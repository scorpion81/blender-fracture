/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor(s): Esteban Tovagliari, Cedric Paille, Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "../ABC_alembic.h"

#include <Alembic/AbcCoreHDF5/All.h>
#include <Alembic/AbcCoreOgawa/All.h>

#include "abc_exporter.h"
#include "abc_camera.h"
#include "abc_mesh.h"
#include "abc_nurbs.h"
#include "abc_util.h"

extern "C" {
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_context.h"
#include "BKE_depsgraph.h"

#include "BLI_math.h"

#include "WM_api.h"
#include "WM_types.h"
}

using Alembic::AbcGeom::ErrorHandler;
using Alembic::AbcGeom::Exception;
using Alembic::AbcGeom::MetaData;
using Alembic::AbcGeom::P3fArraySamplePtr;
using Alembic::AbcGeom::kWrapExisting;

using Alembic::AbcGeom::IArchive;
using Alembic::AbcGeom::ICameraSchema;
using Alembic::AbcGeom::INuPatch;
using Alembic::AbcGeom::IObject;
using Alembic::AbcGeom::IPolyMesh;
using Alembic::AbcGeom::IPolyMeshSchema;
using Alembic::AbcGeom::ISampleSelector;
using Alembic::AbcGeom::IXform;

static IArchive open_archive(const std::string &filename)
{
	Alembic::AbcCoreAbstract::ReadArraySampleCachePtr cache_ptr;
	IArchive archive;

	try {
		archive = IArchive(Alembic::AbcCoreHDF5::ReadArchive(),
		                   filename.c_str(), ErrorHandler::kThrowPolicy,
		                   cache_ptr);
	}
	catch (const Exception &) {
		try {
			archive = IArchive(Alembic::AbcCoreOgawa::ReadArchive(),
			                   filename.c_str(), ErrorHandler::kThrowPolicy,
			                   cache_ptr);
		}
		catch (const Exception &e) {
			std::cerr << e.what() << '\n';
			return IArchive();
		}
	}

	return archive;
}

static size_t update_points(std::pair<IPolyMeshSchema, IObject> schema,
                            const ISampleSelector &sample_sel,
                            MVert *verts, size_t vtx_start, int max_verts = -1,
                            float (*vcos)[3] = 0)
{
	if (!schema.first.valid()) {
		return vtx_start;
	}

	IPolyMeshSchema::Sample smp = schema.first.getValue(sample_sel);
	P3fArraySamplePtr positions = smp.getPositions();

	const size_t vertex_count = positions->size();

	/* don't overflow the buffer! */
	if (max_verts > 0) {
		if ((vtx_start + vertex_count) > max_verts)
			return vtx_start;
	}

	if (verts) {
		int j = vtx_start;
		for (int i = 0; i < vertex_count; ++i, ++j) {
			Imath::V3f pos_in = (*positions)[i];

			verts[j].co[0] = pos_in[0];
			verts[j].co[1] = pos_in[1];
			verts[j].co[2] = pos_in[2];
		}
	}
	else if (vcos) {
		int j = vtx_start;
		for (int i = 0; i < vertex_count; ++i, ++j) {
			Imath::V3f pos_in = (*positions)[i];

			vcos[j][0] = pos_in[0];
			vcos[j][1] = pos_in[1];
			vcos[j][2] = pos_in[2];
		}
	}

	return vtx_start + vertex_count;
}

static void find_mesh_object(const IObject &object, IObject &ret,
                             const std::string &name, bool &found)
{
	if (!object.valid()) {
		return;
	}

	std::vector<std::string> tokens;
	split(name, "/", tokens);

	IObject tmp = object;

	std::vector<std::string>::iterator iter;
	for (iter = tokens.begin(); iter != tokens.end(); ++iter) {
		IObject child = tmp.getChild(*iter);

		if (!child.valid()) {
			continue;
		}

		const MetaData &md = child.getMetaData();

		if (IPolyMesh::matches(md)) {
			ret = child;
			found = true;
			return;
		}

		tmp = child;
	}
}

void ABC_get_vertex_cache(const char *filepath, float time, void *verts,
                          int max_verts, const char *sub_obj, int is_mverts)
{
	IArchive archive = open_archive(filepath);

	if (!archive || !archive.valid()) {
		return;
	}

	IObject object = archive.getTop();

	if (!object.valid()) {
		return;
	}

	IObject mesh_obj;
	bool found = false;

	find_mesh_object(object, mesh_obj, sub_obj, found);

	if (!found) {
		return;
	}

	IPolyMesh mesh(mesh_obj, kWrapExisting);
	IPolyMeshSchema schema = mesh.getSchema();
	ISampleSelector sample_sel(time);

	if (is_mverts) {
		update_points(std::pair<IPolyMeshSchema, IObject>(schema, mesh_obj),
		              sample_sel, (MVert *)verts, 0, max_verts, NULL);
	}
	else {
		float (*vcos)[3] = static_cast<float (*)[3]>(verts);
		update_points(std::pair<IPolyMeshSchema, IObject>(schema, mesh_obj),
		              sample_sel, NULL, 0, max_verts, vcos);
	}
}

int ABC_check_subobject_valid(const char *name, const char *sub_obj)
{
	if ((name[0] == '\0') || (sub_obj[0] == '\0')) {
		return 0;
	}

	IArchive archive = open_archive(name);

	if (!archive.valid()) {
		return 0;
	}

	bool found = false;
	IObject ob;
	find_mesh_object(archive.getTop(), ob, sub_obj, found);

	return (found && ob.valid());
}

int ABC_export(Scene *sce, const char *filename,
               double start, double end,
               double xformstep, double geomstep,
               double shutter_open, double shutter_close,
               int selected_only,
               int uvs, int normals,
               int vcolors,
               int force_meshes,
               int flatten_hierarchy,
               int custom_props_as_geodata,
               int vislayers, int renderable,
               int facesets, int matindices,
               int use_subdiv_schema, int compression, bool packuv,
               int to_forward, int to_up, float scale)
{
	try {
		AbcExportOptions opts(sce);
		opts.startframe = start;
		opts.endframe = end;
		opts.xform_frame_step = xformstep;
		opts.shape_frame_step = geomstep;
		opts.shutter_open = shutter_open;
		opts.shutter_close = shutter_close;
		opts.selected_only = selected_only;
		opts.export_uvs = uvs;
		opts.export_normals = normals;
		opts.export_vcols = vcolors;
		opts.export_subsurfs_as_meshes = force_meshes;
		opts.flatten_hierarchy = flatten_hierarchy;
		opts.export_props_as_geo_params = custom_props_as_geodata;
		opts.visible_layers_only = vislayers;
		opts.renderable_only = renderable;
		opts.use_subdiv_schema = use_subdiv_schema;
		opts.export_ogawa = (compression == ABC_ARCHIVE_OGAWA);
		opts.pack_uv = packuv;
		opts.global_scale = scale;

		// Deprecated
		opts.export_face_sets = facesets;
		opts.export_mat_indices = matindices;

		if (opts.startframe > opts.endframe) {
			std::swap(opts.startframe, opts.endframe);
		}

		opts.do_convert_axis = false;
		if (mat3_from_axis_conversion(1, 2, to_forward, to_up, opts.convert_matrix)) {
			opts.do_convert_axis = true;
		}

		AbcExporter exporter(sce, filename, opts);
		exporter();
	}
	catch (const std::exception &e) {
		std::cout << "Abc Export error: " << e.what() << '\n';
		return BL_ABC_UNKNOWN_ERROR;
	}
	catch (...) {
		return BL_ABC_UNKNOWN_ERROR;
	}

	return BL_ABC_NO_ERR;
}

/* Return whether or not this object is a Maya locator, which is similar to
 * empties used as parent object in Blender. */
static bool is_locator(const IObject &object)
{
	return object.getProperties().getPropertyHeader("locator") != NULL;
}

static void visit_object(const IObject &object,
                         std::vector<AbcObjectReader *> &readers,
                         int from_forward, int from_up)
{
	if (!object.valid()) {
		return;
	}

	for (int i = 0; i < object.getNumChildren(); ++i) {
		IObject child = object.getChild(i);

		if (!child.valid()) {
			continue;
		}

		AbcObjectReader *reader = NULL;

		const MetaData &md = child.getMetaData();

		if (IXform::matches(md) && is_locator(child)) {
			reader = new AbcEmptyReader(child, from_forward, from_up);
		}
		else if (IPolyMesh::matches(md)) {
			reader = new AbcMeshReader(child, from_forward, from_up);
		}
		else if (INuPatch::matches(md)) {
			reader = new AbcNurbsReader(child, from_forward, from_up);
		}
		else if (ICameraSchema::matches(md)) {
			reader = new AbcCameraReader(child, from_forward, from_up);
		}

		if (reader) {
			readers.push_back(reader);
		}

		visit_object(child, readers, from_forward, from_up);
	}
}

static void create_readers(IArchive &archive,
                           std::vector<AbcObjectReader *> &readers,
                           int from_forward, int from_up)
{
	visit_object(archive.getTop(), readers, from_forward, from_up);
}

static Object *find_object(bContext *C, const std::string &name)
{
	Scene *scene = CTX_data_scene(C);
	Base *base;

	for (base = static_cast<Base *>(scene->base.first); base; base = base->next) {
		Object *ob = base->object;

		if (ob->id.name + 2 == name) {
			return ob;
		}
	}

	return NULL;
}

static void create_hierarchy(bContext *C, AbcObjectReader *root)
{
	const IObject &iobjet = root->iobject();
	const std::string &full_name = iobjet.getFullName();

	std::vector<std::string> parts;
	split(full_name, "/", parts);

	Object *parent = NULL;

	std::vector<std::string>::iterator iter;
	for (iter = parts.begin(); iter != parts.end(); ++iter) {
		parent = find_object(C, *iter);

		if (parent != NULL && root->object() != parent) {
			Object *ob = root->object();
			ob->parent = parent;

			DAG_id_tag_update(&ob->id, OB_RECALC_OB);
			DAG_relations_tag_update(CTX_data_main(C));
			WM_main_add_notifier(NC_OBJECT | ND_PARENT, ob);
			break;
		}
	}
}

void ABC_import(bContext *C, const char *filename, int from_forward, int from_up, float scale)
{
	/* get objects strings */
	IArchive archive = open_archive(filename);

	if (!archive.valid()) {
		return;
	}

	std::vector<AbcObjectReader *> readers;
	create_readers(archive, readers, from_forward, from_up);

	std::vector<AbcObjectReader *>::iterator iter;

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		AbcObjectReader *reader = *iter;

		if (reader->valid()) {
			reader->readObjectData(CTX_data_main(C), CTX_data_scene(C), 0.0f);
			reader->readObjectMatrix(0.0f, scale);
		}
	}

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		create_hierarchy(C, *iter);
	}

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		delete *iter;
	}
}
