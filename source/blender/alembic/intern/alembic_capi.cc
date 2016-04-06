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
#include <algorithm>

#include "abc_exporter.h"
#include "abc_camera.h"
#include "abc_mesh.h"
#include "abc_nurbs.h"
#include "abc_util.h"

extern "C" {
#include "MEM_guardedalloc.h"

#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_context.h"
#include "BKE_depsgraph.h"
#include "BKE_global.h"
#include "BKE_idprop.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_scene.h"

#include "BLI_math.h"
#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_threads.h"

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
using Alembic::AbcGeom::IXformSchema;

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

static const int max_alembic_files = 300;

struct AbcInfo {
	Mesh *mesh;
	int current_mat;

	std::string filename, sub_object;
	std::vector< std::pair<IPolyMeshSchema, IObject> > schema_cache;

	std::map<std::string, int> mat_map;
	std::map<std::string, Material *> materials;

	AbcInfo()
	    : mesh(NULL)
	    , current_mat(0)
	{}
};

typedef std::map<void *, AbcInfo> MeshMap;

struct alembicManager {
	alembicManager()
	{
		mutex = BLI_mutex_alloc();
	}

	~alembicManager()
	{
		BLI_mutex_lock(mutex);

		mesh_map.clear();
		mesh_map_cache.clear();
		object_map.clear();
		xform_map.clear();

		std::tr1::unordered_map<std::string, IArchive *>::iterator it = archives.begin();

		for (; it != archives.end(); ++it) {
			/* XXX */
			try {
				delete it->second;
			}
			catch (const Alembic::Util::Exception &e) {
				std::cerr << e.what() << '\n';
			}
		}

		BLI_mutex_unlock(mutex);

		BLI_mutex_free(mutex);
	}

	IArchive *getArchive(const std::string &filename)
	{
		std::tr1::unordered_map<std::string, IArchive*>::iterator it = archives.find(filename);

		if (it != archives.end()) {
			return it->second;
		}

		if ((int)archives.size() > max_alembic_files) {
			std::tr1::unordered_map<IArchive *, std::tr1::unordered_map<std::string, IObject> >::iterator it_ob;
			std::tr1::unordered_map<IArchive *, std::tr1::unordered_map<std::string, IXformSchema> >::iterator it_xf;

			it = archives.begin();

			it_ob = object_map.find(it->second);
			it_xf = xform_map.find(it->second);;

			if (it_ob != object_map.end()) {
				object_map.erase(it_ob);
			}

			if (it_xf != xform_map.end()) {
				xform_map.erase(it_xf);
			}

			delete it->second;
			archives.erase(it);
		}

		IArchive *archive;

		try {
			archive = new IArchive(Alembic::AbcCoreHDF5::ReadArchive(),
			                       filename.c_str(), ErrorHandler::kThrowPolicy,
			                       cache_ptr);
		}
		catch (const Exception &e) {
			try {
				archive = new IArchive(Alembic::AbcCoreOgawa::ReadArchive(),
				                       filename.c_str(), ErrorHandler::kThrowPolicy,
				                       cache_ptr);
			}
			catch (const Exception &e) {
				std::cerr << e.what() << std::endl;
				return NULL;
			}
		}

		archives[filename] = archive;

		return archive;
	}

	void visitObjects(IObject iObj, IObject &ret, std::string path, bool &found)
	{
		if (!iObj.valid()) {
			return;
		}

		IArchive *archive = getArchive(iObj.getArchive().getName());

		const bool archive_ok = object_map.find(archive) != object_map.end();
		std::tr1::unordered_map<std::string, IObject> archive_map;

		if (archive_ok) {
			archive_map = object_map[archive];

			if (archive_map.find(path) != archive_map.end()) {
				ret = archive_map[path];
				found = true;
				return;
			}
		}

		std::vector<std::string> tokens;
		split(path, "/", tokens);

		IObject arch_it = iObj;
		std::string current_path;

		for (int i = 0; i < tokens.size(); ++i) {
			current_path += "/" + tokens[i];
			if (archive_ok) {
				if (archive_map.find(current_path) != archive_map.end()) {
					arch_it = archive_map[current_path];
					continue;
				}
			}
			IObject obj = arch_it.getChild(tokens[i]);
			arch_it = obj;

			if (!obj.valid())
				return;

			if (archive_ok) {
				archive_map[current_path] = arch_it;
			}
		}
		ret = arch_it;
		if (ret.valid())
			found = true;
	}


	IXformSchema getXFormSchema(std::string filename, std::string path, bool &found) {
		IArchive *archive = getArchive(filename);

		if (!archive || !archive->valid())
			return IXformSchema();

		std::tr1::unordered_map< IArchive*, std::tr1::unordered_map<std::string, IXformSchema> >::iterator it = xform_map.find(archive);
		if (it != xform_map.end()) {
			if (it->second.find(path) != it->second.end()) {
				found = true;
				return it->second[path];
			}
		}
		IObject arch_it;
		visitObjects(archive->getTop(), arch_it, path, found);

		if (!found)
			return IXformSchema();

		const MetaData &md = arch_it.getMetaData();
		if (IXformSchema::matches(md)) {
			IXform x(arch_it, kWrapExisting);
			IXformSchema &schema(x.getSchema());
			xform_map[archive][path] = schema;
			return schema;
		}

		return IXformSchema();
	}

	IObject getObject(std::string filename, std::string path, bool &found)
	{
		IObject ret;

		IArchive *archive = getArchive(filename);

		if (!archive || !archive->valid())
			return ret;

		if (object_map.find(archive) != object_map.end()) {
			if (object_map[archive].find(path) != object_map[archive].end()) {
				found = true;
				return object_map[archive][path];
			}
		}

		visitObjects(archive->getTop(), ret, path, found);

		if (found) {
			object_map[archive][path] = ret;
			found = true;
			return ret;
		}

		return IObject();
	}

	// datas
	std::tr1::unordered_map< IArchive*, std::tr1::unordered_map<std::string, IObject> > object_map;
	std::tr1::unordered_map< IArchive*, std::tr1::unordered_map<std::string, IXformSchema> > xform_map;
	std::tr1::unordered_map<std::string, IArchive*> archives;
	::Alembic::AbcCoreAbstract::ReadArraySampleCachePtr cache_ptr;
	ThreadMutex *mutex;

	MeshMap 				  mesh_map;
	MeshMap 				  mesh_map_cache;
};

/* TODO */
static alembicManager __abc_manager;
static alembicManager *abc_manager = &__abc_manager;// new alembicManager();

#if 0
static Material *findMaterial(const char *name)
{
	Main *bmain = G.main;
	Material *material, *found_material = NULL;

	for (material = (Material*)bmain->mat.first; material; material = (Material*)material->id.next) {

		if (BLI_strcaseeq(material->id.name+2, name) == true) {
			found_material = material;
			break;
		}
	}

	return found_material;
}

static void ABC_mutex_lock()
{
	BLI_mutex_lock(abc_manager->mutex);
}

static void ABC_mutex_unlock()
{
	BLI_mutex_unlock(abc_manager->mutex);
}
#endif

static void visitObject(IObject iObj, std::vector< std::pair<IPolyMeshSchema, IObject> > &schemas, std::string sub_obj)
{
	if (!iObj.valid())
		return;

	IObject ret;
	bool found = false;

	abc_manager->visitObjects(iObj, ret, sub_obj, found);
	const MetaData &md = ret.getMetaData();

	if (IPolyMesh::matches(md) && found) {
		IPolyMesh abc_mesh(ret, kWrapExisting);
		IPolyMeshSchema schem = abc_mesh.getSchema();
		schemas.push_back(std::pair<IPolyMeshSchema, IObject>(schem, ret));
	}
}

#if 0
static void visitObjectMatrix(IObject iObj, std::string abc_subobject, float time, float mat[][4])
{
	if (!iObj.valid())
		return;

	for (int i = 0;i < iObj.getNumChildren(); ++i) {
		IObject child(iObj, iObj.getChildHeader(i).getName());

		if (!child.valid())
			continue;

		const MetaData &md = child.getMetaData();

		if (IXformSchema::matches(md) && child.getFullName() == abc_subobject) {
			XformSample xs;
			IXform x(child, kWrapExisting);
			IXformSchema &schema(x.getSchema());
			ISampleSelector sample_sel(time);
			schema.get(xs, sample_sel);
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; j++) {
					mat[i][j] = xs.getMatrix()[i][j];
				}
			}
		}

		visitObjectMatrix(IObject(child), abc_subobject, time, mat);
	}
}

static void getIObjectAsMesh(std::pair<IPolyMeshSchema, IObject> schema,
                             const ISampleSelector &sample_sel, Mesh *blender_mesh,
                             void *key, bool assign_mat, int from_forward, int from_up)
{
	size_t idx_pos  = blender_mesh->totpoly;
	size_t vtx_pos  = blender_mesh->totvert;
	size_t loop_pos = blender_mesh->totloop;

	if (!schema.first.valid()) {
		return;
	}

	IV2fGeomParam 			uv 		= schema.first.getUVsParam();
	IPolyMeshSchema::Sample smp;
	P3fArraySamplePtr   	positions;

	IV2fGeomParam::Sample::samp_ptr_type uvsamp_vals;

	smp 		= schema.first.getValue(sample_sel);
	positions 	= smp.getPositions();

	Int32ArraySamplePtr face_indices = smp.getFaceIndices();
	Int32ArraySamplePtr face_counts  = smp.getFaceCounts();
	size_t 				vertex_count = positions->size();
	size_t 				num_poly 	 = face_counts->size();
	size_t 				num_loops 	 = face_indices->size();

	std::vector<std::string> face_sets;
	schema.first.getFaceSetNames(face_sets);

	mesh_utils::mesh_add_verts(blender_mesh, vertex_count);
	mesh_utils::mesh_add_mpolygons(blender_mesh, num_poly);
	mesh_utils::mesh_add_mloops(blender_mesh, num_loops);

	if (uv.valid()) {
		IV2fGeomParam::Sample uvsamp = uv.getExpandedValue();
		uvsamp_vals = uvsamp.getVals();
	}

	float mat[3][3];
	bool use_mat = false;

	if (mat3_from_axis_conversion(from_forward, from_up, 1, 2, mat)) {
		use_mat = true;
	}

	int j = vtx_pos;
	for (int i = 0; i < vertex_count; ++i, ++j) {
		MVert &mvert = blender_mesh->mvert[j];
		V3f pos_in = (*positions)[i];

		mvert.co[0] = pos_in[0];
		mvert.co[1] = pos_in[1];
		mvert.co[2] = pos_in[2];

		mvert.bweight = 0;
	}

	if (use_mat) {
		j = vtx_pos;
		for (int i = 0; i < vertex_count; ++i, ++j) {
			MVert &mvert = blender_mesh->mvert[j];
			mul_m3_v3(mat, mvert.co);
		}
	}

	j = idx_pos;
	int loopcount = loop_pos;
	for (int i = 0; i < num_poly; ++i, ++j) {
		int face_size = (*face_counts)[i];
		MPoly &poly = blender_mesh->mpoly[j];

		poly.loopstart = loopcount;
		poly.totloop   = face_size;

		// TODO : reverse
		int rev_loop = loopcount;
		for (int f = face_size; f-- ;) {
			MLoop &loop 	= blender_mesh->mloop[rev_loop+f];
			MLoopUV &loopuv = blender_mesh->mloopuv[rev_loop+f];

			if (uvsamp_vals) {
				loopuv.uv[0] = (*uvsamp_vals)[loopcount][0];
				loopuv.uv[1] = (*uvsamp_vals)[loopcount][1];
			}

			loop.v = (*face_indices)[loopcount++];
		}
	}

	if (assign_mat) {
		std::map<std::string, int> &mat_map = abc_manager->mesh_map[key].mat_map;
		int &current_mat = abc_manager->mesh_map[key].current_mat;

		for (int i = 0; i < face_sets.size(); ++i) {
			std::string grp_name = face_sets[i];

			if (mat_map.find(grp_name) == mat_map.end()) {
				mat_map[grp_name] = 1 + current_mat++;
			}

			int assigned_mat = mat_map[grp_name];

			IFaceSet faceset 					= schema.first.getFaceSet(face_sets[i]);
			if (!faceset.valid())
				continue;
			IFaceSetSchema face_schem 			= faceset.getSchema();
			IFaceSetSchema::Sample face_sample 	= face_schem.getValue(sample_sel);
			Int32ArraySamplePtr group_faces 	= face_sample.getFaces();
			size_t num_group_faces 				= group_faces->size();

			for (size_t l = 0; l < num_group_faces; l++) {
				size_t pos = (*group_faces)[l]+idx_pos;
				if (pos >= blender_mesh->totpoly) {
					std::cerr << "Faceset overflow on " << faceset.getName() << std::endl;
					break;
				}

				MPoly  &poly = blender_mesh->mpoly[pos];
				poly.mat_nr = assigned_mat - 1;
			}
		}
	}

	// Compute edge array is done here
	BKE_mesh_validate(blender_mesh, false, false);
}
#endif

static size_t updatePoints(std::pair<IPolyMeshSchema, IObject> schema, const ISampleSelector &sample_sel, MVert *verts, size_t vtx_start, int max_verts = -1, float (*vcos)[3] = 0) {

	if (!schema.first.valid()) {
		return vtx_start;
	}

	IPolyMeshSchema::Sample smp = schema.first.getValue(sample_sel);
	P3fArraySamplePtr positions = smp.getPositions();

	const size_t vertex_count = positions->size();

	// We don't want to overflow the buffer !
	if (max_verts > 0) {
		if ((vtx_start + vertex_count) > max_verts)
			return vtx_start;
	}

	if (verts) {
		int j = vtx_start;
		for (int i = 0; i < vertex_count; ++i, ++j) {
			Imath::V3f pos_in = (*positions)[i];

			// swap from Y-Up to Z-Up
			verts[j].co[0] = pos_in[0];
			verts[j].co[1] = -pos_in[2];
			verts[j].co[2] = pos_in[1];
		}
	}
	else if (vcos) {
		int j = vtx_start;
		for (int i = 0; i < vertex_count; ++i, ++j) {
			Imath::V3f pos_in = (*positions)[i];

			// swap from Y-Up to Z-Up
			vcos[j][0] = pos_in[0];
			vcos[j][1] = -pos_in[2];
			vcos[j][2] = pos_in[1];
		}
	}

	return vtx_start + vertex_count;
}

void ABC_destroy_mesh_data(void *key)
{
	if (abc_manager->mesh_map.find(key) != abc_manager->mesh_map.end()) {
		AbcInfo *info = &abc_manager->mesh_map[key];

		if (info->mesh) {
			BKE_mesh_free(info->mesh, true);
		}

		info->schema_cache.clear();
		info->mesh = NULL;
	}
}

#if 0
static void ABC_destroy_key(void *key)
{
	MeshMap::iterator it;
	if ((it = abc_manager->mesh_map.find(key)) != abc_manager->mesh_map.end()) {
		AbcInfo *info = &abc_manager->mesh_map[key];

		if (info->mesh) {
			BKE_mesh_free(info->mesh, true);
		}

		abc_manager->mesh_map.erase(it);
	}
}

static Mesh *ABC_get_mesh(IArchive *archive, float time, void *key, int assign_mats,
                          const char *sub_obj, bool *p_only, int from_forward, int from_up)
{
	IObject iObj = archive->getTop();

	if (!iObj.valid()) {
		std::cerr << "Cannot get top of " << archive->getName() << '\n';
		return NULL;
	}

	MeshMap::iterator mit = abc_manager->mesh_map.find(key);

	if (mit == abc_manager->mesh_map.end()) {
		Mesh *me = BKE_mesh_add(G.main, "abc_tmp");
		AbcInfo info;
		info.filename 	= archive->getName();
		info.mesh 		= me;
		info.sub_object = sub_obj;
		visitObject(iObj, info.schema_cache, sub_obj);
		abc_manager->mesh_map[key] 	= info;
		*p_only = false;
	}
	else if (mit->second.filename != archive->getName() || mit->second.sub_object != sub_obj) {
		if (mit->second.mesh) {
			BKE_mesh_free(mit->second.mesh, true);
		}

		Mesh *me = BKE_mesh_add(G.main, "abc_tmp");
		AbcInfo info;
		info.filename 	= archive->getName();
		info.mesh 		= me;
		info.sub_object = sub_obj;
		visitObject(iObj, info.schema_cache, sub_obj);
		abc_manager->mesh_map[key] 	= info;
		*p_only = false;
	}
	else {
		return NULL;
	}

	AbcInfo &meshmap = abc_manager->mesh_map[key];
	Mesh *mesh = meshmap.mesh;

	if (!mesh && *p_only) {
		return NULL;
	}

	ISampleSelector sample_sel(time);
	std::vector< std::pair<IPolyMeshSchema, IObject> >::iterator it;

	size_t vtx_count = 0;
	for (it = meshmap.schema_cache.begin(); it != meshmap.schema_cache.end(); ++it) {
		if (*p_only) {
			vtx_count = updatePoints(*it, sample_sel, mesh->mvert, vtx_count);
		}
		else {
			getIObjectAsMesh(*it, sample_sel, mesh, key, assign_mats, from_forward, from_up);
		}
	}

	if (!*p_only) {
		BKE_mesh_validate(mesh, false, true);
	}

	BKE_mesh_calc_normals(mesh);

	return mesh;
}

static void ABC_apply_materials(Object *ob, void *key)
{
	AbcInfo &meshmap = abc_manager->mesh_map[key];

	// Clean up slots
	while (object_remove_material_slot(ob));

	bool can_assign = true;
	std::map<std::string, int>::iterator it = meshmap.mat_map.begin();
	int matcount = 0;
	for (; it != abc_manager->mesh_map[key].mat_map.end(); ++it, matcount++) {
		Material *curmat = give_current_material(ob, matcount);
		if (curmat == NULL) {
			if (!object_add_material_slot(ob)) {
				can_assign = false;
				break;
			}
		}
	}

	if (can_assign) {
		it = abc_manager->mesh_map[key].mat_map.begin();
		for (; it != meshmap.mat_map.end(); ++it) {

			Material *assigned_name;
			std::string mat_name = it->first;

			if (findMaterial(mat_name.c_str()) != NULL) {//meshmap.materials.find(mat_name) != meshmap.materials.end()) {
				assigned_name = findMaterial(mat_name.c_str());//meshmap.materials[mat_name];
			}
			else {
				assigned_name = BKE_material_add(G.main, mat_name.c_str());
				meshmap.materials[mat_name] = assigned_name;
			}

			assign_material(ob, assigned_name, it->second, BKE_MAT_ASSIGN_OBJECT);
		}
	}
}
#endif

void ABC_get_vertex_cache(const char *filepath, float time, void *key, void *verts, int max_verts, const char *sub_obj, int is_mverts)
{
	std::string file_path = filepath;
	std::string sub_object = sub_obj;

	if (file_path.empty()) {
		std::cerr << __func__ << ": file path is empty!\n";
		return;
	}

	IArchive *archive = abc_manager->getArchive(file_path);

	if (!archive || !archive->valid()) {
		return;
	}

	IObject iObj = archive->getTop();

	if (!iObj.valid()) {
		return;
	}

	MeshMap::iterator mit = abc_manager->mesh_map_cache.find(key);

	if (mit == abc_manager->mesh_map_cache.end()) {
		AbcInfo info;
		info.filename 	= file_path;
		info.sub_object = sub_object;
		visitObject(iObj, info.schema_cache, sub_object);
		abc_manager->mesh_map_cache[key] 	= info;
	}
	else if (mit->second.filename != file_path || mit->second.sub_object != sub_object) {
		if (mit->second.mesh)
			BKE_mesh_free(mit->second.mesh, true);
		AbcInfo info;
		info.filename 	= file_path;
		info.sub_object = sub_object;
		visitObject(iObj, info.schema_cache, sub_object);
		abc_manager->mesh_map_cache[key] 	= info;
	}

	ISampleSelector sample_sel(time);
	std::vector< std::pair<IPolyMeshSchema, IObject> >::iterator it;

	size_t vtx_count = 0;
	for (it = abc_manager->mesh_map_cache[key].schema_cache.begin(); it != abc_manager->mesh_map_cache[key].schema_cache.end(); ++it) {
		if (is_mverts)
			vtx_count = updatePoints(*it, sample_sel, (MVert*)verts, vtx_count, max_verts, NULL);
		else {
			float (*vcos)[3] = static_cast<float (*)[3]>(verts);
			vtx_count = updatePoints(*it, sample_sel, NULL, vtx_count, max_verts, vcos);
		}
	}
}

int ABC_check_subobject_valid(const char *name, const char *sub_obj)
{
	if (name[0] == '\0')
		return 0;

	IArchive *archive = abc_manager->getArchive(name);

	if (!archive) {
		std::cerr << "Couldn't find archive!\n";
		return 0;
	}

	if (!archive->valid()) {
		std::cerr << "Alembic archive is not valid!\n";
		return 0;
	}

	if (sub_obj[0] == '\0') {
		std::cerr << "Subobject name is empty!\n";
		return 0;
	}

	bool found = false;
	abc_manager->getObject(name, sub_obj, found);

	return found;
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
               int use_subdiv_schema, bool ogawa, bool packuv,
               int to_forward, int to_up)
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
		opts.export_ogawa = ogawa;
		opts.pack_uv = packuv;
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
		std::cout << "Abc Export error: " << e.what() << std::endl;
		return BL_ABC_UNKNOWN_ERROR;
	}
	catch (...) {
		return BL_ABC_UNKNOWN_ERROR;
	}

	return BL_ABC_NO_ERR;
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

		if (IPolyMesh::matches(md)) {
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

static void create_readers(IArchive *archive,
                           std::vector<AbcObjectReader *> &readers,
                           int from_forward, int from_up)
{
	visit_object(archive->getTop(), readers, from_forward, from_up);
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

void ABC_import(bContext *C, const char *filename, int from_forward, int from_up)
{
	/* get objects strings */
	IArchive *archive = abc_manager->getArchive(filename);

	if (!archive || !archive->valid()) {
		return;
	}

	std::vector<AbcObjectReader *> readers;
	create_readers(archive, readers, from_forward, from_up);

	std::vector<AbcObjectReader *>::iterator iter;

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		AbcObjectReader *reader = *iter;

		if (reader->valid()) {
			reader->readObjectData(CTX_data_main(C), CTX_data_scene(C), 0.0f);
			reader->readObjectMatrix(0.0f);
		}
	}

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		create_hierarchy(C, *iter);
	}

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		delete *iter;
	}
}
