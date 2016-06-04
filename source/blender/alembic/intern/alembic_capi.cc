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
#include <Alembic/AbcMaterial/IMaterial.h>

#include "abc_exporter.h"
#include "abc_camera.h"
#include "abc_hair.h"
#include "abc_mesh.h"
#include "abc_nurbs.h"
#include "abc_points.h"
#include "abc_util.h"

extern "C" {
#include "MEM_guardedalloc.h"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_context.h"
#include "BKE_depsgraph.h"
#include "BKE_global.h"

/* SpaceType struct has a member called 'new' which obviously conflicts with C++
 * so temporarily redefining the new keyword to make it compile. */
#define new extern_new
#include "BKE_screen.h"
#undef new

#include "BLI_math.h"
#include "BLI_string.h"

#include "WM_api.h"
#include "WM_types.h"
}

using Alembic::AbcGeom::ErrorHandler;
using Alembic::AbcGeom::Exception;
using Alembic::AbcGeom::MetaData;
using Alembic::AbcGeom::P3fArraySamplePtr;
using Alembic::AbcGeom::kWrapExisting;

using Alembic::AbcGeom::IArchive;
using Alembic::AbcGeom::ICamera;
using Alembic::AbcGeom::ICurves;
using Alembic::AbcGeom::IFaceSet;
using Alembic::AbcGeom::ILight;
using Alembic::AbcGeom::INuPatch;
using Alembic::AbcGeom::IObject;
using Alembic::AbcGeom::IPoints;
using Alembic::AbcGeom::IPolyMesh;
using Alembic::AbcGeom::IPolyMeshSchema;
using Alembic::AbcGeom::ISampleSelector;
using Alembic::AbcGeom::ISubD;
using Alembic::AbcGeom::IXform;
using Alembic::AbcGeom::IXformSchema;
using Alembic::AbcGeom::XformSample;

using Alembic::AbcMaterial::IMaterial;

int ABC_get_version()
{
	return ALEMBIC_LIBRARY_VERSION;
}

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

static void find_iobject(const IObject &object, IObject &ret,
                        const std::string &path)
{
	if (!object.valid()) {
		return;
	}

	std::vector<std::string> tokens;
	split(path, '/', tokens);

	IObject tmp = object;

	std::vector<std::string>::iterator iter;
	for (iter = tokens.begin(); iter != tokens.end(); ++iter) {
		IObject child = tmp.getChild(*iter);
		tmp = child;
	}

	ret = tmp;
}

void ABC_get_vertex_cache(const char *filepath, float time, void *verts,
                          int max_verts, const char *object_path, int is_mverts)
{
	IArchive archive = open_archive(filepath);

	if (!archive || !archive.valid()) {
		return;
	}

	IObject top = archive.getTop();

	if (!top.valid()) {
		return;
	}

	IObject iobject;
	find_iobject(top, iobject, object_path);

	if (!IPolyMesh::matches(iobject.getHeader())) {
		return;
	}

	IPolyMesh mesh(iobject, kWrapExisting);
	IPolyMeshSchema schema = mesh.getSchema();
	ISampleSelector sample_sel(time);

	if (is_mverts) {
		update_points(std::pair<IPolyMeshSchema, IObject>(schema, iobject),
		              sample_sel, (MVert *)verts, 0, max_verts, NULL);
	}
	else {
		float (*vcos)[3] = static_cast<float (*)[3]>(verts);
		update_points(std::pair<IPolyMeshSchema, IObject>(schema, iobject),
		              sample_sel, NULL, 0, max_verts, vcos);
	}
}

int ABC_check_subobject_valid(const char *filepath, const char *object_path)
{
	if ((filepath[0] == '\0') || (object_path[0] == '\0')) {
		return 0;
	}

	IArchive archive = open_archive(filepath);

	if (!archive.valid()) {
		return 0;
	}

	IObject ob;
	find_iobject(archive.getTop(), ob, object_path);

	return (ob.valid());
}

struct ExportJobData {
	Scene *scene;

	char filename[1024];
	ExportSettings settings;

	short *stop;
	short *do_update;
	float *progress;
};

static void export_startjob(void *customdata, short *stop, short *do_update, float *progress)
{
	ExportJobData *data = static_cast<ExportJobData *>(customdata);

	data->stop = stop;
	data->do_update = do_update;
	data->progress = progress;

	/* XXX annoying hack: needed to prevent data corruption when changing
	 * scene frame in separate threads
     */
    G.is_rendering = true;
    BKE_spacedata_draw_locks(true);

	try {
		AbcExporter exporter(data->scene, data->filename, data->settings);
		exporter(*data->progress);
	}
	catch (const std::exception &e) {
		std::cerr << "Abc Export error: " << e.what() << '\n';
	}
	catch (...) {
		std::cerr << "Abc Export error\n";
	}
}

static void export_endjob(void */*customdata*/)
{
    G.is_rendering = false;
    BKE_spacedata_draw_locks(false);
}

int ABC_export(Scene *scene, bContext *C, const char *filepath,
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
               int use_subdiv_schema, int compression, bool packuv, float scale)
{
	ExportJobData *job = static_cast<ExportJobData *>(MEM_mallocN(sizeof(ExportJobData), "ExportJobData"));
	job->scene = scene;
	BLI_strncpy(job->filename, filepath, 1024);

	job->settings.scene = job->scene;
	job->settings.startframe = start;
	job->settings.endframe = end;
	job->settings.xform_frame_step = xformstep;
	job->settings.shape_frame_step = geomstep;
	job->settings.shutter_open = shutter_open;
	job->settings.shutter_close = shutter_close;
	job->settings.selected_only = selected_only;
	job->settings.export_uvs = uvs;
	job->settings.export_normals = normals;
	job->settings.export_vcols = vcolors;
	job->settings.export_subsurfs_as_meshes = force_meshes;
	job->settings.flatten_hierarchy = flatten_hierarchy;
	job->settings.export_props_as_geo_params = custom_props_as_geodata;
	job->settings.visible_layers_only = vislayers;
	job->settings.renderable_only = renderable;
	job->settings.use_subdiv_schema = use_subdiv_schema;
	job->settings.export_ogawa = (compression == ABC_ARCHIVE_OGAWA);
	job->settings.pack_uv = packuv;
	job->settings.global_scale = scale;

	// Deprecated
	job->settings.export_face_sets = facesets;
	job->settings.export_mat_indices = matindices;

	if (job->settings.startframe > job->settings.endframe) {
		std::swap(job->settings.startframe, job->settings.endframe);
	}

	wmJob *wm_job = WM_jobs_get(CTX_wm_manager(C),
	                            CTX_wm_window(C),
	                            job->scene,
	                            "Alembic Export",
	                            WM_JOB_PROGRESS,
	                            WM_JOB_TYPE_ALEMBIC);

	/* setup job */
	WM_jobs_customdata_set(wm_job, job, MEM_freeN);
	WM_jobs_timer(wm_job, 0.1, NC_SCENE | ND_COMPO_RESULT, NC_SCENE | ND_COMPO_RESULT);
	WM_jobs_callbacks(wm_job, export_startjob, NULL, NULL, export_endjob);

	WM_jobs_start(CTX_wm_manager(C), wm_job);

	return BL_ABC_NO_ERR;
}

/* ********************** Import file ********************** */

static void visit_object(const IObject &object,
                         std::vector<AbcObjectReader *> &readers,
                         ImportSettings &settings)
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

		if (IXform::matches(md)) {
			bool create_xform = false;

			if (is_locator(child)) {
				create_xform = true;
			}
			else {
				/* Avoid creating an empty object if the child of this transform
				 * is not a transform (that is an empty). */
				if (child.getNumChildren() == 1) {
					if (IXform::matches(child.getChild(0).getMetaData())) {
						create_xform = true;
					}
#if 0
					else {
						std::cerr << "Skipping " << child.getFullName() << '\n';
					}
#endif
				}
				else {
					create_xform = true;
				}
			}

			if (create_xform) {
				reader = new AbcEmptyReader(child, settings);
			}
		}
		else if (IPolyMesh::matches(md)) {
			reader = new AbcMeshReader(child, settings, false);
		}
		else if (ISubD::matches(md)) {
			reader = new AbcMeshReader(child, settings, true);
		}
		else if (INuPatch::matches(md)) {
			reader = new AbcNurbsReader(child, settings);
		}
		else if (ICamera::matches(md)) {
			reader = new AbcCameraReader(child, settings);
		}
		else if (IPoints::matches(md)) {
			reader = new AbcPointsReader(child, settings);
		}
		else if (IMaterial::matches(md)) {
			/* Pass for now. */
		}
		else if (ILight::matches(md)) {
			/* Pass for now. */
		}
		else if (IFaceSet::matches(md)) {
			/* Pass, those are handled in the mesh reader. */
		}
		else if (ICurves::matches(md)) {
			reader = new AbcHairReader(child, settings);
		}
		else {
			assert(false);
		}

		if (reader) {
			readers.push_back(reader);
		}

		visit_object(child, readers, settings);
	}
}

static Object *find_object(Scene *scene, const std::string &name)
{
	Base *base;

	for (base = static_cast<Base *>(scene->base.first); base; base = base->next) {
		Object *ob = base->object;

		if (ob->id.name + 2 == name) {
			return ob;
		}
	}

	return NULL;
}

static void create_hierarchy(Main *bmain, Scene *scene, AbcObjectReader *root)
{
	const IObject &iobjet = root->iobject();
	const std::string &full_name = iobjet.getFullName();

	std::vector<std::string> parts;
	split(full_name, '/', parts);

	Object *parent = NULL;

	std::vector<std::string>::reverse_iterator iter;
	for (iter = parts.rbegin() + 1; iter != parts.rend(); ++iter) {
		parent = find_object(scene, *iter);

		if (parent != NULL && root->object() != parent) {
			Object *ob = root->object();
			ob->parent = parent;

			DAG_id_tag_update(&ob->id, OB_RECALC_OB);
			DAG_relations_tag_update(bmain);
			WM_main_add_notifier(NC_OBJECT | ND_PARENT, ob);

			return;
		}
	}
}

struct ImportJobData {
	Main *bmain;
	Scene *scene;

	char filename[1024];
	ImportSettings settings;

	short *stop;
	short *do_update;
	float *progress;
};

static void import_startjob(void *cjv, short *stop, short *do_update, float *progress)
{
	ImportJobData *data = static_cast<ImportJobData *>(cjv);

	data->stop = stop;
	data->do_update = do_update;
	data->progress = progress;

	IArchive archive = open_archive(data->filename);

	if (!archive.valid()) {
		return;
	}

	*data->do_update = true;
	*data->progress = 0.05f;

	std::vector<AbcObjectReader *> readers;
	visit_object(archive.getTop(), readers, data->settings);

	*data->do_update = true;
	*data->progress = 0.1f;

	const float size = static_cast<float>(readers.size());
	size_t i = 0;

	std::vector<AbcObjectReader *>::iterator iter;

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		AbcObjectReader *reader = *iter;

		if (reader->valid()) {
			reader->readObjectData(data->bmain, data->scene, 0.0f);
			reader->readObjectMatrix(0.0f);
		}

		*data->progress = 0.1f + 0.6f * (++i / size);
	}

	i = 0;
	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		create_hierarchy(data->bmain, data->scene, *iter);
		*data->progress = 0.7f + 0.3f * (++i / size);
	}

	for (iter = readers.begin(); iter != readers.end(); ++iter) {
		delete *iter;
	}

	WM_main_add_notifier(NC_SCENE | ND_FRAME, data->scene);
}

void ABC_import(bContext *C, const char *filepath, float scale, bool is_sequence)
{
	ImportJobData *job = static_cast<ImportJobData *>(MEM_mallocN(sizeof(ImportJobData), "ImportJobData"));
	job->bmain = CTX_data_main(C);
	job->scene = CTX_data_scene(C);
	BLI_strncpy(job->filename, filepath, 1024);

	job->settings.scale = scale;
	job->settings.is_sequence = is_sequence;

	wmJob *wm_job = WM_jobs_get(CTX_wm_manager(C),
	                            CTX_wm_window(C),
	                            job->scene,
	                            "Alembic Import",
	                            WM_JOB_PROGRESS,
	                            WM_JOB_TYPE_ALEMBIC);

	/* setup job */
	WM_jobs_customdata_set(wm_job, job, MEM_freeN);
	WM_jobs_timer(wm_job, 0.1, NC_SCENE | ND_COMPO_RESULT, NC_SCENE | ND_COMPO_RESULT);
	WM_jobs_callbacks(wm_job, import_startjob, NULL, NULL, NULL);

	WM_jobs_start(CTX_wm_manager(C), wm_job);
}

/* ******************************* */

void ABC_get_transform(Object *ob, const char *filepath, const char *object_path, float r_mat[4][4], float time)
{
	IArchive archive = open_archive(filepath);

	if (!archive.valid()) {
		return;
	}

	IObject tmp;
	find_iobject(archive.getTop(), tmp, object_path);

	IXform ixform;

	if (IXform::matches(tmp.getHeader())) {
		ixform = IXform(tmp, kWrapExisting);
	}
	else {
		ixform = IXform(tmp.getParent(), kWrapExisting);
	}

	IXformSchema schema = ixform.getSchema();

	if (!schema.valid()) {
		return;
	}

	ISampleSelector sample_sel(time);

	create_input_transform(sample_sel, ixform, ob, r_mat);
}

/* ***************************************** */

#ifdef PARALLEL_READ
#include "BLI_task.h"

struct MeshReadData {
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;

	Alembic::Abc::P3fArraySamplePtr positions;
	Alembic::Abc::Int32ArraySamplePtr face_indices;
	Alembic::Abc::Int32ArraySamplePtr face_counts;
};

static void read_verts_cb(void *userdata, const int x)
{
	MeshReadData *data = static_cast<MeshReadData *>(userdata);

	MVert &mvert = data->mverts[x];
	Imath::V3f pos_in = (*data->positions)[x];

	/* Convert Y-up to Z-up. */
	mvert.co[0] = pos_in[0];
	mvert.co[1] = -pos_in[2];
	mvert.co[2] = pos_in[1];
	mvert.bweight = 0;
}

static void read_faces_cb(void *userdata, const int x)
{
	MeshReadData *data = static_cast<MeshReadData *>(userdata);

	int face_size = (*data->face_counts)[x];
	MPoly &poly = data->mpolys[x];

	poly.loopstart = loopcount;
	poly.totloop = face_size;

	/* TODO: reverse */
	int rev_loop = loopcount;
	for (int f = face_size; f-- ;) {
		MLoop &loop = data->mloops[rev_loop + f];
		loop.v = (*data->face_indices)[loopcount++];
	}
}
#endif

static DerivedMesh *read_mesh_sample(DerivedMesh *dm, const IObject &iobject, const float time)
{
	IPolyMesh mesh(iobject, kWrapExisting);
	IPolyMeshSchema schema = mesh.getSchema();
	ISampleSelector sample_sel(time);
	const IPolyMeshSchema::Sample sample = schema.getValue(sample_sel);

	const P3fArraySamplePtr &positions = sample.getPositions();
	const Alembic::Abc::Int32ArraySamplePtr &face_indices = sample.getFaceIndices();
	const Alembic::Abc::Int32ArraySamplePtr &face_counts = sample.getFaceCounts();

	if (dm->getNumVerts(dm) != positions->size()) {
		dm = CDDM_new(positions->size(), 0, 0, face_indices->size(), face_counts->size());
	}

	MVert *mverts = dm->getVertArray(dm);
	MPoly *mpolys = dm->getPolyArray(dm);
	MLoop *mloops = dm->getLoopArray(dm);

#ifdef PARALLEL_READ
	MeshReadData data;
	data.mverts = mverts;
	data.mpolys = mpolys;
	data.mloops = mloops;
	data.positions = positions;
	data.face_indices = face_indices;
	data.face_counts = face_counts;

	BLI_task_parallel_range(0, num_verts, &data, read_verts_cb, num_verts > 10000);
	BLI_task_parallel_range(0, num_polys, &data, read_faces_cb, num_verts > 10000);
#else
	read_mverts(mverts, positions);
	read_mpolys(mpolys, mloops, NULL, face_indices, face_counts);
#endif

	CDDM_calc_edges(dm);
	dm->dirty = static_cast<DMDirtyFlag>(static_cast<int>(dm->dirty) | static_cast<int>(DM_DIRTY_NORMALS));

	return dm;
}

using Alembic::Abc::ObjectHeader;
using Alembic::AbcGeom::IPointsSchema;
using Alembic::AbcGeom::ICurvesSchema;

static DerivedMesh *read_points_sample(DerivedMesh *dm, const IObject &iobject, const float time)
{
	IPoints points(iobject, kWrapExisting);
	IPointsSchema schema = points.getSchema();
	ISampleSelector sample_sel(time);
	const IPointsSchema::Sample sample = schema.getValue(sample_sel);

	const P3fArraySamplePtr &positions = sample.getPositions();

	if (dm->getNumVerts(dm) != positions->size()) {
		dm = CDDM_new(positions->size(), 0, 0, 0, 0);
	}

	MVert *mverts = dm->getVertArray(dm);

	read_mverts(mverts, positions);

	return dm;
}

void read_curves_sample(float (*vertexCos)[3], int max_verts, const IObject &iobject, const float time)
{
	ICurves points(iobject, kWrapExisting);
	ICurvesSchema schema = points.getSchema();
	ISampleSelector sample_sel(time);
	const ICurvesSchema::Sample sample = schema.getValue(sample_sel);

	const P3fArraySamplePtr &positions = sample.getPositions();

	for (int i = 0; i < min_ff(max_verts, positions->size()); ++i) {
		float *vert = vertexCos[i];
		Imath::V3f pos_in = (*positions)[i];

		/* Convert Y-up to Z-up. */
		vert[0] = pos_in[0];
		vert[1] = -pos_in[2];
		vert[2] = pos_in[1];
	}
}

DerivedMesh *ABC_read_mesh(DerivedMesh *dm, const char *filepath, const char *object_path, const float time)
{
	IArchive archive = open_archive(filepath);

	if (!archive.valid()) {
		return NULL;
	}

	IObject iobject;
	find_iobject(archive.getTop(), iobject, object_path);

	if (!iobject.valid()) {
		return NULL;
	}

	const Alembic::Abc::ObjectHeader &header = iobject.getHeader();

	if (IPolyMesh::matches(header)) {
		return read_mesh_sample(dm, iobject, time);
	}
	else if (IPoints::matches(header)) {
		return read_points_sample(dm, iobject, time);
	}

	return NULL;
}

void ABC_read_vertex_cache(const char *filepath, const char *object_path, const float time,
                           float (*vertexCos)[3], int max_verts)
{
	IArchive archive = open_archive(filepath);

	if (!archive.valid()) {
		return;
	}

	IObject iobject;
	find_iobject(archive.getTop(), iobject, object_path);

	if (!iobject.valid()) {
		return;
	}

	const Alembic::Abc::ObjectHeader &header = iobject.getHeader();

	if (ICurves::matches(header)) {
		return read_curves_sample(vertexCos, max_verts, iobject, time);
	}
}
