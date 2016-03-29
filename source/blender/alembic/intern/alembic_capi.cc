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

extern "C" {
#include "MEM_guardedalloc.h"

#include "DNA_camera_types.h"
#include "DNA_curve_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_camera.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_context.h"
#include "BKE_curve.h"
#include "BKE_depsgraph.h"
#include "BKE_global.h"
#include "BKE_idprop.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_scene.h"

#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_threads.h"
}

enum {
	OBJECT_TYPE_MESH = 0,
	OBJECT_TYPE_NURBS = 1,
	OBJECT_TYPE_CAMERA = 2,
};

static void split(const std::string &s, const char *delim, std::vector<std::string> &v)
{
	/* to avoid modifying original string first duplicate the original string
	 *  and return a char pointer then free the memory */
	char *dup = strdup(s.c_str());
	char *token = strtok(dup, delim);

	while (token != NULL) {
		v.push_back(std::string(token));
		/* the call is treated as a subsequent calls to strtok: the function
		 * continues from where it left in previous invocation */
		token = strtok(NULL, delim);
	}

	free(dup);
}

#if 0
static Object *find_object(Scene *scene, const std::string &sub_object, Object */*parent*/)
{
	Object *ret = NULL;

	std::vector<std::string> parts;
	split(sub_object, "/", parts);

	for (Base *base = static_cast<Base *>(scene->base.first); base; base = base->next) {
		Object *ob = base->object;

		if (ob->id.name == sub_object) {
			ret = ob;
		}
	}

	if (parts.size() == 1) {
		return ret;
	}

	return find_object(scene, sub_object, ret);
}
#endif

using namespace Alembic::AbcGeom;

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

struct AbcNuInfo {
	Curve *curve;
	int current_mat;

	std::string filename, sub_object;
	std::vector< std::pair<INuPatchSchema, IObject> > nu_schema_cache;

	std::map<std::string, int> mat_map;
	std::map<std::string, Material*> materials;

	AbcNuInfo()
	    : curve(NULL)
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

// Some helpers for mesh generation
namespace mesh_utils {

static void mesh_add_verts(Mesh *mesh, size_t len)
{
	if (len == 0) {
		return;
	}

	int totvert = mesh->totvert + len;
	CustomData vdata;
	CustomData_copy(&mesh->vdata, &vdata, CD_MASK_MESH, CD_DEFAULT, totvert);
	CustomData_copy_data(&mesh->vdata, &vdata, 0, 0, mesh->totvert);

	if (!CustomData_has_layer(&vdata, CD_MVERT))
		CustomData_add_layer(&vdata, CD_MVERT, CD_CALLOC, NULL, totvert);

	CustomData_free(&mesh->vdata, mesh->totvert);
	mesh->vdata = vdata;
	BKE_mesh_update_customdata_pointers(mesh, false);

	/* set final vertex list size */
	mesh->totvert = totvert;
}

static void mesh_add_mloops(Mesh *mesh, size_t len)
{
	if (len == 0) {
		return;
	}

	/* new face count */
	const int totloops = mesh->totloop + len;

	CustomData ldata;
	CustomData_copy(&mesh->ldata, &ldata, CD_MASK_MESH, CD_DEFAULT, totloops);
	CustomData_copy_data(&mesh->ldata, &ldata, 0, 0, mesh->totloop);

	if (!CustomData_has_layer(&ldata, CD_MLOOP)) {
		CustomData_add_layer(&ldata, CD_MLOOP, CD_CALLOC, NULL, totloops);
	}

	if (!CustomData_has_layer(&ldata, CD_MLOOPUV)) {
		CustomData_add_layer(&ldata, CD_MLOOPUV, CD_CALLOC, NULL, totloops);
	}

	CustomData_free(&mesh->ldata, mesh->totloop);
	mesh->ldata = ldata;
	BKE_mesh_update_customdata_pointers(mesh, false);

	mesh->totloop = totloops;
}

static void mesh_add_mpolygons(Mesh *mesh, size_t len)
{
	if (len == 0) {
		return;
	}

	const int totpolys = mesh->totpoly + len;   /* new face count */

	CustomData pdata;
	CustomData_copy(&mesh->pdata, &pdata, CD_MASK_MESH, CD_DEFAULT, totpolys);
	CustomData_copy_data(&mesh->pdata, &pdata, 0, 0, mesh->totpoly);

	if (!CustomData_has_layer(&pdata, CD_MPOLY))
		CustomData_add_layer(&pdata, CD_MPOLY, CD_CALLOC, NULL, totpolys);

	if (!CustomData_has_layer(&pdata, CD_MTEXPOLY))
		CustomData_add_layer(&pdata, CD_MTEXPOLY, CD_CALLOC, NULL, totpolys);

	CustomData_free(&mesh->pdata, mesh->totpoly);
	mesh->pdata = pdata;
	BKE_mesh_update_customdata_pointers(mesh, false);

	mesh->totpoly = totpolys;
}

} /* mesh_utils */

template<class TContainer>
bool begins_with(const TContainer &input, const TContainer &match)
{
	return input.size() >= match.size()
	        && std::equal(match.begin(), match.end(), input.begin());
}

static void getCamera(IObject iObj, ICameraSchema &schema, std::string sub_obj, bool &found)
{
	if (!iObj.valid() || found)
		return;

	for (int i = 0;i < iObj.getNumChildren(); ++i) {
		bool ok = true;
		IObject child(iObj, iObj.getChildHeader(i).getName());

		if (!sub_obj.empty() && child.valid() && child.getFullName() == sub_obj) {
			const MetaData &md = child.getMetaData();

			if (ICamera::matches(md) && ok) {
				ICamera abc_cam(child, kWrapExisting);
				schema = abc_cam.getSchema();
				found = true;
				return;
			}
		}

		getCamera(child, schema, sub_obj, found);
	}
}

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

#if 0
	for (int i = 0;i < iObj.getNumChildren(); ++i) {
		bool ok = true;
		IObject child(iObj, iObj.getChildHeader(i).getName());

		if (!sub_obj.empty() && child.valid() && !begins_with(child.getFullName(), sub_obj)) {
			ok = false;
		}

		if (!child.valid())
			continue;

		const MetaData &md = child.getMetaData();

		if (IPolyMesh::matches(md) && ok) {
			IPolyMesh abc_mesh(child, kWrapExisting);
			IPolyMeshSchema schem = abc_mesh.getSchema();
			schemas.push_back(std::pair<IPolyMeshSchema, IObject>(schem, child));
		}

		visitObject(IObject(child), schemas, sub_obj);
	}
#endif
}

static void visitNurbsObject(IObject iObj, std::vector< std::pair<INuPatchSchema, IObject> > &schemas, std::string sub_obj)
{
	if (!iObj.valid())
		return;

	for (int i = 0;i < iObj.getNumChildren(); ++i) {
		bool ok = true;
		IObject child(iObj, iObj.getChildHeader(i).getName());

		if (!sub_obj.empty() && child.valid() && !begins_with(child.getFullName(), sub_obj)) {
			ok = false;
		}

		if (!child.valid())
			continue;

		const MetaData &md = child.getMetaData();

		if (INuPatch::matches(md) && ok) {
			INuPatch abc_nurb(child, kWrapExisting);
			INuPatchSchema schem = abc_nurb.getSchema();
			schemas.push_back(std::pair<INuPatchSchema, IObject>(schem, child));
		}

		visitNurbsObject(IObject(child), schemas, sub_obj);
	}
}

static void visitObjectString(IObject iObj, std::vector<std::string> &objects, int type)
{
	if (!iObj.valid()) {
		return;
	}

	for (int i = 0;i < iObj.getNumChildren(); ++i) {
		IObject child(iObj, iObj.getChildHeader(i).getName());

		if (!child.valid())
			continue;

		const MetaData &md = child.getMetaData();

		if (IPolyMesh::matches(md) && type == OBJECT_TYPE_MESH) {
			objects.push_back(child.getFullName());
		}
		else if (INuPatch::matches(md) && type == OBJECT_TYPE_NURBS) {
			objects.push_back(child.getFullName());
		}
		else if (ICameraSchema::matches(md) && type == OBJECT_TYPE_CAMERA) {
			objects.push_back(child.getFullName());
		}

		visitObjectString(IObject(child), objects, type);
	}
}

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

#include "BLI_math.h"

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
	Alembic::Abc::UInt32ArraySamplePtr 	 uvsamp_ind;

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
		//uvsamp_ind 	= uvsamp.getIndices();
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
			V3f pos_in = (*positions)[i];

			// swap from Y-Up to Z-Up
			verts[j].co[0] = pos_in[0];
			verts[j].co[1] = -pos_in[2];
			verts[j].co[2] = pos_in[1];
		}
	}
	else if (vcos) {
		int j = vtx_start;
		for (int i = 0; i < vertex_count; ++i, ++j) {
			V3f pos_in = (*positions)[i];

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

static Curve *ABC_get_nurbs(IArchive *archive, float time, const char *sub_obj)
{
	IObject iObj = archive->getTop();

	if (!iObj.valid()) {
		std::cerr << "Cannot get top of " << archive->getName() << '\n';
		return NULL;
	}

	Curve *cu = BKE_curve_add(G.main, "abc_tmp", OB_SURF);

	AbcNuInfo info;
	info.filename 	= archive->getName();
	info.curve 		= cu;
	info.sub_object = sub_obj;
	visitNurbsObject(iObj, info.nu_schema_cache, sub_obj);

	std::vector< std::pair<INuPatchSchema, IObject> >::iterator it;

	for (it = info.nu_schema_cache.begin(); it != info.nu_schema_cache.end(); ++it) {
		Nurb *nu = (Nurb*)MEM_callocN(sizeof(Nurb), "abc_getnurb");
		nu->flag  = CU_SMOOTH;
		nu->type = CU_NURBS;
		nu->resolu = 4;
		nu->resolv = 4;
		ISampleSelector sample_sel(time);
		INuPatchSchema::Sample smp = it->first.getValue(sample_sel);

		P3fArraySamplePtr   positions    = smp.getPositions();
		FloatArraySamplePtr positionsW   = smp.getPositionWeights();
		int32_t num_U = smp.getNumU();
		int32_t num_V = smp.getNumV();
		int32_t u_order = smp.getUOrder();
		int32_t v_order = smp.getVOrder();
		FloatArraySamplePtr u_knot   = smp.getUKnot();
		FloatArraySamplePtr v_knot   = smp.getVKnot();

		size_t numPt = positions->size();
		size_t numKnotsU = u_knot->size();
		size_t numKnotsV = v_knot->size();


		nu->orderu = u_order;
		nu->orderv = v_order;
		nu->pntsu  = num_U;
		nu->pntsv  = num_V;
		nu->bezt = NULL;

		nu->bp = (BPoint*)MEM_callocN(numPt * sizeof(BPoint), "abc_setsplinetype");
		nu->knotsu = (float*)MEM_callocN(numKnotsU * sizeof(float), "abc_setsplineknotsu");
		nu->knotsv = (float*)MEM_callocN(numKnotsV * sizeof(float), "abc_setsplineknotsv");
		nu->bp->radius = 1.0f;

		for (int i = 0; i < numPt; ++i) {
			V3f pos_in = (*positions)[i];
			float posw_in = 1.0;
			if (positionsW && i < positionsW->size())
				posw_in = (*positionsW)[i];

			// swap from Y-Up to Z-Up
			nu->bp[i].vec[0] = pos_in[0];
			nu->bp[i].vec[1] = -pos_in[2];
			nu->bp[i].vec[2] = pos_in[1];
			nu->bp[i].vec[3] = posw_in;
		}

		for (size_t i = 0; i < numKnotsU; i++) {
			nu->knotsu[i] = (*u_knot)[i];
		}

		for (size_t i = 0; i < numKnotsV; i++) {
			nu->knotsv[i] = (*v_knot)[i];
		}

		ICompoundProperty userProps = it->first.getUserProperties();
		if (userProps.valid() && userProps.getPropertyHeader("endU") != 0) {
			IBoolProperty enduProp(userProps, "endU");
			bool_t endu;
			enduProp.get(endu, sample_sel);
			if (endu)
				nu->flagu = CU_NURB_ENDPOINT;
		}

		if (userProps.valid() && userProps.getPropertyHeader("endV") != 0) {
			IBoolProperty endvProp(userProps, "endV");
			bool_t endv;
			endvProp.get(endv, sample_sel);
			if (endv)
				nu->flagv = CU_NURB_ENDPOINT;
		}

		BLI_addtail(BKE_curve_nurbs_get(cu), nu);
	}

	return cu;
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

static Camera *ABC_get_camera(IArchive *archive, const char *abc_subobject, float time)
{
	ICameraSchema cam_obj;
	bool found = false;
	getCamera(archive->getTop(), cam_obj, abc_subobject, found);

	if (!cam_obj.valid() || !found) {
		std::cerr << "Warning: Corrupted Alembic archive: " << archive->getName() << '\n';
		return NULL;
	}

	Camera *bcam = static_cast<Camera *>(BKE_camera_add(G.main, "abc_camera"));

	ISampleSelector sample_sel(time);
	CameraSample cam_sample;
	cam_obj.get(cam_sample, sample_sel);

	ICompoundProperty customDataContainer =  cam_obj.getUserProperties();

	if (customDataContainer.valid() && customDataContainer.getPropertyHeader("stereoDistance") &&
	    customDataContainer.getPropertyHeader("eyeSeparation")) {
		Alembic::AbcGeom::IFloatProperty convergence_plane(customDataContainer, "stereoDistance");
		Alembic::AbcGeom::IFloatProperty eye_separation(customDataContainer, "eyeSeparation");

		bcam->stereo.interocular_distance = eye_separation.getValue(sample_sel);
		bcam->stereo.convergence_distance = convergence_plane.getValue(sample_sel);;
	}

	float lens = cam_sample.getFocalLength();
	float apperture_x = cam_sample.getHorizontalAperture();
	float apperture_y = cam_sample.getVerticalAperture();
	float h_film_offset = cam_sample.getHorizontalFilmOffset();
	float v_film_offset = cam_sample.getVerticalFilmOffset();
	float film_aspect = apperture_x / apperture_y;

	bcam->lens = lens;
	bcam->sensor_x = apperture_x * 10;
	bcam->sensor_y = apperture_y * 10;
	bcam->shiftx = h_film_offset / apperture_x;
	bcam->shifty = v_film_offset / (apperture_y * film_aspect);
	bcam->clipsta = cam_sample.getNearClippingPlane();
	bcam->clipend = cam_sample.getFarClippingPlane();
	bcam->gpu_dof.focus_distance = cam_sample.getFocusDistance();
	bcam->gpu_dof.fstop = cam_sample.getFStop();
	bcam->shifty = v_film_offset / apperture_y / film_aspect;
	bcam->clipsta = cam_sample.getNearClippingPlane();
	bcam->clipend = cam_sample.getFarClippingPlane();

	return bcam;
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
                int use_subdiv_schema, bool ogawa, bool packuv)
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

		if (opts.startframe > opts.endframe)
			std::swap(opts.startframe, opts.endframe);

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

#if 0
static Object *create_hierarchy(bContext *C, IArchive */*archive*/, const std::vector<std::string> &parts)
{
	Object *parent = NULL;
	std::string sub_object;

	std::vector<std::string>::const_iterator iter;
	for (iter = parts.begin(); iter != parts.end(); ++iter) {
		sub_object = "/" + *iter;

		Object *ob = find_object(CTX_data_scene(C), sub_object, parent);

		if (ob) {
			parent = ob;
		}

		if (sub_object == "/") {
			parent = NULL;
			continue;
		}

		Object *empty = BKE_object_add(CTX_data_main(C), CTX_data_scene(C), OB_EMPTY, sub_object.c_str());
		empty->parent = parent;

		DAG_id_tag_update(&empty->id, OB_RECALC_OB);
	}

	return parent;
}
#endif

static void import_object(bContext *C, IArchive *archive,
                          const std::string &sub_object, int object_type,
                          Object *parent, int from_forward, int from_up)
{
	std::vector<std::string> parts;
	split(sub_object, "/", parts);

	const std::string &data_name = parts.back();
	const std::string &object_name = parts.front();

	Object *ob = NULL;

	switch (object_type) {
		case OBJECT_TYPE_MESH:
		{
			bool apply_materials = false;
			bool p_only = false;

			ABC_mutex_lock();
			Mesh *mesh = ABC_get_mesh(archive, 0.0f, NULL, apply_materials, sub_object.c_str(), &p_only, from_forward, from_up);
			ABC_mutex_unlock();

			if (!mesh) {
		        ABC_destroy_key(NULL);
				return;
			}

			mesh = BKE_mesh_copy(mesh);
			BLI_strncpy(mesh->id.name + 2, data_name.c_str(), data_name.size() + 1);

			ob = BKE_object_add(CTX_data_main(C), CTX_data_scene(C), OB_MESH, object_name.c_str());
			ob->data = mesh;

			if (apply_materials) {
				ABC_apply_materials(ob, NULL);
		    }

			ABC_destroy_key(NULL);

			break;
		}
		case OBJECT_TYPE_NURBS:
		{
			ABC_mutex_lock();
			Curve *curve = ABC_get_nurbs(archive, 0.0f, sub_object.c_str());
			ABC_mutex_unlock();

			if (!curve) {
				return;
			}

			curve = BKE_curve_copy(curve);
			BLI_strncpy(curve->id.name + 2, data_name.c_str(), data_name.size() + 1);

			ob = BKE_object_add(CTX_data_main(C), CTX_data_scene(C), OB_CURVE, object_name.c_str());
			ob->data = curve;

			break;
		}
		case OBJECT_TYPE_CAMERA:
		{
			ABC_mutex_lock();
			Camera *camera = ABC_get_camera(archive, sub_object.c_str(), 0.0f);
			ABC_mutex_unlock();

			if (!camera) {
				return;
			}

			BLI_strncpy(camera->id.name + 2, data_name.c_str(), data_name.size() + 1);

			ob = BKE_object_add(CTX_data_main(C), CTX_data_scene(C), OB_CAMERA, object_name.c_str());
			ob->data = camera;

			break;
		}
	}

	if (ob == NULL) {
		return;
	}

	ob->parent = parent;
	DAG_id_tag_update(&ob->id, OB_RECALC_OB);
}

static void import_objects(bContext *C, IArchive *archive,
                           int object_type, int from_forward, int from_up)
{

	std::vector<std::string> strings;
	visitObjectString(archive->getTop(), strings, object_type);

	/* Reverse list to go from top to bottom. */
	std::reverse(strings.begin(), strings.end());

	/* Remove empty strings. */
	strings.erase(std::remove(strings.begin(), strings.end(), std::string("")),
	              strings.end());

	std::vector<std::string>::iterator iter;
	for (iter = strings.begin(); iter != strings.end(); ++iter) {
		std::cerr << *iter << '\n';

		std::vector<std::string> parts;
		split(*iter, "/", parts);

		/* Remove empty strings. */
		strings.erase(std::remove(parts.begin(), parts.end(), std::string("")),
		              parts.end());

		Object *parent = NULL; // create_hierarchy(C, archive, parts);
		import_object(C, archive, *iter, object_type, parent, from_forward, from_up);
	}
}

void ABC_import(bContext *C, const char *filename, int from_forward, int from_up)
{
	/* get objects strings */
	IArchive *archive = abc_manager->getArchive(filename);

	if (!archive || !archive->valid()) {
		return;
	}

	import_objects(C, archive, OBJECT_TYPE_MESH, from_forward, from_up);
	import_objects(C, archive, OBJECT_TYPE_NURBS, from_forward, from_up);
	import_objects(C, archive, OBJECT_TYPE_CAMERA, from_forward, from_up);
}
