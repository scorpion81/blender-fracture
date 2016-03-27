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

#ifndef __ABC_ALEMBIC_H__
#define __ABC_ALEMBIC_H__

#ifdef __cplusplus
extern "C" {
#endif

struct Camera;
struct Curve;
struct DerivedMesh;
struct Mesh;
struct MVert;
struct Object;
struct Scene;

#define BL_ABC_NO_ERR 0
#define BL_ABC_UNKNOWN_ERROR 1

int ABC_export(struct Scene *sce, const char *filename,
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
				int geogroups, bool ogawa,
				bool packuv
				);

struct DerivedMesh *abcGetDerivedMesh(const char* filepath, float time, void* mesh_data, struct Object* ob, int assign_mat, const char* sub_obj);
void 			abcGetVertexCache(const char* filepath, float time, void *key, void* verts, int max_verts, const char* sub_obj, int is_mvert);
struct Mesh *abcGetMesh(const char* filepath, float time, void *key, int assign_mats, const char* sub_obj, bool *p_only);
struct Curve *abcGetNurbs(const char* filepath, float time, const char* sub_obj);
void 			abcApplyMaterials(struct Object *ob, void* key);

void 			ABC_getObjects(const char *filename, char* result);
void 			ABC_getNurbs(const char *filename, char* result);
void 			ABC_getCamera(const char *filename, char* result);
void 			ABC_getTransform(const char *filename, const char* abc_subobject, float time, float mat[][4], int to_y_up);
void		 	ABC_setCustomProperties(struct Object* bobj);
void 			ABC_set_camera(const char* filename, const char* abc_subobject, float time, struct Camera* bcam);
void 			abcDestroyMeshData(void* key);
int 			checkSubobjectValid(const char* name, const char* sub_obj);
void			abcDestroyKey(void*);
void 			abcMutexLock(void);
void 			abcMutexUnlock(void);

#ifdef __cplusplus
}
#endif

#endif  /* __ABC_ALEMBIC_H__ */
