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

struct bContext;
struct Object;
struct Scene;

enum {
	ABC_ARCHIVE_OGAWA = 0,
	ABC_ARCHIVE_HDF5  = 1,
};

#define BL_ABC_NO_ERR 0
#define BL_ABC_UNKNOWN_ERROR 1

int ABC_export(struct Scene *scene, struct bContext *C, const char *filename,
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
               int geogroups, int compression,
               bool packuv, float scale);

void ABC_import(struct bContext *C, const char *filename, float scale);

void ABC_get_vertex_cache(const char *filepath, float time, void *verts, int max_verts, const char *sub_obj, int is_mvert);

int ABC_check_subobject_valid(const char *name, const char *sub_obj);

void ABC_get_transform(struct Object *ob, const char *filename, const char *object_path, float r_mat[4][4], float time);

#ifdef __cplusplus
}
#endif

#endif  /* __ABC_ALEMBIC_H__ */
