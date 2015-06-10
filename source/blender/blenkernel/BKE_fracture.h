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
 * The Original Code is Copyright (C) Blender Foundation
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/BKE_fracture.h
 *  \ingroup blenkernel
 */

#ifndef BKE_FRACTURE_H
#define BKE_FRACTURE_H

#include "BLI_sys_types.h"

struct FracMesh;
struct Shard;

struct FractureModifierData;
struct FractureSetting;
struct DerivedMesh;
struct Object;
struct Group;
struct MeshIsland;

struct BoundBox;
struct MVert;
struct MPoly;
struct MLoop;

struct BMesh;
struct BMVert;

typedef int ShardID;

typedef struct FracPoint {
	float co[3];
} FracPoint;

typedef struct FracPointCloud {
	struct FracPoint *points;   /* just a bunch of positions in space*/
	int totpoints; /* number of positions */
} FracPointCloud;

/* direct access */
struct Shard *BKE_shard_by_id(struct FracMesh *mesh, ShardID id, struct DerivedMesh *dm);

/* detailed info to the particular shards */
void BKE_get_shard_minmax(struct FracMesh *mesh, ShardID id, float min_r[3], float max_r[3], struct DerivedMesh *dm);

/* container object handling functions */
struct FracMesh *BKE_create_fracture_container(void);
struct Shard *BKE_create_fracture_shard(struct MVert *mvert, struct MPoly *mpoly, struct MLoop *mloop, int totvert, int totpoly, int totloop, bool copy);
struct Shard *BKE_custom_data_to_shard(struct Shard *s, struct DerivedMesh *dm);

/* utility functions */
bool BKE_fracture_shard_center_median(struct Shard *shard, float cent[3]);
bool BKE_fracture_shard_center_centroid(struct Shard *shard, float cent[3]);
float BKE_shard_calc_minmax(struct Shard *shard);

void BKE_fracmesh_free(struct FracMesh *fm, bool doCustomData);
void BKE_shard_free(struct Shard *s, bool doCustomData);


/* DerivedMesh */
struct DerivedMesh *BKE_fracture_create_dm(struct FractureModifierData *fmd, struct FractureSetting *fs, bool doCustomData, bool join_result);
struct DerivedMesh *BKE_shard_create_dm(struct Shard *s, bool doCustomData);

/* create shards from base mesh and a list of points */
void BKE_fracture_shard_by_points(struct FracMesh *fmesh, ShardID id, struct FracPointCloud *points, int algorithm,
                                  struct Object *obj, struct DerivedMesh *dm, short inner_material_index, float mat[4][4],
                                  int num_cuts, float fractal, bool smooth, int num_levels, int mode, bool reset);

/* create shards from a base mesh and a set of other objects / cutter planes */
void BKE_fracture_shard_by_planes(struct FractureModifierData *fmd, struct Object *obj, short inner_material_index, float mat[4][4]);
void BKE_fracture_shard_by_greasepencil(struct FractureModifierData *fmd, struct Object *obj, short inner_material_index, float mat[4][4]);

void BKE_match_vertex_coords(struct MeshIsland* mi, struct MeshIsland *par, struct Object *ob, int frame, bool is_parent);
bool BKE_lookup_mesh_state(struct FractureModifierData *fmd, int frame, int do_lookup);
void BKE_get_prev_entries(struct FractureModifierData *fmd);
void BKE_get_next_entries(struct FractureModifierData *fmd);
void BKE_free_constraints(struct FractureModifierData *fmd);

struct ConstraintSetting* BKE_fracture_constraint_setting_new(struct FractureModifierData *fmd, const char name[64]);
void BKE_fracture_constraint_setting_remove(struct FractureModifierData *fmd, struct ConstraintSetting *setting);
void BKE_fracture_constraint_setting_remove_all(struct FractureModifierData *fmd);
void BKE_initialize_from_vertex_groups(struct FractureModifierData *fmd, struct Object *ob);
void BKE_mesh_separate_selected(struct BMesh **bm_work, struct BMesh **bm_out, struct BMVert **orig_work, struct BMVert ***orig_out1, struct BMVert ***orig_out2);
void BKE_select_linked(struct BMesh **bm_in);

struct DerivedMesh *BKE_prefracture_mesh(struct FractureModifierData *fmd, struct Object *ob, struct DerivedMesh *derivedData);
void BKE_free_fracture_modifier(struct FractureModifierData *fmd, bool do_free_seq);
struct DerivedMesh *BKE_dynamic_fracture_mesh(struct FractureModifierData *fmd, struct Object *ob, struct DerivedMesh *derivedData);

#endif /* BKE_FRACTURE_H */
