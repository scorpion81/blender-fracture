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
 * Copyright (C) 2014 by Martin Felke.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
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

struct RigidBodyWorld;
struct FractureModifierData;
struct FractureSetting;
struct DerivedMesh;
struct Object;
struct Group;
struct MeshIsland;
struct RigidBodyShardCon;
struct GHash;

struct BoundBox;
struct MVert;
struct MPoly;
struct MLoop;
struct MEdge;

struct BMesh;

typedef int ShardID;

typedef struct FracPoint {
	float co[3];
	float offset[3];
} FracPoint;

typedef struct FracPointCloud {
	struct FracPoint *points;   /* just a bunch of positions in space*/
	int totpoints; /* number of positions */
} FracPointCloud;

/* direct access */
struct Shard *BKE_shard_by_id(struct FracMesh *mesh, ShardID id, struct DerivedMesh *dm);

/* detailed info to the particular shards */
bool BKE_get_shard_minmax(struct FracMesh *mesh, ShardID id, float min_r[3], float max_r[3], struct DerivedMesh *dm);

/* container object handling functions */
struct FracMesh *BKE_create_fracture_container(void);
struct Shard *BKE_create_fracture_shard(struct MVert *mvert, struct MPoly *mpoly, struct MLoop *mloop, struct MEdge *medge,
                                        int totvert, int totpoly, int totloop, int totedge, bool copy);
struct Shard *BKE_custom_data_to_shard(struct Shard *s, struct DerivedMesh *dm);

/* utility functions */
bool BKE_fracture_shard_center_median(struct Shard *shard, float cent[3]);
bool BKE_fracture_shard_center_centroid(struct Shard *shard, float cent[3]);
bool BKE_fracture_shard_center_centroid_area(struct Shard *shard, float cent[3]);
float BKE_shard_calc_minmax(struct Shard *shard);

void BKE_fracmesh_free(struct FracMesh *fm, bool doCustomData);
void BKE_shard_free(struct Shard *s, bool doCustomData);
struct Shard* BKE_fracture_shard_copy(struct Shard *s);


/* DerivedMesh */
struct DerivedMesh *BKE_fracture_create_dm(struct FractureModifierData *fmd, bool doCustomData, bool use_packed);
struct DerivedMesh *BKE_shard_create_dm(struct Shard *s, bool doCustomData);

/* create shards from base mesh and a list of points */
void BKE_fracture_shard_by_points(struct FracMesh *fmesh, ShardID id, struct FracPointCloud *points, int algorithm,
                                  struct Object *obj, struct DerivedMesh *dm, short inner_material_index, float mat[4][4],
                                  int num_cuts, float fractal, bool smooth, int num_levels, int mode, bool reset, int active_setting,
                                  int num_settings, char uv_layer[], bool threaded, int solver, float thresh, bool shards_to_islands,
                                  int override_count, float factor, int point_source, int resolution[], float spacing[]);

/* create shards from a base mesh and a set of other objects / cutter planes */
void BKE_fracture_shard_by_planes(struct FractureModifierData *fmd, struct Object *obj, short inner_material_index, float mat[4][4]);
void BKE_fracture_shard_by_greasepencil(struct FractureModifierData *fmd, struct Object *obj, short inner_material_index, float mat[4][4]);

void BKE_match_vertex_coords(struct MeshIsland* mi, struct MeshIsland *par, struct Object *ob, int frame, bool is_parent, bool shards_to_islands);
bool BKE_lookup_mesh_state(struct FractureModifierData *fmd, int frame, int do_lookup);
void BKE_get_prev_entries(struct FractureModifierData *fmd);
void BKE_get_next_entries(struct FractureModifierData *fmd);
void BKE_free_constraints(struct FractureModifierData *fmd);
void BKE_fracture_load_settings(struct FractureModifierData *fmd, struct FractureSetting *fs);
void BKE_fracture_store_settings(struct FractureModifierData *fs, struct FractureSetting *fmd);
struct Shard* BKE_create_initial_shard(struct DerivedMesh *dm);
void BKE_copy_customdata_layers(struct CustomData* dest, struct CustomData *src, int type, int count);

struct MeshIsland *BKE_fracture_mesh_island_add(struct FractureModifierData *fmd, struct Object *own, struct Object *target);
void BKE_fracture_mesh_island_remove(struct FractureModifierData *fmd, struct MeshIsland *mi);
void BKE_fracture_mesh_island_remove_all(struct FractureModifierData *fmd);

struct RigidBodyShardCon* BKE_fracture_mesh_islands_connect(struct FractureModifierData *fmd, struct MeshIsland *mi1, struct MeshIsland *mi2, short con_type);
void BKE_fracture_mesh_constraint_remove(struct FractureModifierData* fmd, struct RigidBodyShardCon *con);
void BKE_fracture_mesh_constraint_remove_all(struct FractureModifierData *fmd);

void BKE_fracture_free_mesh_island(struct FractureModifierData *rmd, struct MeshIsland *mi, bool remove_rigidbody);
int BKE_fracture_update_visual_mesh(struct FractureModifierData *fmd, struct Object *ob, bool do_custom_data);
short BKE_fracture_collect_materials(struct Object* o, struct Object* ob, int matstart, struct GHash** mat_index_map);

void BKE_bm_mesh_hflag_flush_vert(struct BMesh *bm, const char hflag);
void BKE_meshisland_constraint_create(struct FractureModifierData* fmd, struct MeshIsland *mi1, struct MeshIsland *mi2, int con_type, float thresh);
void BKE_update_acceleration_map(struct FractureModifierData *fmd, struct MeshIsland* mi, struct Object* ob, int ctime, float acc, struct RigidBodyWorld *rbw);
void BKE_update_velocity_layer(struct FractureModifierData *fmd, struct MeshIsland *mi);
void BKE_read_animated_loc_rot(struct FractureModifierData *fmd, Object *ob, bool do_bind);

#endif /* BKE_FRACTURE_H */
