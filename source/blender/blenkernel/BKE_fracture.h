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
struct DerivedMesh;
struct Object;
struct Group;
struct MeshIsland;
struct Main;
struct FractureState;

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

typedef struct FractureID {
	struct FractureID *next, *prev;
	int shardID;
	char pad[4];
} FractureID;

/* direct access */
struct Shard *BKE_shard_by_id(struct FracMesh *mesh, ShardID id);

/* detailed info to the particular shards */
void BKE_get_shard_minmax(struct FracMesh *mesh, ShardID id, float min_r[3], float max_r[3]);

/* container object handling functions */
struct FracMesh *BKE_create_fracmesh(void);
struct Shard *BKE_create_fracture_shard(struct MVert *mvert, struct MPoly *mpoly, struct MLoop *mloop, int totvert, int totpoly, int totloop, bool copy);
struct Shard *BKE_custom_data_to_shard(struct Shard *s, struct DerivedMesh *dm);

/* utility functions */
bool BKE_fracture_shard_center_median(struct Shard *shard, float cent[3]);
bool BKE_fracture_shard_center_centroid(struct Shard *shard, float cent[3]);
float BKE_shard_calc_minmax(struct Shard *shard);

void BKE_fracmesh_free(struct FracMesh *fm, bool doCustomData);
void BKE_shard_free(struct Shard *s, bool doCustomData);


/* DerivedMesh */
struct DerivedMesh *BKE_fracture_create_dm(struct Object *ob, struct FracMesh *fm, bool doCustomData);
struct DerivedMesh *BKE_shard_create_dm(struct Shard *s, bool doCustomData);

/* create shards from base mesh and a list of points */
void BKE_fracture_shard_by_points(struct Object *obj, ShardID id, struct FracPointCloud *points, short inner_material_index, float mat[4][4]);

/* create shards from a base mesh and a set of other objects / cutter planes */
void BKE_fracture_shard_by_planes(struct Object *obj, short inner_material_index, float mat[4][4]);
void BKE_fracture_shard_by_greasepencil(struct Object *obj, short inner_material_index, float mat[4][4]);

void BKE_match_vertex_coords(struct MeshIsland* mi, struct MeshIsland *par, struct Object *ob, int frame, bool is_parent);
void BKE_mesh_separate_selected(struct BMesh **bm_work, struct BMesh **bm_out, struct BMVert **orig_work, struct BMVert ***orig_out1, struct BMVert ***orig_out2);
void BKE_select_linked(struct BMesh **bm_in);

void BKE_fracture_prefracture_mesh(struct Scene *scene, struct Object *ob, ShardID id);
void BKE_dynamic_fracture_mesh(struct Scene* scene, struct Object *ob, ShardID id);
int BKE_initialize_meshisland(struct MeshIsland** mii, struct MVert* mverts, int vertstart);
struct DerivedMesh *BKE_fracture_autohide(struct Object* ob);
void BKE_fracture_constraint_container_free(struct Object *ob);
struct ConstraintContainer *BKE_fracture_constraint_container_create(struct Object* ob);

void BKE_fracture_container_free(struct Object *ob);
struct FractureContainer *BKE_fracture_container_create(struct Object *ob);
void BKE_lookup_mesh_state(struct Object* ob, int frame);

struct FractureContainer *BKE_fracture_container_copy(struct Object *ob, struct Object *obN);
struct MVert* BKE_copy_visual_mesh(struct Object* ob, struct FractureState *fs);
struct FracMesh* BKE_copy_fracmesh(struct FracMesh* fm);
struct DerivedMesh *BKE_fracture_ensure_mesh(struct Scene* scene, struct Object* ob);
void BKE_fracture_container_initialize(struct Object* ob, struct DerivedMesh *dm);
void BKE_fracture_create_islands(struct Object *ob, bool rebuild);
void BKE_fracture_constraint_container_update(struct Object* ob);
void BKE_fracture_prepare_autohide(struct Object *ob);
void BKE_fracture_constraint_container_empty(struct Scene *scene, struct Object *ob);
void BKE_fracture_container_empty(struct Scene *scene, struct Object *ob);
void BKE_fracture_relink_cache(struct Scene *scene, struct Object *ob, bool remove);
void BKE_fracture_synchronize_caches(struct Scene* scene);

#endif /* BKE_FRACTURE_H */
