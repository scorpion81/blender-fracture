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

struct BoundBox;
struct MVert;
struct MPoly;
struct MLoop;

typedef int ShardID;

typedef struct ShardIterator {
	struct FracMesh* frac_mesh;
	int		current;
} ShardIterator;

typedef struct FracMeshIterator {
	struct FracHistory* frac_history;
	int		current_step;
} FracMeshIterator;

typedef struct FracPoint {
	float co[3];
} FracPoint;

typedef struct FracPointCloud {
	struct FracPoint *points;	// just a bunch of positions in space
	int totpoints; // number of positions
} FracPointCloud;

/* iterator functions for efficient looping over shards */
struct ShardIterator* BKE_shards_begin(struct FracMesh *fmesh);
struct ShardIterator* BKE_shards_next(struct ShardIterator *iter);
bool BKE_shards_valid(struct ShardIterator* iter);
void BKE_shards_end(struct ShardIterator* iter); 
struct Shard* BKE_shard_by_iterator(struct ShardIterator *iter);

/*direct access*/
struct Shard* BKE_shard_by_id(struct FracMesh* mesh, ShardID id, struct DerivedMesh *dm);

/* detailed info to the particular shards */
void BKE_get_shard_geometry(struct FracMesh* mesh, ShardID id, struct MVert** vert, int *totvert, struct DerivedMesh *dm);
void BKE_get_shard_minmax(struct FracMesh* mesh, ShardID id, float min_r[3], float max_r[3], struct DerivedMesh *dm);

/* container object handling functions */
struct FracMesh *BKE_create_fracture_container(struct DerivedMesh* dm);
struct Shard *BKE_create_fracture_shard(struct MVert *mvert, struct MPoly *mpoly, struct MLoop *mloop, int totvert, int totpoly, int totloop, bool copy);
struct Shard *BKE_custom_data_to_shard(struct Shard* s, struct DerivedMesh* dm);

/* iterator functions for efficient / abstract iterating over fracture history */
struct FracMeshIterator* BKE_fracture_steps_begin(struct Object *ob);
struct FracMeshIterator* BKE_fracture_steps_next(struct FracMeshIterator *iter);
struct FracMeshIterator* BKE_fracture_steps_prev(struct FracMeshIterator *iter);
bool BKE_fracture_steps_valid(struct FracMeshIterator* iter);
void BKE_fracture_steps_end(struct FracMeshIterator* iter); 

//utility functions
bool BKE_fracture_shard_center_median(struct Shard *shard, float cent[3]);
bool BKE_fracture_shard_center_centroid(struct Shard *shard, float cent[3]);
float BKE_shard_calc_minmax(struct Shard *shard);

void BKE_fracmesh_free(struct FracMesh* fm, bool doCustomData);
void BKE_shard_free(struct Shard* s, bool doCustomData);


/* DerivedMesh */
void BKE_fracture_release_dm(struct FractureModifierData *fmd);
void BKE_fracture_create_dm(struct FractureModifierData *fmd, bool doCustomData);
struct DerivedMesh *BKE_shard_create_dm(struct Shard *s, bool doCustomData);

void BKE_shard_assign_material(struct Shard* s, short mat_nr);

// erzeuge shards aus dem basis mesh und einer liste von points (nicht weiter spezifiziert, können auch particles oder so sein)
void BKE_fracture_shard_by_points(struct FracMesh *fmesh, ShardID id, struct FracPointCloud *points, int algorithm, struct Object *obj, struct DerivedMesh *dm,
                                  short inner_material_index);


#endif // BKE_FRACTURE_H
