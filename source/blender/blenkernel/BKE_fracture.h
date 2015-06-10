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

/* old FM struct definition, used in loading routine to retrieve data from old blends */
typedef struct FractureModifierData_Legacy {
	ModifierData modifier;
	struct FracMesh *frac_mesh; /* store only the current fracmesh here first, later maybe an entire history...*/
	struct DerivedMesh *dm;
	struct Group *extra_group;
	struct Group *dm_group;
	struct Group *cluster_group;
	struct Group *cutter_group;
	struct BMesh *visible_mesh;
	struct DerivedMesh *visible_mesh_cached;
	ListBase meshIslands, meshConstraints;
	ListBase islandShards;
	char thresh_defgrp_name[64];  /* MAX_VGROUP_NAME */
	char ground_defgrp_name[64];  /* MAX_VGROUP_NAME */
	char inner_defgrp_name[64];  /* MAX_VGROUP_NAME */
	struct KDTree *nor_tree; /* store original vertices here (coords), to find them later and reuse their normals */
	struct Material *inner_material;
	struct GHash *face_pairs;
	struct GHash *vert_index_map; /*used for autoconversion of former objects to clusters, marks object membership of each vert*/
	struct GHash *vertex_island_map; /* used for constraint building based on vertex proximity, temporary data */
	ListBase shard_sequence; /* used as mesh cache / history for dynamic fracturing, for shards (necessary for conversion to DM) */
	ListBase meshIsland_sequence; /* used as mesh cache / history for dynamic fracturing, for meshIslands (necessary for loc/rot "pointcache") */
	ShardSequence *current_shard_entry; /*volatile storage of current shard entry, so we dont have to search in the list */
	MeshIslandSequence *current_mi_entry; /*analogous to current shard entry */
	ListBase fracture_ids; /*volatile storage of shards being "hit" or fractured currently, needs to be cleaned up after usage! */

	/* values */
	int frac_algorithm;
	int shard_count;
	int shard_id;
	int point_source;
	int point_seed;
	int percentage;
	int cluster_count;

	int constraint_limit;
	int solver_iterations_override;
	int cluster_solver_iterations_override;
	int breaking_percentage;
	int cluster_breaking_percentage;
	int splinter_axis;
	int fractal_cuts;
	int fractal_iterations;
	int grease_decimate;
	int cutter_axis;
	int cluster_constraint_type;
	int fracture_mode;

	float breaking_angle;
	float breaking_distance;
	float cluster_breaking_angle;
	float cluster_breaking_distance;
	float origmat[4][4];
	float breaking_threshold;
	float cluster_breaking_threshold;
	float contact_dist, autohide_dist;
	float splinter_length;
	float nor_range;
	float fractal_amount;
	float physics_mesh_scale;
	float grease_offset;
	float dynamic_force;

	/* flags */
	int refresh;
	int refresh_constraints;
	int refresh_autohide;
	int reset_shards;

	int use_constraints;
	int use_mass_dependent_thresholds;
	int use_particle_birth_coordinates;
	int use_breaking;
	int use_smooth;
	int use_greasepencil_edges;

	int shards_to_islands;
	int execute_threaded;
	int fix_normals;
	int auto_execute;

	int breaking_distance_weighted;
	int breaking_angle_weighted;
	int breaking_percentage_weighted;
	int constraint_target;
	int limit_impact;

	/* internal flags */
	int use_experimental;
	int explo_shared;
	int refresh_images;
	int update_dynamic;

	/* internal values */
	float max_vol;
	int last_frame;

	//char pad[4];
} FractureModifierData_Legacy ;

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
struct DerivedMesh *BKE_fracture_create_dm(struct FractureModifierData *fmd, bool doCustomData, bool join_result);
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

#endif /* BKE_FRACTURE_H */
