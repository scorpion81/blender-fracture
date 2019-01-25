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
 * The Original Code is Copyright (C) 2013 Blender Foundation
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Joshua Leung, Sergej Reich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file BKE_rigidbody.h
 *  \ingroup blenkernel
 *  \brief API for Blender-side Rigid Body stuff
 */
 

#ifndef __BKE_RIGIDBODY_H__
#define __BKE_RIGIDBODY_H__

struct RigidBodyWorld;
struct RigidBodyOb;
struct RigidBodyShardCon;

struct Scene;
struct Object;
struct Group;
struct MeshIsland;
struct FractureModifierData;

/* -------------- */
/* Memory Management */

void BKE_rigidbody_free_world(struct RigidBodyWorld *rbw);
void BKE_rigidbody_free_object(struct Object *ob);
void BKE_rigidbody_free_constraint(struct Object *ob);

/* ...... */

struct RigidBodyOb *BKE_rigidbody_copy_object(const struct Object *ob);
struct RigidBodyCon *BKE_rigidbody_copy_constraint(const struct Object *ob);

/* Callback format for performing operations on ID-pointers for rigidbody world. */
typedef void (*RigidbodyWorldIDFunc)(struct RigidBodyWorld *rbw, struct ID **idpoin, void *userdata, int cb_flag);

void BKE_rigidbody_world_id_loop(struct RigidBodyWorld *rbw, RigidbodyWorldIDFunc func, void *userdata);

/* -------------- */
/* Setup */

/* create Blender-side settings data - physics objects not initialized yet */
struct RigidBodyWorld *BKE_rigidbody_create_world(struct Scene *scene);
struct RigidBodyOb *BKE_rigidbody_create_object(struct Scene *scene, struct Object *ob, short type, struct MeshIsland *mi);
struct RigidBodyCon *BKE_rigidbody_create_constraint(struct Scene *scene, struct Object *ob, short type, struct RigidBodyShardCon *con);
struct RigidBodyOb *BKE_rigidbody_create_shard(struct Scene *scene, struct Object *ob, struct Object *target, struct MeshIsland *mi);
struct RigidBodyShardCon *BKE_rigidbody_create_shard_constraint(struct Scene *scene, short type, bool reset);

/* copy */
struct RigidBodyWorld *BKE_rigidbody_world_copy(struct RigidBodyWorld *rbw);
void BKE_rigidbody_world_groups_relink(struct RigidBodyWorld *rbw);

/* 'validate' (i.e. make new or replace old) Physics-Engine objects */
void BKE_rigidbody_validate_sim_world(struct Scene *scene, struct RigidBodyWorld *rbw, bool rebuild);
void BKE_rigidbody_validate_sim_object(struct RigidBodyWorld *rbw, struct Object *ob, short rebuild);
void BKE_rigidbody_validate_sim_shape(struct Object *ob, short rebuild);
void BKE_rigidbody_validate_sim_constraint(struct RigidBodyWorld *rbw, struct Object *ob, short rebuild);
void BKE_rigidbody_validate_sim_shard_constraint(struct RigidBodyWorld *rbw, struct FractureModifierData* fmd, struct Object*,
                                                 struct RigidBodyShardCon *rbsc, short rebuild);

void BKE_rigidbody_validate_sim_shard(struct RigidBodyWorld *rbw, struct MeshIsland *mi, struct Object *ob, short rebuild, int transfer_speeds, float size[3]);
void BKE_rigidbody_validate_sim_shard_shape(struct MeshIsland *mi, struct Object *ob, short rebuild);

/* move the islands of the visible mesh according to shard rigidbody movement */
void BKE_rigidbody_update_cell(struct MeshIsland *mi, struct Object* ob, float loc[3], float rot[4], struct FractureModifierData *rmd,
                               int frame, struct RigidBodyWorld *rbw);

void BKE_rigidbody_calc_center_of_mass(struct Object *ob, float r_center[3]);

/* -------------- */
/* Utilities */

struct RigidBodyWorld *BKE_rigidbody_get_world(struct Scene *scene);
void BKE_rigidbody_remove_object(struct Scene *scene, struct Object *ob);
void BKE_rigidbody_remove_constraint(struct Scene *scene, struct Object *ob);
float BKE_rigidbody_calc_volume_dm(struct DerivedMesh *dm, struct RigidBodyOb *rbo, Object *ob);
void BKE_rigidbody_calc_shard_mass(struct Object* ob, struct MeshIsland* mi, struct DerivedMesh* dm);
void BKE_rigidbody_calc_threshold(float max_con_mass, struct FractureModifierData* rmd, struct RigidBodyShardCon *con);
float BKE_rigidbody_calc_max_con_mass(struct Object* ob);
float BKE_rigidbody_calc_min_con_dist(struct Object* ob);
void BKE_rigidbody_start_dist_angle(struct RigidBodyShardCon* con, bool exact, bool both);
void BKE_rigidbody_remove_shard_con(struct Scene* scene, struct RigidBodyShardCon* con);
void BKE_rigidbody_remove_shard(struct Scene* scene, struct MeshIsland *mi);
void BKE_rigidbody_update_ob_array(struct RigidBodyWorld *rbw, bool do_bake_correction);
/* -------------- */
/* Utility Macros */

/* get mass of Rigid Body Object to supply to RigidBody simulators */
#define RBO_GET_MASS(rbo) \
	((rbo && ((rbo->type == RBO_TYPE_PASSIVE) || (rbo->flag & RBO_FLAG_KINEMATIC) || (rbo->flag & RBO_FLAG_DISABLED))) ? (0.0f) : (rbo->mass))
/* get collision margin for Rigid Body Object, triangle mesh and cone shapes cannot embed margin, convex hull always uses custom margin */
#define RBO_GET_MARGIN(rbo) \
	((rbo->flag & RBO_FLAG_USE_MARGIN || rbo->shape == RB_SHAPE_CONVEXH || rbo->shape == RB_SHAPE_TRIMESH || rbo->shape == RB_SHAPE_CONE) ? (rbo->margin) : (0.04f))

/* -------------- */
/* Simulation */

void BKE_rigidbody_aftertrans_update(struct Object *ob, float loc[3], float rot[3], float quat[4], float rotAxis[3], float rotAngle);
void BKE_rigidbody_sync_transforms(struct RigidBodyWorld *rbw, struct Object *ob, float ctime);
bool BKE_rigidbody_check_sim_running(struct RigidBodyWorld *rbw, float ctime);
void BKE_rigidbody_cache_reset(struct RigidBodyWorld *rbw);
void BKE_rigidbody_rebuild_world(struct Scene *scene, float ctime, bool do_resetDynamic);
void BKE_rigidbody_do_simulation(struct Scene *scene, float ctime);

/* -------------------- */
/* Depsgraph evaluation */

struct EvaluationContext;

void BKE_rigidbody_rebuild_sim(struct EvaluationContext *eval_ctx,
                               struct Scene *scene);

void BKE_rigidbody_eval_simulation(struct EvaluationContext *eval_ctx,
                                   struct Scene *scene);

void BKE_rigidbody_object_sync_transforms(struct EvaluationContext *eval_ctx,
                                          struct Scene *scene,
                                          struct Object *ob);

bool BKE_restoreKinematic(struct RigidBodyWorld *rbw, bool override_bind);

#endif /* __BKE_RIGIDBODY_H__ */
