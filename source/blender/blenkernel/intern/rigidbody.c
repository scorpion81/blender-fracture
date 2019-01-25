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
 * Contributor(s): Joshua Leung, Sergej Reich, Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file rigidbody.c
 *  \ingroup blenkernel
 *  \brief Blender-side interface and methods for dealing with Rigid Body simulations
 */

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <float.h>
#include <math.h>
#include <limits.h>

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_callbacks.h"
#include "BLI_math.h"
#include "BLI_kdtree.h"
#include "BLI_rand.h"
#include "BLI_sort.h"
#include "BLI_threads.h"
#include "BLI_utildefines.h"

#ifdef WITH_BULLET
#  include "RBI_api.h"
#endif

#include "DNA_fracture_types.h"
#include "DNA_ID.h"
#include "DNA_group_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_object_force.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_depsgraph.h"
#include "BKE_deform.h"
#include "BKE_effect.h"
#include "BKE_fracture.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_library.h"
#include "BKE_library_query.h"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_pointcache.h"
#include "BKE_rigidbody.h"
#include "BKE_modifier.h"
#include "BKE_depsgraph.h"
#include "BKE_scene.h"
#include "PIL_time.h"
#include "bmesh.h"

#ifdef WITH_BULLET

/* Fracture Modifier related prototypes */

static void resetDynamic(RigidBodyWorld *rbw, bool do_reset_always, bool override_bind);
static void validateShard(RigidBodyWorld *rbw, MeshIsland *mi, Object *ob, int rebuild, int transfer_speed, float size[3]);
static void rigidbody_passive_fake_parenting(FractureModifierData *fmd, Object *ob, RigidBodyOb *rbo, float imat[4][4]);
static void rigidbody_passive_hook(FractureModifierData *fmd, MeshIsland *mi, Object* ob);
static void check_fracture(rbContactPoint *cp, RigidBodyWorld *rbw, Object *obA, Object *obB);
static MeshIsland* findMeshIsland(FractureModifierData *fmd, int id);
static rbCollisionShape* rigidbody_get_shape_convexhull_from_mesh(Mesh *me, float hull_margin, bool* can_embed);
static rbCollisionShape* rigidbody_get_shape_trimesh_from_mesh(Object* ob);
static MeshIsland* find_closest_meshisland_to_point(FractureModifierData* fmd, Object *ob, Object *ob2, RigidBodyWorld* rbw, RigidBodyCon *con);
static int filterCallback(void* world, void* island1, void* island2, void *blenderOb1, void* blenderOb2, bool activate);
static void contactCallback(rbContactPoint* cp, void* world);
static void idCallback(void *world, void* island, int* objectId, int* islandId);
static void tickCallback(float timestep, void *scene);
static bool isModifierActive(FractureModifierData *rmd);
static void activateRigidbody(RigidBodyOb* rbo, RigidBodyWorld *UNUSED_rbw, MeshIsland *mi, Object *ob);
static bool do_update_modifier(Scene* scene, Object* ob, RigidBodyWorld *rbw, bool rebuild);
static bool do_sync_modifier(ModifierData *md, Object *ob, RigidBodyWorld *rbw, float ctime);
static void DM_mesh_boundbox(DerivedMesh *bm, float r_loc[3], float r_size[3]);
static void test_deactivate_rigidbody(RigidBodyOb *rbo, MeshIsland *mi);
static float box_volume(float size[3]);

#endif


/* ************************************** */
/* Memory Management */

/* Freeing Methods --------------------- */

#ifndef WITH_BULLET

static void RB_dworld_remove_constraint(void *UNUSED(world), void *UNUSED(con)) {}
static void RB_dworld_remove_body(void *UNUSED(world), void *UNUSED(body)) {}
static void RB_dworld_delete(void *UNUSED(world)) {}
static void RB_body_delete(void *UNUSED(body)) {}
static void RB_shape_delete(void *UNUSED(shape)) {}
static void RB_constraint_delete(void *UNUSED(con)) {}

#endif

/* Free rigidbody world */
void BKE_rigidbody_free_world(RigidBodyWorld *rbw)
{
	/* sanity check */
	if (!rbw)
		return;

	if (rbw->physics_world) {
		/* free physics references, we assume that all physics objects in will have been added to the world */
		GroupObject *go;
		if (rbw->constraints) {
			for (go = rbw->constraints->gobject.first; go; go = go->next) {
				if (go->ob && go->ob->rigidbody_constraint) {
					RigidBodyCon *rbc = go->ob->rigidbody_constraint;

					if (rbc->physics_constraint)
						RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
				}
			}
		}
		if (rbw->group) {
			for (go = rbw->group->gobject.first; go; go = go->next) {
				if (go->ob && go->ob->rigidbody_object) {
					RigidBodyOb *rbo = go->ob->rigidbody_object;

					if (rbo->physics_object)
						RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
				}
			}
		}
		/* free dynamics world */
		if (rbw->physics_world != NULL)
			RB_dworld_delete(rbw->physics_world);
	}
	if (rbw->objects)
		MEM_freeN(rbw->objects);

	if (rbw->cache_index_map) {
		MEM_freeN(rbw->cache_index_map);
		rbw->cache_index_map = NULL;
	}

	if (rbw->cache_offset_map) {
		MEM_freeN(rbw->cache_offset_map);
		rbw->cache_offset_map = NULL;
	}


	/* free cache */
	BKE_ptcache_free_list(&(rbw->ptcaches));
	rbw->pointcache = NULL;

	/* free effector weights */
	if (rbw->effector_weights)
		MEM_freeN(rbw->effector_weights);

	/* free rigidbody world itself */
	MEM_freeN(rbw);
}

/* Free RigidBody settings and sim instances */
void BKE_rigidbody_free_object(Object *ob)
{
	RigidBodyOb *rbo = (ob) ? ob->rigidbody_object : NULL;

	/* sanity check */
	if (rbo == NULL)
		return;

	/* free physics references */
	if (rbo->physics_object) {
		RB_body_delete(rbo->physics_object);
		rbo->physics_object = NULL;
	}

	if (rbo->physics_shape) {
		RB_shape_delete(rbo->physics_shape);
		rbo->physics_shape = NULL;
	}

	/* free data itself */
	MEM_freeN(rbo);
	ob->rigidbody_object = NULL;
}

/* Free RigidBody constraint and sim instance */
void BKE_rigidbody_free_constraint(Object *ob)
{
	RigidBodyCon *rbc = (ob) ? ob->rigidbody_constraint : NULL;

	/* sanity check */
	if (rbc == NULL)
		return;

	/* free physics reference */
	if (rbc->physics_constraint) {
		RB_constraint_delete(rbc->physics_constraint);
		rbc->physics_constraint = NULL;
	}

	/* free data itself */
	MEM_freeN(rbc);
	ob->rigidbody_constraint = NULL;
}

#ifdef WITH_BULLET

/* Copying Methods --------------------- */

/* These just copy the data, clearing out references to physics objects.
 * Anything that uses them MUST verify that the copied object will
 * be added to relevant groups later...
 */

RigidBodyOb *BKE_rigidbody_copy_object(const Object *ob)
{
	RigidBodyOb *rboN = NULL;

	if (ob->rigidbody_object) {
		/* just duplicate the whole struct first (to catch all the settings) */
		rboN = MEM_dupallocN(ob->rigidbody_object);

		/* tag object as needing to be verified */
		rboN->flag |= RBO_FLAG_NEEDS_VALIDATE;

		/* clear out all the fields which need to be revalidated later */
		rboN->physics_object = NULL;
		rboN->physics_shape = NULL;
	}

	/* return new copy of settings */
	return rboN;
}

RigidBodyCon *BKE_rigidbody_copy_constraint(const Object *ob)
{
	RigidBodyCon *rbcN = NULL;

	if (ob->rigidbody_constraint) {
		/* just duplicate the whole struct first (to catch all the settings) */
		rbcN = MEM_dupallocN(ob->rigidbody_constraint);

		/* tag object as needing to be verified */
		rbcN->flag |= RBC_FLAG_NEEDS_VALIDATE;

		/* clear out all the fields which need to be revalidated later */
		rbcN->physics_constraint = NULL;
	}

	/* return new copy of settings */
	return rbcN;
}

/* although this may be slow, it also may help to correct the mesh bounding problem */
static DerivedMesh* dm_solidify(DerivedMesh *dm, float thickness)
{
	DerivedMesh *result = NULL;
	BMesh *bm = BM_mesh_create(&bm_mesh_allocsize_default, &((struct BMeshCreateParams){.use_toolflags = true,}));
	BMOperator bmop;

	DM_to_bmesh_ex(dm, bm, true);

	BMO_op_initf(bm, &bmop, BMO_FLAG_DEFAULTS, "solidify geom=%af thickness=%f offset=%f", thickness, 0.0f, true);

	/* deselect only the faces in the region to be solidified (leave wire
	 * edges and loose verts selected, as there will be no corresponding
	 * geometry selected below) */
	BMO_slot_buffer_hflag_disable(bm, bmop.slots_in, "geom", BM_FACE, BM_ELEM_SELECT, true);

	/* run the solidify operator */
	BMO_op_exec(bm, &bmop);

	/* select the newly generated faces */
	BMO_slot_buffer_hflag_enable(bm, bmop.slots_out, "geom.out", BM_FACE, BM_ELEM_SELECT, true);

	BMO_op_finish(bm, &bmop);

	result = CDDM_from_bmesh(bm, true);
	BM_mesh_free(bm);

	return result;
}

/* ************************************** */
/* Setup Utilities - Validate Sim Instances */

/* get the appropriate DerivedMesh based on rigid body mesh source */
static DerivedMesh *rigidbody_get_mesh(Object *ob)
{
	if (ob->rigidbody_object->mesh_source == RBO_MESH_DEFORM) {
		return ob->derivedDeform;
	}
	else if (ob->rigidbody_object->mesh_source == RBO_MESH_FINAL) {
		return ob->derivedFinal;
	}
	else if (ob->rigidbody_object->mesh_source == RBO_MESH_FINAL_SOLID) {
		if (ob->derivedFinal) {
			return dm_solidify(ob->derivedFinal, ob->rigidbody_object->margin);
		}
		else {
			// fallback to avoid crash
			return CDDM_from_mesh(ob->data);
		}
	}
	else {
		return CDDM_from_mesh(ob->data);
	}
}

/* Create new physics sim collision shape for object and store it,
 * or remove the existing one first and replace...
 */
static void rigidbody_validate_sim_shape(Object *ob, bool rebuild)
{
	RigidBodyOb *rbo = ob->rigidbody_object;
	rbCollisionShape *new_shape = NULL;
	BoundBox *bb = NULL;
	float size[3] = {1.0f, 1.0f, 1.0f};
	float radius = 1.0f;
	float height = 1.0f;
	float capsule_height;
	float hull_margin = 0.0f;
	bool can_embed = true;
	bool has_volume;

	/* sanity check */
	if (rbo == NULL)
		return;

	/* don't create a new shape if we already have one and don't want to rebuild it */
	if (rbo->physics_shape && !rebuild)
		return;

	/* if automatically determining dimensions, use the Object's boundbox
	 *	- assume that all quadrics are standing upright on local z-axis
	 *	- assume even distribution of mass around the Object's pivot
	 *	  (i.e. Object pivot is centralized in boundbox)
	 */
	// XXX: all dimensions are auto-determined now... later can add stored settings for this
	/* get object dimensions without scaling */
	bb = BKE_object_boundbox_get(ob);
	if (bb) {
		size[0] = (bb->vec[4][0] - bb->vec[0][0]);
		size[1] = (bb->vec[2][1] - bb->vec[0][1]);
		size[2] = (bb->vec[1][2] - bb->vec[0][2]);
	}
	mul_v3_fl(size, 0.5f);

	if (ELEM(rbo->shape, RB_SHAPE_CAPSULE, RB_SHAPE_CYLINDER, RB_SHAPE_CONE)) {
		/* take radius as largest x/y dimension, and height as z-dimension */
		radius = MAX2(size[0], size[1]);
		height = size[2];
	}
	else if (rbo->shape == RB_SHAPE_SPHERE) {
		/* take radius to the largest dimension to try and encompass everything */
		radius = MAX3(size[0], size[1], size[2]);
	}

	/* create new shape */
	switch (rbo->shape) {
		case RB_SHAPE_BOX:
			new_shape = RB_shape_new_box(size[0], size[1], size[2]);
			break;

		case RB_SHAPE_SPHERE:
			new_shape = RB_shape_new_sphere(radius + RBO_GET_MARGIN(rbo));
			break;

		case RB_SHAPE_CAPSULE:
			capsule_height = (height - radius) * 2.0f;
			new_shape = RB_shape_new_capsule(radius, (capsule_height > 0.0f) ? capsule_height : 0.0f);
			break;
		case RB_SHAPE_CYLINDER:
			new_shape = RB_shape_new_cylinder(radius, height);
			break;
		case RB_SHAPE_CONE:
			new_shape = RB_shape_new_cone(radius, height * 2.0f);
			break;

		case RB_SHAPE_CONVEXH:
			/* try to emged collision margin */
			has_volume = (MIN3(size[0], size[1], size[2]) > 0.0f);

			if (!(rbo->flag & RBO_FLAG_USE_MARGIN) && has_volume)
				hull_margin = 0.04f;
			if (ob->type == OB_MESH && ob->data) {
				new_shape = rigidbody_get_shape_convexhull_from_mesh((Mesh *)ob->data, hull_margin, &can_embed);
			}
			else {
				printf("ERROR: cannot make Convex Hull collision shape for non-Mesh object\n");
			}

			if (!(rbo->flag & RBO_FLAG_USE_MARGIN))
				rbo->margin = (can_embed && has_volume) ? 0.04f : 0.0f;  /* RB_TODO ideally we shouldn't directly change the margin here */
			break;
		case RB_SHAPE_TRIMESH:
			new_shape = rigidbody_get_shape_trimesh_from_mesh(ob);
			break;
	}
	/* use box shape if we can't fall back to old shape */
	if (new_shape == NULL && rbo->physics_shape == NULL) {
		new_shape = RB_shape_new_box(size[0], size[1], size[2]);
	}
	/* assign new collision shape if creation was successful */
	if (new_shape) {
		if (rbo->physics_shape)
			RB_shape_delete(rbo->physics_shape);
		rbo->physics_shape = new_shape;
		RB_shape_set_margin(rbo->physics_shape, RBO_GET_MARGIN(rbo));
	}
}

void BKE_rigidbody_calc_center_of_mass(Object *ob, float r_center[3])
{
	RigidBodyOb *rbo = ob->rigidbody_object;

	float size[3]  = {1.0f, 1.0f, 1.0f};
	float height = 1.0f;

	zero_v3(r_center);

	/* if automatically determining dimensions, use the Object's boundbox
	 *	- assume that all quadrics are standing upright on local z-axis
	 *	- assume even distribution of mass around the Object's pivot
	 *	  (i.e. Object pivot is centralized in boundbox)
	 *	- boundbox gives full width
	 */
	// XXX: all dimensions are auto-determined now... later can add stored settings for this
	BKE_object_dimensions_get(ob, size);

	/* calculate volume as appropriate  */
	switch (rbo->shape) {
		case RB_SHAPE_BOX:
		case RB_SHAPE_SPHERE:
		case RB_SHAPE_CAPSULE:
		case RB_SHAPE_CYLINDER:
			break;

		case RB_SHAPE_CONE:
			/* take radius as largest x/y dimension, and height as z-dimension */
			height = size[2];
			/* cone is geometrically centered on the median,
			 * center of mass is 1/4 up from the base
			 */
			r_center[2] = -0.25f * height;
			break;

		case RB_SHAPE_CONVEXH:
		case RB_SHAPE_TRIMESH:
		{
			if (ob->type == OB_MESH) {
				DerivedMesh *dm = rigidbody_get_mesh(ob);
				MVert *mvert;
				const MLoopTri *looptri;
				int totvert, tottri;
				const MLoop *mloop;
				
				/* ensure mesh validity, then grab data */
				if (dm == NULL)
					return;
			
				mvert   = dm->getVertArray(dm);
				totvert = dm->getNumVerts(dm);
				looptri = dm->getLoopTriArray(dm);
				tottri = dm->getNumLoopTri(dm);
				mloop = dm->getLoopArray(dm);
				
				if (totvert > 0 && tottri > 0) {
					BKE_mesh_calc_volume(mvert, totvert, looptri, tottri, mloop, NULL, r_center);
				}
				
				/* cleanup temp data */
				if (ob->rigidbody_object->mesh_source == RBO_MESH_BASE) {
					dm->release(dm);
				}
			}
			break;
		}

#if 0 // XXX: not defined yet
		case RB_SHAPE_COMPOUND:
			volume = 0.0f;
			break;
#endif
	}
}

/* --------------------- */

/**
 * Create physics sim representation of object given RigidBody settings
 *
 * \param rebuild Even if an instance already exists, replace it
 */
static void rigidbody_validate_sim_object(RigidBodyWorld *rbw, Object *ob, bool rebuild, bool transfer_speeds)
{
	RigidBodyOb *rbo = (ob) ? ob->rigidbody_object : NULL;
	float loc[3];
	float rot[4];

	/* sanity checks:
	 *	- object doesn't have RigidBody info already: then why is it here?
	 */
	if (rbo == NULL)
		return;

	/* make sure collision shape exists */
	/* FIXME we shouldn't always have to rebuild collision shapes when rebuilding objects, but it's needed for constraints to update correctly */
	if (rbo->physics_shape == NULL || rebuild)
		rigidbody_validate_sim_shape(ob, true);

	if (rbo->physics_object && rebuild == false) {
		RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
	}
	if (!rbo->physics_object || rebuild) {

		float locbb[3], size[3];
		/* remove rigid body if it already exists before creating a new one */
		if (rbo->physics_object) {
			RB_body_delete(rbo->physics_object);
		}

		mat4_to_loc_quat(loc, rot, ob->obmat);

		/* a hacky check whether the dm might be valid or not*/
		if (ob->derivedFinal && ob->derivedFinal->getNumLoopTri(ob->derivedFinal) > 0)
			DM_mesh_boundbox(ob->derivedFinal, locbb, size);
		else  //fallback
			BKE_mesh_boundbox_calc((Mesh*)ob->data, locbb, size);
		mul_v3_v3(size, ob->size);

		rbo->physics_object = RB_body_new(rbo->physics_shape, loc, rot, false, 0.0f, 0.0f, 0.0f, 0.0f, size);

		RB_body_set_friction(rbo->physics_object, rbo->friction);
		RB_body_set_restitution(rbo->physics_object, rbo->restitution);

		RB_body_set_damping(rbo->physics_object, rbo->lin_damping, rbo->ang_damping);
		RB_body_set_sleep_thresh(rbo->physics_object, rbo->lin_sleep_thresh, rbo->ang_sleep_thresh);
		RB_body_set_activation_state(rbo->physics_object, rbo->flag & RBO_FLAG_USE_DEACTIVATION);

		if (rbo->type == RBO_TYPE_PASSIVE || rbo->flag & RBO_FLAG_START_DEACTIVATED)
			RB_body_deactivate(rbo->physics_object);


		RB_body_set_linear_factor(rbo->physics_object,
		                          (ob->protectflag & OB_LOCK_LOCX) == 0,
		                          (ob->protectflag & OB_LOCK_LOCY) == 0,
		                          (ob->protectflag & OB_LOCK_LOCZ) == 0);
		RB_body_set_angular_factor(rbo->physics_object,
		                           (ob->protectflag & OB_LOCK_ROTX) == 0,
		                           (ob->protectflag & OB_LOCK_ROTY) == 0,
		                           (ob->protectflag & OB_LOCK_ROTZ) == 0);

		RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
		RB_body_set_kinematic_state(rbo->physics_object, rbo->flag & RBO_FLAG_KINEMATIC || rbo->flag & RBO_FLAG_DISABLED);

		if (transfer_speeds)
		{
			if ((len_squared_v3(rbo->lin_vel) > (rbo->lin_sleep_thresh * rbo->lin_sleep_thresh)))
			{
				printf("Setting linear velocity (%f, %f, %f)\n", rbo->lin_vel[0], rbo->lin_vel[1], rbo->lin_vel[2]);
				RB_body_set_linear_velocity(rbo->physics_object, rbo->lin_vel);
			}

			if ((len_squared_v3(rbo->ang_vel) > (rbo->ang_sleep_thresh * rbo->ang_sleep_thresh)))
			{
				printf("Setting angular velocity (%f, %f, %f)\n", rbo->ang_vel[0], rbo->ang_vel[1], rbo->ang_vel[2]);
				RB_body_set_angular_velocity(rbo->physics_object, rbo->ang_vel);
			}
		}
	}
	else if (rbo->flag & RBO_FLAG_KINEMATIC_REBUILD )
	{
		RB_body_deactivate(rbo->physics_object);
		RB_body_set_mass(rbo->physics_object, 0.0f);
		RB_body_set_kinematic_state(rbo->physics_object, true);
	}

	if (rbw && rbw->physics_world)
		RB_dworld_add_body(rbw->physics_world, rbo->physics_object, rbo->col_groups, NULL, ob, rbo->meshisland_index);
}

/**
 * Create physics sim representation of constraint given rigid body constraint settings
 *
 * < rebuild: even if an instance already exists, replace it
 */
static void rigidbody_validate_sim_constraint(RigidBodyWorld *rbw, Object *ob, bool rebuild)
{
	RigidBodyCon *rbc = (ob) ? ob->rigidbody_constraint : NULL;
	float loc[3];
	float rot[4];
	float lin_lower;
	float lin_upper;
	float ang_lower;
	float ang_upper;

	/* sanity checks:
	 *	- object should have a rigid body constraint
	 *  - rigid body constraint should have at least one constrained object
	 */
	if (rbc == NULL) {
		return;
	}

	if (ELEM(NULL, rbc->ob1, rbc->ob1->rigidbody_object, rbc->ob2, rbc->ob2->rigidbody_object)) {
		if (rbc->physics_constraint) {
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}
		return;
	}

	if (rbc->physics_constraint && rebuild == false) {
		RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
	}
	if (rbc->physics_constraint == NULL || rebuild) {
		rbRigidBody *rb1 = NULL, *rb2 = NULL;
		FractureModifierData *fmd1 = NULL, *fmd2 = NULL;
		MeshIsland *mi1, *mi2;

		/*add logic to find closest shards of 2 FM objects, or to attach one shard to one regular */
		/*for now, just take the first meshislands... later find the closest ones or the closest to center*/
		fmd1 = (FractureModifierData*)modifiers_findByType(rbc->ob1, eModifierType_Fracture);
		fmd2 = (FractureModifierData*)modifiers_findByType(rbc->ob2, eModifierType_Fracture);

		if (fmd1 && fmd2) {
			mi1 = find_closest_meshisland_to_point(fmd1, rbc->ob1, ob, rbw, rbc);
			mi2 = find_closest_meshisland_to_point(fmd2, rbc->ob2, ob, rbw, rbc);

			if (mi1 && mi2) {
				rb1 = mi1->rigidbody->physics_object;
				rb2 = mi2->rigidbody->physics_object;
			}
		}
		else if (fmd1) {
			mi1 = find_closest_meshisland_to_point(fmd1, rbc->ob1, ob, rbw, rbc);
			if (mi1) {
				rb1 = mi1->rigidbody->physics_object;
				rb2 = rbc->ob2->rigidbody_object->physics_object;
			}
		}
		else if (fmd2) {
			mi2 = find_closest_meshisland_to_point(fmd2, rbc->ob2, ob, rbw, rbc);
			if (mi2)
			{
				rb2 = mi2->rigidbody->physics_object;
				rb1 = rbc->ob1->rigidbody_object->physics_object;
			}
		}
		else {
			rb1 = rbc->ob1->rigidbody_object->physics_object;
			rb2 = rbc->ob2->rigidbody_object->physics_object;
		}

		/* remove constraint if it already exists before creating a new one */
		if (rbc->physics_constraint) {
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}

		mat4_to_loc_quat(loc, rot, ob->obmat);

		if (rb1 && rb2) {
			switch (rbc->type) {
				case RBC_TYPE_POINT:
					rbc->physics_constraint = RB_constraint_new_point(loc, rb1, rb2);
					break;
				case RBC_TYPE_FIXED:
					rbc->physics_constraint = RB_constraint_new_fixed(loc, rot, rb1, rb2);
					break;
				case RBC_TYPE_HINGE:
					rbc->physics_constraint = RB_constraint_new_hinge(loc, rot, rb1, rb2);
					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Z) {
						RB_constraint_set_limits_hinge(rbc->physics_constraint, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
					}
					else
						RB_constraint_set_limits_hinge(rbc->physics_constraint, 0.0f, -1.0f);
					break;
				case RBC_TYPE_SLIDER:
					rbc->physics_constraint = RB_constraint_new_slider(loc, rot, rb1, rb2);
					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
						RB_constraint_set_limits_slider(rbc->physics_constraint, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
					else
						RB_constraint_set_limits_slider(rbc->physics_constraint, 0.0f, -1.0f);
					break;
				case RBC_TYPE_PISTON:
					rbc->physics_constraint = RB_constraint_new_piston(loc, rot, rb1, rb2);
					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X) {
						lin_lower = rbc->limit_lin_x_lower;
						lin_upper = rbc->limit_lin_x_upper;
					}
					else {
						lin_lower = 0.0f;
						lin_upper = -1.0f;
					}
					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_X) {
						ang_lower = rbc->limit_ang_x_lower;
						ang_upper = rbc->limit_ang_x_upper;
					}
					else {
						ang_lower = 0.0f;
						ang_upper = -1.0f;
					}
					RB_constraint_set_limits_piston(rbc->physics_constraint, lin_lower, lin_upper, ang_lower, ang_upper);
					break;
				case RBC_TYPE_6DOF_SPRING:
					rbc->physics_constraint = RB_constraint_new_6dof_spring(loc, rot, rb1, rb2);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->flag & RBC_FLAG_USE_SPRING_X);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->spring_stiffness_x);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->spring_damping_x);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->flag & RBC_FLAG_USE_SPRING_Y);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->spring_stiffness_y);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->spring_damping_y);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->flag & RBC_FLAG_USE_SPRING_Z);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->spring_stiffness_z);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->spring_damping_z);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->flag & RBC_FLAG_USE_SPRING_ANG_X);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->spring_stiffness_ang_x);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->spring_damping_ang_x);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Y);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->spring_stiffness_ang_y);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->spring_damping_ang_y);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Z);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->spring_stiffness_ang_z);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->spring_damping_ang_z);

					RB_constraint_set_equilibrium_6dof_spring(rbc->physics_constraint);
					ATTR_FALLTHROUGH;
				case RBC_TYPE_6DOF:
					if (rbc->type == RBC_TYPE_6DOF) /* a litte awkward but avoids duplicate code for limits */
						rbc->physics_constraint = RB_constraint_new_6dof(loc, rot, rb1, rb2);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Y)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->limit_lin_y_lower, rbc->limit_lin_y_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Z)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->limit_lin_z_lower, rbc->limit_lin_z_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_X)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->limit_ang_x_lower, rbc->limit_ang_x_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Y)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->limit_ang_y_lower, rbc->limit_ang_y_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Z)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, 0.0f, -1.0f);
					break;
				case RBC_TYPE_MOTOR:
					rbc->physics_constraint = RB_constraint_new_motor(loc, rot, rb1, rb2);

					RB_constraint_set_enable_motor(rbc->physics_constraint, rbc->flag & RBC_FLAG_USE_MOTOR_LIN, rbc->flag & RBC_FLAG_USE_MOTOR_ANG);
					RB_constraint_set_max_impulse_motor(rbc->physics_constraint, rbc->motor_lin_max_impulse, rbc->motor_ang_max_impulse);
					RB_constraint_set_target_velocity_motor(rbc->physics_constraint, rbc->motor_lin_target_velocity, rbc->motor_ang_target_velocity);
					break;
			}
		}
		else { /* can't create constraint without both rigid bodies */
			return;
		}

		RB_constraint_set_enabled(rbc->physics_constraint, rbc->flag & RBC_FLAG_ENABLED);

		if (rbc->flag & RBC_FLAG_USE_BREAKING)
			RB_constraint_set_breaking_threshold(rbc->physics_constraint, rbc->breaking_threshold);
		else
			RB_constraint_set_breaking_threshold(rbc->physics_constraint, FLT_MAX);

		if (rbc->flag & RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS)
			RB_constraint_set_solver_iterations(rbc->physics_constraint, rbc->num_solver_iterations);
		else
			RB_constraint_set_solver_iterations(rbc->physics_constraint, -1);
	}

	if (rbw && rbw->physics_world && rbc->physics_constraint) {
		RB_dworld_add_constraint(rbw->physics_world, rbc->physics_constraint, rbc->flag & RBC_FLAG_DISABLE_COLLISIONS);
	}

	if (rbc->physics_constraint)
	{
		/*
		char id[64];
		char *rest = id;
		char *ptr;
		char ptr2[64] = "0";

		strncpy(id, ob->id.name + 2, strlen(ob->id.name) - 2);

		ptr = strtok_r(id, ".", &rest);
		while (ptr != NULL)
		{
			strncpy(ptr2, ptr, strlen(ptr));
			ptr = strtok_r(NULL, ".", &rest);
		}*/

		RB_constraint_set_id(rbc->physics_constraint, ob->id.name + 2);
	}
}



/* --------------------- */

/* Create physics sim world given RigidBody world settings */
// NOTE: this does NOT update object references that the scene uses, in case those aren't ready yet!
void BKE_rigidbody_validate_sim_world(Scene *scene, RigidBodyWorld *rbw, bool rebuild)
{
	/* sanity checks */
	if (rbw == NULL)
		return;

	/* create new sim world */
	if (rebuild || rbw->physics_world == NULL) {
		if (rbw->physics_world)
			RB_dworld_delete(rbw->physics_world);
		rbw->physics_world = RB_dworld_new(scene->physics_settings.gravity, rbw, scene, filterCallback, contactCallback, idCallback, tickCallback);
	}

	RB_dworld_set_solver_iterations(rbw->physics_world, rbw->num_solver_iterations);
	RB_dworld_set_split_impulse(rbw->physics_world, rbw->flag & RBW_FLAG_USE_SPLIT_IMPULSE);
}

/* ************************************** */
/* Setup Utilities - Create Settings Blocks */

/* Set up RigidBody world */
RigidBodyWorld *BKE_rigidbody_create_world(Scene *scene)
{
	/* try to get whatever RigidBody world that might be representing this already */
	RigidBodyWorld *rbw;

	/* sanity checks
	 *	- there must be a valid scene to add world to
	 *	- there mustn't be a sim world using this group already
	 */
	if (scene == NULL)
		return NULL;

	/* create a new sim world */
	rbw = MEM_callocN(sizeof(RigidBodyWorld), "RigidBodyWorld");

	/* set default settings */
	rbw->effector_weights = BKE_add_effector_weights(NULL);

	rbw->ltime = PSFRA;

	rbw->time_scale = 1.0f;

	rbw->steps_per_second = 60; /* Bullet default (60 Hz) */
	rbw->num_solver_iterations = 10; /* 10 is bullet default */

	rbw->pointcache = BKE_ptcache_add(&(rbw->ptcaches));
	rbw->pointcache->step = 1;
	rbw->flag &=~ RBW_FLAG_OBJECT_CHANGED;
	rbw->flag &=~ RBW_FLAG_REFRESH_MODIFIERS;

	rbw->objects = MEM_mallocN(sizeof(Object *), "objects");
	rbw->cache_index_map = MEM_mallocN(sizeof(RigidBodyOb *), "cache_index_map");
	rbw->cache_offset_map = MEM_mallocN(sizeof(int), "cache_offset_map");

	/* return this sim world */
	return rbw;
}

/* Add rigid body settings to the specified shard */
RigidBodyOb *BKE_rigidbody_create_shard(Scene *scene, Object *ob, Object *target, MeshIsland *mi)
{
	RigidBodyOb *rbo;
	RigidBodyWorld *rbw = BKE_rigidbody_get_world(scene);
	float centr[3], size[3], mat[4][4];

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- shard must exist
	 *	- cannot add rigid body if it already exists
	 */
	if (mi == NULL || (mi->rigidbody != NULL))
		return NULL;

	if (ob->type != OB_MESH && ob->type != OB_FONT && ob->type != OB_CURVE && ob->type != OB_SURF) {
		return NULL;
	}
	
	if ((ob->type == OB_MESH) && (((Mesh *)ob->data)->totvert == 0)) {
		return NULL;
	}

	/* Add rigid body world and group if they don't exist for convenience */
	if (rbw == NULL) {
		rbw = BKE_rigidbody_create_world(scene);
		BKE_rigidbody_validate_sim_world(scene, rbw, false);
		scene->rigidbody_world = rbw;
	}
	if (rbw->group == NULL) {
		rbw->group = BKE_group_add(G.main, "RigidBodyWorld");
	}

	/* make rigidbody object settings */
	if (ob->rigidbody_object == NULL) {
		ob->rigidbody_object = BKE_rigidbody_create_object(scene, ob, RBO_TYPE_ACTIVE, NULL);
	}
	else {
		ob->rigidbody_object->type = RBO_TYPE_ACTIVE;
		ob->rigidbody_object->flag |= RBO_FLAG_NEEDS_VALIDATE;
	}

	if (!BKE_group_object_exists(rbw->group, ob))
		BKE_group_object_add(rbw->group, ob, scene, NULL);

	DAG_id_tag_update(&ob->id, OB_RECALC_OB);

	/* since we are always member of an object, dupe its settings,
	 * create new settings data, and link it up */
	if (target && target->rigidbody_object)
	{
		rbo = BKE_rigidbody_copy_object(target);
		mat4_to_loc_quat(rbo->pos, rbo->orn, target->obmat);
		zero_v3(rbo->lin_vel);
		zero_v3(rbo->ang_vel);
	}
	else
	{
		/* regular FM case */
		rbo = BKE_rigidbody_copy_object(ob);
		rbo->type = mi->ground_weight > 0.01f ? RBO_TYPE_PASSIVE : RBO_TYPE_ACTIVE;

		/* set initial transform */
		mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
		mat4_to_size(size, ob->obmat);

		//add initial "offset" (centroid), maybe subtract ob->obmat ?? (not sure)
		copy_v3_v3(centr, mi->centroid);
		mul_v3_v3(centr, size);
		mul_qt_v3(rbo->orn, centr);
		add_v3_v3(rbo->pos, centr);
		zero_v3(rbo->lin_vel);
		zero_v3(rbo->ang_vel);
	}

	/* return this object */
	return rbo;
}

RigidBodyWorld *BKE_rigidbody_world_copy(RigidBodyWorld *rbw)
{
	RigidBodyWorld *rbwn = MEM_dupallocN(rbw);

	if (rbw->effector_weights)
		rbwn->effector_weights = MEM_dupallocN(rbw->effector_weights);
	if (rbwn->group)
		id_us_plus(&rbwn->group->id);
	if (rbwn->constraints)
		id_us_plus(&rbwn->constraints->id);

	rbwn->pointcache = BKE_ptcache_copy_list(&rbwn->ptcaches, &rbw->ptcaches, true);

	rbwn->objects = NULL;
	rbwn->physics_world = NULL;
	rbwn->numbodies = 0;

	rbwn->cache_index_map = NULL;
	rbwn->cache_offset_map = NULL;

	return rbwn;
}

void BKE_rigidbody_world_groups_relink(RigidBodyWorld *rbw)
{
	ID_NEW_REMAP(rbw->group);
	ID_NEW_REMAP(rbw->constraints);
	ID_NEW_REMAP(rbw->effector_weights->group);
}

void BKE_rigidbody_world_id_loop(RigidBodyWorld *rbw, RigidbodyWorldIDFunc func, void *userdata)
{
	GroupObject *go;

	func(rbw, (ID **)&rbw->group, userdata, IDWALK_CB_NOP);
	func(rbw, (ID **)&rbw->constraints, userdata, IDWALK_CB_NOP);
	func(rbw, (ID **)&rbw->effector_weights->group, userdata, IDWALK_CB_NOP);

	/* in regular blender one rigidbody is equivalent to one object, but in FM build you can have more
	 * rigidbodies in the world than objects in the object array... thats why loop over the group of objects
	 * of the rigidbody world... the rbw->objects array would contain group objects only anyway, also dont forget
	 * regular constraints if there are any */

	/*if (rbw->objects) {
		int i;
		int count = BLI_listbase_count(&rbw->group->gobject);
		for (i = 0; i < count; i++) {
			func(rbw, (ID **)&rbw->objects[i], userdata, IDWALK_CB_NOP);
		}
	}*/

	if (rbw->group) {
		for (go = rbw->group->gobject.first; go; go = go->next) {
			func(rbw, (ID **)&go->ob, userdata, IDWALK_CB_NOP);
		}
	}

	if (rbw->constraints) {
		for (go = rbw->constraints->gobject.first; go; go = go->next) {
			func(rbw, (ID **)&go->ob, userdata, IDWALK_CB_NOP);
		}
	}
}

/* Add rigid body settings to the specified object */
RigidBodyOb *BKE_rigidbody_create_object(Scene *scene, Object *ob, short type, MeshIsland *mi)
{
	RigidBodyOb *rbo;
	RigidBodyWorld *rbw = scene->rigidbody_world;
	FractureModifierData *fmd = NULL;

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- object must exist
	 *	- cannot add rigid body if it already exists
	 */
	if (ob == NULL || (ob->rigidbody_object != NULL))
		return NULL;

	/* create new settings data, and link it up */
	rbo = MEM_callocN(sizeof(RigidBodyOb), "RigidBodyOb");

	if (mi != NULL && mi->rigidbody != NULL)
	{
		rbo->flag = mi->rigidbody->flag;

		rbo->physics_object = NULL;
		rbo->physics_shape = NULL;

		rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;

		rbo->type = mi->rigidbody->type;
		rbo->mass = mi->rigidbody->mass;

		rbo->friction = mi->rigidbody->friction;
		rbo->restitution = mi->rigidbody->restitution;

		rbo->margin = mi->rigidbody->margin;

		rbo->lin_sleep_thresh = mi->rigidbody->lin_sleep_thresh;
		rbo->ang_sleep_thresh = mi->rigidbody->ang_sleep_thresh;
		rbo->force_thresh = mi->rigidbody->force_thresh;

		rbo->lin_damping = mi->rigidbody->lin_damping;
		rbo->ang_damping = mi->rigidbody->ang_damping;

		rbo->col_groups = mi->rigidbody->col_groups;

		rbo->shape = mi->rigidbody->shape;
		rbo->mesh_source = mi->rigidbody->mesh_source;
		rbo->meshisland_index = mi->rigidbody->meshisland_index;
		rbo->is_fractured = true;
		copy_v3_v3(rbo->pos, mi->rigidbody->pos);
		copy_qt_qt(rbo->orn, mi->rigidbody->orn);
		//mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
	}
	else
	{
		/* set default settings */
		rbo->type = type;

		rbo->mass = 1.0f;

		rbo->friction = 0.5f; /* best when non-zero. 0.5 is Bullet default */
		rbo->restitution = 0.0f; /* best when zero. 0.0 is Bullet default */

		rbo->margin = 0.04f; /* 0.04 (in meters) is Bullet default */

		rbo->lin_sleep_thresh = 0.4f; /* 0.4 is half of Bullet default */
		rbo->ang_sleep_thresh = 0.5f; /* 0.5 is half of Bullet default */
		rbo->force_thresh = 0.0f; /*dont activate by force by default */

		rbo->lin_damping = 0.04f; /* 0.04 is game engine default */
		rbo->ang_damping = 0.1f; /* 0.1 is game engine default */

		rbo->col_groups = 1;

		/* use triangle meshes for passive objects
		 * use convex hulls for active objects since dynamic triangle meshes are very unstable
		 */
		if (type == RBO_TYPE_ACTIVE)
			rbo->shape = RB_SHAPE_CONVEXH;
		else
			rbo->shape = RB_SHAPE_TRIMESH;

		rbo->mesh_source = RBO_MESH_DEFORM;

		/* set initial transform */
		mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);

		rbo->meshisland_index = -1;
		rbo->is_fractured = false;
	}

	zero_v3(rbo->lin_vel);
	zero_v3(rbo->ang_vel);

	fmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);
	if (fmd && fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{	//keep cache here
		return rbo;
	}

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);

	/* return this object */
	return rbo;
}

/* Add rigid body constraint to the specified object */
RigidBodyCon *BKE_rigidbody_create_constraint(Scene *scene, Object *ob, short type, RigidBodyShardCon *con)
{
	RigidBodyCon *rbc;
	RigidBodyWorld *rbw = scene->rigidbody_world;

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- object must exist
	 *	- cannot add constraint if it already exists
	 */
	if (ob == NULL || (ob->rigidbody_constraint != NULL))
		return NULL;

	/* create new settings data, and link it up */
	rbc = MEM_callocN(sizeof(RigidBodyCon), "RigidBodyCon");

	/* set default settings */
	rbc->type = type;

	rbc->ob1 = NULL;
	rbc->ob2 = NULL;

	rbc->flag |= RBC_FLAG_ENABLED;
	rbc->flag |= RBC_FLAG_DISABLE_COLLISIONS;

	if (con)
	{
		rbc->flag = con->flag;
		rbc->breaking_threshold = con->breaking_threshold;
		rbc->num_solver_iterations = con->num_solver_iterations;

		rbc->limit_lin_x_lower = con->limit_lin_x_lower;
		rbc->limit_lin_x_upper = con->limit_lin_x_upper;
		rbc->limit_lin_y_lower = con->limit_lin_y_lower;
		rbc->limit_lin_y_upper = con->limit_lin_y_upper;
		rbc->limit_lin_z_lower = con->limit_lin_z_lower;
		rbc->limit_lin_z_upper = con->limit_lin_z_upper;
		rbc->limit_ang_x_lower = con->limit_ang_x_lower;
		rbc->limit_ang_x_upper = con->limit_ang_x_upper;
		rbc->limit_ang_y_lower = con->limit_ang_y_lower;
		rbc->limit_ang_y_upper = con->limit_ang_y_upper;
		rbc->limit_ang_z_lower = con->limit_ang_z_lower;
		rbc->limit_ang_z_upper = con->limit_ang_z_upper;

		rbc->spring_damping_x = con->spring_damping_x;
		rbc->spring_damping_y = con->spring_damping_y;
		rbc->spring_damping_z = con->spring_damping_z;
		rbc->spring_damping_ang_x = con->spring_damping_ang_x;
		rbc->spring_damping_ang_y = con->spring_damping_ang_y;
		rbc->spring_damping_ang_z = con->spring_damping_ang_z;
		rbc->spring_stiffness_x = con->spring_stiffness_x;
		rbc->spring_stiffness_y = con->spring_stiffness_y;
		rbc->spring_stiffness_z = con->spring_stiffness_z;
		rbc->spring_stiffness_ang_x = con->spring_stiffness_ang_x;
		rbc->spring_stiffness_ang_y = con->spring_stiffness_ang_y;
		rbc->spring_stiffness_ang_z = con->spring_stiffness_ang_z;

		rbc->motor_lin_max_impulse = con->motor_lin_max_impulse;
		rbc->motor_lin_target_velocity = con->motor_lin_target_velocity;
		rbc->motor_ang_max_impulse = con->motor_ang_max_impulse;
		rbc->motor_ang_target_velocity = con->motor_ang_target_velocity;
	}
	else
	{
		rbc->breaking_threshold = 10.0f; /* no good default here, just use 10 for now */
		rbc->num_solver_iterations = 10; /* 10 is Bullet default */

		rbc->limit_lin_x_lower = -1.0f;
		rbc->limit_lin_x_upper = 1.0f;
		rbc->limit_lin_y_lower = -1.0f;
		rbc->limit_lin_y_upper = 1.0f;
		rbc->limit_lin_z_lower = -1.0f;
		rbc->limit_lin_z_upper = 1.0f;
		rbc->limit_ang_x_lower = -M_PI_4;
		rbc->limit_ang_x_upper = M_PI_4;
		rbc->limit_ang_y_lower = -M_PI_4;
		rbc->limit_ang_y_upper = M_PI_4;
		rbc->limit_ang_z_lower = -M_PI_4;
		rbc->limit_ang_z_upper = M_PI_4;

		rbc->spring_damping_x = 0.5f;
		rbc->spring_damping_y = 0.5f;
		rbc->spring_damping_z = 0.5f;
		rbc->spring_damping_ang_x = 0.5f;
		rbc->spring_damping_ang_y = 0.5f;
		rbc->spring_damping_ang_z = 0.5f;
		rbc->spring_stiffness_x = 10.0f;
		rbc->spring_stiffness_y = 10.0f;
		rbc->spring_stiffness_z = 10.0f;
		rbc->spring_stiffness_ang_x = 10.0f;
		rbc->spring_stiffness_ang_y = 10.0f;
		rbc->spring_stiffness_ang_z = 10.0f;

		rbc->motor_lin_max_impulse = 1.0f;
		rbc->motor_lin_target_velocity = 1.0f;
		rbc->motor_ang_max_impulse = 1.0f;
		rbc->motor_ang_target_velocity = 1.0f;
	}

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);

	/* return this object */
	return rbc;
}

/* Add rigid body constraint to the specified object */
RigidBodyShardCon *BKE_rigidbody_create_shard_constraint(Scene *scene, short type, bool reset)
{
	RigidBodyShardCon *rbc;
	RigidBodyWorld *rbw = scene->rigidbody_world;

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- object must exist
	 *	- cannot add constraint if it already exists
	 */

	/* create new settings data, and link it up */
	rbc = MEM_callocN(sizeof(RigidBodyShardCon), "RigidBodyShardCon");

	/* set default settings */
	rbc->type = type;

	rbc->mi1 = NULL;
	rbc->mi2 = NULL;

	rbc->flag |= RBC_FLAG_ENABLED;
	rbc->flag &= ~RBC_FLAG_DISABLE_COLLISIONS;
	rbc->flag |= RBC_FLAG_USE_BREAKING;

	rbc->breaking_threshold = 10.0f; /* no good default here, just use 10 for now */
	rbc->num_solver_iterations = 10; /* 10 is Bullet default */

	rbc->limit_lin_x_lower = -1.0f;
	rbc->limit_lin_x_upper = 1.0f;
	rbc->limit_lin_y_lower = -1.0f;
	rbc->limit_lin_y_upper = 1.0f;
	rbc->limit_lin_z_lower = -1.0f;
	rbc->limit_lin_z_upper = 1.0f;
	rbc->limit_ang_x_lower = -M_PI_4;
	rbc->limit_ang_x_upper = M_PI_4;
	rbc->limit_ang_y_lower = -M_PI_4;
	rbc->limit_ang_y_upper = M_PI_4;
	rbc->limit_ang_z_lower = -M_PI_4;
	rbc->limit_ang_z_upper = M_PI_4;

	rbc->spring_damping_x = 0.5f;
	rbc->spring_damping_y = 0.5f;
	rbc->spring_damping_z = 0.5f;
	rbc->spring_damping_ang_x = 0.5f;
	rbc->spring_damping_ang_y = 0.5f;
	rbc->spring_damping_ang_z = 0.5f;
	rbc->spring_stiffness_x = 10.0f;
	rbc->spring_stiffness_y = 10.0f;
	rbc->spring_stiffness_z = 10.0f;
	rbc->spring_stiffness_ang_x = 10.0f;
	rbc->spring_stiffness_ang_y = 10.0f;
	rbc->spring_stiffness_ang_z = 10.0f;

	rbc->motor_lin_max_impulse = 1.0f;
	rbc->motor_lin_target_velocity = 1.0f;
	rbc->motor_ang_max_impulse = 1.0f;
	rbc->motor_ang_target_velocity = 1.0f;
	strcpy(rbc->name, "");
	zero_v3(rbc->pos);
	unit_qt(rbc->orn);
	rbc->breaking_angle = 0.0f;
	rbc->breaking_dist = 0.0f;

	/* flag cache as outdated */
	if (reset)
		BKE_rigidbody_cache_reset(rbw);

	/* return this object */
	return rbc;
}

/* ************************************** */
/* Utilities API */

/* Get RigidBody world for the given scene, creating one if needed
 *
 * \param scene Scene to find active Rigid Body world for
 */
RigidBodyWorld *BKE_rigidbody_get_world(Scene *scene)
{
	/* sanity check */
	if (scene == NULL)
		return NULL;

	return scene->rigidbody_world;
}

void BKE_rigidbody_remove_shard_con(Scene *scene, RigidBodyShardCon *con)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	if (rbw && rbw->physics_world && con && con->physics_constraint) {
		RB_dworld_remove_constraint(rbw->physics_world, con->physics_constraint);
		RB_constraint_delete(con->physics_constraint);
		con->physics_constraint = NULL;
	}
}

void BKE_rigidbody_remove_shard(Scene *scene, MeshIsland *mi)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	int i = 0;
	
	/* rbw can be NULL directly after linking / appending objects without their original scenes
	 * if an attempt to refracture is done then, this would crash here with null pointer access */
	if (mi->rigidbody != NULL && rbw != NULL) {
		
		RigidBodyShardCon *con;

		for (i = 0; i < mi->participating_constraint_count; i++) {
			con = mi->participating_constraints[i];
			BKE_rigidbody_remove_shard_con(scene, con);
		}
		
		if (rbw->physics_world && mi->rigidbody && mi->rigidbody->physics_object)
			RB_dworld_remove_body(rbw->physics_world, mi->rigidbody->physics_object);

		if (mi->rigidbody->physics_object) {
			RB_body_delete(mi->rigidbody->physics_object);
			mi->rigidbody->physics_object = NULL;
		}

		if (mi->rigidbody->physics_shape) {
			RB_shape_delete(mi->rigidbody->physics_shape);
			mi->rigidbody->physics_shape = NULL;
		}

		/* this SHOULD be the correct global index, mark with NULL as 'dirty' BEFORE deleting */
		/* need to check whether we didnt create the rigidbody world manually already, prior to fracture, in this
		 * case cache_index_map might be not initialized ! checking numbodies here, they should be 0 in a fresh
		 * rigidbody world */
#if 0
		if ((rbw->cache_index_map != NULL) && (rbw->numbodies > 0) && mi->linear_index < rbw->numbodies) {
			//mi->rigidbody = NULL;
			rbw->cache_index_map[mi->linear_index] = NULL;
		}
#endif

		if (rbw->cache_index_map != NULL) {
			MEM_freeN(rbw->cache_index_map);
			rbw->cache_index_map = NULL;
		}

		//BKE_rigidbody_update_ob_array(rbw);
	}
}

static bool do_remove_modifier(RigidBodyWorld* rbw, ModifierData *md, Object *ob)
{
	RigidBodyShardCon *con;
	MeshIsland *mi;
	FractureModifierData *fmd;
	bool modFound = false;

	if (md->type == eModifierType_Fracture)
	{
		fmd = (FractureModifierData *)md;
		modFound = true;
		GroupObject *go;

		for (con = fmd->meshConstraints.first; con; con = con->next) {
			if (rbw && rbw->physics_world && con->physics_constraint) {
				RB_dworld_remove_constraint(rbw->physics_world, con->physics_constraint);
				RB_constraint_delete(con->physics_constraint);
				con->physics_constraint = NULL;
			}
		}

		/*if we are part of a connected object, delete the parent's constraints here too*/
		for (go = rbw->group->gobject.first; go; go = go->next)
		{
			FractureModifierData *fmdi = (FractureModifierData*)modifiers_findByType(go->ob, eModifierType_Fracture);
			if (fmdi && ob != go->ob)
			{
				for (con = fmdi->meshConstraints.first; con; con = con->next) {
					if (rbw && rbw->physics_world && con->physics_constraint) {
						RB_dworld_remove_constraint(rbw->physics_world, con->physics_constraint);
						RB_constraint_delete(con->physics_constraint);
						con->physics_constraint = NULL;
					}
				}
			}
		}


		for (mi = fmd->meshIslands.first; mi; mi = mi->next) {
			if (mi->rigidbody != NULL) {
				if (rbw->physics_world && mi->rigidbody && mi->rigidbody->physics_object)
					RB_dworld_remove_body(rbw->physics_world, mi->rigidbody->physics_object);
				if (mi->rigidbody->physics_object) {
					RB_body_delete(mi->rigidbody->physics_object);
					mi->rigidbody->physics_object = NULL;
				}

				if (mi->rigidbody->physics_shape) {
					RB_shape_delete(mi->rigidbody->physics_shape);
					mi->rigidbody->physics_shape = NULL;
				}

				/* this SHOULD be the correct global index*/
				/*if ((rbw->cache_index_map != NULL) && (rbw->numbodies > 0))
				{
					rbw->cache_index_map[mi->linear_index] = NULL;
				}*/

				if (rbw->cache_index_map != NULL) {
					MEM_freeN(rbw->cache_index_map);
					rbw->cache_index_map = NULL;
				}

				MEM_freeN(mi->rigidbody);
				mi->rigidbody = NULL;
			}
		}
	}

	return modFound;
}

void BKE_rigidbody_remove_object(Scene *scene, Object *ob)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyOb *rbo = ob->rigidbody_object;
	RigidBodyCon *rbc;
	GroupObject *go;
	ModifierData *md;
	int i;
	bool modFound = false;

	if (rbw) {
		for (md = ob->modifiers.first; md; md = md->next) {
			modFound = do_remove_modifier(rbw, md, ob);
		}

		if (!modFound) {
			/* remove from rigidbody world, free object won't do this */
			if (rbw->physics_world && rbo->physics_object)
				RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);

			/* remove object from array */
			if (rbw && rbw->objects) {
				for (i = 0; i < rbw->numbodies; i++) {
					int index = rbw->cache_offset_map[i];
					if (rbw->objects[index] == ob) {
						rbw->objects[index] = NULL;
					}
					
					/*if (rbo == rbw->cache_index_map[i]) {
						rbw->cache_index_map[i] = NULL;
						break;
					}*/
					if (rbw->cache_index_map != NULL) {
						MEM_freeN(rbw->cache_index_map);
						rbw->cache_index_map = NULL;
					}
				}
			}

			/* remove object from rigid body constraints */
			if (rbw->constraints) {
				for (go = rbw->constraints->gobject.first; go; go = go->next) {
					Object *obt = go->ob;
					if (obt && obt->rigidbody_constraint) {
						rbc = obt->rigidbody_constraint;
						if (rbc->ob1 == ob) {
							rbc->ob1 = NULL;
							rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
						}
						if (rbc->ob2 == ob) {
							rbc->ob2 = NULL;
							rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
						}
					}
				}
			}
			
			/* remove object's settings */
			BKE_rigidbody_free_object(ob);
		}
	}

	/* force removal of object settings even if world may be invalid e.g. after link/append */
	if (!rbw && rbo)
		BKE_rigidbody_free_object(ob);

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);
}

void BKE_rigidbody_remove_constraint(Scene *scene, Object *ob)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyCon *rbc = ob->rigidbody_constraint;

	/* remove from rigidbody world, free object won't do this */
	if (rbw && rbw->physics_world && rbc->physics_constraint) {
		RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
	}
	/* remove object's settings */
	BKE_rigidbody_free_constraint(ob);

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);
}

static int rigidbody_group_count_items(const ListBase *group, int *r_num_objects, int *r_num_shards)
{
	int num_gobjects = 0;
	ModifierData *md;
	FractureModifierData *rmd;
	GroupObject *gob;

	if (r_num_objects == NULL || r_num_shards == NULL)
	{
		return num_gobjects;
	}

	*r_num_objects = 0;
	*r_num_shards = 0;

	for (gob = group->first; gob; gob = gob->next) {
		bool found_modifiers = false;
		for (md = gob->ob->modifiers.first; md; md = md->next) {
			if (md->type == eModifierType_Fracture) {
				rmd = (FractureModifierData *)md;
				if (isModifierActive(rmd))
				{
					found_modifiers = true;
					if (rmd->meshIslands.first != NULL)
					{
						*r_num_shards += BLI_listbase_count(&rmd->meshIslands);
					}
				}
			}
		}
		if (found_modifiers == false && gob->ob->rigidbody_object) {
			(*r_num_objects)++;
		}
		num_gobjects++;
	}

	return num_gobjects;
}

static int object_sort_eval(const void *s1, const void *s2, void* context)
{
	Object **o1 = (Object**)s1;
	Object **o2 = (Object**)s2;

	FractureModifierData *fmd1 = (FractureModifierData*)modifiers_findByType(*o1, eModifierType_Fracture);
	FractureModifierData *fmd2 = (FractureModifierData*)modifiers_findByType(*o2, eModifierType_Fracture);


	if ((fmd1 && fmd1->dm_group && fmd1->use_constraint_group) &&
	   (fmd2 && fmd2->dm_group && fmd2->use_constraint_group))
	{
		return 0;
	}

	if ((fmd1 && fmd1->dm_group && fmd1->use_constraint_group) &&
	   (fmd2 && !(fmd2->dm_group && fmd2->use_constraint_group)))
	{
		return 1;
	}

	if ((fmd1 && fmd1->dm_group && fmd1->use_constraint_group) && !fmd2)
	{
		return 1;
	}

	if ((fmd1 && !(fmd1->dm_group && fmd1->use_constraint_group)) &&
	   (fmd2 && (fmd2->dm_group && fmd2->use_constraint_group)))
	{
		return -1;
	}

	if ((!fmd1) && (fmd2 && fmd2->dm_group && fmd2->use_constraint_group))
	{
		return -1;
	}

	return 0;
}

/* ************************************** */
/* Simulation Interface - Bullet */

/* Update object array and rigid body count so they're in sync with the rigid body group */
void BKE_rigidbody_update_ob_array(RigidBodyWorld *rbw, bool do_bake_correction)
{
	GroupObject *go;
	FractureModifierData *rmd;
	MeshIsland *mi;
	int i, j = 0, l = 0, m = 0, n = 0, counter = 0;
	bool ismapped = false;
	RigidBodyOb ** tmp_index = NULL;
	int *tmp_offset = NULL;
	
	if (rbw->objects != NULL) {
		MEM_freeN(rbw->objects);
		rbw->objects = NULL;
	}
	
	if (rbw->cache_index_map != NULL) {
		MEM_freeN(rbw->cache_index_map);
		rbw->cache_index_map = NULL;
	}
	
	if (rbw->cache_offset_map != NULL) {
		MEM_freeN(rbw->cache_offset_map);
		rbw->cache_offset_map = NULL;
	}

	l = rigidbody_group_count_items(&rbw->group->gobject, &m, &n);

	rbw->numbodies = m + n;
	rbw->objects = MEM_mallocN(sizeof(Object *) * l, "objects");
	rbw->cache_index_map = MEM_mallocN(sizeof(RigidBodyOb *) * rbw->numbodies, "cache_index_map");
	rbw->cache_offset_map = MEM_mallocN(sizeof(int) * rbw->numbodies, "cache_offset_map");
	tmp_index =  MEM_mallocN(sizeof(RigidBodyOb *) * rbw->numbodies, "cache_index_map temp");
	tmp_offset = MEM_mallocN(sizeof(int) * rbw->numbodies, "cache_offset_map");

	printf("RigidbodyCount changed: %d\n", rbw->numbodies);

	//pre-sort rbw->objects... put such with fmd->dm_group and fmd->use_constraint_group after all without,
	//to enforce proper eval order

	for (go = rbw->group->gobject.first, i = 0; go; go = go->next, i++) {

		if (go->ob->rigidbody_object)
		{
			rbw->objects[i] = go->ob;
		}
	}

	BLI_qsort_r(rbw->objects, l, sizeof(Object *), object_sort_eval, NULL);

	//correct map if baked, it might be shifted
	for (i = 0; i < l; i++) {
		Object *ob = rbw->objects[i];
		//printf("%s\n", ob->id.name + 2);

		rmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);
		if (rmd) {
			for (mi = rmd->meshIslands.first, j = 0; mi; mi = mi->next) {
				//store original position of the object in the object array, to be able to rearrange it later so it matches the baked cache
				tmp_index[counter] = mi->rigidbody; /* map all shards of an object to this object index*/
				tmp_offset[counter] = i;
				mi->linear_index = counter;
				if (mi->rigidbody && !do_bake_correction) {
					//as we search by id now in the pointcache, we set the id here too
					mi->rigidbody->meshisland_index = counter;
					if (mi->object_index != -1)
					{
						mi->object_index = i;
					}
				}
				counter++;
				j++;
			}
			ismapped = true;
		}

		if (!ismapped && ob->rigidbody_object) {
			tmp_index[counter] = ob->rigidbody_object; /*1 object 1 index here (normal case)*/
			tmp_offset[counter] = i;
			if (ob->rigidbody_object && !do_bake_correction)
				ob->rigidbody_object->meshisland_index = counter;
			counter++;
		}

		ismapped = false;
	}

	//do shuffle for each rigidbody
	for (i = 0; i < rbw->numbodies; i++)
	{
		RigidBodyOb *rbo = tmp_index[i];
		int offset = tmp_offset[i];

		if (rbo->meshisland_index != i && do_bake_correction)
		{
			rbw->cache_index_map[rbo->meshisland_index] = rbo;
			rbw->cache_offset_map[rbo->meshisland_index] = offset;
			//printf("Shuffle %d -> %d\n", i, rbo->meshisland_index);
		}
		else {
			rbw->cache_index_map[i] = rbo;
			rbw->cache_offset_map[i] = offset;
		}
	}

	MEM_freeN(tmp_index);
	MEM_freeN(tmp_offset);
}

static void rigidbody_update_sim_world(Scene *scene, RigidBodyWorld *rbw, bool rebuild)
{
	float adj_gravity[3];
	bool skip_correction = rbw->flag & RBW_FLAG_REFRESH_MODIFIERS;

	/* adjust gravity to take effector weights into account */
	if (scene->physics_settings.flag & PHYS_GLOBAL_GRAVITY) {
		copy_v3_v3(adj_gravity, scene->physics_settings.gravity);
		mul_v3_fl(adj_gravity, rbw->effector_weights->global_gravity * rbw->effector_weights->weight[0]);
	}
	else {
		zero_v3(adj_gravity);
	}

	/* update gravity, since this RNA setting is not part of RigidBody settings */
	RB_dworld_set_gravity(rbw->physics_world, adj_gravity);

	/* update object array in case there are changes */
	if (rebuild)
	{
		BKE_rigidbody_update_ob_array(rbw, (rbw->pointcache->flag & PTCACHE_BAKED) && !skip_correction);
	}
}

static void rigidbody_update_sim_ob(Scene *scene, RigidBodyWorld *rbw, Object *ob, RigidBodyOb *rbo, float centroid[3],
                                    MeshIsland *mi, float size[3], FractureModifierData *fmd)
{
	float loc[3];
	float rot[4];
	float scale[3], centr[3];

	/* only update if rigid body exists */
	if (rbo->physics_object == NULL)
		return;

	if (rbo->shape == RB_SHAPE_TRIMESH && rbo->flag & RBO_FLAG_USE_DEFORM) {
		DerivedMesh *dm = NULL;

		if (rbo->mesh_source == RBO_MESH_DEFORM) {
			dm = ob->derivedDeform;
		}
		else if (rbo->mesh_source == RBO_MESH_FINAL) {
			dm = ob->derivedFinal;
		}
		else if (rbo->mesh_source == RBO_MESH_FINAL_SOLID)
		{
			dm = dm_solidify(ob->derivedFinal, rbo->margin);
		}

		if (dm) {
			MVert *mvert = dm->getVertArray(dm);
			int totvert = dm->getNumVerts(dm);
			BoundBox *bb = BKE_object_boundbox_get(ob);

			if (RB_shape_get_num_verts(rbo->physics_shape) != totvert)
			{
				if (mi != NULL)
				{
#if 0
					if (mi->rigidbody->type == RBO_TYPE_PASSIVE)
					{
						MVert *mv = mvert + mi->vertex_indices[0];
						rigidbody_passive_fake_hook(mi, mv);
					}
					else
#endif
					{
						//fracture modifier case TODO, update mi->physicsmesh somehow and redraw
						rbo->flag |= RBO_FLAG_NEEDS_RESHAPE;
						validateShard(rbw, mi, ob, false, false, size);
					}
				}
				else
				{
					//regular rigidbody case
					rigidbody_validate_sim_shape(ob, true);
					RB_body_set_collision_shape(rbo->physics_object, rbo->physics_shape);
				}
			}
			else
			{
				RB_shape_trimesh_update(rbo->physics_shape, (float *)mvert, totvert, sizeof(MVert), bb->vec[0], bb->vec[6]);
			}

			if (rbo->mesh_source == RBO_MESH_FINAL_SOLID)
			{
				dm->release(dm);
			}
		}
	}

	copy_v3_v3(centr, centroid);
	mat4_decompose(loc, rot, scale, ob->obmat);
	mul_v3_v3(scale, size);

	/* update scale for all objects */
	RB_body_set_scale(rbo->physics_object, scale);
	/* compensate for embedded convex hull collision margin */
	if (!(rbo->flag & RBO_FLAG_USE_MARGIN) && rbo->shape == RB_SHAPE_CONVEXH)
		RB_shape_set_margin(rbo->physics_shape, RBO_GET_MARGIN(rbo) * MIN3(scale[0], scale[1], scale[2]));

	/* make transformed objects temporarily kinmatic so that they can be moved by the user during simulation */
	if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) {
		RB_body_set_kinematic_state(rbo->physics_object, true);
		RB_body_set_mass(rbo->physics_object, 0.0f);
	}

	/* update rigid body location and rotation for kinematic bodies */
	if ((rbo->flag & RBO_FLAG_KINEMATIC && rbo->force_thresh == 0.0f) || (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)) {
		if (((rbo->type == RBO_TYPE_ACTIVE || mi == NULL) && (rbo->flag & RBO_FLAG_KINEMATIC_REBUILD) == 0))
		{
			if (!(fmd && fmd->anim_mesh_ob && fmd->use_animated_mesh))
			{
				//override for externally animated rbs
				mul_v3_v3(centr, scale);
				mul_qt_v3(rot, centr);
				add_v3_v3(loc, centr);
				RB_body_activate(rbo->physics_object);
				RB_body_set_loc_rot(rbo->physics_object, loc, rot);
			}
		}
	}
	/* update influence of effectors - but don't do it on an effector (why not ?)*/
	/* only dynamic bodies need effector update */
	else if (rbo->type == RBO_TYPE_ACTIVE /* && ((ob->pd == NULL) || (ob->pd->forcefield == PFIELD_NULL))*/) {
		EffectorWeights *effector_weights = rbw->effector_weights;
		EffectedPoint epoint = {0};
		ListBase *effectors;

		/* get effectors present in the group specified by effector_weights */
		effectors = pdInitEffectors(scene, ob, NULL, effector_weights, true);
		if (effectors) {
			float eff_force[3] = {0.0f, 0.0f, 0.0f};
			float eff_loc[3], eff_vel[3];
			float thresh = rbo->force_thresh * rbo->force_thresh; /*use this to compare against squared length of vector */

			/* create dummy 'point' which represents last known position of object as result of sim */
			// XXX: this can create some inaccuracies with sim position, but is probably better than using unsimulated vals?
			RB_body_get_position(rbo->physics_object, eff_loc);
			//mul_v3_v3(centr, scale);
			//add_v3_v3(eff_loc, centr);

			RB_body_get_linear_velocity(rbo->physics_object, eff_vel);

			pd_point_from_loc(scene, eff_loc, eff_vel, 0, &epoint);

			/* calculate net force of effectors, and apply to sim object
			 *	- we use 'central force' since apply force requires a "relative position" which we don't have...
			 */
			pdDoEffectors(effectors, NULL, effector_weights, &epoint, eff_force, NULL);
			if ((rbo->flag & RBO_FLAG_KINEMATIC) && (thresh < len_squared_v3(eff_force)))
			{
				activateRigidbody(rbo, rbw, mi, ob);
				RB_body_apply_central_force(rbo->physics_object, eff_force);
			}
			else if (rbo->flag & RBO_FLAG_KINEMATIC)
			{
				if ((rbo->flag & RBO_FLAG_KINEMATIC_REBUILD) == 0)
				{   //XXXXX TODO, maybe this is wrong here
					/* do the same here as above, but here we needed the eff_force value to compare against threshold */

					if (!(fmd && fmd->anim_mesh_ob && fmd->use_animated_mesh))
					{	//same override as above
						mul_v3_v3(centr, scale);
						mul_qt_v3(rot, centr);
						add_v3_v3(loc, centr);
						RB_body_activate(rbo->physics_object);
						RB_body_set_loc_rot(rbo->physics_object, loc, rot);
					}
				}
			}
			else
			{
				if (G.f & G_DEBUG)
					printf("\tapplying force (%f,%f,%f) to '%s'\n", eff_force[0], eff_force[1], eff_force[2], ob->id.name + 2);
				/* activate object in case it is deactivated */
				if (!is_zero_v3(eff_force))
					RB_body_activate(rbo->physics_object);
				RB_body_apply_central_force(rbo->physics_object, eff_force);
			}
		}
		else if (G.f & G_DEBUG)
			printf("\tno forces to apply to '%s'\n", ob->id.name + 2);

		/* cleanup */
		pdEndEffectors(&effectors);
	}
	/* NOTE: passive objects don't need to be updated since they don't move */

	/* NOTE: no other settings need to be explicitly updated here,
	 * since RNA setters take care of the rest :)
	 */

	// per-object constant linear force //
	if (rbo->type == RBO_TYPE_ACTIVE){
		RB_body_apply_central_force(rbo->physics_object, rbo->apply_force);
	}
}

/* Updates and validates world, bodies and shapes.
 * < rebuild: rebuild entire simulation
 */
static void rigidbody_update_simulation(Scene *scene, RigidBodyWorld *rbw, bool rebuild)
{
	GroupObject *go;
	bool did_modifier = false;
	float centroid[3] = {0, 0, 0};
	float size[3] = {1.0f, 1.0f, 1.0f};
	float count = BLI_listbase_count(&rbw->group->gobject);
	int i = 0;

	/* update world */
	if (rebuild) {
		BKE_rigidbody_validate_sim_world(scene, rbw, true);
	}

	rigidbody_update_sim_world(scene, rbw, rebuild);

	/* update objects */
	for (i = 0; i < count; i++) {
		Object *ob = rbw->objects[i];

		if (ob && (ob->type == OB_MESH || ob->type == OB_CURVE || ob->type == OB_SURF || ob->type == OB_FONT)) {
			did_modifier = do_update_modifier(scene, ob, rbw, rebuild);
		}

		if (!did_modifier && ob->type == OB_MESH) {
			/* validate that we've got valid object set up here... */
			RigidBodyOb *rbo = ob->rigidbody_object;
			/* update transformation matrix of the object so we don't get a frame of lag for simple animations */
			BKE_object_where_is_calc(scene, ob);

			if (rbw->flag & RBW_FLAG_REBUILD_CONSTRAINTS && rbo)
			{
				//reset speeds
				//printf("ZEROIZING speed (object)\n");
				zero_v3(rbo->lin_vel);
				zero_v3(rbo->ang_vel);
			}

			if (rbo == NULL) {
				/* Since this object is included in the sim group but doesn't have
				 * rigid body settings (perhaps it was added manually), add!
				 *	- assume object to be active? That is the default for newly added settings...
				 */
				ob->rigidbody_object = BKE_rigidbody_create_object(scene, ob, RBO_TYPE_ACTIVE, NULL);
				rigidbody_validate_sim_object(rbw, ob, true, true);

				rbo = ob->rigidbody_object;
			}
			else {
				/* perform simulation data updates as tagged */
				/* refresh object... */
				if (rebuild) {
					/* World has been rebuilt so rebuild object */
					rigidbody_validate_sim_object(rbw, ob, true, true);
				}
				else if (rbo->flag & RBO_FLAG_NEEDS_VALIDATE) {
					rigidbody_validate_sim_object(rbw, ob, false, true);
				}
				/* refresh shape... */
				if (rbo->flag & RBO_FLAG_NEEDS_RESHAPE) {
					/* mesh/shape data changed, so force shape refresh */
					rigidbody_validate_sim_shape(ob, true);
					/* now tell RB sim about it */
					// XXX: we assume that this can only get applied for active/passive shapes that will be included as rigidbodies
					RB_body_set_collision_shape(rbo->physics_object, rbo->physics_shape);
				}
				rbo->flag &= ~(RBO_FLAG_NEEDS_VALIDATE | RBO_FLAG_NEEDS_RESHAPE);
			}

			/* update simulation object... */
			rigidbody_update_sim_ob(scene, rbw, ob, rbo, centroid, NULL, size, NULL);
		}
	}

	if (rbw->physics_world && rbw->flag & RBW_FLAG_REBUILD_CONSTRAINTS)
	{
		double start = PIL_check_seconds_timer();
		RB_dworld_init_compounds(rbw->physics_world);
		printf("Building compounds done, %g\n", PIL_check_seconds_timer() - start);
	}

	rbw->flag &= ~RBW_FLAG_REFRESH_MODIFIERS;

	/* update constraints */
	if (rbw->constraints == NULL) /* no constraints, move on */
		return;
	for (go = rbw->constraints->gobject.first; go; go = go->next) {
		Object *ob = go->ob;

		if (ob) {
			/* validate that we've got valid object set up here... */
			RigidBodyCon *rbc = ob->rigidbody_constraint;
			/* update transformation matrix of the object so we don't get a frame of lag for simple animations */
			BKE_object_where_is_calc(scene, ob);

			if (rbc == NULL) {
				/* Since this object is included in the group but doesn't have
				 * constraint settings (perhaps it was added manually), add!
				 */
				ob->rigidbody_constraint = BKE_rigidbody_create_constraint(scene, ob, RBC_TYPE_FIXED, NULL);
				rigidbody_validate_sim_constraint(rbw, ob, true);

				rbc = ob->rigidbody_constraint;
			}
			else {
				/* perform simulation data updates as tagged */
				if (rebuild) {
					/* World has been rebuilt so rebuild constraint */
					rigidbody_validate_sim_constraint(rbw, ob, true);
				}
				else if (rbc->flag & RBC_FLAG_NEEDS_VALIDATE) {
					rigidbody_validate_sim_constraint(rbw, ob, false);
				}
				rbc->flag &= ~RBC_FLAG_NEEDS_VALIDATE;
			}

			if (rbc->physics_constraint)
			{
				RigidBodyOb *rbo1 = rbc->ob1->rigidbody_object;
				RigidBodyOb *rbo2 = rbc->ob2->rigidbody_object;

				if ((rbo1->force_thresh > 0 || rbo2->force_thresh > 0))
				{
					if (RB_constraint_get_applied_impulse(rbc->physics_constraint) >= rbo1->force_thresh + rbo2->force_thresh)
					{
						RB_constraint_set_enabled(rbc->physics_constraint, false);
						activateRigidbody(rbo1, rbw, NULL, rbc->ob1);
						activateRigidbody(rbo2, rbw, NULL, rbc->ob2);
					}
				}
			}
		}
	}
}

static ThreadMutex post_step_lock = BLI_MUTEX_INITIALIZER;
static void rigidbody_update_simulation_post_step(RigidBodyWorld *rbw)
{
	GroupObject *go;
	ModifierData *md;
	FractureModifierData *rmd;
	int modFound = false;
	RigidBodyOb *rbo;
	MeshIsland *mi;

	for (go = rbw->group->gobject.first; go; go = go->next) {

		Object *ob = go->ob;
		//handle fractured rigidbodies, maybe test for psys as well ?
		BLI_mutex_lock(&post_step_lock);
		for (md = ob->modifiers.first; md; md = md->next) {
			if (md->type == eModifierType_Fracture) {
				rmd = (FractureModifierData *)md;
				if (isModifierActive(rmd)) {
					for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
						rbo = mi->rigidbody;
						if (!rbo) continue;
						/* reset kinematic state for transformed objects */
						if (rbo->physics_object && ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) {
							RB_body_set_kinematic_state(rbo->physics_object, rbo->flag & RBO_FLAG_KINEMATIC || rbo->flag & RBO_FLAG_DISABLED);
							RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
							/* deactivate passive objects so they don't interfere with deactivation of active objects */
							if (rbo->type == RBO_TYPE_PASSIVE)
								RB_body_deactivate(rbo->physics_object);
						}

						/* update stored velocities, can be set again after sim rebuild */
						if (rmd->fracture_mode == MOD_FRACTURE_DYNAMIC && rbo->physics_object)
						{
							if (!(rbo->flag & RBO_FLAG_KINEMATIC))
							{
								RB_body_get_linear_velocity(rbo->physics_object, rbo->lin_vel);
								RB_body_get_angular_velocity(rbo->physics_object, rbo->ang_vel);
							}
						}
					}
					modFound = true;
					break;
				}
			}
		}
		BLI_mutex_unlock(&post_step_lock);

		/* handle regular rigidbodies */
		if (ob && ob->rigidbody_object && !modFound) {
			rbo = ob->rigidbody_object;
			/* reset kinematic state for transformed objects */
			if (rbo && (ob->flag & SELECT) && (G.moving & G_TRANSFORM_OBJ)) {
				RB_body_set_kinematic_state(rbo->physics_object, rbo->flag & RBO_FLAG_KINEMATIC || rbo->flag & RBO_FLAG_DISABLED);
				RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
				/* deactivate passive objects so they don't interfere with deactivation of active objects */
				if (rbo->type == RBO_TYPE_PASSIVE)
					RB_body_deactivate(rbo->physics_object);
			}

			if (!(rbo->flag & RBO_FLAG_KINEMATIC))
			{
				RB_body_get_linear_velocity(rbo->physics_object, rbo->lin_vel);
				RB_body_get_angular_velocity(rbo->physics_object, rbo->ang_vel);
			}
		}
		modFound = false;
	}
}

bool BKE_rigidbody_check_sim_running(RigidBodyWorld *rbw, float ctime)
{
	return (rbw && (rbw->flag & RBW_FLAG_MUTED) == 0 && ctime > rbw->pointcache->startframe);
}

/* Sync rigid body and object transformations */
static ThreadMutex modifier_lock = BLI_MUTEX_INITIALIZER;
void BKE_rigidbody_sync_transforms(RigidBodyWorld *rbw, Object *ob, float ctime)
{
	RigidBodyOb *rbo = NULL;
	ModifierData *md;
	int modFound = false;

	if (rbw == NULL)
		return;

	BLI_mutex_lock(&modifier_lock);
	for (md = ob->modifiers.first; md; md = md->next) {
		modFound = do_sync_modifier(md, ob, rbw, ctime);
		if (modFound)
			break;
	}
	BLI_mutex_unlock(&modifier_lock);

	if (!modFound)
	{
		rbo = ob->rigidbody_object;

		/* keep original transform for kinematic and passive objects */
		if (ELEM(NULL, rbw, rbo) || (((rbo->flag & RBO_FLAG_KINEMATIC) && ((rbo->flag & RBO_FLAG_KINEMATIC_REBUILD) == 0)) || rbo->type == RBO_TYPE_PASSIVE))
			return;

		/* use rigid body transform after cache start frame if objects is not being transformed */
		if (BKE_rigidbody_check_sim_running(rbw, ctime) && !(ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)) {
			float mat[4][4], size_mat[4][4], size[3];

			normalize_qt(rbo->orn); // RB_TODO investigate why quaternion isn't normalized at this point
			quat_to_mat4(mat, rbo->orn);
			copy_v3_v3(mat[3], rbo->pos);

			/* keep original transform when the simulation is muted */
			if (rbw->flag & RBW_FLAG_MUTED)
				return;

			mat4_to_size(size, ob->obmat);
			size_to_mat4(size_mat, size);
			mul_m4_m4m4(mat, mat, size_mat);

			copy_m4_m4(ob->obmat, mat);
		}
		/* otherwise set rigid body transform to current obmat */
		else {
			if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)
				rbw->flag |= RBW_FLAG_OBJECT_CHANGED;

			mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
		}
	}
}

static void do_reset_rigidbody(RigidBodyOb *rbo, Object *ob, MeshIsland* mi, float loc[3],
                              float rot[3], float quat[4], float rotAxis[3], float rotAngle)
{
	bool correct_delta = !(rbo->flag & RBO_FLAG_KINEMATIC || rbo->type == RBO_TYPE_PASSIVE);

	/* return rigid body and object to their initial states */
	copy_v3_v3(rbo->pos, ob->loc);
	if (mi)
		add_v3_v3(rbo->pos, mi->centroid);
	copy_v3_v3(ob->loc, loc);

	if (correct_delta) {
		add_v3_v3(rbo->pos, ob->dloc);
	}

	if (ob->rotmode > 0) {
		float qt[4];
		eulO_to_quat(qt, ob->rot, ob->rotmode);

		if (correct_delta) {
			float dquat[4];
			eulO_to_quat(dquat, ob->drot, ob->rotmode);

			mul_qt_qtqt(rbo->orn, dquat, qt);
		}
		else {
			copy_qt_qt(rbo->orn, qt);
		}

		if (mi)
			mul_qt_qtqt(rbo->orn, qt, mi->rot);

		copy_v3_v3(ob->rot, rot);
	}
	else if (ob->rotmode == ROT_MODE_AXISANGLE) {
		float qt[4];
		axis_angle_to_quat(qt, ob->rotAxis, ob->rotAngle);

		if (correct_delta) {
			float dquat[4];
			axis_angle_to_quat(dquat, ob->drotAxis, ob->drotAngle);

			mul_qt_qtqt(rbo->orn, dquat, qt);
		}
		else {
			copy_qt_qt(rbo->orn, qt);
		}

		if (mi)
			mul_qt_qtqt(rbo->orn, qt, mi->rot);

		copy_v3_v3(ob->rotAxis, rotAxis);
		ob->rotAngle = rotAngle;
	}
	else {
		if (correct_delta) {
			mul_qt_qtqt(rbo->orn, ob->dquat, ob->quat);
		}
		else {
			copy_qt_qt(rbo->orn, ob->quat);
		}

		if (mi)
			mul_qt_qtqt(rbo->orn, rbo->orn, mi->rot);

		copy_qt_qt(ob->quat, quat);
	}

	if (rbo->physics_object) {
		/* allow passive objects to return to original transform */
		if (rbo->type == RBO_TYPE_PASSIVE)
			RB_body_set_kinematic_state(rbo->physics_object, true);
		RB_body_set_loc_rot(rbo->physics_object, rbo->pos, rbo->orn);
	}
}

/* Used when cancelling transforms - return rigidbody and object to initial states */
void BKE_rigidbody_aftertrans_update(Object *ob, float loc[3], float rot[3], float quat[4], float rotAxis[3], float rotAngle)
{
	RigidBodyOb *rbo;
	ModifierData *md;
	FractureModifierData *rmd;
	float imat[4][4];
	
	md = modifiers_findByType(ob, eModifierType_Fracture);
	if (md != NULL)
	{
		MeshIsland *mi;
		rmd = (FractureModifierData *)md;
		invert_m4_m4(imat, ob->obmat);
		for (mi = rmd->meshIslands.first; mi; mi = mi->next)
		{
			rbo = mi->rigidbody;
			do_reset_rigidbody(rbo, ob, mi, loc, rot, quat, rotAxis, rotAngle);

			if (rbo->flag & RBO_FLAG_KINEMATIC)
			{
				rigidbody_passive_fake_parenting(rmd, ob, rbo, imat);
			}
			else
			{
				rigidbody_passive_hook(rmd, mi, ob);
			}
		}

		if (rmd->use_animated_mesh && rmd->anim_mesh_ob)
		{
			BKE_read_animated_loc_rot(rmd, ob, false);
		}

		//then update origmat
		copy_m4_m4(rmd->origmat, ob->obmat);
	}
	else {
		rbo = ob->rigidbody_object;
		do_reset_rigidbody(rbo, ob, NULL, loc, rot, quat, rotAxis, rotAngle);

		// RB_TODO update rigid body physics object's loc/rot for dynamic objects here as well (needs to be done outside bullet's update loop)
	}

	// RB_TODO update rigid body physics object's loc/rot for dynamic objects here as well (needs to be done outside bullet's update loop)
}

void BKE_rigidbody_cache_reset(RigidBodyWorld *rbw)
{
	if (rbw) {
		rbw->pointcache->flag |= PTCACHE_OUTDATED;
		//restoreKinematic(rbw);
		//if (!(rbw->pointcache->flag & PTCACHE_BAKED))
		resetDynamic(rbw, false, false);
		//resetPrefractured(rbw);
	}
}

/* ------------------ */

/* Rebuild rigid body world */
/* NOTE: this needs to be called before frame update to work correctly */
void BKE_rigidbody_rebuild_world(Scene *scene, float ctime, bool do_resetDynamic)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	PointCache *cache;
	PTCacheID pid;
	int startframe, endframe;
	int shards = 0, objects = 0;

	if (ctime == -1)
	{
		if (rbw && rbw->group)
		{
			/*hack to be able to update the simulation data after loading from FM*/
			rigidbody_update_simulation(scene, rbw, true);
		}
		return;
	}

	BKE_ptcache_id_from_rigidbody(&pid, NULL, rbw);
	BKE_ptcache_id_time(&pid, scene, ctime, &startframe, &endframe, NULL);
	cache = rbw->pointcache;

	/* flag cache as outdated if we don't have a world or number of objects in the simulation has changed */
	rigidbody_group_count_items(&rbw->group->gobject, &shards, &objects);
	if (rbw->physics_world == NULL /*|| rbw->numbodies != (shards + objects)*/) {
		cache->flag |= PTCACHE_OUTDATED;
	}

	if (ctime == startframe + 1 && rbw->ltime == startframe) {
		if (cache->flag & PTCACHE_OUTDATED) {

			//if we destroy the cache, also reset dynamic data (if not baked)
			if (!(cache->flag & PTCACHE_BAKED))
			{
				resetDynamic(rbw, true, do_resetDynamic);
				//resetExternal(rbw);
				//resetPrefractured(rbw);
			}

			BKE_ptcache_id_reset(scene, &pid, PTCACHE_RESET_OUTDATED);
			rigidbody_update_simulation(scene, rbw, true);
			BKE_ptcache_validate(cache, (int)ctime);
			cache->last_exact = 0;
			cache->flag &= ~PTCACHE_REDO_NEEDED;
		}
	}
}

/* Run RigidBody simulation for the specified physics world */
void BKE_rigidbody_do_simulation(Scene *scene, float ctime)
{
	float timestep;
	RigidBodyWorld *rbw = scene->rigidbody_world;
	PointCache *cache;
	PTCacheID pid;
	int startframe, endframe;
	bool was_changed = false;

	BKE_ptcache_id_from_rigidbody(&pid, NULL, rbw);
	BKE_ptcache_id_time(&pid, scene, ctime, &startframe, &endframe, NULL);
	cache = rbw->pointcache;

	/*trigger dynamic update*/
	if (rbw->flag & RBW_FLAG_OBJECT_CHANGED)
	{
		rbw->flag &= ~RBW_FLAG_OBJECT_CHANGED;
		was_changed = rbw->flag & RBW_FLAG_REFRESH_MODIFIERS;
		rbw->flag &= ~RBW_FLAG_REFRESH_MODIFIERS;
		//if (!(cache->flag & PTCACHE_BAKED))
		{
			bool baked = cache->flag & PTCACHE_BAKED;
			bool from_cache = cache->last_exact > cache->simframe;
			if ((from_cache || baked) && ctime > startframe + 1)
			{
				/* dont mess with baked data */
				rigidbody_update_simulation(scene, rbw, true);
			}
		}
		//rbw->flag &= ~RBW_FLAG_REFRESH_MODIFIERS;
	}

	if (ctime <= startframe) {
		/* rebuild constraints */
		rbw->flag |= RBW_FLAG_REBUILD_CONSTRAINTS;

		rbw->ltime = startframe;
		if (rbw->flag & RBW_FLAG_OBJECT_CHANGED)
		{       /* flag modifier refresh at their next execution XXX TODO -> still used ? */
			rbw->flag |= RBW_FLAG_REFRESH_MODIFIERS;
			rbw->flag &= ~RBW_FLAG_OBJECT_CHANGED;
			rigidbody_update_simulation(scene, rbw, true);
		}
		return;
	}
	/* make sure we don't go out of cache frame range */
	else if (ctime > endframe) {
		ctime = endframe;
	}

	/* don't try to run the simulation if we don't have a world yet but allow reading baked cache */
	if (rbw->physics_world == NULL && !(cache->flag & PTCACHE_BAKED))
		return;
	else if ((rbw->objects == NULL) || (rbw->cache_index_map == NULL))
	{
		if (!was_changed)
		{
			BKE_rigidbody_cache_reset(rbw);
			BKE_ptcache_id_reset(scene, &pid, PTCACHE_RESET_OUTDATED);
		}

		BKE_rigidbody_update_ob_array(rbw, cache->flag & PTCACHE_BAKED);
	}

	/* try to read from cache */
	// RB_TODO deal with interpolated, old and baked results
	bool can_simulate = (ctime == rbw->ltime + 1) && !(cache->flag & PTCACHE_BAKED);

	if (BKE_ptcache_read(&pid, ctime, can_simulate) == PTCACHE_READ_EXACT) {
		BKE_ptcache_validate(cache, (int)ctime);

		rbw->ltime = ctime;

		return;
	}
	else if (rbw->ltime == startframe)
	{
		/*bool did_it = */BKE_restoreKinematic(rbw, false);
		//if (did_it)

		//make 1st run like later runs... hack...
		rigidbody_update_simulation(scene, rbw, true);
	}

	/* advance simulation, we can only step one frame forward */
	if (can_simulate) {
		/* write cache for first frame when on second frame */
		if (rbw->ltime == startframe && (cache->flag & PTCACHE_OUTDATED || cache->last_exact == 0)) {
			BKE_ptcache_write(&pid, startframe);
		}

		if (ctime >= startframe) {
			rbw->flag &= ~RBW_FLAG_REBUILD_CONSTRAINTS;
		}

		/* update and validate simulation */
		rigidbody_update_simulation(scene, rbw, false);

		/* calculate how much time elapsed since last step in seconds */
		timestep = 1.0f / (float)FPS * (ctime - rbw->ltime) * rbw->time_scale;
		/* step simulation by the requested timestep, steps per second are adjusted to take time scale into account */
		RB_dworld_step_simulation(rbw->physics_world, timestep, INT_MAX, 1.0f / (float)rbw->steps_per_second * min_ff(rbw->time_scale, 1.0f));

		rigidbody_update_simulation_post_step(rbw);

		/* write cache for current frame */
		BKE_ptcache_validate(cache, (int)ctime);
		BKE_ptcache_write(&pid, (unsigned int)ctime);

		rbw->ltime = ctime;
	}
}
/* ************************************** */

#else  /* WITH_BULLET */

/* stubs */
#ifdef __GNUC__
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

struct RigidBodyOb *BKE_rigidbody_copy_object(const Object *ob) { return NULL; }
struct RigidBodyCon *BKE_rigidbody_copy_constraint(const Object *ob) { return NULL; }
void BKE_rigidbody_validate_sim_world(Scene *scene, RigidBodyWorld *rbw, bool rebuild) {}
void BKE_rigidbody_calc_volume(Object *ob, float *r_vol) { if (r_vol) *r_vol = 0.0f; }
void BKE_rigidbody_calc_center_of_mass(Object *ob, float r_center[3]) { zero_v3(r_center); }
struct RigidBodyWorld *BKE_rigidbody_create_world(Scene *scene) { return NULL; }
struct RigidBodyWorld *BKE_rigidbody_world_copy(RigidBodyWorld *rbw) { return NULL; }
void BKE_rigidbody_world_groups_relink(struct RigidBodyWorld *rbw) {}
void BKE_rigidbody_world_id_loop(struct RigidBodyWorld *rbw, RigidbodyWorldIDFunc func, void *userdata) {}
struct RigidBodyOb *BKE_rigidbody_create_object(Scene *scene, Object *ob, short type) { return NULL; }
struct RigidBodyCon *BKE_rigidbody_create_constraint(Scene *scene, Object *ob, short type) { return NULL; }
struct RigidBodyWorld *BKE_rigidbody_get_world(Scene *scene) { return NULL; }
void BKE_rigidbody_remove_object(Scene *scene, Object *ob) {}
void BKE_rigidbody_remove_constraint(Scene *scene, Object *ob) {}
void BKE_rigidbody_sync_transforms(RigidBodyWorld *rbw, Object *ob, float ctime) {}
void BKE_rigidbody_aftertrans_update(Object *ob, float loc[3], float rot[3], float quat[4], float rotAxis[3], float rotAngle) {}
bool BKE_rigidbody_check_sim_running(RigidBodyWorld *rbw, float ctime) { return false; }
void BKE_rigidbody_cache_reset(RigidBodyWorld *rbw) {}
void BKE_rigidbody_rebuild_world(Scene *scene, float ctime, bool do_resetDynamic) {}
void BKE_rigidbody_do_simulation(Scene *scene, float ctime) {}

#ifdef __GNUC__
#  pragma GCC diagnostic pop
#endif

#endif  /* WITH_BULLET */

/* -------------------- */
/* Depsgraph evaluation */

void BKE_rigidbody_rebuild_sim(EvaluationContext *UNUSED(eval_ctx),
                               Scene *scene)
{
	float ctime = BKE_scene_frame_get(scene);

	if (G.debug & G_DEBUG_DEPSGRAPH) {
		printf("%s at %f\n", __func__, ctime);
	}

	/* rebuild sim data (i.e. after resetting to start of timeline) */
	if (BKE_scene_check_rigidbody_active(scene)) {
		BKE_rigidbody_rebuild_world(scene, ctime, false);
	}
}

void BKE_rigidbody_eval_simulation(EvaluationContext *UNUSED(eval_ctx),
                                   Scene *scene)
{
	float ctime = BKE_scene_frame_get(scene);

	if (G.debug & G_DEBUG_DEPSGRAPH) {
		printf("%s at %f\n", __func__, ctime);
	}

	/* evaluate rigidbody sim */
	if (BKE_scene_check_rigidbody_active(scene)) {
		BKE_rigidbody_do_simulation(scene, ctime);
	}
}

void BKE_rigidbody_object_sync_transforms(EvaluationContext *UNUSED(eval_ctx),
                                          Scene *scene,
                                          Object *ob)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	float ctime = BKE_scene_frame_get(scene);

	if (G.debug & G_DEBUG_DEPSGRAPH) {
		printf("%s on %s\n", __func__, ob->id.name);
	}

	/* read values pushed into RBO from sim/cache... */
	BKE_rigidbody_sync_transforms(rbw, ob, ctime);
}


/*====================================================================================================================*/

/* Fracture Modifier stuff */

#ifdef WITH_BULLET

static void activateRigidbody(RigidBodyOb* rbo, RigidBodyWorld *UNUSED(rbw), MeshIsland *mi, Object *ob)
{
	RigidBodyShardCon *con;
	int i;

	if (rbo->flag & RBO_FLAG_KINEMATIC && rbo->type == RBO_TYPE_ACTIVE)
	{
		rbo->flag &= ~RBO_FLAG_KINEMATIC;
		rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;

		//propagate trigger on impact / activation
		if (ob->rigidbody_object->flag & RBO_FLAG_PROPAGATE_TRIGGER) {
			rbo->flag |= RBO_FLAG_PROPAGATE_TRIGGER;
		}

		//RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
		RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
		RB_body_set_kinematic_state(rbo->physics_object, false);
		//RB_dworld_add_body(rbw->physics_world, rbo->physics_object, rbo->col_groups, mi, ob);
		RB_body_activate(rbo->physics_object);
	}

	if (mi && ob->rigidbody_object->flag & RBO_FLAG_CONSTRAINT_DISSOLVE) {
		for (i = 0; i < mi->participating_constraint_count; i++) {
			bool different_cluster = false;
			bool dissolve_plastic = (ob->rigidbody_object->flag & RBO_FLAG_PLASTIC_DISSOLVE);
			con = mi->participating_constraints[i];

			different_cluster = ((con->mi1->particle_index != con->mi2->particle_index) ||
			                    ((con->mi1->particle_index == -1) && (con->mi2->particle_index == -1)));

			if (con->physics_constraint && different_cluster) {
				if (dissolve_plastic) {
					con->flag |= RBC_FLAG_PLASTIC_ACTIVE;
				}

				if (con->breaking_threshold >= 0)
				{
					RB_constraint_set_enabled(con->physics_constraint, false);
				}
			}
		}
	}
}

static bool isModifierActive(FractureModifierData *rmd) {
	return ((rmd != NULL) && (rmd->modifier.mode & (eModifierMode_Realtime | eModifierMode_Render)) &&
	        (rmd->refresh == false || rmd->fracture_mode == MOD_FRACTURE_DYNAMIC));
}

static void calc_dist_angle(RigidBodyShardCon *con, float *dist, float *angle, bool exact)
{
	float q1[4], q2[4], qdiff[4], axis[3];

	if (con == NULL || con->mi1 == NULL || con->mi2 == NULL ||
	    con->mi1->rigidbody == NULL || con->mi2->rigidbody == NULL)
	{
		*dist = 0;
		*angle = 0;
		return;
	}

	sub_v3_v3v3(axis, con->mi1->rigidbody->pos, con->mi2->rigidbody->pos);
	*dist = len_v3(axis);

	copy_qt_qt(q1, con->mi1->rigidbody->orn);
	copy_qt_qt(q2, con->mi2->rigidbody->orn);

	if (exact)
	{
		float iquat1[4], iquat2[4];
		invert_qt_qt(iquat1, con->mi1->rot);
		invert_qt_qt(iquat2, con->mi2->rot);
		mul_qt_qtqt(q1, q1, iquat1);
		mul_qt_qtqt(q2, q2, iquat2);
		rotation_between_quats_to_quat(qdiff, q1, q2);
		normalize_qt(qdiff);
		*angle = 2.0f * saacos(qdiff[0]);
		if (!isfinite(*angle)) {
			*angle = 0.0f;
		}
	}
	else
	{
		//XXX TODO probably very wrong here
		invert_qt(q1);
		mul_qt_qtqt(qdiff, q1, q2);
		quat_to_axis_angle(axis, angle, qdiff);
	}
}

void BKE_rigidbody_start_dist_angle(RigidBodyShardCon *con, bool exact, bool both)
{
	/* store starting angle and distance per constraint*/
	float dist, angle;
	calc_dist_angle(con, &dist, &angle, exact);

	if (both)
	{
		con->start_dist = dist;
		con->start_angle = angle;
		//printf("Start Values(dist, angle) %f %f %f %f\n", con->start_dist, con->start_angle, dist, angle);
	}

	con->start_dist_deform = dist;
	con->start_angle_deform = angle;
}

float BKE_rigidbody_calc_max_con_mass(Object *ob)
{
	FractureModifierData *rmd;
	ModifierData *md;
	RigidBodyShardCon *con;
	float max_con_mass = 0, con_mass;

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_Fracture) {
			rmd = (FractureModifierData *)md;
			for (con = rmd->meshConstraints.first; con; con = con->next) {
				if ((con->mi1 != NULL && con->mi1->rigidbody != NULL) &&
				    (con->mi2 != NULL && con->mi2->rigidbody != NULL)) {
					con_mass = con->mi1->rigidbody->mass + con->mi2->rigidbody->mass;
					if (con_mass > max_con_mass) {
						max_con_mass = con_mass;
					}
				}
			}

			return max_con_mass;
		}
	}

	return 0;
}

float BKE_rigidbody_calc_min_con_dist(Object *ob)
{
	FractureModifierData *rmd;
	ModifierData *md;
	RigidBodyShardCon *con;
	float min_con_dist = FLT_MAX, con_dist, con_vec[3];

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_Fracture) {
			rmd = (FractureModifierData *)md;
			for (con = rmd->meshConstraints.first; con; con = con->next) {
				if ((con->mi1 != NULL && con->mi1->rigidbody != NULL) &&
				    (con->mi2 != NULL && con->mi2->rigidbody != NULL)) {
					sub_v3_v3v3(con_vec, con->mi1->centroid, con->mi2->centroid);
					con_dist = len_v3(con_vec);
					if (con_dist < min_con_dist) {
						min_con_dist = con_dist;
					}
				}
			}

			return min_con_dist;
		}
	}

	return FLT_MAX;
}


void BKE_rigidbody_calc_threshold(float max_con_mass, FractureModifierData *rmd, RigidBodyShardCon *con) {

	float max_thresh, thresh = 0.0f, con_mass;
	if ((max_con_mass == 0) && (rmd->use_mass_dependent_thresholds)) {
		return;
	}

	if ((con->mi1 == NULL) || (con->mi2 == NULL)) {
		return;
	}

	max_thresh = thresh = rmd->breaking_threshold;
	if ((con->mi1->rigidbody != NULL) && (con->mi2->rigidbody != NULL)) {

		if (rmd->use_compounds)
		{
			float min_mass = MIN2(con->mi1->rigidbody->mass, con->mi2->rigidbody->mass);
			float max_mass = MAX2(con->mi1->rigidbody->mass, con->mi2->rigidbody->mass);

			thresh = ((min_mass + (rmd->mass_threshold_factor * max_mass)) / (min_mass + max_mass)) * max_thresh;
		}
		else
		{
			con_mass = con->mi1->rigidbody->mass + con->mi2->rigidbody->mass;
			if (rmd->use_mass_dependent_thresholds)
			{
				thresh = (con_mass / max_con_mass) * max_thresh;
			}
		}

		con->breaking_threshold = thresh;
	}
}

static int DM_mesh_minmax(DerivedMesh *dm, float r_min[3], float r_max[3])
{
	MVert *mvert;
	int i = 0, totvert;

	mvert = dm->getVertArray(dm);
	totvert = dm->getNumVerts(dm);
	for (i = 0; i < totvert; i++) {
		minmax_v3v3_v3(r_min, r_max, mvert[i].co);
	}

	return (dm->numVertData != 0);
}

static void DM_mesh_boundbox(DerivedMesh *bm, float r_loc[3], float r_size[3])
{
	float min[3], max[3];
	float mloc[3], msize[3];

	if (!r_loc) r_loc = mloc;
	if (!r_size) r_size = msize;

	INIT_MINMAX(min, max);
	if (!DM_mesh_minmax(bm, min, max)) {
		min[0] = min[1] = min[2] = -1.0f;
		max[0] = max[1] = max[2] = 1.0f;
	}

	mid_v3_v3v3(r_loc, min, max);

	r_size[0] = (max[0] - min[0]) / 2.0f;
	r_size[1] = (max[1] - min[1]) / 2.0f;
	r_size[2] = (max[2] - min[2]) / 2.0f;
}

static float box_volume(float size[3])
{
	float volume;

	volume = size[0] * size[1] * size[2];
	if (volume == 0 && size[0] == 0) {
		volume = size[1] * size[2];
	}
	else if (volume == 0 && size[1] == 0) {
		volume = size[0] * size[2];
	}
	else if (volume == 0 && size[2] == 0) {
		volume = size[0] * size[1];
	}

	return volume;
}

/* helper function to calculate volume of rigidbody object */
float BKE_rigidbody_calc_volume_dm(DerivedMesh *dm, RigidBodyOb *rbo, Object* ob)
{
	float loc[3]  = {0.0f, 0.0f, 0.0f};
	float size[3]  = {1.0f, 1.0f, 1.0f};
	float scale[3] = {1.0f, 1.0f, 1.0f};
	float radius = 1.0f;
	float height = 1.0f;

	float volume = 0.0f;

	/* if automatically determining dimensions, use the Object's boundbox
	 *	- assume that all quadrics are standing upright on local z-axis
	 *	- assume even distribution of mass around the Object's pivot
	 *	  (i.e. Object pivot is centralised in boundbox)
	 *	- boundbox gives full width
	 */
	/* XXX: all dimensions are auto-determined now... later can add stored settings for this*/
	DM_mesh_boundbox(dm, loc, size);
	mul_v3_fl(size, 2.0f);

	/* also take object scale into account */
	if (ob)
	{
		mat4_to_size(scale, ob->obmat);
		mul_v3_v3(size, scale);
	}

	if (ELEM(rbo->shape, RB_SHAPE_CAPSULE, RB_SHAPE_CYLINDER, RB_SHAPE_CONE)) {
		/* take radius as largest x/y dimension, and height as z-dimension */
		radius = MAX2(size[0], size[1]) * 0.5f;
		height = size[2];
	}
	else if (rbo->shape == RB_SHAPE_SPHERE) {
		/* take radius to the the largest dimension to try and encompass everything */
		radius = max_fff(size[0], size[1], size[2]) * 0.5f;
	}

	/* calculate volume as appropriate  */
	switch (rbo->shape) {

		case RB_SHAPE_SPHERE:
			volume = 4.0f / 3.0f * (float)M_PI * radius * radius * radius;
			break;

		/* for now, assume that capsule is close enough to a cylinder... */
		case RB_SHAPE_CAPSULE:
		case RB_SHAPE_CYLINDER:
			volume = (float)M_PI * radius * radius * height;
			break;

		case RB_SHAPE_CONE:
			volume = (float)M_PI / 3.0f * radius * radius * height;
			break;

		/* for now, all mesh shapes are just treated as boxes...
		 * NOTE: this may overestimate the volume, but other methods are overkill
		 */
		case RB_SHAPE_BOX:
			volume = box_volume(size);
			break;

		case RB_SHAPE_CONVEXH:
		case RB_SHAPE_TRIMESH:
		{
			if (!ob) {
				/* for quick shard mass (re)calculation approximate,
				 * else for many shards on complex geometries this becomes
				 * very slow */
				volume = box_volume(size);
			}
			else
			{
				//for initial mass calculation take exact values
				MVert *mvert = dm->getVertArray(dm);
				int totvert = dm->getNumVerts(dm);
				const MLoopTri *mlooptri = dm->getLoopTriArray(dm);
				int tottri = dm->getNumLoopTri(dm);
				MLoop *mloop = dm->getLoopArray(dm);

				BKE_mesh_calc_volume(mvert, totvert, mlooptri, tottri, mloop, &volume, NULL);

				if (volume == 0.0f)
					volume = box_volume(size);
			}
			break;
		}

#if 0 // XXX: not defined yet
		case RB_SHAPE_COMPOUND:
			volume = 0.0f;
			break;
#endif
	}

	/* return the volume calculated */
	return volume;
}

void BKE_rigidbody_calc_shard_mass(Object *ob, MeshIsland *mi, DerivedMesh *orig_dm)
{
	DerivedMesh *dm_ob = orig_dm, *dm_mi;
	float vol_mi = 0, mass_mi = 0, vol_ob = 0, mass_ob = 0;

	if (dm_ob == NULL) {
		/* fallback method */
		if (ob->type == OB_MESH) {
			/* if we have a mesh, determine its volume */
			dm_ob = CDDM_from_mesh(ob->data);
			vol_ob = BKE_rigidbody_calc_volume_dm(dm_ob, ob->rigidbody_object, ob);
		}
		else {
			/* else get object boundbox as last resort */
			float dim[3];
			BKE_object_dimensions_get(ob, dim);
			vol_ob = dim[0] * dim[1] * dim[2];
		}
	}
	else
	{
		vol_ob = BKE_rigidbody_calc_volume_dm(dm_ob, ob->rigidbody_object, ob);
	}

	mass_ob = ob->rigidbody_object->mass;

	if (vol_ob > 0) {
		dm_mi = mi->physics_mesh;
		vol_mi = BKE_rigidbody_calc_volume_dm(dm_mi, mi->rigidbody, NULL);
		mass_mi = (vol_mi / vol_ob) * mass_ob;
		mi->rigidbody->mass = mass_mi;
	}

	if (mi->rigidbody->type == RBO_TYPE_ACTIVE) {
		if (mi->rigidbody->mass < 0.001)
			mi->rigidbody->mass = 0.001;  /* set a minimum mass for active objects */
	}

	/* only active bodies need mass update */
	if ((mi->rigidbody->physics_object) && (mi->rigidbody->type == RBO_TYPE_ACTIVE)) {
		RB_body_set_mass(mi->rigidbody->physics_object, RBO_GET_MASS(mi->rigidbody));
	}

	if (orig_dm == NULL && dm_ob != NULL)
	{
		/* free temp dm, if it hasnt been passed in */
		dm_ob->needsFree = 1;
		dm_ob->release(dm_ob);
	}
}

static void initNormals(struct MeshIsland *mi, Object *ob, FractureModifierData *fmd)
{
	/* hrm have to init Normals HERE, because we cant do this in readfile.c in case the file is loaded (have no access to the Object there) */
	if (mi->vertno == NULL && mi->vertices_cached != NULL) {
		KDTreeNearest n;
		int index = 0, i = 0;
		MVert mvrt;

		DerivedMesh *dm = ob->derivedFinal;
		if (dm == NULL) {
			dm = CDDM_from_mesh(ob->data);
		}

		if (fmd->nor_tree == NULL) {
			/* HRRRRRMMMM need to build the kdtree here as well if we start the sim after loading and not refreshing, again, no access to object.... */
			int i = 0, totvert;
			KDTree *tree;
			MVert *mv, *mvert;

			mvert = dm->getVertArray(dm);
			totvert = dm->getNumVerts(dm);
			tree = BLI_kdtree_new(totvert);

			for (i = 0, mv = mvert; i < totvert; i++, mv++) {
				BLI_kdtree_insert(tree, i, mv->co);
			}

			BLI_kdtree_balance(tree);
			fmd->nor_tree = tree;
		}

		mi->vertno = MEM_callocN(sizeof(short) * 3 * mi->vertex_count, "mi->vertno");
		for (i = 0; i < mi->vertex_count; i++) {
			MVert *v = mi->vertices_cached[i];
			index = BLI_kdtree_find_nearest(fmd->nor_tree, v->co, &n);
			dm->getVert(dm, index, &mvrt);
			mi->vertno[i * 3] = mvrt.no[0];
			mi->vertno[i * 3 + 1] = mvrt.no[1];
			mi->vertno[i * 3 + 2] = mvrt.no[2];
		}

		if (ob->derivedFinal == NULL) {
			dm->needsFree = 1;
			dm->release(dm);
			dm = NULL;
		}
	}
}

void BKE_rigidbody_update_cell(struct MeshIsland *mi, Object *ob, float loc[3], float rot[4], FractureModifierData *rmd, int frame,
                               struct RigidBodyWorld *rbw)
{
	float startco[3], centr[3], size[3];
	short startno[3];
	int j, n = 0, x = 0;
	bool invalidData;

	/* hrm have to init Normals HERE, because we cant do this in readfile.c in case the file is loaded (have no access to the Object there)*/
	if (mi->vertno == NULL && rmd->fix_normals) {
		initNormals(mi, ob, rmd);
	}

	invalidData = (loc[0] == FLT_MIN) || (rot[0] == FLT_MIN);

	if (invalidData) {
		return;
	}

	//invert_m4_m4(ob->imat, ob->obmat);
	mat4_to_size(size, ob->obmat);

	if (rmd->fracture_mode != MOD_FRACTURE_DYNAMIC && frame >= mi->start_frame) {
		/*record only in prefracture case here, when you want to convert to keyframes*/
		n = frame - mi->start_frame + 1;
		x = frame - mi->start_frame;

		if (mi->locs == NULL || mi->rots == NULL || mi->acc_sequence == NULL)
		{
			float loca[3], rota[4], quat[4];
			mi->locs = MEM_mallocN(sizeof(float)*3, "mi->locs");
			mi->rots = MEM_mallocN(sizeof(float)*4, "mi->rots");
			mi->acc_sequence = MEM_callocN(sizeof(float) * 2, "mi->acc_sequence");
			mi->frame_count = 0;

			copy_v3_v3(loca, mi->centroid);
			mul_m4_v3(ob->obmat, loca);
			mat4_to_quat(quat, ob->obmat);

			copy_qt_qt(rota, mi->rot);
			mul_qt_qtqt(rota, quat, rota);

			mi->locs[0] = loca[0];
			mi->locs[1] = loca[1];
			mi->locs[2] = loca[2];

			mi->rots[0] = rota[0];
			mi->rots[1] = rota[1];
			mi->rots[2] = rota[2];
			mi->rots[3] = rota[3];
		}

		if (n > mi->frame_count) {
			mi->locs = MEM_reallocN(mi->locs, sizeof(float) * 3 * n);
			mi->rots = MEM_reallocN(mi->rots, sizeof(float) * 4 * n);
			mi->acc_sequence = MEM_reallocN(mi->acc_sequence, sizeof(float) * (n+1));
			//mi->acc_sequence[x] = 0.0f;
			mi->acc_sequence[n] = 0.0f;

			mi->locs[x*3] = loc[0];
			mi->locs[x*3+1] = loc[1];
			mi->locs[x*3+2] = loc[2];

			mi->rots[x*4] = rot[0];
			mi->rots[x*4+1] = rot[1];
			mi->rots[x*4+2] = rot[2];
			mi->rots[x*4+3] = rot[3];
			mi->frame_count = n;
		}
	}

	for (j = 0; j < mi->vertex_count; j++) {
		struct MVert *vert;
		float fno[3];

		if (!mi->vertices_cached) {
			return;
		}

		vert = mi->vertices_cached[j];
		if (vert == NULL) break;
		if (vert->co == NULL) break;
		//if (rmd->refresh == true) break;

		startco[0] = mi->vertco[j * 3];
		startco[1] = mi->vertco[j * 3 + 1];
		startco[2] = mi->vertco[j * 3 + 2];

		if (rmd->fix_normals) {
			float irot[4], qrot[4];
			startno[0] = mi->vertno[j * 3];
			startno[1] = mi->vertno[j * 3 + 1];
			startno[2] = mi->vertno[j * 3 + 2];

			/*ignore global quaternion rotation here */
			normal_short_to_float_v3(fno, startno);
			mat4_to_quat(qrot, ob->obmat);
			invert_qt_qt(irot, qrot);
			mul_qt_v3(rot, fno);
			mul_qt_v3(irot, fno);
			normal_float_to_short_v3(vert->no, fno);
		}

		copy_v3_v3(vert->co, startco);
		mul_v3_v3(vert->co, size);
		mul_qt_v3(rot, vert->co);
		copy_v3_v3(centr, mi->centroid);
		mul_v3_v3(centr, size);
		mul_qt_v3(rot, centr);
		sub_v3_v3(vert->co, centr);
		add_v3_v3(vert->co, loc);
		mul_m4_v3(ob->imat, vert->co);
	}

	ob->recalc |= OB_RECALC_ALL;
}

/* create collision shape of mesh - convex hull */
static rbCollisionShape *rigidbody_get_shape_convexhull_from_mesh(Mesh *me, float margin, bool *can_embed)
{
	rbCollisionShape *shape = NULL;
	int totvert = me->totvert;
	MVert *mvert = me->mvert;

	if (me && totvert) {
		shape = RB_shape_new_convex_hull((float *)mvert, sizeof(MVert), totvert, margin, can_embed);
	}
	else {
		printf("ERROR: no vertices to define Convex Hull collision shape with\n");
	}

	return shape;
}

static rbCollisionShape *rigidbody_get_shape_convexhull_from_dm(DerivedMesh *dm, float margin, bool *can_embed)
{
	rbCollisionShape *shape = NULL;
	int totvert = dm->getNumVerts(dm);
	MVert *mvert = dm->getVertArray(dm);

	if (dm && totvert) {
		shape = RB_shape_new_convex_hull((float *)mvert, sizeof(MVert), totvert, margin, can_embed);
	}
	else {
		printf("ERROR: no vertices to define Convex Hull collision shape with\n");
	}

	return shape;
}


static void scale_physics_mesh(DerivedMesh** dm, float loc[3], float scale)
{
	/*location is the centroid, scale factor comes from ob->rigidbody (main dummy rigidbody object) */
	MVert* mv, *mvert;
	int totvert, i;
	float l[3], size[3], scale_v[3], cent[3];

	DM_mesh_boundbox(*dm, l, size);

	scale_v[0] = (size[0] * scale) / size[0];
	scale_v[1] = (size[1] * scale) / size[1];
	scale_v[2] = (size[2] * scale) / size[2];

	copy_v3_v3(cent, loc);
	mul_v3_fl(cent, scale);

	mvert   = (*dm)->getVertArray(*dm);
	totvert = (*dm)->getNumVerts(*dm);

	for (i = 0, mv = mvert; i < totvert; i++, mv++ )
	{
		add_v3_v3(mv->co, loc);
		mul_v3_v3(mv->co, scale_v);
		sub_v3_v3(mv->co, cent);
	}
}

/* create collision shape of mesh - triangulated mesh
 * returns NULL if creation fails.
 */
static rbCollisionShape *rigidbody_get_shape_trimesh_from_mesh_shard(MeshIsland *mi, Object *ob)
{
	rbCollisionShape *shape = NULL;

	if (mi && mi->physics_mesh) {
		DerivedMesh *dm = NULL;
		MVert *mvert;
		const MLoopTri *looptri;
		int totvert;
		int tottri;
		const MLoop *mloop;

		//dm = rigidbody_get_mesh(ob);
		dm = mi->physics_mesh;

		/* ensure mesh validity, then grab data */
		if (dm == NULL)
			return NULL;

		//DM_ensure_looptri(dm);

		mvert   = dm->getVertArray(dm);
		totvert = dm->getNumVerts(dm);
		looptri = dm->getLoopTriArray(dm);
		tottri = dm->getNumLoopTri(dm);
		mloop = dm->getLoopArray(dm);

		/* sanity checking - potential case when no data will be present */
		if ((totvert == 0) || (tottri == 0)) {
			printf("WARNING: no geometry data converted for Mesh Collision Shape (ob = %s)\n", ob->id.name + 2);
		}
		else {
			rbMeshData *mdata;
			int i;

			/* init mesh data for collision shape */
			mdata = RB_trimesh_data_new(tottri, totvert);

			RB_trimesh_add_vertices(mdata, (float *)mvert, totvert, sizeof(MVert));

			/* loop over all faces, adding them as triangles to the collision shape
			 * (so for some faces, more than triangle will get added)
			 */
			if (mvert && looptri) {
				for (i = 0; i < tottri; i++) {
					/* add first triangle - verts 1,2,3 */
					const MLoopTri *lt = &looptri[i];
					int vtri[3];

					vtri[0] = mloop[lt->tri[0]].v;
					vtri[1] = mloop[lt->tri[1]].v;
					vtri[2] = mloop[lt->tri[2]].v;

					RB_trimesh_add_triangle_indices(mdata, i, UNPACK3(vtri));
				}
			}

			RB_trimesh_finish(mdata);

			/* construct collision shape
			 *
			 * These have been chosen to get better speed/accuracy tradeoffs with regards
			 * to limitations of each:
			 *    - BVH-Triangle Mesh: for passive objects only. Despite having greater
			 *                         speed/accuracy, they cannot be used for moving objects.
			 *    - GImpact Mesh:      for active objects. These are slower and less stable,
			 *                         but are more flexible for general usage.
			 */
			if (ob->rigidbody_object->type == RBO_TYPE_PASSIVE) {
				shape = RB_shape_new_trimesh(mdata);
			}
			else {
				shape = RB_shape_new_gimpact_mesh(mdata);
			}
		}

#if 0
		/* cleanup temp data */
		if (dm && ob->rigidbody_object->mesh_source == RBO_MESH_BASE) {
			dm->needsFree = 1;
			dm->release(dm);
			dm = NULL;
		}
#endif

	}
	else {
		printf("ERROR: cannot make Triangular Mesh collision shape for non-Mesh object\n");
	}

	return shape;
}

/* create collision shape of mesh - triangulated mesh
 * returns NULL if creation fails.
 */
static rbCollisionShape *rigidbody_get_shape_trimesh_from_mesh(Object *ob)
{
	rbCollisionShape *shape = NULL;

	if (ob->type == OB_MESH) {
		DerivedMesh *dm = NULL;
		MVert *mvert;
		MFace *mface;
		int totvert;
		int totface;
		int tottris = 0;
		int triangle_index = 0;

		dm = rigidbody_get_mesh(ob);

		/* ensure mesh validity, then grab data */
		if (dm == NULL)
			return NULL;

		DM_ensure_tessface(dm);

		mvert   = dm->getVertArray(dm);
		totvert = dm->getNumVerts(dm);
		mface   = dm->getTessFaceArray(dm);
		totface = dm->getNumTessFaces(dm);

		/* sanity checking - potential case when no data will be present */
		if ((totvert == 0) || (totface == 0)) {
			printf("WARNING: no geometry data converted for Mesh Collision Shape (ob = %s)\n", ob->id.name + 2);
		}
		else {
			rbMeshData *mdata;
			int i;

			/* count triangles */
			for (i = 0; i < totface; i++) {
				(mface[i].v4) ? (tottris += 2) : (tottris += 1);
			}

			/* init mesh data for collision shape */
			mdata = RB_trimesh_data_new(tottris, totvert);

			RB_trimesh_add_vertices(mdata, (float *)mvert, totvert, sizeof(MVert));

			/* loop over all faces, adding them as triangles to the collision shape
			 * (so for some faces, more than triangle will get added)
			 */
			for (i = 0; (i < totface) && (mface) && (mvert); i++, mface++) {
				/* add first triangle - verts 1,2,3 */
				RB_trimesh_add_triangle_indices(mdata, triangle_index, mface->v1, mface->v2, mface->v3);
				triangle_index++;

				/* add second triangle if needed - verts 1,3,4 */
				if (mface->v4) {
					RB_trimesh_add_triangle_indices(mdata, triangle_index, mface->v1, mface->v3, mface->v4);
					triangle_index++;
				}
			}
			RB_trimesh_finish(mdata);

			/* construct collision shape
			 *
			 * These have been chosen to get better speed/accuracy tradeoffs with regards
			 * to limitations of each:
			 *    - BVH-Triangle Mesh: for passive objects only. Despite having greater
			 *                         speed/accuracy, they cannot be used for moving objects.
			 *    - GImpact Mesh:      for active objects. These are slower and less stable,
			 *                         but are more flexible for general usage.
			 */
			if (ob->rigidbody_object->type == RBO_TYPE_PASSIVE) {
				shape = RB_shape_new_trimesh(mdata);
			}
			else {
				shape = RB_shape_new_gimpact_mesh(mdata);
			}
		}

		/* cleanup temp data */
		if (ob->rigidbody_object->mesh_source == RBO_MESH_BASE || ob->rigidbody_object->mesh_source == RBO_MESH_FINAL_SOLID) {
			dm->release(dm);
		}
	}
	else {
		printf("ERROR: cannot make Triangular Mesh collision shape for non-Mesh object\n");
	}

	return shape;
}

/* --------------------- */

/* Create new physics sim collision shape for object and store it,
 * or remove the existing one first and replace...
 */
void BKE_rigidbody_validate_sim_shard_shape(MeshIsland *mi, Object *ob, short rebuild)
{
	RigidBodyOb *rbo = mi->rigidbody;
	rbCollisionShape *new_shape = NULL;
	float size[3] = {1.0f, 1.0f, 1.0f}, loc[3] = {0.0f, 0.0f, 0.0f};
	float radius = 1.0f;
	float height = 1.0f;
	float capsule_height;
	float hull_margin = 0.0f;
	bool can_embed = true;
	bool has_volume;
	float min[3], max[3], margin;

	/* sanity check */
	if (rbo == NULL)
		return;

	/* don't create a new shape if we already have one and don't want to rebuild it */
	if (rbo->physics_shape && !rebuild)
		return;

	/* if automatically determining dimensions, use the Object's boundbox
	 *	- assume that all quadrics are standing upright on local z-axis
	 *	- assume even distribution of mass around the Object's pivot
	 *	  (i.e. Object pivot is centralized in boundbox)
	 *	- boundbox gives full width
	 */
	// XXX: all dimensions are auto-determined now... later can add stored settings for this
	/* get object dimensions without scaling */

	INIT_MINMAX(min, max);
	if (!DM_mesh_minmax(mi->physics_mesh, min, max)) {
		min[0] = min[1] = min[2] = -1.0f;
		max[0] = max[1] = max[2] = 1.0f;
	}

	mid_v3_v3v3(loc, min, max);
	size[0] = (max[0] - min[0]) / 2.0f;
	size[1] = (max[1] - min[1]) / 2.0f;
	size[2] = (max[2] - min[2]) / 2.0f;

	if (ELEM(rbo->shape, RB_SHAPE_CAPSULE, RB_SHAPE_CYLINDER, RB_SHAPE_CONE)) {
		/* take radius as largest x/y dimension, and height as z-dimension */
		radius = MAX2(size[0], size[1]);
		height = size[2];
	}
	else if (rbo->shape == RB_SHAPE_SPHERE) {

		/* take radius to the largest dimension to try and encompass everything */
		radius = (rbo->flag & RBO_FLAG_USE_MARGIN) ? min_fff(size[0], size[1], size[2]) :
				 max_fff(size[0], size[1], size[2]);
	}

	/* create new shape */
	switch (rbo->shape) {
		case RB_SHAPE_BOX:
			new_shape = RB_shape_new_box(size[0], size[1], size[2]);
			break;

		case RB_SHAPE_SPHERE:
			margin = (rbo->flag & RBO_FLAG_USE_MARGIN) ? rbo->margin : 0.0f;
			margin = (rbo->flag & RBO_FLAG_RANDOM_MARGIN) ? BLI_frand() * margin : margin;

			new_shape = RB_shape_new_sphere(radius + margin);
			break;

		case RB_SHAPE_CAPSULE:
			capsule_height = (height - radius) * 2.0f;
			new_shape = RB_shape_new_capsule(radius, (capsule_height > 0.0f) ? capsule_height : 0.0f);
			break;
		case RB_SHAPE_CYLINDER:
			new_shape = RB_shape_new_cylinder(radius, height);
			break;

		case RB_SHAPE_CONE:
			new_shape = RB_shape_new_cone(radius, height * 2.0f);
			break;

		case RB_SHAPE_CONVEXH:
			/* try to embed collision margin */
			has_volume = (MIN3(size[0], size[1], size[2]) > 0.0f);

			if (!(rbo->flag & RBO_FLAG_USE_MARGIN) && has_volume)
				hull_margin = 0.04f;
			new_shape = rigidbody_get_shape_convexhull_from_dm(mi->physics_mesh, hull_margin, &can_embed);
			if (!(rbo->flag & RBO_FLAG_USE_MARGIN))
				rbo->margin = (can_embed && has_volume) ? 0.04f : 0.0f;      /* RB_TODO ideally we shouldn't directly change the margin here */
			break;
		case RB_SHAPE_TRIMESH:
		{
			new_shape = rigidbody_get_shape_trimesh_from_mesh_shard(mi, ob);
			break;
		}
	}
	/* assign new collision shape if creation was successful */
	if (new_shape) {
		margin = RBO_GET_MARGIN(rbo);
		if (rbo->physics_shape)
			RB_shape_delete(rbo->physics_shape);
		rbo->physics_shape = new_shape;

		if ((rbo->flag & RBO_FLAG_RANDOM_MARGIN) && (rbo->shape != RB_SHAPE_SPHERE))
			margin = (BLI_frand() * 0.5 - 1) * margin;

		RB_shape_set_margin(rbo->physics_shape, margin);
	}
	else { /* otherwise fall back to box shape */
		rbo->shape = RB_SHAPE_BOX;
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
	}
}

/* --------------------- */

/* Create physics sim representation of shard given RigidBody settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_shard(RigidBodyWorld *rbw, MeshIsland *mi, Object *ob, short rebuild, int transfer_speeds, float isize[3])
{
	FractureModifierData *fmd = NULL;
	RigidBodyOb *rbo = (mi) ? mi->rigidbody : NULL;
	float loc[3];
	float rot[4];

	/* sanity checks:
	 *	- object doesn't have RigidBody info already: then why is it here?
	 */
	if (rbo == NULL)
		return;

	/* at validation, reset frame count as well */
	/* XXX removed due to dynamic, HACK !!! */
//	mi->start_frame = rbw->pointcache->startframe;
//	mi->frame_count = 0;

	fmd = (FractureModifierData*) modifiers_findByType(ob, eModifierType_Fracture);

	/* make sure collision shape exists */
	/* FIXME we shouldn't always have to rebuild collision shapes when rebuilding objects, but it's needed for constraints to update correctly */
	if (rbo->physics_shape == NULL || rebuild)
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);

	if (rbo->physics_object) {
		if (rebuild == false /*|| mi->rigidbody->flag & RBO_FLAG_KINEMATIC_REBUILD*/)
			RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
	}

	if (!rbo->physics_object || rebuild /*|| (fmd->use_animated_mesh && fmd->anim_mesh_ob)*/) {
		float size[3];

		/* remove rigid body if it already exists before creating a new one */
		if (rbo->physics_object) {
			RB_body_delete(rbo->physics_object);
		}

		//if (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL)
		{
			copy_v3_v3(loc, rbo->pos);
			copy_qt_qt(rot, rbo->orn);
		}
#if 0
		else
		{
			copy_v3_v3(loc, mi->centroid);
			copy_qt_qt(rot, mi->rot);
		}
#endif

		/*if (ob->derivedFinal)
		{
			DM_mesh_boundbox(ob->derivedFinal, locbb, size);
		}
		else
		{
			BKE_mesh_boundbox_calc((Mesh*)ob->data, locbb, size);
		}*/

		copy_v3_v3(size, isize);
		mul_v3_v3(size, ob->size);
		rbo->physics_object = RB_body_new(rbo->physics_shape, loc, rot, fmd->use_compounds, fmd->impulse_dampening,
		                                  fmd->directional_factor, fmd->minimum_impulse, fmd->mass_threshold_factor, size);

		RB_body_set_friction(rbo->physics_object, rbo->friction);
		RB_body_set_restitution(rbo->physics_object, rbo->restitution);

		RB_body_set_damping(rbo->physics_object, rbo->lin_damping, rbo->ang_damping);
		RB_body_set_sleep_thresh(rbo->physics_object, rbo->lin_sleep_thresh, rbo->ang_sleep_thresh);
		RB_body_set_activation_state(rbo->physics_object, rbo->flag & RBO_FLAG_USE_DEACTIVATION);

		if (rbo->type == RBO_TYPE_PASSIVE || rbo->flag & RBO_FLAG_START_DEACTIVATED)
			RB_body_deactivate(rbo->physics_object);


		RB_body_set_linear_factor(rbo->physics_object,
		                          (ob->protectflag & OB_LOCK_LOCX) == 0,
		                          (ob->protectflag & OB_LOCK_LOCY) == 0,
		                          (ob->protectflag & OB_LOCK_LOCZ) == 0);
		RB_body_set_angular_factor(rbo->physics_object,
		                           (ob->protectflag & OB_LOCK_ROTX) == 0,
		                           (ob->protectflag & OB_LOCK_ROTY) == 0,
		                           (ob->protectflag & OB_LOCK_ROTZ) == 0);

		RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
		RB_body_set_kinematic_state(rbo->physics_object, rbo->flag & RBO_FLAG_KINEMATIC || rbo->flag & RBO_FLAG_DISABLED);

		if (transfer_speeds)
		{
			//if ((len_squared_v3(rbo->lin_vel) > (rbo->lin_sleep_thresh * rbo->lin_sleep_thresh)))
			{
				//printf("Setting linear velocity (%f, %f, %f)\n", rbo->lin_vel[0], rbo->lin_vel[1], rbo->lin_vel[2]);
				RB_body_set_linear_velocity(rbo->physics_object, rbo->lin_vel);
			}

			//if ((len_squared_v3(rbo->ang_vel) > (rbo->ang_sleep_thresh * rbo->ang_sleep_thresh)))
			{
				//printf("Setting angular velocity (%f, %f, %f)\n", rbo->ang_vel[0], rbo->ang_vel[1], rbo->ang_vel[2]);
				RB_body_set_angular_velocity(rbo->physics_object, rbo->ang_vel);
			}
		}
	}
	else if (mi->rigidbody->flag & RBO_FLAG_KINEMATIC_REBUILD )
	{
		RB_body_deactivate(rbo->physics_object);
		RB_body_set_mass(rbo->physics_object, 0.0f);
		RB_body_set_kinematic_state(rbo->physics_object, true);
	}

	if (rbw && rbw->physics_world && rbo->physics_object)
	{
		RB_dworld_add_body(rbw->physics_world, rbo->physics_object, rbo->col_groups, mi, ob, mi->linear_index);
	}

	rbo->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
	//rbo->flag &= ~RBO_FLAG_KINEMATIC_REBUILD;
}

/* --------------------- */

static MeshIsland* find_closest_meshisland_to_point(FractureModifierData* fmd, Object *ob, Object *ob2, RigidBodyWorld* rbw, RigidBodyCon *con) {

	MeshIsland *mi, **mi_array = NULL;
	KDTree *tree;
	KDTreeNearest *n = NULL;
	int count = 0;
	int index = 0;
	float loc[3], min[3], max[3], vec[3] = {1, 1, 1};
	int i = 0, r = 0;

	count = BLI_listbase_count(&fmd->meshIslands);
	tree = BLI_kdtree_new(count);
	mi_array = MEM_mallocN(sizeof(MeshIsland*) * count, "mi_array find_closest_meshisland");

	for (mi = fmd->meshIslands.first; mi; mi = mi->next) {
		mul_v3_m4v3(loc, ob->obmat, mi->centroid);
		BLI_kdtree_insert(tree, i, loc);
		mi_array[i] = mi;
		i++;
	}

	BLI_kdtree_balance(tree);

	index = BLI_kdtree_find_nearest(tree, ob2->loc, n);

	//create "aabb"
	mul_v3_v3(vec, ob2->size);
	sub_v3_v3v3(min, ob2->loc, vec);
	add_v3_v3v3(max, ob2->loc, vec);

	if (index == -1) {
		MEM_freeN(mi_array);
		BLI_kdtree_free(tree);
		return NULL;
	}

	if (index >= count) {
		index = count-1;
	}

	mi = mi_array[index];

	//do a range search and clip against "aabb" of empty scale for additional inner constraint
	r = BLI_kdtree_range_search(tree, ob2->loc, &n, max_fff(UNPACK3(ob2->size)));
	for (i = 0; i < r; i++)
	{
		float co[3];
		copy_v3_v3(co, n[i].co);

		if ((co[0] > min[0] && co[0] < max[0]) &&
		    (co[1] > min[1] && co[1] < max[1]) &&
		    (co[2] > min[2] && co[2] < max[2]))
		{
			MeshIsland* mi2;
			int ind = n[i].index;

			if (ind >= count) {
				ind = count-1;
			}

			mi2 = mi_array[ind];

			//connect ?
			BKE_meshisland_constraint_create(fmd, mi, mi2, fmd->constraint_type, fmd->breaking_threshold);
		}
	}

	if (n) {
		MEM_freeN(n);
	}

	MEM_freeN(mi_array);
	BLI_kdtree_free(tree);

	return mi;
}

static void rigidbody_set_springs_active(RigidBodyShardCon *rbc, bool active)
{
	if (rbc && rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING)
	{
		if (active) //XXX TEST purpose only
		{
			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->flag & RBC_FLAG_USE_SPRING_X);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->spring_stiffness_x);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->spring_damping_x);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->flag & RBC_FLAG_USE_SPRING_Y);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->spring_stiffness_y);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->spring_damping_y);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->flag & RBC_FLAG_USE_SPRING_Z);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->spring_stiffness_z);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->spring_damping_z);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->flag & RBC_FLAG_USE_SPRING_ANG_X);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->spring_stiffness_ang_x);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->spring_damping_ang_x);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Y);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->spring_stiffness_ang_y);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->spring_damping_ang_y);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Z);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->spring_stiffness_ang_z);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->spring_damping_ang_z);
		}
		else
		{
			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->flag & RBC_FLAG_USE_SPRING_X);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, 0);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->spring_damping_x);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->flag & RBC_FLAG_USE_SPRING_Y);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, 0);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->spring_damping_y);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->flag & RBC_FLAG_USE_SPRING_Z);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, 0);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->spring_damping_z);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->flag & RBC_FLAG_USE_SPRING_ANG_X);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, 0);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->spring_damping_ang_x);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Y);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, 0);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->spring_damping_ang_y);

			RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Z);
			RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, 0);
			RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->spring_damping_ang_z);
		}
	}
}

static void rigidbody_create_shard_physics_constraint(FractureModifierData* fmd, Object* ob, RigidBodyShardCon *rbc, RigidBodyWorld *rbw)
{
	float loc[3];
	float rot[4];
	float lin_lower;
	float lin_upper;
	float ang_lower;
	float ang_upper;
	rbRigidBody *rb1;
	rbRigidBody *rb2;

	if (rbc && rbc->mi1 && rbc->mi2)
	{
		rb1 = rbc->mi1->rigidbody->physics_object;
		rb2 = rbc->mi2->rigidbody->physics_object;
	}
	else
	{
		return;
	}

	if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL || (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && fmd->is_dynamic_external))
	{
		mul_v3_m4v3(loc, ob->obmat, rbc->pos);
		mat4_to_quat(rot, ob->obmat);
		mul_qt_qtqt(rot, rot, rbc->orn);
	}
	else
	{
		/* keep old constraint calculation for other fracture modes ! */
		/* do this for all constraints */
		/* location for fixed constraints doesnt matter, so keep old setting */
/*		if (rbc->type == RBC_TYPE_FIXED) {
			copy_v3_v3(rbc->pos, rbc->mi1->rigidbody->pos);
		}
		else*/ {
			/* else set location to center */
			add_v3_v3v3(rbc->pos, rbc->mi1->rigidbody->pos, rbc->mi2->rigidbody->pos);
			mul_v3_fl(rbc->pos, 0.5f);
		}

		copy_qt_qt(rbc->orn, rbc->mi1->rigidbody->orn);
		copy_v3_v3(loc, rbc->pos);
		copy_qt_qt(rot, rbc->orn);
	}

	if (rb1 && rb2) {
		switch (rbc->type) {
			case RBC_TYPE_POINT:
				rbc->physics_constraint = RB_constraint_new_point(loc, rb1, rb2);
				break;
			case RBC_TYPE_FIXED:
				rbc->physics_constraint = RB_constraint_new_fixed(loc, rot, rb1, rb2);
				break;
			case RBC_TYPE_HINGE:
				rbc->physics_constraint = RB_constraint_new_hinge(loc, rot, rb1, rb2);
				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Z) {
					RB_constraint_set_limits_hinge(rbc->physics_constraint, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
				}
				else
					RB_constraint_set_limits_hinge(rbc->physics_constraint, 0.0f, -1.0f);
				break;
			case RBC_TYPE_SLIDER:
				rbc->physics_constraint = RB_constraint_new_slider(loc, rot, rb1, rb2);
				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
					RB_constraint_set_limits_slider(rbc->physics_constraint, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				else
					RB_constraint_set_limits_slider(rbc->physics_constraint, 0.0f, -1.0f);
				break;
			case RBC_TYPE_PISTON:
				rbc->physics_constraint = RB_constraint_new_piston(loc, rot, rb1, rb2);
				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X) {
					lin_lower = rbc->limit_lin_x_lower;
					lin_upper = rbc->limit_lin_x_upper;
				}
				else {
					lin_lower = 0.0f;
					lin_upper = -1.0f;
				}
				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_X) {
					ang_lower = rbc->limit_ang_x_lower;
					ang_upper = rbc->limit_ang_x_upper;
				}
				else {
					ang_lower = 0.0f;
					ang_upper = -1.0f;
				}
				RB_constraint_set_limits_piston(rbc->physics_constraint, lin_lower, lin_upper, ang_lower, ang_upper);
				break;
			case RBC_TYPE_6DOF_SPRING:
				rbc->physics_constraint = RB_constraint_new_6dof_spring(loc, rot, rb1, rb2);

				if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL || (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && fmd->is_dynamic_external))
				{
					if ((rbc->plastic_angle < 0.0f) && (rbc->plastic_dist < 0.0f))
					{
						/* no plastic mode */
						rigidbody_set_springs_active(rbc, true);
					}
					else
					{
						/*plastic mode, activate depending on flag */
						/* mark immediate activation, so we dont activate again */

						if (rbc->flag & RBC_FLAG_USE_PLASTIC)
						{
							rbc->flag |= RBC_FLAG_PLASTIC_ACTIVE;
							rigidbody_set_springs_active(rbc, true);
						}
						else
						{
							rbc->flag &= ~RBC_FLAG_PLASTIC_ACTIVE;
							rigidbody_set_springs_active(rbc, false);
						}
					}
				}
				else
				{
					/* no plastic mode available in other fracture modes */
					rigidbody_set_springs_active(rbc, true);
				}

				RB_constraint_set_equilibrium_6dof_spring(rbc->physics_constraint);

			/* fall through */
			case RBC_TYPE_6DOF:
				if (rbc->type == RBC_TYPE_6DOF)     /* a litte awkward but avoids duplicate code for limits */
					rbc->physics_constraint = RB_constraint_new_6dof(loc, rot, rb1, rb2);

				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				else
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, 0.0f, -1.0f);

				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Y)
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->limit_lin_y_lower, rbc->limit_lin_y_upper);
				else
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, 0.0f, -1.0f);

				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Z)
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->limit_lin_z_lower, rbc->limit_lin_z_upper);
				else
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, 0.0f, -1.0f);

				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_X)
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->limit_ang_x_lower, rbc->limit_ang_x_upper);
				else
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, 0.0f, -1.0f);

				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Y)
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->limit_ang_y_lower, rbc->limit_ang_y_upper);
				else
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, 0.0f, -1.0f);

				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Z)
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
				else
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, 0.0f, -1.0f);
				break;
			case RBC_TYPE_MOTOR:
				rbc->physics_constraint = RB_constraint_new_motor(loc, rot, rb1, rb2);

				RB_constraint_set_enable_motor(rbc->physics_constraint, rbc->flag & RBC_FLAG_USE_MOTOR_LIN, rbc->flag & RBC_FLAG_USE_MOTOR_ANG);
				RB_constraint_set_max_impulse_motor(rbc->physics_constraint, rbc->motor_lin_max_impulse, rbc->motor_ang_max_impulse);
				RB_constraint_set_target_velocity_motor(rbc->physics_constraint, rbc->motor_lin_target_velocity, rbc->motor_ang_target_velocity);
				break;
			case RBC_TYPE_COMPOUND:
				rbc->physics_constraint = RB_constraint_new_compound(rb1, rb2);
				break;
		}
	}
	else { /* can't create constraint without both rigid bodies */
		return;
	}

	RB_constraint_set_enabled(rbc->physics_constraint, rbc->flag & RBC_FLAG_ENABLED);

	if (rbc->flag & RBC_FLAG_USE_BREAKING)
		RB_constraint_set_breaking_threshold(rbc->physics_constraint, rbc->breaking_threshold);
	else
		RB_constraint_set_breaking_threshold(rbc->physics_constraint, FLT_MAX);

	if (rbc->flag & RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS)
		RB_constraint_set_solver_iterations(rbc->physics_constraint, rbc->num_solver_iterations);
	else
		RB_constraint_set_solver_iterations(rbc->physics_constraint, -1);

	if (rbc->physics_constraint)
	{
		//char id[64];
		//sprintf(id, "%d", rbc->id);
		RB_constraint_set_id(rbc->physics_constraint, rbc->name);
	}
}

/* Create physics sim representation of constraint given rigid body constraint settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_shard_constraint(RigidBodyWorld *rbw, FractureModifierData *fmd, Object* ob, RigidBodyShardCon *rbc, short rebuild)
{
	/* sanity checks:
	 *	- object should have a rigid body constraint
	 *  - rigid body constraint should have at least one constrained object
	 */
	if (rbc == NULL) {
		return;
	}

	if (ELEM(NULL, rbc->mi1, rbc->mi2)) {
		if (rbc->physics_constraint) {
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}
		return;
	}

	if (rbc->mi1 == rbc->mi2) {
		//owch, this happened... better check for sanity  here
		return;
	}

	if (ELEM(NULL, rbc->mi1->rigidbody, rbc->mi2->rigidbody)) {
		return;
	}

	if (rbc->physics_constraint) {
		if (rebuild == false)
		{
			if (!(rbc->flag & RBC_FLAG_USE_KINEMATIC_DEACTIVATION))
			{
				RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
			}
		}
	}
	if (rbc->physics_constraint == NULL || rebuild || (rbc->flag & RBC_FLAG_USE_KINEMATIC_DEACTIVATION) || (rbc->flag & RBC_FLAG_NEEDS_VALIDATE)) {

		/* remove constraint if it already exists before creating a new one */
		if (rbc->physics_constraint) {
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}

		rigidbody_create_shard_physics_constraint(fmd, ob, rbc, rbw);
	}

	if ((rbw && rbw->physics_world && rbc->physics_constraint)) {
		RB_dworld_add_constraint(rbw->physics_world, rbc->physics_constraint, rbc->flag & RBC_FLAG_DISABLE_COLLISIONS);
	}

	rbc->flag &= ~RBC_FLAG_USE_KINEMATIC_DEACTIVATION;
	rbc->flag &= ~RBC_FLAG_NEEDS_VALIDATE;
}

static bool colgroup_check(int group1, int group2)
{
	int i = 0;
	for (i = 0; i < 20; i++)
	{
		int v1, v2;
		v1 = (group1 & (1 << i));
		v2 = (group2 & (1 << i));

		//printf("%d, %d, %d\n", i, v1, v2);

		if ((v1 > 0) && (v1 == v2))
		{
			return true;
		}
	}

	return false;
}

static bool do_activate(Object* ob, Object *ob2, MeshIsland *mi_compare, RigidBodyWorld *rbw, MeshIsland *mi_trigger, bool activate)
{
	FractureModifierData *fmd;
	bool valid = true;
	bool antiValid = ob2->rigidbody_object->flag & RBO_FLAG_ANTI_TRIGGER;
	bool wouldActivate = false;
	MeshIsland *mi;

	fmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);
	valid = valid && (fmd != NULL);
	antiValid = antiValid && (fmd != NULL);

	valid = valid && (ob->rigidbody_object->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION);
	valid = valid && ((ob2->rigidbody_object->flag & RBO_FLAG_IS_TRIGGER) || ((ob2->rigidbody_object->flag & RBO_FLAG_PROPAGATE_TRIGGER) &&
	        ((mi_trigger) && (mi_trigger->rigidbody->flag & RBO_FLAG_PROPAGATE_TRIGGER))));

	if (valid || antiValid)
	{
		for (mi = fmd->meshIslands.first; mi; mi = mi->next)
		{
			bool dissolve = ob->rigidbody_object->flag & RBO_FLAG_CONSTRAINT_DISSOLVE;

			bool same_cluster = ((mi->particle_index != -1) &&
			                    (mi->particle_index == mi_compare->particle_index));

			bool different_cluster = !same_cluster && dissolve;

			RigidBodyOb* rbo = mi->rigidbody;
			if ((((rbo->flag & RBO_FLAG_KINEMATIC) || different_cluster) &&
			     ((mi_compare == mi) || (same_cluster && !dissolve && fmd->activate_broken))) && valid)
			{
				if (rbo->physics_object && activate) {
					activateRigidbody(rbo, rbw, mi, ob);
				}

				wouldActivate = true;
			}

			if ((mi_compare == mi) && antiValid && activate)
			{
				if (rbo->physics_object) {
					test_deactivate_rigidbody(rbo, mi);
				}
			}
		}
	}
	else if (!fmd)
	{
		RigidBodyOb* rbo = ob->rigidbody_object;
		valid = ob2->rigidbody_object->flag & RBO_FLAG_IS_TRIGGER;
		antiValid = ob2->rigidbody_object->flag & RBO_FLAG_ANTI_TRIGGER;

		if (rbo && valid && activate)
		{
			if (activate)
				activateRigidbody(rbo, rbw, NULL, ob);

			wouldActivate = true;
		}

		if (rbo && antiValid && activate)
		{
			test_deactivate_rigidbody(rbo, NULL);
		}
	}

	return wouldActivate;
}

static int check_colgroup_ghost(Object* ob1, Object *ob2)
{
	int ret = 0;
	ret = colgroup_check(ob1->rigidbody_object->col_groups, ob2->rigidbody_object->col_groups);
	return ret && (!(ob1->rigidbody_object->flag & RBO_FLAG_IS_GHOST) && !(ob2->rigidbody_object->flag & RBO_FLAG_IS_GHOST));
}

static void fake_dynamic_collide(Object *ob1, Object *ob2, MeshIsland *mi1, MeshIsland *mi2, RigidBodyWorld *rbw)
{
	if (((ob1->rigidbody_object->flag & RBO_FLAG_IS_GHOST) && (ob1->rigidbody_object->flag & RBO_FLAG_DYNAMIC_TRIGGER)) ||
		((ob1->rigidbody_object->flag & RBO_FLAG_IS_GHOST) && (ob2->rigidbody_object->flag & RBO_FLAG_DYNAMIC_TRIGGER)) ||
		((ob2->rigidbody_object->flag & RBO_FLAG_IS_GHOST) && (ob1->rigidbody_object->flag & RBO_FLAG_DYNAMIC_TRIGGER)) ||
		((ob2->rigidbody_object->flag & RBO_FLAG_IS_GHOST) && (ob2->rigidbody_object->flag & RBO_FLAG_DYNAMIC_TRIGGER)))
	{
		rbContactPoint point;
		point.contact_force = 0;

		if (mi1 || mi2) {
			if (mi1) {
				copy_v3_v3(point.contact_pos_world_onA, mi1->centroid);
				point.contact_body_indexA = mi1->linear_index;
			}
			else {
				copy_v3_v3(point.contact_pos_world_onA, ob1->loc);
				point.contact_body_indexA = -1;
			}

			if (mi2) {
				copy_v3_v3(point.contact_pos_world_onB, mi2->centroid);
				point.contact_body_indexB = mi2->linear_index;
			}
			else {
				copy_v3_v3(point.contact_pos_world_onB, ob2->loc);
				point.contact_body_indexB = -1;
			}

			check_fracture(&point, rbw, ob1, ob2);
		}
	}
}

static bool check_constraint_island(FractureModifierData* fmd, MeshIsland *mi1, MeshIsland *mi2)
{	
	if (mi1 && mi2 && !fmd->use_compounds && (!fmd->use_constraint_collision || fmd->use_self_collision)) {

		float dist_sq = len_squared_v3v3(mi1->centroid, mi2->centroid);
		bool is_near = len_squared_v3v3(mi1->rigidbody->pos, mi2->rigidbody->pos) < dist_sq;
		bool same_island = mi1->constraint_index == mi2->constraint_index;
		bool same_near_self = same_island && fmd->use_self_collision && is_near;
		bool diff_clust_near_self = mi1->particle_index != mi2->particle_index && fmd->use_self_collision && is_near;
		bool regular_case = mi1 && mi2 && !fmd->use_constraint_collision && !same_island;

		if (fmd->use_self_collision)
		{
			if (mi1->rigidbody->physics_shape)
			{
				RB_shape_set_margin(mi1->rigidbody->physics_shape, is_near ? 0.0f : RBO_GET_MARGIN(mi1->rigidbody));
			}

			if (mi2->rigidbody->physics_shape)
			{
				RB_shape_set_margin(mi2->rigidbody->physics_shape, is_near ? 0.0f : RBO_GET_MARGIN(mi2->rigidbody));
			}
		}

		//collide if: same island and near, different cluster if clustered and same island and near

		return regular_case || same_near_self || diff_clust_near_self;

	}

	if (fmd)
	{
		return !(fmd->dm_group && fmd->use_constraint_collision);
	}

	return true;
}

/* this allows partial object activation, only some shards will be activated, called from bullet(!) */
static int filterCallback(void* world, void* island1, void* island2, void *blenderOb1, void* blenderOb2, bool activate) {
	MeshIsland* mi1, *mi2;
	RigidBodyWorld *rbw = (RigidBodyWorld*)world;
	Object* ob1, *ob2;
	int ob_index1 = -1, ob_index2 = -1;
	bool validOb = true, check_activate = false;

	mi1 = (MeshIsland*)island1;
	mi2 = (MeshIsland*)island2;

	FractureModifierData *fmd1 = (FractureModifierData*)modifiers_findByType((Object*)blenderOb1, eModifierType_Fracture);
	FractureModifierData *fmd2 = (FractureModifierData*)modifiers_findByType((Object*)blenderOb2, eModifierType_Fracture);

#if 0
	if ((fmd1 && fmd1->fracture_mode == MOD_FRACTURE_EXTERNAL) ||
	   (fmd2 && fmd2->fracture_mode == MOD_FRACTURE_EXTERNAL))
	{
		/*external doesnt need triggering, maybe the prefractured object (and dynamic ?)... later TODO */
		/* XXXX remove this in case of external, it interferes */
		ob1 = blenderOb1;
		ob2 = blenderOb2;
		return check_colgroup_ghost(ob1, ob2);
	}
#endif

	if (rbw == NULL)
	{
		/* just check for ghost flags here, do not activate anything */
		ob1 = blenderOb1;
		ob2 = blenderOb2;
		return check_colgroup_ghost(ob1, ob2);
	}

	/* cache offset map is a dull name for that... */
	if (mi1 != NULL && rbw->cache_offset_map)
	{
		ob_index1 = rbw->cache_offset_map[mi1->linear_index];
		ob1 = rbw->objects[ob_index1];
	}
	else
	{
		ob1 = blenderOb1;
		ob_index1 = -1;
	}

	if (mi2 != NULL && rbw->cache_offset_map)
	{
		ob_index2 = rbw->cache_offset_map[mi2->linear_index];
		ob2 = rbw->objects[ob_index2];
	}
	else
	{
		ob2 = blenderOb2;
		ob_index2 = -1;
	}

	if ((!ob1 && ob_index1 == -1) || (!ob2 && ob_index2 == -1))
		return false;

	if ((mi1 != NULL) && (mi2 != NULL) && ob_index1 != -1 && ob_index2 != -1) {
		validOb = (ob_index1 != ob_index2 && colgroup_check(ob1->rigidbody_object->col_groups, ob2->rigidbody_object->col_groups) &&
				  ((mi1->rigidbody->flag & RBO_FLAG_KINEMATIC) || (mi2->rigidbody->flag & RBO_FLAG_KINEMATIC)) &&
		          ((mi1->rigidbody->type == RBO_TYPE_ACTIVE) && (mi2->rigidbody->type == RBO_TYPE_ACTIVE)));
	}
	else if ((mi1 == NULL) && (mi2 != NULL)) {
		validOb = (colgroup_check(ob1->rigidbody_object->col_groups, ob2->rigidbody_object->col_groups) &&
		          ((ob1->rigidbody_object->flag & RBO_FLAG_KINEMATIC) || (mi2->rigidbody->flag & RBO_FLAG_KINEMATIC)) &&
		          ((ob1->rigidbody_object->type == RBO_TYPE_ACTIVE) && (mi2->rigidbody->type == RBO_TYPE_ACTIVE)));
	}
	else if ((mi1 != NULL) && (mi2 == NULL)) {
		validOb = (colgroup_check(ob1->rigidbody_object->col_groups, ob2->rigidbody_object->col_groups) &&
		          ((mi1->rigidbody->flag & RBO_FLAG_KINEMATIC) || (ob2->rigidbody_object->flag & RBO_FLAG_KINEMATIC)) &&
		          ((mi1->rigidbody->type == RBO_TYPE_ACTIVE) && (ob2->rigidbody_object->type == RBO_TYPE_ACTIVE)));
	}
	else
	{
		validOb = (colgroup_check(ob1->rigidbody_object->col_groups, ob2->rigidbody_object->col_groups) &&
		          ((ob1->rigidbody_object->flag & RBO_FLAG_KINEMATIC) || (ob2->rigidbody_object->flag & RBO_FLAG_KINEMATIC)) &&
		          ((ob1->rigidbody_object->type == RBO_TYPE_ACTIVE) && (ob2->rigidbody_object->type == RBO_TYPE_ACTIVE)));
	}

	if (validOb || ((colgroup_check(ob1->rigidbody_object->col_groups, ob2->rigidbody_object->col_groups) &&
	               ((ob1->rigidbody_object->flag & RBO_FLAG_CONSTRAINT_DISSOLVE) ||
	                (ob2->rigidbody_object->flag & RBO_FLAG_CONSTRAINT_DISSOLVE) ||
                    (ob1->rigidbody_object->flag & RBO_FLAG_ANTI_TRIGGER) ||
	                (ob2->rigidbody_object->flag & RBO_FLAG_ANTI_TRIGGER)))))
	{
		//override for 2 regular rigidbodies to enable ghost trigger functionality; else bullet wont call this again here with "activate == true"


		if (ob1->rigidbody_object->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION)
		{
			bool override = activate || (ob2->rigidbody_object->flag & RBO_FLAG_ANTI_TRIGGER);
			check_activate = do_activate(ob1, ob2, mi1, rbw, mi2, override);
		}

		if (ob2->rigidbody_object->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION)
		{
			bool override = activate || (ob1->rigidbody_object->flag & RBO_FLAG_ANTI_TRIGGER);
			check_activate = do_activate(ob2, ob1, mi2, rbw, mi1, override);
		}
	}

	//if ghost is involved, and dynafrac trigger is enabled, try to call check_fracture manually here, without forces and with centroid as contact point... Update: should have real contact point now from bullet
	//fake_dynamic_collide(ob1, ob2, mi1, mi2, rbw);
	//fake_dynamic_collide(ob2, ob1, mi2, mi1, rbw);

	validOb = (check_colgroup_ghost(ob1, ob2) && ((check_constraint_island(fmd1, mi1, mi2) &&
		          check_constraint_island(fmd2, mi2, mi1)) || (ob1 != ob2)));

	//return activate ? validOb : check_activate || validOb;
	return validOb;
}

static bool can_break(Object* collider, Object* ob, bool limit)
{
	//allow limit impact only on initial shard and 1st level shards ?
	if (collider && collider->rigidbody_object && (collider->rigidbody_object->flag & RBO_FLAG_DYNAMIC_TRIGGER))
	{
		if (limit && (collider == ob)) {
			return false;
		}

		//dont allow limit impact with ground
		if (collider->rigidbody_object->type == RBO_TYPE_PASSIVE) {
			return false;
		}

		return true;
	}

	return !limit;
}

static Shard* findShard(FractureModifierData *fmd, int id)
{
	Shard *t = fmd->frac_mesh->shard_map.first;
	Shard *s = NULL;

	while (t)
	{
		if (t->shard_id == id && t->flag & SHARD_INTACT)
		{
			//printf("FOUND: %d\n", id);
			s = t;
			break;
		}
		t = t->next;
	}

	return s;
}

static MeshIsland* findMeshIsland(FractureModifierData *fmd, int id)
{
	MeshIsland *mi = fmd->meshIslands.first;

	while (mi)
	{
		if (mi->rigidbody->meshisland_index == id)
		{
			return mi;
		}
		mi = mi->next;
	}

	return NULL;
}

static bool check_shard_size(FractureModifierData *fmd, int id)
{
	FractureID *fid;
	float size = fmd->dynamic_min_size, diff[3];
	Shard *s = NULL;

	s = findShard(fmd, id);

	if (s == NULL)
	{
		return false;
	}

	BKE_shard_calc_minmax(s);

	sub_v3_v3v3(diff, s->max, s->min);
	if (diff[max_axis_v3(diff)] < size)
	{
		return false;
	}

	for (fid = fmd->fracture_ids.first; fid; fid = fid->next)
	{
		if (fid->shardID == id)
		{
			return false;
		}
	}

	printf("FRACTURE : %d\n", id);

	return true;
}

static bool check_constraints(FractureModifierData *fmd, MeshIsland *mi, RigidBodyWorld *rbw) {
	//count broken constraints
	RigidBodyShardCon *con;
	GroupObject* go;
	int i = 0, broken = 0;
	float percentage;

	if (rbw && rbw->constraints) {
		//delete outer constraints here, to be sure
		for (go = rbw->constraints->gobject.first; go; go = go->next) {
			RigidBodyCon *rbc = go->ob->rigidbody_constraint;
			FractureModifierData *fmd1 = (FractureModifierData*)modifiers_findByType(rbc->ob1, eModifierType_Fracture);
			FractureModifierData *fmd2 = (FractureModifierData*)modifiers_findByType(rbc->ob2, eModifierType_Fracture);

			if ((rbc && rbc->physics_constraint && rbw && rbw->physics_world) && ((fmd1 == fmd) || (fmd2 == fmd))) {
				RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
				RB_constraint_delete(rbc->physics_constraint);
				rbc->physics_constraint = NULL;
			}
		}
	}

	if (mi->participating_constraint_count == 0) {
		return true;
	}

	for (i = 0; i < mi->participating_constraint_count; i++) {
		con = mi->participating_constraints[i];
		if (con->physics_constraint && !RB_constraint_is_enabled(con->physics_constraint)) {
			broken++;
		}
	}

	percentage = (float)broken / (float)mi->participating_constraint_count;

	if ((percentage * 100) >= fmd->dynamic_percentage) {
		for (i = 0; i < mi->participating_constraint_count; i++) {
			con = mi->participating_constraints[i];
			if (con->physics_constraint) {
				RB_constraint_set_enabled(con->physics_constraint, false);
			}
		}

		return true;
	}

	return false;
}



static void check_fracture(rbContactPoint* cp, RigidBodyWorld *rbw, Object *obA, Object *obB)
{
	int linear_index1, linear_index2;
	Object* ob1 = NULL, *ob2 = NULL;
	int ob_index1, ob_index2;
	FractureModifierData *fmd1, *fmd2;
	float force;

	//printf("checkfracture\n");

	if (cp == NULL)
		return;

	force = cp->contact_force;

	linear_index1 = cp->contact_body_indexA;
	linear_index2 = cp->contact_body_indexB;

	if (rbw == NULL)
	{
		return;
	}

	if (linear_index2 > -1 && linear_index2 < rbw->numbodies)
	{
		ob_index2 = rbw->cache_offset_map[linear_index2];
		ob2 = rbw->objects[ob_index2];
	}
	else if (obB) {
		ob2 = obB;
	}

	if (linear_index1 > -1 && linear_index1 < rbw->numbodies)
	{
		ob_index1 = rbw->cache_offset_map[linear_index1];
		ob1 = rbw->objects[ob_index1];
		fmd1 = (FractureModifierData*)modifiers_findByType(ob1, eModifierType_Fracture);

		if (fmd1 && fmd1->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			if (fmd1->current_shard_entry && fmd1->current_shard_entry->is_new)
			{
				RigidBodyOb *rbo = rbw->cache_index_map[linear_index1];
				int id = rbo->meshisland_index;
				MeshIsland* mi = findMeshIsland(fmd1, id);
				Shard *s = findShard(fmd1, mi->id);
				if (mi->rigidbody->mass > 0) {
					force = force / mi->rigidbody->mass;
				}

				//printf("FORCE1:%f\n",force);
				bool canbreak = (force > fmd1->dynamic_force) || ((fmd1->limit_impact || obB) && can_break(ob2, ob1, fmd1->limit_impact));

				if (canbreak && check_constraints(fmd1, mi, rbw))
				{
					if (s) {
						float size[3] =  {1.0f, 1.0f, 1.0f};

						if (ob1 == ob2 || (ob2 && ob2->rigidbody_object && ob1->rigidbody_object->type == RBO_TYPE_PASSIVE)) {
							//todo calculate shard...
							size[0] = size[1] = size[2] = -1.0f;
						}
						else if (ob2) {
							BKE_object_dimensions_get(ob2, size);
						}

						copy_v3_v3(s->impact_loc, cp->contact_pos_world_onA);
						copy_v3_v3(s->impact_size, size);
					}
					/*only fracture on new entries, this is necessary because after loading a file
					 *the pointcache thinks it is empty and a fracture is attempted ! */
					if (check_shard_size(fmd1, mi->id))
					{
						FractureID* fid1 = MEM_mallocN(sizeof(FractureID), "contact_callback_fractureid1");
						fid1->shardID = mi->id;
						BLI_addtail(&fmd1->fracture_ids, fid1);
						fmd1->update_dynamic = true;
					}
				}
			}
		}
	}
	else if (obA) {
		ob1 = obA;
	}

	if (linear_index2 > -1 && linear_index2 < rbw->numbodies)
	{
		//ob_index2 = rbw->cache_offset_map[linear_index2];
		//ob2 = rbw->objects[ob_index2];
		fmd2 = (FractureModifierData*)modifiers_findByType(ob2, eModifierType_Fracture);

		if (fmd2 && fmd2->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			if (fmd2->current_shard_entry && fmd2->current_shard_entry->is_new)
			{
				RigidBodyOb *rbo = rbw->cache_index_map[linear_index2];
				int id = rbo->meshisland_index;
				MeshIsland* mi = findMeshIsland(fmd2, id);
				Shard *s = findShard(fmd2, mi->id);

				if (mi->rigidbody->mass > 0) {
					force = force / mi->rigidbody->mass;
				}

				//printf("FORCE2:%f\n",force);
				bool canbreak = (force > fmd2->dynamic_force) || ((fmd2->limit_impact || obA) && can_break(ob1, ob2, fmd2->limit_impact));

				if (canbreak && check_constraints(fmd2, mi, rbw))
				{
					if (s) {
						float size[3] = {1.0f, 1.0f, 1.0f};

						if (ob1 == ob2 || (ob1 && ob1->rigidbody_object && ob1->rigidbody_object->type == RBO_TYPE_PASSIVE)) {
							size[0] = size[1] = size[2] = -1.0f;
						}
						else if (ob1) {
							BKE_object_dimensions_get(ob1, size);
						}
						copy_v3_v3(s->impact_loc, cp->contact_pos_world_onB);
						copy_v3_v3(s->impact_size, size);
					}

					if (check_shard_size(fmd2, mi->id))
					{
						FractureID* fid2 = MEM_mallocN(sizeof(FractureID), "contact_callback_fractureid2");
						fid2->shardID = mi->id;
						BLI_addtail(&fmd2->fracture_ids, fid2);
						fmd2->update_dynamic = true;
					}
				}
			}
		}
	}

	cp = NULL;
}

static ThreadMutex acceleration_lock = BLI_MUTEX_INITIALIZER;
static void contactCallback(rbContactPoint* cp, void* world)
{
	RigidBodyWorld *rbw = (RigidBodyWorld*)world;

	BLI_mutex_lock(&acceleration_lock);
	check_fracture(cp, rbw, NULL, NULL);
	BLI_mutex_unlock(&acceleration_lock);
}

static void idCallback(void *world, void* island, int* objectId, int* islandId)
{
	MeshIsland *mi = (MeshIsland*)island;
	RigidBodyWorld *rbw = (RigidBodyWorld*)world;

	*objectId = -1;
	*islandId = -1;

	if (mi)
	{
		*objectId = rbw->cache_offset_map[mi->linear_index];
		*islandId = mi->id;
	}
}

static void tickCallback(float timestep, void *scene)
{
	Scene* sce = (Scene*)scene;
	RigidBodyWorld *rbw = sce->rigidbody_world;
	rbw->internal_tick = timestep;

	BLI_callback_exec(G.main, &sce->id, BLI_CB_EVT_BULLET_TICK);
}

static void rigidbody_passive_fake_hook(MeshIsland *mi, float co[3])
{
	//no reshape necessary as vertcount didnt change, but update rbo->pos / orn ? according to change of 1st vertex
	//fake hook system
	if (mi->rigidbody->type == RBO_TYPE_PASSIVE &&
	    mi->rigidbody->physics_object && !(mi->rigidbody->flag & RBO_FLAG_KINEMATIC))
	{
		float oldloc[3], loc[3], diff[3], pos[3];
		oldloc[0] = mi->vertco[0];
		oldloc[1] = mi->vertco[1];
		oldloc[2] = mi->vertco[2];

		//this location comes from the final DM, which might be changed by hook modifiers for example
		//XXX TODO maybe need a proper switch for this behavior, too
		copy_v3_v3(loc, co);
		sub_v3_v3v3(diff, oldloc, loc);
		//sub_v3_v3(diff, mi->centroid);

		//RB_body_get_position(mi->rigidbody->physics_object, pos);
		copy_v3_v3(pos, mi->rigidbody->pos);
		//print_v3("Pos:", pos);
		//print_v3("Diff", diff);

		sub_v3_v3(pos, diff);
		RB_body_set_kinematic_state(mi->rigidbody->physics_object, true);

		//XXX TODO how to handle rotation properly ? and omit if kinematic, else it will interfere
		//copy_v3_v3(mi->rigidbody->pos, pos);
		RB_body_set_loc_rot(mi->rigidbody->physics_object, pos, mi->rigidbody->orn);
		//BKE_rigidbody_update_cell(mi, ob, pos, mi->rigidbody->orn, fmd, -1);
	}
}

static void validateShard(RigidBodyWorld *rbw, MeshIsland *mi, Object *ob, int rebuild, int transfer_speed, float size[3])
{
	if (mi == NULL || mi->rigidbody == NULL) {
		return;
	}

	if (rebuild /*|| (mi->rigidbody->flag & RBO_FLAG_KINEMATIC_REBUILD)*/) {
		/* World has been rebuilt so rebuild object */
		BKE_rigidbody_validate_sim_shard(rbw, mi, ob, true, transfer_speed, size);
	}
	else if (mi->rigidbody->flag & RBO_FLAG_NEEDS_VALIDATE) {
		BKE_rigidbody_validate_sim_shard(rbw, mi, ob, false, transfer_speed, size);
	}
	/* refresh shape... */
	if (mi->rigidbody->physics_object && (mi->rigidbody->flag & RBO_FLAG_NEEDS_RESHAPE)) {
		/* mesh/shape data changed, so force shape refresh */
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
		/* now tell RB sim about it */
		// XXX: we assume that this can only get applied for active/passive shapes that will be included as rigidbodies
		RB_body_set_collision_shape(mi->rigidbody->physics_object, mi->rigidbody->physics_shape);
	}
	mi->rigidbody->flag &= ~(RBO_FLAG_NEEDS_VALIDATE | RBO_FLAG_NEEDS_RESHAPE);
}

static void activateCluster(MeshIsland *mi, int particle_index, RigidBodyWorld *rbw, Object *ob) {
	RigidBodyShardCon *con;
	int i = 0;
	for (i = 0; i < mi->participating_constraint_count; i++)
	{
		con = mi->participating_constraints[i];
		if (con->physics_constraint && con->mi1->particle_index == particle_index) /*|| particle_index == -1)*/
		{
			if (con->mi1->rigidbody->flag & RBO_FLAG_KINEMATIC) {
				activateRigidbody(con->mi1->rigidbody, rbw, con->mi1, ob);
				//activateCluster(con->mi1, particle_index, rbw, ob);
			}
		}

		if (con->physics_constraint && con->mi2->particle_index == particle_index) /*|| particle_index == -1)*/
		{
			if (con->mi2->rigidbody->flag & RBO_FLAG_KINEMATIC) {
				activateRigidbody(con->mi2->rigidbody, rbw, con->mi2, ob);
				//activateCluster(con->mi2, particle_index, rbw, ob);
			}
		}
	}
}

static void set_constraint_index(FractureModifierData *fmd, RigidBodyShardCon *con)
{
	if (con->physics_constraint)
	{
		if (!RB_constraint_is_enabled(con->physics_constraint))
		{
			fmd->constraint_island_count++;
			con->mi1->constraint_index = fmd->constraint_island_count;

			fmd->constraint_island_count++;
			con->mi2->constraint_index = fmd->constraint_island_count;
		}
		else
		{
			con->mi1->constraint_index = fmd->constraint_island_count;
			con->mi2->constraint_index = fmd->constraint_island_count;
		}
	}
}

static void handle_breaking_percentage(FractureModifierData* fmd, Object *ob, MeshIsland *mi, RigidBodyWorld *rbw, int breaking_percentage)
{
	int broken_cons = 0, cons = 0, i = 0, cluster_cons = 0, broken_cluster_cons = 0;
	RigidBodyShardCon *con;

	cons = mi->participating_constraint_count;
	/* calc ratio of broken cons here, per MeshIsland and flag the rest to be broken too*/
	for (i = 0; i < cons; i++) {
		con = mi->participating_constraints[i];
		if (con) {
			if (fmd->cluster_breaking_percentage > 0)
			{
				/*only count as broken if between clusters!*/
				if (con->mi1->particle_index != con->mi2->particle_index)
				{
					cluster_cons++;

					if (con->physics_constraint)
					{
						if (!RB_constraint_is_enabled(con->physics_constraint)) {
							broken_cluster_cons++;
						}
					}
				}
			}

			if (con->physics_constraint && !RB_constraint_is_enabled(con->physics_constraint)) {
				broken_cons++;
			}
		}
	}

	if (cluster_cons > 0) {
		if ((float)broken_cluster_cons / (float)cluster_cons * 100 >= fmd->cluster_breaking_percentage) {
			for (i = 0; i < cons; i++) {
				con = mi->participating_constraints[i];
				if (con && con->mi1->particle_index != con->mi2->particle_index) {
					if (fmd->use_breaking)
					{
						if (con->physics_constraint) {

							RB_constraint_set_enabled(con->physics_constraint, false);
							/*if (con->mi1->rigidbody->flag & RBO_FLAG_KINEMATIC ||
							    con->mi2->rigidbody->flag & RBO_FLAG_KINEMATIC ) */
							if (fmd->activate_broken)
							{
								activateRigidbody(con->mi1->rigidbody, rbw, con->mi1, ob);
								activateCluster(con->mi1, con->mi1->particle_index, rbw, ob);

								activateRigidbody(con->mi2->rigidbody, rbw, con->mi2, ob);
								activateCluster(con->mi2, con->mi2->particle_index, rbw, ob);
							}
						}
					}
				}
			}
		}
	}

	if (cons > 0 && breaking_percentage > 0) {
		if ((float)broken_cons / (float)cons * 100 >= breaking_percentage) {
			/* break all cons if over percentage */
			for (i = 0; i < cons; i++) {
				con = mi->participating_constraints[i];
				if (con && fmd->use_breaking)
				{
					if (con->physics_constraint) {
						RB_constraint_set_enabled(con->physics_constraint, false);
						if (fmd->activate_broken)
						{
							activateRigidbody(con->mi1->rigidbody, rbw, con->mi1, ob);
							activateRigidbody(con->mi2->rigidbody, rbw, con->mi2, ob);
						}
					}
				}
			}
		}
	}
}

static void test_deactivate_rigidbody(RigidBodyOb *rbo, MeshIsland* mi)
{
	//make kinematic again (un-trigger)
	//printf("Untrigger\n");
	//if (rbo->physics_object)

	if (rbo->physics_object && ((rbo->flag & RBO_FLAG_KINEMATIC) == 0)) {
		float lin_vel[3], ang_vel[3];

		RB_body_get_linear_velocity(rbo->physics_object, lin_vel);
		RB_body_get_angular_velocity(rbo->physics_object, ang_vel);

		if (((len_squared_v3(lin_vel) < (rbo->lin_sleep_thresh * rbo->lin_sleep_thresh))) ||
		   ((len_squared_v3(ang_vel) < (rbo->ang_sleep_thresh * rbo->ang_sleep_thresh))))
		{
			//RB_body_set_kinematic_state(rbo->physics_object, true);
			//RB_body_set_mass(rbo->physics_object, 0.0f);
			//rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
			//RB_body_deactivate(rbo->physics_object);

			int i = 0;

			rbo->flag |= RBO_FLAG_KINEMATIC;
			rbo->flag |= RBO_FLAG_KINEMATIC_REBUILD;
			rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;

			if ((mi != NULL) && (rbo->flag & RBO_FLAG_PROPAGATE_TRIGGER))
			{
				for (i = 0; i < mi->participating_constraint_count; i++)
				{
					RigidBodyShardCon *con;
					con = mi->participating_constraints[i];

					if (con && con->physics_constraint && RB_constraint_is_enabled(con->physics_constraint))
					{
						RigidBodyOb *rb1 = con->mi1->rigidbody;
						RigidBodyOb *rb2 = con->mi2->rigidbody;

						RB_constraint_set_enabled(con->physics_constraint, false);

						rb1->flag |= RBO_FLAG_KINEMATIC;
						rb1->flag |= RBO_FLAG_KINEMATIC_REBUILD;
						rb1->flag |= RBO_FLAG_NEEDS_VALIDATE;

						rb2->flag |= RBO_FLAG_KINEMATIC;
						rb2->flag |= RBO_FLAG_KINEMATIC_REBUILD;
						rb2->flag |= RBO_FLAG_NEEDS_VALIDATE;
					}
				}
			}
		}
	}
}

static void deform_constraint(FractureModifierData *fmd, Object *ob, RigidBodyShardCon* rbsc, RigidBodyWorld *rbw)
{
	float thresh = RB_constraint_get_breaking_threshold(rbsc->physics_constraint);
	float weakening = 1.0f - fmd->deform_weakening;

	RB_dworld_remove_constraint(rbw->physics_world, rbsc->physics_constraint);

	BKE_rigidbody_start_dist_angle(rbsc, true, false);
	//rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;
	BKE_rigidbody_validate_sim_shard_constraint(rbw, fmd, ob, rbsc, true);
	//set_constraint_index(fmd, rbsc);

	//thresh = RB_constraint_get_breaking_threshold(rbsc->physics_constraint);
	RB_constraint_set_breaking_threshold(rbsc->physics_constraint, thresh * weakening);

	RB_body_deactivate(rbsc->mi1->rigidbody->physics_object);
	RB_body_deactivate(rbsc->mi2->rigidbody->physics_object);
}

static void handle_deform_angle(FractureModifierData *fmd, Object *ob, RigidBodyShardCon *rbsc, RigidBodyWorld *rbw,
                                float anglediff, float weight, float deform_angle)
{
	if ((fmd->deform_angle > 0 || (fmd->deform_angle_weighted && weight > 0)) &&
		(anglediff > deform_angle))
	{
		/* if we have cluster breaking angle, then only treat equal cluster indexes like the default, else all */
		if ((fmd->cluster_deform_angle > 0 && rbsc->mi1->particle_index == rbsc->mi2->particle_index) ||
			 fmd->cluster_deform_angle == 0)
		{
			deform_constraint(fmd, ob, rbsc, rbw);
		}
	}

	if ((fmd->cluster_deform_angle > 0) && (rbsc->mi1->particle_index != rbsc->mi2->particle_index)
		&& anglediff > fmd->cluster_deform_angle)
	{
		deform_constraint(fmd, ob, rbsc, rbw);
	}
}

static void handle_deform_dist(FractureModifierData *fmd, Object *ob, RigidBodyShardCon *rbsc, RigidBodyWorld *rbw,
                                float distdiff, float weight, float deform_dist)
{
	if ((fmd->deform_distance > 0 || (fmd->deform_angle_weighted && weight > 0)) &&
		(distdiff > deform_dist))
	{
		/* if we have cluster breaking angle, then only treat equal cluster indexes like the default, else all */
		if ((fmd->cluster_deform_distance > 0 && rbsc->mi1->particle_index == rbsc->mi2->particle_index) ||
			 fmd->cluster_deform_distance == 0)
		{
			deform_constraint(fmd, ob, rbsc, rbw);
		}
	}

	if ((fmd->cluster_deform_distance > 0) && (rbsc->mi1->particle_index != rbsc->mi2->particle_index)
		&& distdiff > fmd->cluster_deform_distance)
	{
		deform_constraint(fmd, ob, rbsc, rbw);
	}
}


static void handle_breaking_angle(FractureModifierData *fmd, Object *ob, RigidBodyShardCon *rbsc, RigidBodyWorld *rbw,
                                  float anglediff, float weight, float breaking_angle)
{
	if ((fmd->breaking_angle > 0 || (fmd->breaking_angle_weighted && weight > 0)) &&
		(anglediff > breaking_angle))
	{
		/* if we have cluster breaking angle, then only treat equal cluster indexes like the default, else all */
		if ((fmd->cluster_breaking_angle > 0 && rbsc->mi1->particle_index == rbsc->mi2->particle_index &&
		     rbsc->mi1->particle_index != -1) || fmd->cluster_breaking_angle == 0)
		{
			if (fmd->use_breaking)
			{
				//break constraint
				if (rbsc->physics_constraint) {
					RB_constraint_set_enabled(rbsc->physics_constraint, false);
					if (fmd->activate_broken)
					{
						activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
						activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
					}
				}
			}
		}
	}

	if ((fmd->cluster_breaking_angle > 0) && (rbsc->mi1->particle_index != rbsc->mi2->particle_index)
		&& anglediff > fmd->cluster_breaking_angle)
	{
		if (fmd->use_breaking)
		{
			if (rbsc->physics_constraint) {
				RB_constraint_set_enabled(rbsc->physics_constraint, false);
				if (fmd->activate_broken)
				{
					activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
					activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
				}
			}
		}
	}
}

static void handle_breaking_distance(FractureModifierData *fmd, Object *ob, RigidBodyShardCon *rbsc, RigidBodyWorld *rbw,
                                     float distdiff, float weight, float breaking_distance)
{
	if ((fmd->breaking_distance > 0 || (fmd->breaking_distance_weighted && weight > 0)) &&
		(distdiff > breaking_distance))
	{
		/* if we have cluster breaking distance, then only treat equal cluster indexes like the default, else all */
		if ((fmd->cluster_breaking_distance > 0 && rbsc->mi1->particle_index == rbsc->mi2->particle_index &&
		     rbsc->mi1->particle_index != -1) || fmd->cluster_breaking_distance == 0)
		{
			if (fmd->use_breaking)
			{
				if (rbsc->physics_constraint) {
					RB_constraint_set_enabled(rbsc->physics_constraint, false);
					if (fmd->activate_broken)
					{
						activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
						activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
					}
				}
			}
		}
	}

	if ((fmd->cluster_breaking_distance > 0) && (rbsc->mi1->particle_index != rbsc->mi2->particle_index)
		&& distdiff > fmd->cluster_breaking_distance)
	{
		if (fmd->use_breaking)
		{
			if (rbsc->physics_constraint) {
				RB_constraint_set_enabled(rbsc->physics_constraint, false);
				if (fmd->activate_broken)
				{
					activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
					activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
				}
			}
		}
	}
}

static void enable_plastic(RigidBodyShardCon *rbsc)
{
	if (!(rbsc->flag & RBC_FLAG_PLASTIC_ACTIVE) && rbsc->plastic_dist >= 0.0f && rbsc->plastic_angle >= 0.0f)
	{
		if (rbsc->physics_constraint)
		{
			/* activate only once */
			rbsc->flag |= RBC_FLAG_PLASTIC_ACTIVE;
			rigidbody_set_springs_active(rbsc, true);
			RB_constraint_set_equilibrium_6dof_spring(rbsc->physics_constraint);
			RB_constraint_set_enabled(rbsc->physics_constraint, true);
		}
	}
}

static void handle_plastic_breaking(RigidBodyShardCon *rbsc, RigidBodyWorld* rbw, short laststeps, float lastscale)
{
	float dist, angle, distdiff, anglediff;
	bool exceededAngle = false, exceededDist = false, regularBroken = false;

	/*match breaking threshold according to timescale and steps */
	if (rbsc->physics_constraint)
	{
		float step_ratio = (float)rbw->steps_per_second / (float)laststeps;
		float time_ratio = lastscale / rbw->time_scale;

		/*in case of generic without linear locks, ignore time ratio*/
		if ((rbsc->type == RBC_TYPE_6DOF || rbsc->type == RBC_TYPE_6DOF_SPRING) &&
		    ((rbsc->flag & (RBC_FLAG_USE_LIMIT_LIN_X | RBC_FLAG_USE_LIMIT_LIN_Y | RBC_FLAG_USE_LIMIT_LIN_Z)) == 0))
		{
			time_ratio = 1.0f;
		}

		RB_constraint_set_breaking_threshold(rbsc->physics_constraint, (rbsc->breaking_threshold / step_ratio) * time_ratio);
	}

	calc_dist_angle(rbsc, &dist, &angle, true);

	/* note, relative change in percentage is expected, -1 disables */
	distdiff = fabs(1.0f -(rbsc->start_dist/dist));

	/* TODO, ensure rigidbody orn is equal to quaternion of object !!! */
	// The construct "asin(sin(x))" is a triangle function to achieve a seamless rotation loop from input
	anglediff = asin(sin(fabs(rbsc->start_angle - angle) * 0.5f));

	//printf("Dist, Angle: %f %f %f %f %f %f\n", rbsc->start_dist, rbsc->start_angle, dist, angle, distdiff, anglediff);

	exceededAngle = ((rbsc->breaking_angle > 0.0f) && (anglediff > rbsc->breaking_angle));
	exceededDist = ((rbsc->breaking_dist > 0.0f) && (distdiff > (rbsc->breaking_dist + (anglediff / M_PI))));

	if (exceededDist || exceededAngle) //|| regularBroken)
	{
		if (rbsc->type == RBC_TYPE_6DOF_SPRING)
		{
			enable_plastic(rbsc);
		}
		else if (rbsc->physics_constraint)
		{
			/* break regular connections */
			RB_constraint_set_enabled(rbsc->physics_constraint, false);
		}
	}

	exceededAngle = ((rbsc->plastic_angle > 0.0f) && (anglediff > rbsc->plastic_angle));
	exceededDist = ((rbsc->plastic_dist > 0.0f) && (distdiff > (rbsc->plastic_dist + (anglediff / M_PI))));

	/* break plastic connections */
	if ((exceededDist || exceededAngle) /*&& !regularBroken*/)
	{
		if (rbsc->type == RBC_TYPE_6DOF_SPRING && rbsc->flag & RBC_FLAG_PLASTIC_ACTIVE)
		{
			if (rbsc->physics_constraint)
			{
				rigidbody_set_springs_active(rbsc, false);
				RB_constraint_set_enabled(rbsc->physics_constraint, rbsc->flag & RBC_FLAG_ENABLED);
				//rbsc->flag &= ~RBC_FLAG_PLASTIC_ACTIVE;
			}
		}
	}
}

static void handle_regular_breaking(FractureModifierData *fmd, Object *ob, RigidBodyWorld *rbw, RigidBodyShardCon *rbsc, float max_con_mass, bool rebuild)
{
	float weight = MIN2(rbsc->mi1->thresh_weight, rbsc->mi2->thresh_weight);
	float breaking_angle = fmd->breaking_angle_weighted ? fmd->breaking_angle * weight : fmd->breaking_angle;
	float breaking_distance = fmd->breaking_distance_weighted ? fmd->breaking_distance * weight : fmd->breaking_distance;
	float deform_angle = fmd->deform_angle_weighted ? fmd->deform_angle * weight : fmd->deform_angle;
	float deform_distance = fmd->deform_distance_weighted ? fmd->deform_distance * weight : fmd->deform_distance;
	float dist, angle, distdiff, anglediff;

	if ((fmd->use_mass_dependent_thresholds || fmd->use_compounds /*|| fmd->mass_threshold_factor > 0.0f*/)) {
		BKE_rigidbody_calc_threshold(max_con_mass, fmd, rbsc);
	}

	if ((((fmd->breaking_angle) > 0) || (fmd->breaking_angle_weighted && weight > 0) ||
		(fmd->breaking_distance > 0) || (fmd->breaking_distance_weighted && weight > 0) ||
		 (fmd->cluster_breaking_angle > 0 || (fmd->cluster_breaking_distance > 0))) /*&& !rebuild*/)
	{
		calc_dist_angle(rbsc, &dist, &angle, false);
		anglediff = fabs(angle - rbsc->start_angle);
		distdiff = fabs(dist - rbsc->start_dist);

		/* handle breaking */
		handle_breaking_angle(fmd, ob, rbsc, rbw, anglediff, weight, breaking_angle);
		handle_breaking_distance(fmd, ob, rbsc, rbw, distdiff, weight, breaking_distance);
	}

	if ((((fmd->deform_angle) > 0) || (fmd->deform_angle_weighted && weight > 0) ||
		(fmd->deform_distance > 0) || (fmd->deform_distance_weighted && weight > 0) ||
		 (fmd->cluster_deform_angle > 0 || (fmd->cluster_deform_distance > 0))) /*&& !rebuild*/)
	{
		if (rbsc->physics_constraint && RB_constraint_is_enabled(rbsc->physics_constraint))
		{
			calc_dist_angle(rbsc, &dist, &angle, false);
			anglediff = fabs(angle - rbsc->start_angle_deform);
			distdiff = fabs(dist - rbsc->start_dist_deform);

			/* handle deform */
			handle_deform_angle(fmd, ob, rbsc, rbw, anglediff, weight, deform_angle);
			handle_deform_dist(fmd, ob, rbsc, rbw, distdiff, weight, deform_distance);
		}
	}
}

static void handle_solver_iterations(RigidBodyWorld *rbw, FractureModifierData *fmd, RigidBodyShardCon *rbsc)
{
	int iterations;

	if (fmd->solver_iterations_override == 0) {
		iterations = rbw->num_solver_iterations;
	}
	else {
		if (rbsc && rbsc->mi1 && rbsc->mi2)
		{
			if ((rbsc->mi1->particle_index != -1) && (rbsc->mi1->particle_index == rbsc->mi2->particle_index)) {
				iterations = fmd->cluster_solver_iterations_override;
			}
			else {
				iterations = fmd->solver_iterations_override;
			}
		}
		else {
			iterations = rbw->num_solver_iterations;
		}
	}

	/* dont automatically enable in External mode */
	if ((iterations > 0) && (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL)) {
		rbsc->flag |= RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS;
		rbsc->num_solver_iterations = iterations;
	}
}

static bool do_update_modifier(Scene* scene, Object* ob, RigidBodyWorld *rbw, bool rebuild)
{
	ModifierData *md = NULL;
	FractureModifierData *fmd = NULL;
	MeshIsland *mi;
	RigidBodyShardCon *rbsc;
	short laststeps = rbw->steps_per_second;
	float lastscale = rbw->time_scale;
	int i = 0;


	/* check for fractured objects which want to participate first, then handle other normal objects*/
	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_Fracture) {
			fmd = (FractureModifierData *)md;
			break;
		}
	}

	if (isModifierActive(fmd)) {
		float max_con_mass = 0;
		bool is_empty = BLI_listbase_is_empty(&fmd->meshIslands);
		int count = 0, brokencount = 0, plastic = 0;
		float frame = 0;
		float size[3] = {1.0f, 1.0f, 1.0f};
		float bbsize[3];
		float locbb[3];

		//hacky check for ob->derivedFinal validity
		/*if (ob->derivedFinal && ob->derivedFinal->getNumLoopTri(ob->derivedFinal) > 0)
		{
			DM_mesh_boundbox(ob->derivedFinal, locbb, bbsize);
		}
		else
		{
			BKE_mesh_boundbox_calc((Mesh*)ob->data, locbb, bbsize);
		}*/


		if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			int fr = (int)BKE_scene_frame_get(scene);
			if (BKE_lookup_mesh_state(fmd, fr, true))
			{
				BKE_rigidbody_update_ob_array(rbw, false);
			}
		}
		//else
		{
			if (rebuild || is_zero_m4(fmd->passive_parent_mat))
			{
				copy_m4_m4(fmd->passive_parent_mat, ob->obmat);
				//print_m4("Passivemat: \n", fmd->passive_parent_mat);
			}

			//print_m4("Obmat: \n", ob->obmat);
			//print_m4("Passivemat: \n", fmd->passive_parent_mat);
			//printf("WHERE IS CALC\n");
		}

		BKE_object_where_is_calc(scene, ob);
		fmd->constraint_island_count = 1;

		if ((ob->rigidbody_object && (ob->rigidbody_object->flag & RBO_FLAG_KINEMATIC) //&&
		     /*fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED*/)) {

			if (fmd->use_animated_mesh && fmd->anim_mesh_ob)
			{
				BKE_read_animated_loc_rot(fmd, ob, false);
			}
		}

		for (mi = fmd->meshIslands.first; mi; mi = mi->next) {
			if (mi->rigidbody == NULL) {
				continue;
			}
			else {  /* as usual, but for each shard now, and no constraints*/
				/* perform simulation data updates as tagged */
				/* refresh object... */
				int do_rebuild = rebuild;

				DM_mesh_boundbox(mi->physics_mesh, locbb, bbsize);

				if ((rbw->flag & RBW_FLAG_REBUILD_CONSTRAINTS) && fmd->fracture_mode != MOD_FRACTURE_DYNAMIC)
				{
					//reset speeds
					//printf("ZEROIZING speed (shard)\n");
					//zero_v3(mi->rigidbody->lin_vel);
					//zero_v3(mi->rigidbody->ang_vel);
					mi->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
					mi->rigidbody->flag &= ~RBO_FLAG_PROPAGATE_TRIGGER;
				}

				rigidbody_passive_hook(fmd, mi, ob);

				if (fmd->use_breaking && fmd->fracture_mode != MOD_FRACTURE_EXTERNAL)
				{
					float weight = mi->thresh_weight;
					int breaking_percentage = fmd->breaking_percentage_weighted ? (fmd->breaking_percentage * weight) : fmd->breaking_percentage;

					if (fmd->breaking_percentage > 0 || (fmd->breaking_percentage_weighted && weight > 0) || (fmd->cluster_breaking_percentage > 0)) {
						handle_breaking_percentage(fmd, ob, mi, rbw, breaking_percentage);
					}
				}

				validateShard(rbw, is_empty ? NULL : mi, ob, do_rebuild, fmd->fracture_mode == MOD_FRACTURE_DYNAMIC, bbsize);

				mi->constraint_index = mi->id;

			}

			/* update simulation object... */
			if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
			{
				Shard *s = BLI_findlink(&fmd->frac_mesh->shard_map, mi->id);
				if (s)
					copy_v3_v3(size, s->impact_size);
			}

			rigidbody_update_sim_ob(scene, rbw, ob, mi->rigidbody, mi->centroid, mi, size, fmd);
		}

		if (fmd->use_mass_dependent_thresholds) {
			max_con_mass = BKE_rigidbody_calc_max_con_mass(ob);
		}

		frame = BKE_scene_frame_get(scene);

		for (rbsc = fmd->meshConstraints.first; rbsc; rbsc = rbsc->next) {

			//sanity check
			if (!rbsc || !rbsc->mi1 || !rbsc->mi2)
				continue;

			if (rebuild)
			{
				rbsc->start_angle = 0.0f;
				rbsc->start_dist = 0.0f;
			}

			/* needs probably taken into account BEFORE validation already, instead of after it
			 * (causing a delay by one frame with old value and thus different simulation behavior at start) */
			handle_solver_iterations(rbw, fmd, rbsc);

			if (rbsc->physics_constraint && !(RB_constraint_is_enabled(rbsc->physics_constraint)))
			{
				brokencount++;
			}

			if (rbsc->type == RBC_TYPE_6DOF_SPRING && (rbsc->flag & RBC_FLAG_PLASTIC_ACTIVE))
			{
				plastic++;
			}

			count++;


			if (rbsc->physics_constraint && rbw && (rbw->flag & RBW_FLAG_REBUILD_CONSTRAINTS) && !rebuild) {
				//printf("Rebuilding constraints\n");
				if (!fmd->is_dynamic_external) {
					RB_constraint_set_enabled(rbsc->physics_constraint, rbsc->flag & RBC_FLAG_ENABLED);
					rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;
				}

				if (((fmd->fracture_mode == MOD_FRACTURE_EXTERNAL) || ((fmd->fracture_mode == MOD_FRACTURE_DYNAMIC) && fmd->is_dynamic_external))
				    && (rbsc->type == RBC_TYPE_6DOF_SPRING))
				{
					if (rbsc->plastic_angle >= 0.0f || rbsc->plastic_dist >= 0.0f)
					{
						/*reset plastic constraints with immediate activation*/
						if (rbsc->flag & RBC_FLAG_USE_PLASTIC)
						{
							if (!(rbsc->flag & RBC_FLAG_PLASTIC_ACTIVE))
							{
								rbsc->flag |= RBC_FLAG_PLASTIC_ACTIVE;
								rigidbody_set_springs_active(rbsc, true);
								RB_constraint_set_enabled(rbsc->physics_constraint, true);
								if (rbsc->physics_constraint)
									RB_constraint_set_equilibrium_6dof_spring(rbsc->physics_constraint);
							}
						}
						else
						{
							rigidbody_set_springs_active(rbsc, false);
							RB_constraint_set_enabled(rbsc->physics_constraint, false);
							rbsc->flag &= ~RBC_FLAG_PLASTIC_ACTIVE;
						}
					}
				}
			}

			if (rebuild) { // || rbsc->mi1->rigidbody->flag & RBO_FLAG_KINEMATIC_REBUILD ||
				//rbsc->mi2->rigidbody->flag & RBO_FLAG_KINEMATIC_REBUILD) {
				/* World has been rebuilt so rebuild constraint */
				BKE_rigidbody_validate_sim_shard_constraint(rbw, fmd, ob, rbsc, true);
				BKE_rigidbody_start_dist_angle(rbsc, fmd->fracture_mode == MOD_FRACTURE_EXTERNAL ||
				                               (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && fmd->is_dynamic_external), true);
				//TODO ensure evaluation on transform change too
			}

			else if (rbsc->flag & RBC_FLAG_NEEDS_VALIDATE || fmd->fracture_mode == MOD_FRACTURE_DYNAMIC) {
				BKE_rigidbody_validate_sim_shard_constraint(rbw, fmd, ob, rbsc, false);
				//if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
				//	BKE_rigidbody_start_dist_angle(rbsc, true);
			}

			if (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL && !rebuild)
			{
				handle_regular_breaking(fmd, ob, rbw, rbsc, max_con_mass, rebuild);
			}

			if (((fmd->fracture_mode == MOD_FRACTURE_EXTERNAL) || (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && fmd->is_dynamic_external)) &&
			    (rbsc->flag & RBC_FLAG_USE_BREAKING) && !rebuild)
			{
				handle_plastic_breaking(rbsc, rbw, laststeps, lastscale);
			}

			if (rbsc->physics_constraint)
			{
				RigidBodyOb *rbo1 = rbsc->mi1->rigidbody;
				RigidBodyOb *rbo2 = rbsc->mi2->rigidbody;

				if ((rbo1->force_thresh > 0 || rbo2->force_thresh > 0))
				{
					if (RB_constraint_get_applied_impulse(rbsc->physics_constraint) >= rbo1->force_thresh + rbo2->force_thresh)
					{
						//TODO, should be the actual objects, not just "ob"... can differ in case of external constraints...
						RB_constraint_set_enabled(rbsc->physics_constraint, false);
						activateRigidbody(rbo1, rbw, NULL, ob);
						activateRigidbody(rbo2, rbw, NULL, ob);
					}
				}
			}

			set_constraint_index(fmd, rbsc);

			rbsc->flag &= ~RBC_FLAG_NEEDS_VALIDATE;
			lastscale = rbw->time_scale;
			laststeps = rbw->steps_per_second;

			i++;
		}

		printf("Constraints: Frame %d , Total %d,  Intact %d,  Broken %d, Plastic %d\n", (int)frame, count, count-brokencount, brokencount, plastic);
		return true;
	}
	else
	{
		return false;
	}
}

static void rigidbody_passive_hook(FractureModifierData *fmd, MeshIsland *mi, Object* ob)
{
	RigidBodyOb *rbo = mi->rigidbody;
	Scene *scene = fmd->modifier.scene;

	if (rbo->type == RBO_TYPE_PASSIVE && !(rbo->flag & RBO_FLAG_KINEMATIC))
	{
		DerivedMesh *dm = fmd->visible_mesh_cached;
		ModifierData *md;
		bool found = false;

		if (dm)
		{
			int totvert = dm->getNumVerts(dm);
			float acc[3];
			dm->getVertCo(dm, mi->vertex_indices[0], acc);

			for (md = ob->modifiers.first; md; md = md->next)
			{
				if (md->type == eModifierType_Fracture)
				{
					if ((FractureModifierData*)md == fmd)
					{
						found = true;
					}
				}

				//only eval following hookmodifiers, based on our derivedmesh
				if (md->type == eModifierType_Hook && found)
				{
					float (*vertexCos)[3], old[3], diff[3];
					const ModifierTypeInfo *mti = modifierType_getInfo(md->type);
					HookModifierData *hmd = (HookModifierData*)md;

					//skip hook modifiers which were just added and arent valid yet
					if (!hmd->object)
						continue;

					BKE_object_where_is_calc(scene, hmd->object);

					vertexCos = MEM_callocN(sizeof(float) * 3 * totvert, "Vertex Cos");
					dm->getVertCos(dm, vertexCos);
					copy_v3_v3(old, vertexCos[mi->vertex_indices[0]]);

					mti->deformVerts(md, ob, dm, vertexCos, totvert, 0);

					sub_v3_v3v3(diff, vertexCos[mi->vertex_indices[0]], old);
					add_v3_v3(acc, diff);
					MEM_freeN(vertexCos);
				}
			}

			rigidbody_passive_fake_hook(mi, acc);
		}
	}
}

static bool do_sync_modifier(ModifierData *md, Object *ob, RigidBodyWorld *rbw, float ctime)
{
	bool modFound = false;
	FractureModifierData *fmd = NULL;
	MeshIsland *mi;
	bool exploOK = false;
	RigidBodyOb *rbo;
	float size[3] = {1, 1, 1};
	float centr[3];
	float imat[4][4];


	if (md->type == eModifierType_Fracture) {

		fmd = (FractureModifierData *)md;
		bool mode = fmd->fracture_mode == MOD_FRACTURE_EXTERNAL;

		exploOK = !fmd->explo_shared || (fmd->explo_shared && fmd->frac_mesh && fmd->dm) || mode || fmd->is_dynamic_external;

		if (isModifierActive(fmd) && exploOK) {
			modFound = true;

			if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && !(ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ))
			{
				int frame = (int)ctime;

				if (BKE_lookup_mesh_state(fmd, frame, true))
				{
					BKE_rigidbody_update_ob_array(rbw, false);
				}
			}

			invert_m4_m4(imat, fmd->passive_parent_mat);
			invert_m4_m4(ob->imat, ob->obmat);

			if ((ob->rigidbody_object && (ob->rigidbody_object->flag & RBO_FLAG_KINEMATIC)))
			{
				if (fmd->use_animated_mesh && fmd->anim_mesh_ob)
				{
					BKE_read_animated_loc_rot(fmd, ob, false);
				}
			}

			for (mi = fmd->meshIslands.first; mi; mi = mi->next) {

				rbo = mi->rigidbody;
				if (!rbo) {
					continue;
				}

				if (ob->rigidbody_object->type == RBO_TYPE_ACTIVE) {
#if 0
					if (rbo->type == RBO_TYPE_PASSIVE)
					{
						printf("PASSIVE: %s\n", mi->name);
						if (rbo->flag & RBO_FLAG_KINEMATIC) {
							printf("KINEMATIC: %s\n", mi->name);
						}
					}
#endif
					rigidbody_passive_fake_parenting(fmd, ob, rbo, imat);
				}

				/* use rigid body transform after cache start frame if objects is not being transformed */
				if (BKE_rigidbody_check_sim_running(rbw, ctime) && !(ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)) {

					/* keep original transform when the simulation is muted */
					if (rbw->flag & RBW_FLAG_MUTED)
						break;
				}
				/* otherwise set rigid body transform to current obmat*/
				else {

					mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
					mat4_to_size(size, ob->obmat);
					copy_v3_v3(centr, mi->centroid);
					mul_v3_v3(centr, size);
					mul_qt_v3(rbo->orn, centr);
					add_v3_v3(rbo->pos, centr);

					if (mode)
					{
						mul_qt_qtqt(rbo->orn, rbo->orn, mi->rot);
					}

					zero_v3(rbo->lin_vel);
					zero_v3(rbo->ang_vel);

					//reset at start, there no cache read seems to happen
					BKE_update_acceleration_map(fmd, mi, ob, (int)ctime, 0.0f, rbw);
					//BKE_update_velocity_layer(fmd, mi);
				}

				if ((ob->rigidbody_object->type == RBO_TYPE_ACTIVE) && (rbo->type == RBO_TYPE_ACTIVE || rbo->flag & RBO_FLAG_KINEMATIC)) {

					float quat[4];

					if (mode)
					{
						float iquat[4];
						invert_qt_qt(iquat, mi->rot);
						mul_qt_qtqt(quat, rbo->orn, iquat);
					}
					else {
						copy_qt_qt(quat, rbo->orn);
					}

					BKE_rigidbody_update_cell(mi, ob, rbo->pos, quat, fmd, (int)ctime, rbw);
				}
			}

			copy_m4_m4(fmd->passive_parent_mat, ob->obmat);
			//print_m4("Passivemat2: \n", fmd->passive_parent_mat);

			if ((ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) /* || (mode && rbw) */||
				((ob->rigidbody_object) && (ob->rigidbody_object->flag & RBO_FLAG_KINEMATIC)))
			{
				/* update "original" matrix */
				copy_m4_m4(fmd->origmat, ob->obmat);
				if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ && rbw) {
					RigidBodyShardCon *con;

					rbw->flag |= RBW_FLAG_OBJECT_CHANGED;
					BKE_rigidbody_cache_reset(rbw);
					/* re-enable all constraints as well */
					for (con = fmd->meshConstraints.first; con; con = con->next) {
						//con->flag |= RBC_FLAG_ENABLED;
						//con->flag |= RBC_FLAG_NEEDS_VALIDATE;
						if (con->physics_constraint)
							RB_constraint_set_enabled(con->physics_constraint, con->flag & RBC_FLAG_ENABLED);
					}
				}
			}

			if (!is_zero_m4(fmd->origmat) && rbw && !(rbw->flag & RBW_FLAG_OBJECT_CHANGED))
			{
				if (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL /*&& !fmd->is_dynamic_external*/)
				{
					copy_m4_m4(ob->obmat, fmd->origmat);
					zero_m4(fmd->origmat);
				}
			}

			return modFound;
		}
	}

	return modFound;
}

void rigidbody_passive_fake_parenting(FractureModifierData *fmd, Object *ob, RigidBodyOb *rbo, float imat[4][4])
{
	if (rbo->type == RBO_TYPE_PASSIVE && rbo->physics_object)
	{
		//fake parenting, move all passive rbos together with original object in FM case
		float quat[4];
		//float imat[4][4];

		//first get rid of old obmat (=passive_parent_mat) -> do outside loop, expensive function due to profiler
		//invert_m4_m4(imat, fmd->passive_parent_mat);
		mat4_to_quat(quat, imat);
		mul_m4_v3(imat, rbo->pos);
		mul_qt_qtqt(rbo->orn, quat, rbo->orn);

		//then apply new obmat
		mat4_to_quat(quat, ob->obmat);
		mul_m4_v3(ob->obmat, rbo->pos);
		mul_qt_qtqt(rbo->orn, quat, rbo->orn);

		RB_body_set_kinematic_state(rbo->physics_object, true);
		RB_body_set_loc_rot(rbo->physics_object, rbo->pos, rbo->orn);
	}
}

bool BKE_restoreKinematic(RigidBodyWorld *rbw, bool override_bind)
{
	GroupObject *go;
	bool did_it = false;

	/*restore kinematic state of shards if object is kinematic*/
	for (go = rbw->group->gobject.first; go; go = go->next)	{
		bool kinematic = false, triggered = false;

		if ((go->ob) && (go->ob->rigidbody_object)) {
			kinematic = go->ob->rigidbody_object->flag & RBO_FLAG_KINEMATIC;
			triggered = go->ob->rigidbody_object->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION;

			FractureModifierData *fmd = (FractureModifierData*)modifiers_findByType(go->ob, eModifierType_Fracture);
			if (fmd && triggered)
			{
				MeshIsland* mi;
				for (mi = fmd->meshIslands.first; mi; mi = mi->next)
				{
					if (mi->rigidbody)
					{
						mi->rigidbody->flag &= ~RBO_FLAG_KINEMATIC_REBUILD;
						if (kinematic)
						{
							if(!fmd->use_animated_mesh ||
							  (fmd->use_animated_mesh && (override_bind || mi->rigidbody->flag & RBO_FLAG_KINEMATIC_BOUND)))
							{
								if (fmd->use_animated_mesh && override_bind)
								{
									mi->rigidbody->flag |= RBO_FLAG_KINEMATIC_BOUND;
								}

								mi->rigidbody->flag |= RBO_FLAG_KINEMATIC;
							}
						}
						else
						{
							//might happen if being hit by a stop trigger, remove kinematic here in this case
							mi->rigidbody->flag &= ~RBO_FLAG_KINEMATIC;
						}
						mi->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
						did_it = true;
					}
				}
			}
			else if (!fmd && triggered)
			{	/* restore regular triggered objects back to kinematic at all, they very likely were kinematic before...
				 * user has to disable triggered if behavior is not desired */
				go->ob->rigidbody_object->flag &= ~RBO_FLAG_KINEMATIC_REBUILD;
				go->ob->rigidbody_object->flag |= RBO_FLAG_KINEMATIC;
				go->ob->rigidbody_object->flag |= RBO_FLAG_NEEDS_VALIDATE;
				did_it = true;
			}
		}
	}

	return did_it;
}

static ThreadMutex reset_lock = BLI_MUTEX_INITIALIZER;
static void resetDynamic(RigidBodyWorld *rbw, bool do_reset_always, bool override_bind)
{
	GroupObject *go;
	if (!rbw->group)
		return;

	for (go = rbw->group->gobject.first; go; go = go->next)
	{
		Object *ob = go->ob;
		FractureModifierData *fmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);

		if ((fmd && fmd->fracture_mode != MOD_FRACTURE_DYNAMIC && !(rbw->pointcache->flag & PTCACHE_BAKED)))
		{
			//also purge distortion cache here too (called from cache reset
			fmd->distortion_cached = false;
			fmd->refresh_autohide = true;
		}

		if (fmd && fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			//Scene *scene = fmd->modifier.scene;
			MeshIsland *mi;

			if (do_reset_always && (!fmd->use_animated_mesh || (fmd->use_animated_mesh && override_bind)))
			{
				ModifierData *md;
				DerivedMesh *dm = NULL;
				bool found = false;

				if (ob->type == OB_MESH)
				{
					dm = CDDM_from_mesh((Mesh*)ob->data);
				}
				else if (ELEM(ob->type, OB_CURVE, OB_SURF, OB_FONT))
				{
					dm = CDDM_from_curve(ob);
				}

				if (!dm)
				{
					return;
				}

				BLI_mutex_lock(&reset_lock);
				fmd->refresh = true;
				fmd->reset_shards = true;
				fmd->last_frame = INT_MAX;

				//sigh, apply all modifiers before fracture
				for (md = ob->modifiers.first; md; md = md->next)
				{
					const ModifierTypeInfo *mti = modifierType_getInfo(md->type);

					/*if (md == (ModifierData*)fmd)
					{
						BLI_mutex_unlock(&reset_lock);
						found = true;
						break;
					}*/

					if (!found)
					{
						if (mti->deformVerts && (md->mode & (eModifierMode_Realtime | eModifierMode_Render)))
						{
							float (*vertexCos)[3];
							int totvert = dm->getNumVerts(dm);

							vertexCos = MEM_callocN(sizeof(float) * 3 * totvert, "Vertex Cos");
							dm->getVertCos(dm, vertexCos);
							mti->deformVerts(md, ob, dm, vertexCos, totvert, 0);
							CDDM_apply_vert_coords(dm, vertexCos);
							MEM_freeN(vertexCos);
						}

						if (mti->applyModifier && (md->mode & (eModifierMode_Realtime | eModifierMode_Render)))
						{
							DerivedMesh *ndm;

							if (md == (ModifierData*)fmd) {
								BLI_mutex_unlock(&reset_lock);
							}

							ndm = mti->applyModifier(md, ob, dm, 0);
							if (ndm != dm)
							{
								dm->needsFree = 1;
								dm->release(dm);
							}
							dm = ndm;
						}
					}

					if (md == (ModifierData*)fmd)
					{
						found = true;
						break;
					}
				}
				//BLI_mutex_unlock(&reset_lock);

				//DAG_id_tag_update(go->ob, OB_RECALC_ALL);
				//WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, go->ob);
				//WM_main_add_notifier(NC_OBJECT | ND_TRANSFORM, go->ob);
			}

			for (mi = fmd->meshIslands.first; mi; mi = mi->next)
			{
				zero_v3(mi->rigidbody->lin_vel);
				zero_v3(mi->rigidbody->ang_vel);
			}
		}
	}
}

#endif

