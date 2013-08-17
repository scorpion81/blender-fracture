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
#include "BLI_math.h"

#ifdef WITH_BULLET
#  include "RBI_api.h"
#endif

#include "DNA_anim_types.h"
#include "DNA_group_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_object_force.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"

#include "BKE_animsys.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_effect.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_library.h"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_pointcache.h"
#include "BKE_rigidbody.h"
#include "BKE_utildefines.h"
#include "BKE_library.h"
#include "BKE_main.h"
#include "BKE_modifier.h"
#include "BKE_scene.h"

#include "RNA_access.h"
#include "bmesh.h"

/*#ifdef WIN32
    #ifndef NAN
        static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
        #define NAN (*(const float *) __nan)
    #endif
#endif*/

#ifdef WITH_BULLET

static bool isModifierActive(RigidBodyModifierData* rmd) {
	return ((rmd != NULL) && (rmd->modifier.mode & eModifierMode_Realtime) && (rmd->refresh == FALSE));// rmd->modifier.mode & eModifierMode_Render));
}

void calc_dist_angle(RigidBodyShardCon* con, float* dist, float* angle)
{
	float q1[4], q2[4], qdiff[4], axis[3];
	if ((con->mi1->rigidbody == NULL) ||
	   (con->mi2->rigidbody == NULL))
	{
		*dist = 0;
		*angle = 0;
		return;
	}
	
	sub_v3_v3v3(axis, con->mi1->rigidbody->pos, con->mi2->rigidbody->pos);
	*dist = len_v3(axis);
	copy_qt_qt(q1, con->mi1->rigidbody->orn);
	copy_qt_qt(q2, con->mi2->rigidbody->orn);
	invert_qt(q1);
	mul_qt_qtqt(qdiff, q1, q2);
	quat_to_axis_angle(axis, angle, qdiff);
	
	*angle = RAD2DEGF(*angle);
}

void BKE_rigidbody_start_dist_angle(RigidBodyShardCon* con)
{
	//store starting angle and distance per constraint
	float dist, angle;
	calc_dist_angle(con, &dist, &angle);
	con->start_dist = dist;
	con->start_angle = angle;
}

float BKE_rigidbody_calc_max_con_mass(Object* ob)
{
	RigidBodyModifierData *rmd;
	ModifierData *md;
	RigidBodyShardCon *con;
	float max_con_mass = 0, con_mass;

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_RigidBody) {
			rmd = (RigidBodyModifierData*)md;
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

float BKE_rigidbody_calc_min_con_dist(Object* ob)
{
	RigidBodyModifierData *rmd;
	ModifierData *md;
	RigidBodyShardCon *con;
	float min_con_dist = FLT_MAX, con_dist, con_vec[3];

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_RigidBody) {
			rmd = (RigidBodyModifierData*)md;
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


void BKE_rigidbody_calc_threshold(float max_con_mass, float min_con_dist,  RigidBodyModifierData *rmd, RigidBodyShardCon *con) {

	float max_thresh, thresh, con_mass, con_dist, con_vec[3];
	if ((max_con_mass == 0) && (rmd->mass_dependent_thresholds))
	{
		return;
	}

	if ((min_con_dist == FLT_MAX) && (rmd->dist_dependent_thresholds))
	{
		return;
	}

	if ((con->mi1 == NULL) || (con->mi2 == NULL))
	{
		return;
	}

	max_thresh = ((con->mi1->parent_mod == con->mi2->parent_mod) ? con->mi1->parent_mod->breaking_threshold : rmd->group_breaking_threshold);
	if ((con->mi1->rigidbody != NULL) && (con->mi2->rigidbody != NULL)) {
		con_mass = con->mi1->rigidbody->mass + con->mi2->rigidbody->mass;
		sub_v3_v3v3(con_vec, con->mi1->centroid, con->mi2->centroid);
		con_dist = len_v3(con_vec);

		if (rmd->mass_dependent_thresholds && rmd->dist_dependent_thresholds)
		{
			//multiply both factors if desired
			float thresh1 = 0;
			thresh1 = (con_mass / max_con_mass) * max_thresh;
			thresh = (min_con_dist / con_dist) * thresh1;
		}
		else if (rmd->mass_dependent_thresholds)
		{
			thresh = (con_mass / max_con_mass) * max_thresh;
		}
		else if (rmd->dist_dependent_thresholds)
		{
			thresh = (min_con_dist / con_dist) * max_thresh;
		}

		con->breaking_threshold = thresh;
	}
}

static int DM_mesh_minmax(DerivedMesh *dm, float r_min[3], float r_max[3])
{
	MVert* v;
	int i = 0;
	//BMIter iter;
	/*BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
		minmax_v3v3_v3(r_min, r_max, v->co);
	}*/
	for (i = 0; i < dm->numVertData; i++)
	{
		v = CDDM_get_vert(dm, i);
		minmax_v3v3_v3(r_min, r_max, v->co);
	}

	//BM_mesh_normals_update(bm, FALSE);
	return (dm->numVertData != 0);
}

static void DM_mesh_boundbox(DerivedMesh* bm, float r_loc[3], float r_size[3])
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

/* helper function to calculate volume of rigidbody object */
// TODO: allow a parameter to specify method used to calculate this?
float BKE_rigidbody_calc_volume(DerivedMesh *dm, RigidBodyOb *rbo)
{
	//RigidBodyOb *rbo = mi->rigidbody;

	float loc[3]  = {0.0f, 0.0f, 0.0f};
	float size[3]  = {1.0f, 1.0f, 1.0f};
	float radius = 1.0f;
	float height = 1.0f;

	float volume = 0.0f;

	/* if automatically determining dimensions, use the Object's boundbox
	 *	- assume that all quadrics are standing upright on local z-axis
	 *	- assume even distribution of mass around the Object's pivot
	 *	  (i.e. Object pivot is centralised in boundbox)
	 *	- boundbox gives full width
	 */
	// XXX: all dimensions are auto-determined now... later can add stored settings for this
	//BKE_object_dimensions_get(ob, size);
	DM_mesh_boundbox(dm, loc, size); //maybe *2 ??

	if (ELEM3(rbo->shape, RB_SHAPE_CAPSULE, RB_SHAPE_CYLINDER, RB_SHAPE_CONE)) {
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
		case RB_SHAPE_CONVEXH:
		case RB_SHAPE_TRIMESH:
			volume = size[0] * size[1] * size[2];
			if (size[0] == 0)
			{
				volume = size[1] * size[2];
			}
			else if (size[1] == 0)
			{
				volume = size[0] * size[2];
			}
			else if (size[2] == 0)
			{
				volume = size[0] * size[1];
			}
			break;

#if 0 // XXX: not defined yet
		case RB_SHAPE_COMPOUND:
			volume = 0.0f;
			break;
#endif
	}

	/* return the volume calculated */
	return volume;
}

void BKE_rigidbody_calc_shard_mass(Object *ob, MeshIsland* mi)
{
	DerivedMesh *dm_ob, *dm_mi;
	float vol_mi, mass_mi, vol_ob, mass_ob;

	dm_ob = CDDM_from_mesh(ob->data, ob); //ob->derivedFinal;
	vol_ob = BKE_rigidbody_calc_volume(dm_ob, ob->rigidbody_object);
	mass_ob = ob->rigidbody_object->mass;

	if (vol_ob > 0)
	{
		//bm_mi = DM_to_bmesh(mi->physics_mesh);
		dm_mi = mi->physics_mesh;
		vol_mi = BKE_rigidbody_calc_volume(dm_mi, mi->rigidbody);
		//BM_mesh_free(bm_mi);
		mass_mi = (vol_mi / vol_ob) * mass_ob;
		mi->rigidbody->mass = mass_mi;
	}
	
	if (mi->rigidbody->type == RBO_TYPE_ACTIVE)
	{
		if (mi->rigidbody->mass == 0)
			mi->rigidbody->mass = 0.001; //set a minimum mass for active objects 
	}
	/* only active bodies need mass update */
	if ((mi->rigidbody->physics_object) && (mi->rigidbody->type == RBO_TYPE_ACTIVE)) {
		RB_body_set_mass(mi->rigidbody->physics_object, RBO_GET_MASS(mi->rigidbody));
	}
	
	DM_release(dm_ob);
	MEM_freeN(dm_ob);
}


void BKE_rigidbody_update_cell(struct MeshIsland* mi, Object* ob, float loc[3], float rot[4], float cfra, bool baked)
{
	float startco[3], centr[3], size[3];
	int i, j;
	bool invalidData, invalidFrame, invalidBake, invalidBakeFrame;
	
	if ((mi->destruction_frame >= 0) && (cfra > mi->destruction_frame))
	{
		if (mi->rigidbody)
			mi->rigidbody->flag &= ~RBO_FLAG_BAKED_COMPOUND;
	}
	
	invalidData = (loc[0] == FLT_MIN) || (rot[0] == FLT_MIN);
	invalidFrame = (mi->destruction_frame >= 0) && (cfra > mi->destruction_frame) && !baked;
	invalidBake = !(mi->rigidbody->flag & RBO_FLAG_BAKED_COMPOUND) && baked;
	invalidBakeFrame = ((mi->destruction_frame >= 0) && (cfra > mi->destruction_frame)) || (mi->destruction_frame < 0);
	
	if (invalidData || invalidFrame || (invalidBake && invalidBakeFrame))
	{
		//printf("Frames %f %f\n", mi->destruction_frame, cfra);
		//skip dummy cache entries
		printf("SKIPPED: %d %e %f %d\n", mi->linear_index, loc[0], mi->destruction_frame, mi->rigidbody->flag);
		return;
	}

	invert_m4_m4(ob->imat, ob->obmat);
	mat4_to_size(size, ob->obmat);
	
	//update compound children centroids if any
	/*for (i = 0; i < mi->compound_count; i++)
	{
		//hrm, maybe need compound startco as well
		float co[3];
		copy_v3_v3(co, mi->compound_children[i]->start_co);
		mul_v3_v3(co, size);
		mul_qt_v3(rot, co);
		copy_v3_v3(centr, mi->centroid);
		mul_v3_v3(centr, size);
		mul_qt_v3(rot, centr);
		sub_v3_v3(co, centr);
		add_v3_v3(co, loc);
		//mul_m4_v3(ob->imat, co);
		copy_v3_v3(mi->compound_children[i]->centroid, co);
	}*/
	
	for (j = 0; j < mi->vertex_count; j++) {
		// BMVert *vert = BM_vert_at_index(bm, ind);
		struct BMVert* vert = mi->vertices[j];
		if (vert == NULL) break;
		if (vert->co == NULL) break;
		if (mi->parent_mod->refresh == TRUE) break; //if refresh in progress, dont try to access stuff here

		//reset to original coords // stored at fracture time
		startco[0] = mi->vertco[j*3];
		startco[1] = mi->vertco[j*3+1];
		startco[2] = mi->vertco[j*3+2];

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
}

/* ************************************** */
/* Memory Management */

/* Freeing Methods --------------------- */

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
		RB_dworld_delete(rbw->physics_world);
	}
	if (rbw->objects)
		MEM_freeN(rbw->objects);

	if (rbw->cache_index_map)
	{
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

/* Copying Methods --------------------- */

/* These just copy the data, clearing out references to physics objects.
 * Anything that uses them MUST verify that the copied object will
 * be added to relevant groups later...
 */

RigidBodyOb *BKE_rigidbody_copy_object(Object *ob)
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

RigidBodyCon *BKE_rigidbody_copy_constraint(Object *ob)
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

/* preserve relationships between constraints and rigid bodies after duplication */
void BKE_rigidbody_relink_constraint(RigidBodyCon *rbc)
{
	ID_NEW(rbc->ob1);
	ID_NEW(rbc->ob2);
}

/* ************************************** */
/* Setup Utilities - Validate Sim Instances */

/* create collision shape of mesh - convex hull */
/*static rbCollisionShape *rigidbody_get_shape_convexhull_from_mesh(Object *ob, float margin, bool *can_embed)
{
	rbCollisionShape *shape = NULL;
	Mesh *me = NULL;

	if (ob->type == OB_MESH && ob->data) {
		me = ob->data;
	}
	else {
		printf("ERROR: cannot make Convex Hull collision shape for non-Mesh object\n");
	}

	if (me && me->totvert) {
		shape = RB_shape_new_convex_hull((float *)me->mvert, sizeof(MVert), me->totvert, margin, can_embed);
	}
	else {
		printf("ERROR: no vertices to define Convex Hull collision shape with\n");
	}

	return shape;
}*/

/* create collision shape of mesh - convex hull */
static rbCollisionShape *rigidbody_get_shape_convexhull_from_mesh(Mesh* me, float margin, bool *can_embed)
{
	rbCollisionShape *shape = NULL;

	if (me && me->totvert) {
		shape = RB_shape_new_convex_hull((float *)me->mvert, sizeof(MVert), me->totvert, margin, can_embed);
	}
	else {
		printf("ERROR: no vertices to define Convex Hull collision shape with\n");
	}

	return shape;
}


/* create collision shape of mesh - triangulated mesh
 * returns NULL if creation fails.
 */
static rbCollisionShape *rigidbody_get_shape_trimesh_from_mesh_shard(Mesh *me, Object *ob)
{
	rbCollisionShape *shape = NULL;
	DerivedMesh *dm = CDDM_from_mesh(me, NULL);

	MVert *mvert;
	MFace *mface;
	int totvert;
	int totface;

	/* ensure mesh validity, then grab data */
	DM_ensure_tessface(dm);

	mvert   = (dm) ? dm->getVertArray(dm) : NULL;
	totvert = (dm) ? dm->getNumVerts(dm) : 0;
	mface   = (dm) ? dm->getTessFaceArray(dm) : NULL;
	totface = (dm) ? dm->getNumTessFaces(dm) : 0;

	/* sanity checking - potential case when no data will be present */
	if ((totvert == 0) || (totface == 0)) {
		printf("WARNING: no geometry data converted for Mesh Collision Shape (ob = %s)\n", ob->id.name + 2);
	}
	else {
		rbMeshData *mdata;
		int i;

		/* init mesh data for collision shape */
		mdata = RB_trimesh_data_new();

		/* loop over all faces, adding them as triangles to the collision shape
		 * (so for some faces, more than triangle will get added)
		 */
		for (i = 0; (i < totface) && (mface) && (mvert); i++, mface++) {
			/* add first triangle - verts 1,2,3 */
			{
				MVert *va = (IN_RANGE(mface->v1, 0, totvert)) ? (mvert + mface->v1) : (mvert);
				MVert *vb = (IN_RANGE(mface->v2, 0, totvert)) ? (mvert + mface->v2) : (mvert);
				MVert *vc = (IN_RANGE(mface->v3, 0, totvert)) ? (mvert + mface->v3) : (mvert);

				RB_trimesh_add_triangle(mdata, va->co, vb->co, vc->co);
			}

			/* add second triangle if needed - verts 1,3,4 */
			if (mface->v4) {
				MVert *va = (IN_RANGE(mface->v1, 0, totvert)) ? (mvert + mface->v1) : (mvert);
				MVert *vb = (IN_RANGE(mface->v3, 0, totvert)) ? (mvert + mface->v3) : (mvert);
				MVert *vc = (IN_RANGE(mface->v4, 0, totvert)) ? (mvert + mface->v4) : (mvert);

				RB_trimesh_add_triangle(mdata, va->co, vb->co, vc->co);
			}
		}

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
	if (dm) {
		dm->release(dm);
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
		DerivedMesh *dm = CDDM_from_mesh(ob->data, ob);

		MVert *mvert;
		MFace *mface;
		int totvert;
		int totface;

		/* ensure mesh validity, then grab data */
		DM_ensure_tessface(dm);

		mvert   = (dm) ? dm->getVertArray(dm) : NULL;
		totvert = (dm) ? dm->getNumVerts(dm) : 0;
		mface   = (dm) ? dm->getTessFaceArray(dm) : NULL;
		totface = (dm) ? dm->getNumTessFaces(dm) : 0;

		/* sanity checking - potential case when no data will be present */
		if ((totvert == 0) || (totface == 0)) {
			printf("WARNING: no geometry data converted for Mesh Collision Shape (ob = %s)\n", ob->id.name + 2);
		}
		else {
			rbMeshData *mdata;
			int i;

			/* init mesh data for collision shape */
			mdata = RB_trimesh_data_new();

			/* loop over all faces, adding them as triangles to the collision shape
			 * (so for some faces, more than triangle will get added)
			 */
			for (i = 0; (i < totface) && (mface) && (mvert); i++, mface++) {
				/* add first triangle - verts 1,2,3 */
				{
					MVert *va = (IN_RANGE(mface->v1, 0, totvert)) ? (mvert + mface->v1) : (mvert);
					MVert *vb = (IN_RANGE(mface->v2, 0, totvert)) ? (mvert + mface->v2) : (mvert);
					MVert *vc = (IN_RANGE(mface->v3, 0, totvert)) ? (mvert + mface->v3) : (mvert);

					RB_trimesh_add_triangle(mdata, va->co, vb->co, vc->co);
				}

				/* add second triangle if needed - verts 1,3,4 */
				if (mface->v4) {
					MVert *va = (IN_RANGE(mface->v1, 0, totvert)) ? (mvert + mface->v1) : (mvert);
					MVert *vb = (IN_RANGE(mface->v3, 0, totvert)) ? (mvert + mface->v3) : (mvert);
					MVert *vc = (IN_RANGE(mface->v4, 0, totvert)) ? (mvert + mface->v4) : (mvert);

					RB_trimesh_add_triangle(mdata, va->co, vb->co, vc->co);
				}
			}

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
		if (dm) {
			dm->release(dm);
		}
	}
	else {
		printf("ERROR: cannot make Triangular Mesh collision shape for non-Mesh object\n");
	}

	return shape;
}

static rbCollisionShape *rigidbody_get_shape_compound_from_mi(MeshIsland* mi, Object* ob)
{
	int i = 0;
	rbCollisionShape *child;
	rbCollisionShape *compound;
	RigidBodyOb *rbo = mi->rigidbody;
	float size[3];
	
	invert_m4_m4(ob->imat, ob->obmat);
	mat4_to_size(size, ob->imat);
	//copy_v3_v3(size, ob->size);
	
	if (mi->compound_count > 0)
	{
		compound = RB_shape_new_compound();
	}
	else
	{
		//fall back to convex hull if no children available
		bool has_volume, can_embed;
		float hull_margin, loc[3] = {0,0,0}, size[3] = {1,1,1};
		Mesh* me = BKE_mesh_add(G.main, "_mesh_");
		
		DM_to_mesh(mi->physics_mesh, me, ob, 0);
		BKE_mesh_boundbox_calc(me, loc, size);
		has_volume = (MIN3(size[0], size[1], size[2]) > 0.0f);
		
		if (!(rbo->flag & RBO_FLAG_USE_MARGIN) && has_volume) 
			hull_margin = 0.04f;
		
		compound = rigidbody_get_shape_convexhull_from_mesh(me, hull_margin, &can_embed);
			
		if (!(rbo->flag & RBO_FLAG_USE_MARGIN))
			rbo->margin = (can_embed && has_volume) ? 0.04f : 0.0f;  /* RB_TODO ideally we shouldn't directly change the margin here */
		
		BKE_libblock_free_us(&(G.main->mesh), me);
		me = NULL;
		return compound;
	}
	
	for (i = 0; i < mi->compound_count; i++)
	{
		MeshIsland *mi2 = mi->compound_children[i];
		Mesh* me = BKE_mesh_add(G.main, "_mesh_"); 
		bool has_volume, can_embed;
		float hull_margin, loc[3] = {0,0,0}, bbsize[3] = {1,1,1}, rot[4], centr[3];
		
		DM_to_mesh(mi2->physics_mesh, me, ob, 0);
		
		BKE_mesh_boundbox_calc(me, loc, bbsize);
		has_volume = (MIN3(bbsize[0], bbsize[1], bbsize[2]) > 0.0f);
		
		//mat4_to_loc_quat(loc, rot, ob->obmat);
		
		zero_v3(loc); //size only
		unit_qt(rot); //needs to be zeroized, hmm
		
		if (!(rbo->flag & RBO_FLAG_USE_MARGIN) && has_volume) 
			hull_margin = 0.04f;
		
		child = rigidbody_get_shape_convexhull_from_mesh(me, hull_margin, &can_embed);
			
		if (!(rbo->flag & RBO_FLAG_USE_MARGIN))
			rbo->margin = (can_embed && has_volume) ? 0.04f : 0.0f;  /* RB_TODO ideally we shouldn't directly change the margin here */
			
		copy_v3_v3(centr, mi2->centroid);
		//mul_v3_v3(centr, size);
		mul_qt_v3(rot, centr);
		add_v3_v3(loc, centr);
			
		RB_shape_add_compound_child(&compound, child, loc, rot);

		BKE_libblock_free_us(&(G.main->mesh), me);
		me = NULL;
	}
	
	if (mi->compound_count > 0)
	{
		RB_shape_compound_set_scaling(compound, size);
	}
	
	return compound;
}

/* Create new physics sim collision shape for object and store it,
 * or remove the existing one first and replace...
 */
void BKE_rigidbody_validate_sim_shape(Object *ob, short rebuild)
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

	if (ELEM3(rbo->shape, RB_SHAPE_CAPSULE, RB_SHAPE_CYLINDER, RB_SHAPE_CONE)) {
		/* take radius as largest x/y dimension, and height as z-dimension */
		radius = MAX2(size[0], size[1]);
		height = size[2];
	}
	else if (rbo->shape == RB_SHAPE_SPHERE) {
		/* take radius to the the largest dimension to try and encompass everything */
		radius = MAX3(size[0], size[1], size[2]);
	}

	/* create new shape */
	switch (rbo->shape) {
		case RB_SHAPE_BOX:
			new_shape = RB_shape_new_box(size[0], size[1], size[2]);
			break;

		case RB_SHAPE_SPHERE:
			new_shape = RB_shape_new_sphere(radius);
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
		case RB_SHAPE_COMPOUND: //for now... 
			/* try to emged collision margin */
			has_volume = (MIN3(size[0], size[1], size[2]) > 0.0f);
	
			if (!(rbo->flag & RBO_FLAG_USE_MARGIN) && has_volume)
				hull_margin = 0.04f;
			if (ob->type == OB_MESH && ob->data) {
				new_shape = rigidbody_get_shape_convexhull_from_mesh((Mesh*)ob->data, hull_margin, &can_embed);
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
	/* assign new collision shape if creation was successful */
	if (new_shape) {
		if (rbo->physics_shape)
			RB_shape_delete(rbo->physics_shape);
		rbo->physics_shape = new_shape;
		RB_shape_set_margin(rbo->physics_shape, RBO_GET_MARGIN(rbo));
	}
	else { /* otherwise fall back to box shape */
		rbo->shape = RB_SHAPE_BOX;
		BKE_rigidbody_validate_sim_shape(ob, true);
	}
}


/* Create new physics sim collision shape for object and store it,
 * or remove the existing one first and replace...
 */
void BKE_rigidbody_validate_sim_shard_shape(MeshIsland* mi, Object* ob, short rebuild)
{
	RigidBodyOb *rbo = mi->rigidbody;
	rbCollisionShape *new_shape = NULL;
	//BoundBox *bb = NULL;
	float size[3] = {1.0f, 1.0f, 1.0f}, loc[3] = {0.0f, 0.0f, 0.0f};
	float radius = 1.0f;
	float height = 1.0f;
	float capsule_height;
	float hull_margin = 0.0f;
	bool can_embed = true;
	bool has_volume;
	int v;
	Mesh *me;
	
	/* sanity check */
	if (rbo == NULL)
		return;

	/* don't create a new shape if we already have one and don't want to rebuild it */
	if (rbo->physics_shape && !rebuild)
		return;
	
	if (rbo->shape != RB_SHAPE_COMPOUND)
	{
		me = BKE_mesh_add(G.main, "_mesh_"); // TODO need to delete this again
		
		/* if automatically determining dimensions, use the Object's boundbox
		 *	- assume that all quadrics are standing upright on local z-axis
		 *	- assume even distribution of mass around the Object's pivot
		 *	  (i.e. Object pivot is centralized in boundbox)
		 */
		// XXX: all dimensions are auto-determined now... later can add stored settings for this
		/* get object dimensions without scaling */
		//BM_mesh_bm_to_me(mi->physics_mesh, me, FALSE);
		DM_to_mesh(mi->physics_mesh, me, ob, 0);
	
		BKE_mesh_boundbox_calc(me, loc, size);
	
		if (ELEM3(rbo->shape, RB_SHAPE_CAPSULE, RB_SHAPE_CYLINDER, RB_SHAPE_CONE)) {
			/* take radius as largest x/y dimension, and height as z-dimension */
			radius = MAX2(size[0], size[1]);
			height = size[2];
		}
		else if (rbo->shape == RB_SHAPE_SPHERE) {
			/* take radius to the the largest dimension to try and encompass everything */
			radius = MAX3(size[0], size[1], size[2]);
		}
	}
	
	/* create new shape */
	switch (rbo->shape) {
		case RB_SHAPE_BOX:
			new_shape = RB_shape_new_box(size[0], size[1], size[2]);
			break;
	
		case RB_SHAPE_SPHERE:
			new_shape = RB_shape_new_sphere(radius);
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
			new_shape = rigidbody_get_shape_convexhull_from_mesh(me, hull_margin, &can_embed);
			if (!(rbo->flag & RBO_FLAG_USE_MARGIN))
				rbo->margin = (can_embed && has_volume) ? 0.04f : 0.0f;  /* RB_TODO ideally we shouldn't directly change the margin here */
			break;
		case RB_SHAPE_TRIMESH:
			new_shape = rigidbody_get_shape_trimesh_from_mesh_shard(me, ob);
			break;
		case RB_SHAPE_COMPOUND:
			new_shape = rigidbody_get_shape_compound_from_mi(mi, ob);
			break;
	}
	/* assign new collision shape if creation was successful */
	if (new_shape) {
		if (rbo->physics_shape)
			RB_shape_delete(rbo->physics_shape);
		rbo->physics_shape = new_shape;
		RB_shape_set_margin(rbo->physics_shape, RBO_GET_MARGIN(rbo));
	}
	else { /* otherwise fall back to box shape */
		rbo->shape = RB_SHAPE_BOX;
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
	}

	//delete mesh block, bullet shouldnt care about blender blocks
	if (rbo->shape != RB_SHAPE_COMPOUND)
	{
		BKE_libblock_free_us(&(G.main->mesh), me);
		me = NULL;
	}
}


/* --------------------- */

/* Create physics sim representation of shard given RigidBody settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_shard(RigidBodyWorld *rbw, MeshIsland *mi, Object *ob, short rebuild)
{
	RigidBodyOb *rbo = (mi) ? mi->rigidbody : NULL;
	float loc[3];
	float rot[4];
	float centr[3], size[3];

	/* sanity checks:
	 *	- object doesn't have RigidBody info already: then why is it here?
	 */
	if (rbo == NULL)
		return;

	/* make sure collision shape exists */
	/* FIXME we shouldn't always have to rebuild collision shapes when rebuilding objects, but it's needed for constraints to update correctly */
	if (rbo->physics_shape == NULL || rebuild)
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
	
	if (rbo->physics_object) {
		if (rebuild == false)
			RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
	}
	if (!rbo->physics_object || rebuild) {
		/* remove rigid body if it already exists before creating a new one */
		if (rbo->physics_object) {
			RB_body_delete(rbo->physics_object);
		}

		/*copy_v3_v3(centr, mi->centroid);
		mat4_to_loc_quat(loc, rot, ob->obmat); //offset
		mat4_to_size(size, ob->obmat);
		mul_v3_v3(centr, size);
		mul_qt_v3(rot, centr);
		add_v3_v3(loc, centr);*/
		copy_v3_v3(loc, rbo->pos);
		copy_v4_v4(rot, rbo->orn);
		
		rbo->physics_object = RB_body_new(rbo->physics_shape, loc, rot);

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
	}

	if (rbw && rbw->physics_world && rbo->physics_object)
		RB_dworld_add_body(rbw->physics_world, rbo->physics_object, rbo->col_groups, mi);

	rbo->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
}




/* --------------------- */

/* Create physics sim representation of object given RigidBody settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_object(RigidBodyWorld *rbw, Object *ob, short rebuild)
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
		BKE_rigidbody_validate_sim_shape(ob, true);

	if (rbo->physics_object) {
		if (rebuild == false)
			RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
	}
	if (!rbo->physics_object || rebuild) {
		/* remove rigid body if it already exists before creating a new one */
		if (rbo->physics_object) {
			RB_body_delete(rbo->physics_object);
		}

		mat4_to_loc_quat(loc, rot, ob->obmat);

		rbo->physics_object = RB_body_new(rbo->physics_shape, loc, rot);

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
	}

	if (rbw && rbw->physics_world)
		RB_dworld_add_body(rbw->physics_world, rbo->physics_object, rbo->col_groups, NULL);
}

/* --------------------- */

/* Create physics sim representation of constraint given rigid body constraint settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_constraint(RigidBodyWorld *rbw, Object *ob, short rebuild)
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

	if (ELEM4(NULL, rbc->ob1, rbc->ob1->rigidbody_object, rbc->ob2, rbc->ob2->rigidbody_object)) {
		if (rbc->physics_constraint) {
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}
		return;
	}

	if (rbc->physics_constraint) {
		if (rebuild == false)
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
	}
	if (rbc->physics_constraint == NULL || rebuild) {
		rbRigidBody *rb1 = rbc->ob1->rigidbody_object->physics_object;
		rbRigidBody *rb2 = rbc->ob2->rigidbody_object->physics_object;

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

					RB_constraint_set_equilibrium_6dof_spring(rbc->physics_constraint);
				/* fall through */
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
}

/* Create physics sim representation of constraint given rigid body constraint settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_shard_constraint(RigidBodyWorld *rbw, RigidBodyShardCon *rbc, Object *ob, short rebuild)
{
	//RigidBodyShardCon *rbc = (mi) ? mi->rigidbody_constraint : NULL;
	RigidBodyModifierData* rmd = NULL;
	ModifierData* md = NULL;
	float loc[3];
	float rot[4];
	float lin_lower;
	float lin_upper;
	float ang_lower;
	float ang_upper;
	rbRigidBody *rb1;
	rbRigidBody *rb2;
	bool use_deact = false;

	/* sanity checks:
	 *	- object should have a rigid body constraint
	 *  - rigid body constraint should have at least one constrained object
	 */
	if (rbc == NULL) {
		return;
	}

	if (ELEM4(NULL, rbc->mi1, rbc->mi1->rigidbody, rbc->mi2, rbc->mi2->rigidbody)) {
		if (rbc->physics_constraint) {
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}
		return;
	}
	
	/*if ((rbc->mi1->rigidbody->flag & RBO_FLAG_INACTIVE_COMPOUND) || (rbc->mi2->rigidbody->flag & RBO_FLAG_INACTIVE_COMPOUND))
	{
		//disable those constraints....
		if (rbc->physics_constraint) {
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}
		return;
	}*/
	
	rb1 = rbc->mi1->rigidbody->physics_object;
	rb2 = rbc->mi2->rigidbody->physics_object;
	use_deact = ((rbc->mi1->rigidbody->flag & RBO_FLAG_USE_DEACTIVATION) && (rbc->mi2->rigidbody->flag & RBO_FLAG_USE_DEACTIVATION));
	
	
	if (rb1 && rb2 && use_deact) // rbc->physics_constraint && RB_constraint_is_enabled(rbc->physics_constraint))
	{
		//printf("STATE %d %d\n", RB_body_get_activation_state(rb1), RB_body_get_activation_state(rb2));
		if ((RB_body_get_activation_state(rb1) == RBO_STATE_ISLAND_SLEEPING) ||
			(RB_body_get_activation_state(rb2) == RBO_STATE_ISLAND_SLEEPING))
		{
			//deactivate both islands if one is sleeping
			RB_body_deactivate(rb1);
			RB_body_deactivate(rb2);
		}
	}

	if (rbc->physics_constraint) {
		if (rebuild == false)
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
	}
	if (rbc->physics_constraint == NULL || rebuild) {
		//rbRigidBody *rb1 = rbc->mi1->rigidbody->physics_object;
		//rbRigidBody *rb2 = rbc->mi2->rigidbody->physics_object;
		/* remove constraint if it already exists before creating a new one */
		if (rbc->physics_constraint) {
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}

		//mat4_to_loc_quat(loc, rot, ob->obmat);
		//copy_v3_v3(loc, rbc->mi1->rigidbody->pos);
		//add_v3_v3(loc, rbc->mi2->rigidbody->pos);
		//mul_v3_fl(loc, 0.5f);

		//do this for all inner constraints
		copy_v3_v3(loc, rbc->mi1->rigidbody->pos);

		for (md = ob->modifiers.first; md; md = md->next) {
			if (md->type == eModifierType_RigidBody) {
				int index1, index2;
				rmd = (RigidBodyModifierData*)md;

				index1 = BLI_findindex(&rmd->meshIslands, rbc->mi1);
				index2 = BLI_findindex(&rmd->meshIslands, rbc->mi2);

				if ((index1 == -1) || (index2 == -1)) //outer constraint if not both meshislands in same object
				{
					float master[3], slave[3], center[3];
					copy_v3_v3(master, rbw->objects[rbw->cache_offset_map[rbc->mi1->linear_index]]->loc);
					copy_v3_v3(slave, rbw->objects[rbw->cache_offset_map[rbc->mi2->linear_index]]->loc);
					add_v3_v3v3(center, master, slave);
					mul_v3_fl(center, 0.5f);

					if (rmd->outer_constraint_location == MOD_RIGIDBODY_ACTIVE)
					{
						if (rbc->type == RBC_TYPE_FIXED)
						{
							copy_v3_v3(loc, rbc->mi2->rigidbody->pos);
						}
						else
						{
							copy_v3_v3(loc, slave);
						}
					}
					else if (rmd->outer_constraint_location == MOD_RIGIDBODY_SELECTED)
					{
						if (rbc->type == RBC_TYPE_FIXED)
						{
							copy_v3_v3(loc, rbc->mi1->rigidbody->pos);
						}
						else
						{
							copy_v3_v3(loc, master);
						}
					}
					else if (rmd->outer_constraint_location == MOD_RIGIDBODY_CENTER)
					{
						if (rbc->type == RBC_TYPE_FIXED)
						{
							copy_v3_v3(loc, rbc->mi1->rigidbody->pos);
							add_v3_v3(loc, rbc->mi2->rigidbody->pos);
							mul_v3_fl(loc, 0.5f);
						}
						else
						{
							copy_v3_v3(loc, center);
						}
					}
				}
			}
		}

		copy_v4_v4(rot, rbc->mi1->rigidbody->orn);

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

					RB_constraint_set_equilibrium_6dof_spring(rbc->physics_constraint);
				/* fall through */
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
}

bool isDisconnected(MeshIsland *mi)
{
	int cons = 0, broken_cons = 0, i;
	RigidBodyShardCon* con;
	cons = mi->participating_constraint_count;
	//calc ratio of broken cons here, per Mi and flag the rest to be broken too
	for (i = 0; i < cons; i++)
	{
		con = mi->participating_constraints[i];
		if (con && con->physics_constraint)
		{
			if (!RB_constraint_is_enabled(con->physics_constraint))
			{
				broken_cons++;
			}
		}
	}
	
	return (cons == broken_cons) && (cons > 0);
}

//this allows partial object activation, only some shards will be activated, called from bullet(!)
int filterCallback(void* world, void* island1, void* island2) {
	RigidBodyWorld* rbw = (RigidBodyWorld*)world;
	MeshIsland* mi1, *mi2;
	int ob_index1, ob_index2;

	mi1 = (MeshIsland*)island1;
	mi2 = (MeshIsland*)island2;

	if ((mi1 == NULL) || (mi2 == NULL)) {
		return TRUE;
	}
	
	//cache offset map is a dull name for that... 
	ob_index1 = rbw->cache_offset_map[mi1->linear_index];
	ob_index2 = rbw->cache_offset_map[mi2->linear_index];
	
	if ((mi1->compound_count > 0 && mi1->participating_constraint_count > 0) && 
		(mi2->compound_count > 0 && mi2->participating_constraint_count > 0))
	{
		//disallow collision between intact compounds of same object
		return ob_index1 != ob_index2; //FALSE;
	}
	
	if ((mi1->destruction_frame > -1) || (mi2->destruction_frame > -1))
	{
		//disallow collision between destroyed compounds... of same object
		return ob_index1 != ob_index2; //FALSE;
	}
	return TRUE;

	/*ob_index1 = rbw->cache_index_map[mi1->linear_index];
	ob_index2 = rbw->cache_index_map[mi2->linear_index];

	if ((mi1->rigidbody->flag & RBO_FLAG_START_DEACTIVATED) ||
		(mi2->rigidbody->flag & RBO_FLAG_START_DEACTIVATED)) {
		return ob_index1 != ob_index2; //only allow collision between different objects
	}
	else
	{
		return TRUE;
	}*/
}

/* --------------------- */
/* Create physics sim world given RigidBody world settings */
// NOTE: this does NOT update object references that the scene uses, in case those aren't ready yet!
void BKE_rigidbody_validate_sim_world(Scene *scene, RigidBodyWorld *rbw, short rebuild)
{
	/* sanity checks */
	if (rbw == NULL)
		return;

	/* create new sim world */
	if (rebuild || rbw->physics_world == NULL) {
		if (rbw->physics_world)
			RB_dworld_delete(rbw->physics_world);
		rbw->physics_world = RB_dworld_new(scene->physics_settings.gravity, rbw, filterCallback);
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
	rbw->object_changed = FALSE;
	rbw->refresh_modifiers = FALSE;

	rbw->objects = MEM_mallocN(sizeof(Object*), "objects");
	rbw->cache_index_map = MEM_mallocN(sizeof(RigidBodyOb*), "cache_index_map");
	rbw->cache_offset_map = MEM_mallocN(sizeof(int), "cache_offset_map");

	/* return this sim world */
	return rbw;
}

/* Add rigid body settings to the specified shard */
RigidBodyOb *BKE_rigidbody_create_shard(Scene *scene, Object *ob, MeshIsland *mi)
{
	RigidBodyOb *rbo;
	RigidBodyWorld *rbw = BKE_rigidbody_get_world(scene);
	float centr[3], size[3];

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- shard must exist
	 *	- cannot add rigid body if it already exists
	 */
	if (mi == NULL || (mi->rigidbody != NULL))
		return NULL;

	if (ob->type != OB_MESH) {
		return NULL;
	}
	
	if (((Mesh *)ob->data)->totvert == 0) {
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
		ob->rigidbody_object = BKE_rigidbody_create_object(scene, ob, RBO_TYPE_ACTIVE);
	}
	else {
		//ob->rigidbody_object->type = RBO_TYPE_ACTIVE;
		ob->rigidbody_object->flag |= RBO_FLAG_NEEDS_VALIDATE;

		/* add object to rigid body group */
		if (!BKE_group_object_exists(rbw->group, ob))
			BKE_group_object_add(rbw->group, ob, scene, NULL);

		//DAG_id_tag_update(&ob->id, OB_RECALC_OB);
	}

	//since we are always member of an object, dupe its settings,
	/* create new settings data, and link it up */
	rbo = BKE_rigidbody_copy_object(ob);

	/* set initial transform */
	mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
	mat4_to_size(size, ob->obmat);

	//add initial "offset" (centroid), maybe subtract ob->obmat ?? (not sure)
	copy_v3_v3(centr, mi->centroid);
	mul_v3_v3(centr, size);
	mul_qt_v3(rbo->orn, centr);
	add_v3_v3(rbo->pos, centr);

	/* flag cache as outdated */
	//BKE_rigidbody_cache_reset(rbw);

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

	rbwn->pointcache = BKE_ptcache_copy_list(&rbwn->ptcaches, &rbw->ptcaches, FALSE);

	rbwn->objects = NULL;
	rbwn->physics_world = NULL;
	rbwn->numbodies = 0;

	return rbwn;
}

void BKE_rigidbody_world_groups_relink(RigidBodyWorld *rbw)
{
	if (rbw->group && rbw->group->id.newid)
		rbw->group = (Group *)rbw->group->id.newid;
	if (rbw->constraints && rbw->constraints->id.newid)
		rbw->constraints = (Group *)rbw->constraints->id.newid;
	if (rbw->effector_weights->group && rbw->effector_weights->group->id.newid)
		rbw->effector_weights->group = (Group *)rbw->effector_weights->group->id.newid;
}

/* Add rigid body settings to the specified object */
RigidBodyOb *BKE_rigidbody_create_object(Scene *scene, Object *ob, short type)
{
	RigidBodyOb *rbo;
	RigidBodyWorld *rbw = scene->rigidbody_world;

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- object must exist
	 *	- cannot add rigid body if it already exists
	 */
	if (ob == NULL || (ob->rigidbody_object != NULL))
		return NULL;

	/* create new settings data, and link it up */
	rbo = MEM_callocN(sizeof(RigidBodyOb), "RigidBodyOb");

	/* set default settings */
	rbo->type = type;

	rbo->mass = 1.0f;

	rbo->friction = 0.5f; /* best when non-zero. 0.5 is Bullet default */
	rbo->restitution = 0.0f; /* best when zero. 0.0 is Bullet default */

	rbo->margin = 0.04f; /* 0.04 (in meters) is Bullet default */

	rbo->lin_sleep_thresh = 0.4f; /* 0.4 is half of Bullet default */
	rbo->ang_sleep_thresh = 0.5f; /* 0.5 is half of Bullet default */

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

	/* set initial transform */
	mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);

	/* return this object */
	return rbo;
}

/* Add rigid body constraint to the specified object */
RigidBodyCon *BKE_rigidbody_create_constraint(Scene *scene, Object *ob, short type)
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
	rbc->spring_stiffness_x = 10.0f;
	rbc->spring_stiffness_y = 10.0f;
	rbc->spring_stiffness_z = 10.0f;

	rbc->motor_lin_max_impulse = 1.0f;
	rbc->motor_lin_target_velocity = 1.0f;
	rbc->motor_ang_max_impulse = 1.0f;
	rbc->motor_ang_target_velocity = 1.0f;

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);

	/* return this object */
	return rbc;
}

/* Add rigid body constraint to the specified object */
RigidBodyShardCon *BKE_rigidbody_create_shard_constraint(Scene *scene, short type)
{
	RigidBodyShardCon *rbc;
	RigidBodyWorld *rbw = scene->rigidbody_world;

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- object must exist
	 *	- cannot add constraint if it already exists
	 */
	/*if (mi == NULL || (mi->rigidbody_constraint != NULL))
		return NULL;*/

	/* create new settings data, and link it up */
	rbc = MEM_callocN(sizeof(RigidBodyShardCon), "RigidBodyCon");

	/* set default settings */
	rbc->type = type;

	rbc->mi1 = NULL;
	rbc->mi2 = NULL;

	rbc->flag |= RBC_FLAG_ENABLED;
	rbc->flag &=~RBC_FLAG_DISABLE_COLLISIONS;
	rbc->flag |= RBC_FLAG_USE_BREAKING;

	rbc->breaking_threshold = 1.0f; /* no good default here, just use 10 for now */
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
	rbc->spring_stiffness_x = 10.0f;
	rbc->spring_stiffness_y = 10.0f;
	rbc->spring_stiffness_z = 10.0f;

	rbc->motor_lin_max_impulse = 1.0f;
	rbc->motor_lin_target_velocity = 1.0f;
	rbc->motor_ang_max_impulse = 1.0f;
	rbc->motor_ang_target_velocity = 1.0f;

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);

	/* return this object */
	return rbc;
}

/* ************************************** */
/* Utilities API */

/* Get RigidBody world for the given scene, creating one if needed
 * < scene: Scene to find active Rigid Body world for
 */
RigidBodyWorld *BKE_rigidbody_get_world(Scene *scene)
{
	/* sanity check */
	if (scene == NULL)
		return NULL;

	return scene->rigidbody_world;
}

void BKE_rigidbody_remove_shard_con(Scene* scene, RigidBodyShardCon* con)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	if (rbw && rbw->physics_world && con->physics_constraint) {
		RB_dworld_remove_constraint(rbw->physics_world, con->physics_constraint);
		RB_constraint_delete(con->physics_constraint);
		con->physics_constraint = NULL;
	}
}

void BKE_rigidbody_remove_shard(Scene* scene, MeshIsland *mi)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	int i = 0;
	
	if (mi->rigidbody != NULL) {
		
		RigidBodyShardCon *con;
		for (i = 0; i < mi->participating_constraint_count; i++)
		{	
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
		
		//this SHOULD be the correct global index
		if (rbw->cache_index_map != NULL)
			rbw->cache_index_map[mi->linear_index] = NULL;
	}
}

void BKE_rigidbody_remove_object(Scene *scene, Object *ob)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyOb *rbo = ob->rigidbody_object;
	RigidBodyCon *rbc;
	GroupObject *go;
	ModifierData* md;
	RigidBodyModifierData* rmd;
	RigidBodyShardCon* con;
	MeshIsland* mi;
	int i;
	int modFound = FALSE;

	if (rbw) {
		for (md = ob->modifiers.first; md; md = md->next) {

			if (md->type == eModifierType_RigidBody)
			{
				rmd = (RigidBodyModifierData*)md;
				modFound = TRUE;
				for (con = rmd->meshConstraints.first; con; con = con->next) {
					if (rbw && rbw->physics_world && con->physics_constraint) {
						RB_dworld_remove_constraint(rbw->physics_world, con->physics_constraint);
						RB_constraint_delete(con->physics_constraint);
						con->physics_constraint = NULL;
					}
				}

				for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
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
						
						//this SHOULD be the correct global index
						if (rbw->cache_index_map)
							rbw->cache_index_map[mi->linear_index] = NULL;
						MEM_freeN(mi->rigidbody);
						mi->rigidbody = NULL;
					}
				}
			}
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
					
					if (rbo == rbw->cache_index_map[i]){
						rbw->cache_index_map[i] = NULL;
						break;
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

		/* remove object's settings */
		//BKE_rigidbody_free_object(ob);
	}

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);
}

void BKE_rigidbody_remove_constraint(Scene *scene, Object *ob)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyCon *rbc = ob->rigidbody_constraint;

	if (rbw) {
		/* remove from rigidbody world, free object won't do this */
		if (rbw && rbw->physics_world && rbc->physics_constraint)
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
	}
	/* remove object's settings */
	BKE_rigidbody_free_constraint(ob);

	/* flag cache as outdated */
	BKE_rigidbody_cache_reset(rbw);
}

static int rigidbody_count_regular_objects(ListBase obs)
{
	int count = 0;
	struct GroupObject *gob = NULL;
	struct ModifierData *md = NULL;

	for (gob = obs.first; gob; gob = gob->next) {
		count++;
		for (md = gob->ob->modifiers.first; md; md = md->next) {
			if (md->type == eModifierType_RigidBody) {
				count--;
				break;
			}
		}
	}
	return count;
}

static int rigidbody_count_shards(ListBase obs)
{
	int count = 0;
	struct GroupObject *gob = NULL;
	struct ModifierData *md = NULL;
	struct RigidBodyModifierData *rmd = NULL;

	for (gob = obs.first; gob; gob = gob->next) {
		for (md = gob->ob->modifiers.first; md; md = md->next) {
			if (md->type == eModifierType_RigidBody) {
				rmd = (RigidBodyModifierData*)md;
				if (isModifierActive(rmd))
					count += BLI_countlist(&rmd->meshIslands);
			}
		}
	}
	return count;
}

/* ************************************** */
/* Simulation Interface - Bullet */

/* Update object array and rigid body count so they're in sync with the rigid body group */
static void rigidbody_update_ob_array(RigidBodyWorld *rbw)
{
	GroupObject *go;
	ModifierData *md;
	RigidBodyModifierData *rmd;
	MeshIsland *mi;
	int i, j, l, m, n, counter = 0;
	int ismapped = FALSE;
	
	if (rbw->objects == NULL)
	{
		rbw->objects = MEM_callocN(sizeof (Object*), "rbw->objects");
	}
	
	if (rbw->cache_index_map == NULL)
	{
		rbw->cache_index_map = MEM_callocN(sizeof(RigidBodyOb*), "cache_index_map");
	}
	
	if (rbw->cache_offset_map == NULL)
	{
		rbw->cache_offset_map = MEM_callocN(sizeof(int), "cache_offset_map");
	}

	l = BLI_countlist(&rbw->group->gobject); // all objects
	m = rigidbody_count_regular_objects(rbw->group->gobject);
	n = rigidbody_count_shards(rbw->group->gobject);

	if (rbw->numbodies != (m+n)) {
		rbw->numbodies = m+n;
		rbw->objects = MEM_reallocN(rbw->objects, sizeof(Object *) * l);
		rbw->cache_index_map = MEM_reallocN(rbw->cache_index_map, sizeof(RigidBodyOb*) * rbw->numbodies);
		rbw->cache_offset_map = MEM_reallocN(rbw->cache_offset_map, sizeof(int) * rbw->numbodies);
		printf("RigidbodyCount changed: %d\n", rbw->numbodies);
	}
	for (go = rbw->group->gobject.first, i = 0; go; go = go->next, i++) {
		Object *ob = go->ob;
		rbw->objects[i] = ob;

		for (md = ob->modifiers.first; md; md = md->next) {
			if (md->type == eModifierType_RigidBody) {
				rmd = (RigidBodyModifierData*)md;
				if (isModifierActive(rmd)) {
					for (mi = rmd->meshIslands.first, j = 0; mi; mi = mi->next) {
						rbw->cache_index_map[counter] = mi->rigidbody; //map all shards of an object to this object index
						rbw->cache_offset_map[counter] = i;
						mi->linear_index = counter;
						counter++;
						j++;
					}
					ismapped = TRUE;
					break;
				}
			}
		}

		if (!ismapped) {
			//printf("index map:  %d %d\n", counter, i);
			rbw->cache_index_map[counter] = ob->rigidbody_object; //i; 1 object 1 index here (normal case)
			rbw->cache_offset_map[counter] = i;
			//mi->linear_index = counter;
			counter++;
		}

		ismapped = FALSE;
	}
}

static void rigidbody_update_sim_world(Scene *scene, RigidBodyWorld *rbw)
{
	float adj_gravity[3];

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
	rigidbody_update_ob_array(rbw);
}

static void rigidbody_update_sim_ob(Scene *scene, RigidBodyWorld *rbw, Object *ob, RigidBodyOb *rbo, float centroid[3])
{
	float loc[3];
	float rot[4];
	float scale[3], centr[3];

	/* only update if rigid body exists */
	if (rbo->physics_object == NULL)
		return;

	copy_v3_v3(centr, centroid);
	mat4_decompose(loc, rot, scale, ob->obmat);

	/* update scale for all objects */
	RB_body_set_scale(rbo->physics_object, scale);
	/* compensate for embedded convex hull collision margin */
	if (!(rbo->flag & RBO_FLAG_USE_MARGIN) && rbo->shape == RB_SHAPE_CONVEXH)
		RB_shape_set_margin(rbo->physics_shape, RBO_GET_MARGIN(rbo) * MIN3(scale[0], scale[1], scale[2]));

	/* make transformed objects temporarily kinmatic so that they can be moved by the user during simulation */
	if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) {
		RB_body_set_kinematic_state(rbo->physics_object, TRUE);
		RB_body_set_mass(rbo->physics_object, 0.0f);
	}

	/* update rigid body location and rotation for kinematic bodies */
	if (rbo->flag & RBO_FLAG_KINEMATIC || (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)) {
		mul_v3_v3(centr, scale);
		mul_qt_v3(rot, centr);
		add_v3_v3(loc, centr);
		RB_body_activate(rbo->physics_object);
		RB_body_set_loc_rot(rbo->physics_object, loc, rot);
	}
	/* update influence of effectors - but don't do it on an effector */
	/* only dynamic bodies need effector update */
	else if (rbo->type == RBO_TYPE_ACTIVE && ((ob->pd == NULL) || (ob->pd->forcefield == PFIELD_NULL))) {
		EffectorWeights *effector_weights = rbw->effector_weights;
		EffectedPoint epoint;
		ListBase *effectors;

		/* get effectors present in the group specified by effector_weights */
		effectors = pdInitEffectors(scene, ob, NULL, effector_weights);
		if (effectors) {
			float eff_force[3] = {0.0f, 0.0f, 0.0f};
			float eff_loc[3], eff_vel[3];

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
			if (G.f & G_DEBUG)
				printf("\tapplying force (%f,%f,%f) to '%s'\n", eff_force[0], eff_force[1], eff_force[2], ob->id.name + 2);
			/* activate object in case it is deactivated */
			if (!is_zero_v3(eff_force))
				RB_body_activate(rbo->physics_object);
			RB_body_apply_central_force(rbo->physics_object, eff_force);
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
}

static void validateShard(RigidBodyWorld *rbw, MeshIsland* mi, Object* ob, int rebuild)
{
	if (mi == NULL || mi->rigidbody == NULL)
	{
		return;
	}
	
	if (rebuild) { // && (mi->rigidbody->flag & RBO_FLAG_NEEDS_VALIDATE)) {
		/* World has been rebuilt so rebuild object */
		BKE_rigidbody_validate_sim_shard(rbw, mi, ob, true);
	}
	else if (mi->rigidbody->flag & RBO_FLAG_NEEDS_VALIDATE) {
		BKE_rigidbody_validate_sim_shard(rbw, mi, ob, false);
	}
	/* refresh shape... */
	if (mi->rigidbody->flag & RBO_FLAG_NEEDS_RESHAPE) {
		/* mesh/shape data changed, so force shape refresh */
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
		/* now tell RB sim about it */
		// XXX: we assume that this can only get applied for active/passive shapes that will be included as rigidbodies
		//if (mi->rigidbody->physics_shape != NULL)
		RB_body_set_collision_shape(mi->rigidbody->physics_object, mi->rigidbody->physics_shape);
	}
	mi->rigidbody->flag &= ~(RBO_FLAG_NEEDS_VALIDATE | RBO_FLAG_NEEDS_RESHAPE);
}

/* Updates and validates world, bodies and shapes.
 * < rebuild: rebuild entire simulation
 */
static void rigidbody_update_simulation(Scene *scene, RigidBodyWorld *rbw, int rebuild)
{
	GroupObject *go;
	MeshIsland* mi = NULL;
	float centroid[3] = {0, 0, 0};
	RigidBodyShardCon *rbsc;
	//int rebuildcon = rebuild;

	/* update world */
	if (rebuild)
		BKE_rigidbody_validate_sim_world(scene, rbw, true);
	rigidbody_update_sim_world(scene, rbw);

	/* update objects */
	for (go = rbw->group->gobject.first; go; go = go->next) {
		Object *ob = go->ob;
		ModifierData *md = NULL;
		RigidBodyModifierData *rmd = NULL;

		if (ob && ob->type == OB_MESH) {

			/* check for fractured objects which want to participate first, then handle other normal objects*/
			for (md = ob->modifiers.first; md; md = md->next) {
				if (md->type == eModifierType_RigidBody) {
					rmd = (RigidBodyModifierData*)md;
					//BKE_rigidbody_sync_all_shards(ob);
					if (rbw->refresh_modifiers)
					{
						//trigger refresh of modifier at next execution (if we jumped back to startframea after changing an object)
						rmd->refresh = TRUE;
						BKE_rigidbody_remove_object(scene, ob);
						//rbw->refresh_modifiers = FALSE;
						rbw->object_changed = FALSE;
						//rebuildcon = TRUE;
					}
					
					/*if (rbw->rebuild_comp_con) {
						rmd->refresh_constraints = TRUE;
						rbw->rebuild_comp_con = FALSE;
					}*/
					break;
				}
			}

			if (isModifierActive(rmd)) {
				float max_con_mass = 0;
				float min_con_dist = FLT_MAX;
			
				//BKE_object_where_is_calc(scene, ob);
				int count = BLI_countlist(&rmd->meshIslands);
				for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
					if (mi->rigidbody == NULL) {
						continue;
						/*if (mi->compound_count == 0)
						{
							//treat compound parents separately - do not reassign deleted rigidbodies
							mi->rigidbody = BKE_rigidbody_create_shard(scene, ob, mi);
							BKE_rigidbody_calc_shard_mass(ob, mi);
							BKE_rigidbody_validate_sim_shard(rbw, mi, ob, true);
							mi->rigidbody->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
						}*/
					}
					else {  //as usual, but for each shard now, and no constraints
						/* perform simulation data updates as tagged */
						/* refresh object... */
						int do_rebuild = rebuild;
						
						if (rmd->breaking_percentage > 0)
						{
							int broken_cons = 0, cons = 0, i = 0;
							RigidBodyShardCon* con;
							
							cons = mi->participating_constraint_count;
							//calc ratio of broken cons here, per Mi and flag the rest to be broken too
							for (i = 0; i < cons; i++)
							{
								con = mi->participating_constraints[i];
								if (con && con->physics_constraint)
								{
									if (!RB_constraint_is_enabled(con->physics_constraint))
									{
										broken_cons++;
									}
								}
							}
							
							if (cons > 0)
							{
								if ((float)broken_cons / (float)cons * 100 >= rmd->breaking_percentage) {
									//break all cons if over percentage
									for (i = 0; i < cons; i++)
									{
										con = mi->participating_constraints[i];
										if (con)
										{
											con->flag &= ~RBC_FLAG_ENABLED;
											con->flag |= RBC_FLAG_NEEDS_VALIDATE;
											
											if (con->physics_constraint)
											{
												RB_constraint_set_enabled(con->physics_constraint, FALSE);
											}
										}
									}
								}
							}
						}
						
						validateShard(rbw, count == 0 ? NULL : mi, ob, do_rebuild);
					}

					/* update simulation object... */
					rigidbody_update_sim_ob(scene, rbw, ob, mi->rigidbody, mi->centroid);
				}

				if (rmd->mass_dependent_thresholds)
				{
					max_con_mass = BKE_rigidbody_calc_max_con_mass(ob);
				}

				if (rmd->dist_dependent_thresholds)
				{
					min_con_dist = BKE_rigidbody_calc_min_con_dist(ob);
				}

				for (rbsc = rmd->meshConstraints.first; rbsc; rbsc = rbsc->next) {

					if ((rmd->mass_dependent_thresholds) || (rmd->dist_dependent_thresholds))
					{
						BKE_rigidbody_calc_threshold(max_con_mass, min_con_dist, rmd, rbsc);
					}
					
					if (((rmd->breaking_angle > 0) || (rmd->breaking_distance > 0)) && !rebuild)
					{
						float dist, angle, distdiff, anglediff;
						calc_dist_angle(rbsc, &dist, &angle);
						
						anglediff = fabs(angle - rbsc->start_angle);
						distdiff = fabs(dist - rbsc->start_dist);
						
						
						if ((rmd->breaking_angle > 0) && (anglediff > rmd->breaking_angle))
						{
							rbsc->flag &= ~RBC_FLAG_ENABLED;
							rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;
							
							if (rbsc->physics_constraint)
							{
								RB_constraint_set_enabled(rbsc->physics_constraint, FALSE);
							}
						}
						
						if ((rmd->breaking_distance > 0) && (distdiff > rmd->breaking_distance))
						{
							rbsc->flag &= ~RBC_FLAG_ENABLED;
							rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;
							
							if (rbsc->physics_constraint)
							{
								RB_constraint_set_enabled(rbsc->physics_constraint, FALSE);
							}
						}
					}

					if (rebuild) {
						/* World has been rebuilt so rebuild constraint */
						BKE_rigidbody_validate_sim_shard_constraint(rbw, rbsc, ob, true);
						BKE_rigidbody_start_dist_angle(rbsc);
					}

					else if (rbsc->flag & RBC_FLAG_NEEDS_VALIDATE) {
						BKE_rigidbody_validate_sim_shard_constraint(rbw, rbsc, ob, false);
					}

					rbsc->flag &= ~RBC_FLAG_NEEDS_VALIDATE;
					
					if (rmd->use_cellbased_sim) //bullet crash, todo...
					{
						for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
							if (isDisconnected(mi))
							{
								float cfra = BKE_scene_frame_get(scene);
								rmd->split(rmd, ob, mi, cfra);
							}
						}
					}
				}
			}
			else
			{
				/* validate that we've got valid object set up here... */
				RigidBodyOb *rbo = ob->rigidbody_object;
				/* update transformation matrix of the object so we don't get a frame of lag for simple animations */
				BKE_object_where_is_calc(scene, ob);

				if (rbo == NULL) {
					/* Since this object is included in the sim group but doesn't have
					 * rigid body settings (perhaps it was added manually), add!
					 *	- assume object to be active? That is the default for newly added settings...
					 */
					ob->rigidbody_object = BKE_rigidbody_create_object(scene, ob, RBO_TYPE_ACTIVE);
					BKE_rigidbody_validate_sim_object(rbw, ob, true);

					rbo = ob->rigidbody_object;
				}
				else {
					/* perform simulation data updates as tagged */
					/* refresh object... */
					if (rebuild) {
						/* World has been rebuilt so rebuild object */
						BKE_rigidbody_validate_sim_object(rbw, ob, true);
					}
					else if (rbo->flag & RBO_FLAG_NEEDS_VALIDATE) {
						BKE_rigidbody_validate_sim_object(rbw, ob, false);
					}
					/* refresh shape... */
					if (rbo->flag & RBO_FLAG_NEEDS_RESHAPE) {
						/* mesh/shape data changed, so force shape refresh */
						BKE_rigidbody_validate_sim_shape(ob, true);
						/* now tell RB sim about it */
						// XXX: we assume that this can only get applied for active/passive shapes that will be included as rigidbodies
						RB_body_set_collision_shape(rbo->physics_object, rbo->physics_shape);
					}
					rbo->flag &= ~(RBO_FLAG_NEEDS_VALIDATE | RBO_FLAG_NEEDS_RESHAPE);
				}

				/* update simulation object... */
				rigidbody_update_sim_ob(scene, rbw, ob, rbo, centroid);
			}
		}
		rigidbody_update_ob_array(rbw);
		rbw->refresh_modifiers = FALSE;
	}

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
				ob->rigidbody_constraint = BKE_rigidbody_create_constraint(scene, ob, RBC_TYPE_FIXED);
				BKE_rigidbody_validate_sim_constraint(rbw, ob, true);

				rbc = ob->rigidbody_constraint;
			}
			else {
				/* perform simulation data updates as tagged */
				if (rebuild) {
					/* World has been rebuilt so rebuild constraint */
					BKE_rigidbody_validate_sim_constraint(rbw, ob, true);
				}
				else if (rbc->flag & RBC_FLAG_NEEDS_VALIDATE) {
					BKE_rigidbody_validate_sim_constraint(rbw, ob, false);
				}
				rbc->flag &= ~RBC_FLAG_NEEDS_VALIDATE;
			}
		}
	}
}

static void rigidbody_update_simulation_post_step(RigidBodyWorld *rbw)
{
	GroupObject *go;
	ModifierData *md;
	RigidBodyModifierData *rmd;
	int modFound = FALSE;
	RigidBodyOb *rbo;
	MeshIsland *mi;

	for (go = rbw->group->gobject.first; go; go = go->next) {

		Object *ob = go->ob;
		//handle fractured rigidbodies, maybe test for psys as well ?
		for (md = ob->modifiers.first; md; md = md->next) {
			if (md->type == eModifierType_RigidBody) {
				rmd = (RigidBodyModifierData*)md;
				if (isModifierActive(rmd)) {
					for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
						rbo = mi->rigidbody;
						if (!rbo) continue;
						/* reset kinematic state for transformed objects */
						if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) {
							RB_body_set_kinematic_state(rbo->physics_object, rbo->flag & RBO_FLAG_KINEMATIC || rbo->flag & RBO_FLAG_DISABLED);
							RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
							/* deactivate passive objects so they don't interfere with deactivation of active objects */
							if (rbo->type == RBO_TYPE_PASSIVE)
								RB_body_deactivate(rbo->physics_object);
						}
					}
					modFound = TRUE;
					break;
				}
			}
		}

		//handle regular rigidbodies
		if (ob && !modFound) {
			RigidBodyOb *rbo = ob->rigidbody_object;
			/* reset kinematic state for transformed objects */
			if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) {
				RB_body_set_kinematic_state(rbo->physics_object, rbo->flag & RBO_FLAG_KINEMATIC || rbo->flag & RBO_FLAG_DISABLED);
				RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
				/* deactivate passive objects so they don't interfere with deactivation of active objects */
				if (rbo->type == RBO_TYPE_PASSIVE)
					RB_body_deactivate(rbo->physics_object);
			}
		}
		modFound = FALSE;
	}
}

/*int is_zero_m4(float mat[4][4]) {
	return is_zero_v4(mat[0]) &&
		   is_zero_v4(mat[1]) &&
		   is_zero_v4(mat[2]) &&
		   is_zero_v4(mat[3]);
}*/

bool BKE_rigidbody_check_sim_running(RigidBodyWorld *rbw, float ctime)
{
	return (rbw && (rbw->flag & RBW_FLAG_MUTED) == 0 && ctime > rbw->pointcache->startframe);
}

/* Sync rigid body and object transformations */
void BKE_rigidbody_sync_transforms(RigidBodyWorld *rbw, Object *ob, float ctime)
{
	RigidBodyOb *rbo = NULL;
	RigidBodyModifierData *rmd = NULL;
	ExplodeModifierData *emd = NULL;
	MeshIsland *mi;
	ModifierData * md;
	float centr[3], size[3];
	int modFound = FALSE;
	bool exploPresent = false, exploOK = false;

	for (md = ob->modifiers.first; md; md = md->next)
	{
		if (md->type == eModifierType_Explode)
		{
			emd = (ExplodeModifierData*)md;
			if (emd->mode == eFractureMode_Cells && emd->cells != NULL){
				exploPresent = true;
			}
		}
		if (md->type == eModifierType_RigidBody)
		{
			rmd = (RigidBodyModifierData*)md;
			exploOK = !rmd->explo_shared || (rmd->explo_shared && exploPresent);
			
			if (isModifierActive(rmd) && exploOK) {
				int count;
				modFound = TRUE;
				count = BLI_countlist(&rmd->meshIslands);
				
				if ((ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) ||
				((ob->rigidbody_object) && (ob->rigidbody_object->flag & RBO_FLAG_KINEMATIC))) {
					//update "original" matrix
					copy_m4_m4(rmd->origmat, ob->obmat);
					if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ && rbw)
					{
						RigidBodyShardCon* con;
						
						rbw->object_changed = TRUE;
						BKE_rigidbody_cache_reset(rbw);
						//re-enable all constraints as well
						for (con = rmd->meshConstraints.first; con; con = con->next) {
							con->flag |= RBC_FLAG_ENABLED;
							con->flag |= RBC_FLAG_NEEDS_VALIDATE;
						}
					}
				}

				if (!is_zero_m4(rmd->origmat) && rbw && !rbw->object_changed) {
					copy_m4_m4(ob->obmat, rmd->origmat);
				}

				for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
					rbo = mi->rigidbody;
					if (!rbo)
					{
						continue;
					}
					
					/* use rigid body transform after cache start frame if objects is not being transformed */
					if (BKE_rigidbody_check_sim_running(rbw, ctime) && !(ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)) {

					/* keep original transform when the simulation is muted */
						if (rbw->flag & RBW_FLAG_MUTED)
							return;
					}
					/* otherwise set rigid body transform to current obmat*/
					else {
						//offset
						mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
						mat4_to_size(size, ob->obmat);
						copy_v3_v3(centr, mi->centroid);
						mul_v3_v3(centr, size);
						mul_qt_v3(rbo->orn, centr);
						add_v3_v3(rbo->pos, centr);
					}
					BKE_rigidbody_update_cell(mi, ob, rbo->pos, rbo->orn, ctime, 
											 rbw->pointcache->flag & PTCACHE_BAKED && rmd->use_cellbased_sim);
				}
				
				break;
			}
		}

		modFound = FALSE;
	}

	if (!modFound)
	{
		rbo = ob->rigidbody_object;

		/* keep original transform for kinematic and passive objects */
		if (ELEM(NULL, rbw, rbo) || rbo->flag & RBO_FLAG_KINEMATIC || rbo->type == RBO_TYPE_PASSIVE)
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

			/*normalize_qt(rbo->orn); // RB_TODO investigate why quaternion isn't normalized at this point
			quat_to_mat4(mat, rbo->orn);
			copy_v3_v3(mat[3], rbo->pos);*/

			mat4_to_size(size, ob->obmat);
			size_to_mat4(size_mat, size);
			mult_m4_m4m4(mat, mat, size_mat);

			copy_m4_m4(ob->obmat, mat);
		}
			/* otherwise set rigid body transform to current obmat */
		else {
			if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)
				rbw->object_changed = TRUE;
			mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
		}
	}
}

/* Used when cancelling transforms - return rigidbody and object to initial states */
void BKE_rigidbody_aftertrans_update(Object *ob, float loc[3], float rot[3], float quat[4], float rotAxis[3], float rotAngle)
{
	RigidBodyOb *rbo;
	ModifierData *md;
	RigidBodyModifierData *rmd;
	
	md = modifiers_findByType(ob, eModifierType_RigidBody);
	if (md != NULL)
	{
		MeshIsland* mi;
		rmd = (RigidBodyModifierData*)md;
		copy_m4_m4(rmd->origmat, ob->obmat);
		for (mi = rmd->meshIslands.first; mi; mi = mi->next)
		{
			rbo = mi->rigidbody;
			/* return rigid body and object to their initial states */
			copy_v3_v3(rbo->pos, ob->loc);
			add_v3_v3(rbo->pos, mi->centroid);
			copy_v3_v3(ob->loc, loc);
		
			if (ob->rotmode > 0) {
				eulO_to_quat(rbo->orn, ob->rot, ob->rotmode);
				copy_v3_v3(ob->rot, rot);
			}
			else if (ob->rotmode == ROT_MODE_AXISANGLE) {
				axis_angle_to_quat(rbo->orn, ob->rotAxis, ob->rotAngle);
				copy_v3_v3(ob->rotAxis, rotAxis);
				ob->rotAngle = rotAngle;
			}
			else {
				copy_qt_qt(rbo->orn, ob->quat);
				copy_qt_qt(ob->quat, quat);
			}
			if (rbo->physics_object) {
				/* allow passive objects to return to original transform */
				if (rbo->type == RBO_TYPE_PASSIVE)
					RB_body_set_kinematic_state(rbo->physics_object, TRUE);
				RB_body_set_loc_rot(rbo->physics_object, rbo->pos, rbo->orn);
			}
		}
	}
	else
	{
		rbo = ob->rigidbody_object;
		/* return rigid body and object to their initial states */
		copy_v3_v3(rbo->pos, ob->loc);
		copy_v3_v3(ob->loc, loc);
	
		if (ob->rotmode > 0) {
			eulO_to_quat(rbo->orn, ob->rot, ob->rotmode);
			copy_v3_v3(ob->rot, rot);
		}
		else if (ob->rotmode == ROT_MODE_AXISANGLE) {
			axis_angle_to_quat(rbo->orn, ob->rotAxis, ob->rotAngle);
			copy_v3_v3(ob->rotAxis, rotAxis);
			ob->rotAngle = rotAngle;
		}
		else {
			copy_qt_qt(rbo->orn, ob->quat);
			copy_qt_qt(ob->quat, quat);
		}
		if (rbo->physics_object) {
			/* allow passive objects to return to original transform */
			if (rbo->type == RBO_TYPE_PASSIVE)
				RB_body_set_kinematic_state(rbo->physics_object, TRUE);
			RB_body_set_loc_rot(rbo->physics_object, rbo->pos, rbo->orn);
		}
		// RB_TODO update rigid body physics object's loc/rot for dynamic objects here as well (needs to be done outside bullet's update loop)
	}
}

void BKE_rigidbody_cache_reset(RigidBodyWorld *rbw)
{
	if (rbw)
		rbw->pointcache->flag |= PTCACHE_OUTDATED;
}


/* ------------------ */

/* Rebuild rigid body world */
/* NOTE: this needs to be called before frame update to work correctly */
void BKE_rigidbody_rebuild_world(Scene *scene, float ctime)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	PointCache *cache;
	PTCacheID pid;
	int startframe, endframe;

	BKE_ptcache_id_from_rigidbody(&pid, NULL, rbw);
	BKE_ptcache_id_time(&pid, scene, ctime, &startframe, &endframe, NULL);
	cache = rbw->pointcache;

	/* flag cache as outdated if we don't have a world or number of objects in the simulation has changed */
	if (rbw->physics_world == NULL || rbw->numbodies != (rigidbody_count_regular_objects(rbw->group->gobject) + rigidbody_count_shards(rbw->group->gobject))) {
		cache->flag |= PTCACHE_OUTDATED;
	}

	if (ctime <= startframe + 1 && rbw->ltime == startframe) {
		if (cache->flag & PTCACHE_OUTDATED) {
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

	BKE_ptcache_id_from_rigidbody(&pid, NULL, rbw);
	BKE_ptcache_id_time(&pid, scene, ctime, &startframe, &endframe, NULL);
	cache = rbw->pointcache;

	if (ctime <= startframe) {
		rbw->ltime = startframe;
		if ((rbw->object_changed))
		{	//flag modifier refresh at their next execution
			rbw->refresh_modifiers = TRUE;
			rbw->object_changed = FALSE;
			rigidbody_update_simulation(scene, rbw, true);
		}
		rbw->rebuild_comp_con = TRUE;
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
		rigidbody_update_ob_array(rbw);
	
	/* try to read from cache */
	// RB_TODO deal with interpolated, old and baked results
	if (BKE_ptcache_read(&pid, ctime)) {
		BKE_ptcache_validate(cache, (int)ctime);
		rbw->ltime = ctime;
		return;
	}

	/* advance simulation, we can only step one frame forward */
	if (ctime == rbw->ltime + 1 && !(cache->flag & PTCACHE_BAKED)) {
		/* write cache for first frame when on second frame */
		if (rbw->ltime == startframe && (cache->flag & PTCACHE_OUTDATED || cache->last_exact == 0)) {
			BKE_ptcache_write(&pid, startframe);
			//rbw->object_changed = TRUE; //flag refresh of modifiers ONCE if cache is empty
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

void BKE_rigidbody_free_world(RigidBodyWorld *rbw) {}
void BKE_rigidbody_free_object(Object *ob) {}
void BKE_rigidbody_free_constraint(Object *ob) {}
struct RigidBodyOb *BKE_rigidbody_copy_object(Object *ob) { return NULL; }
struct RigidBodyCon *BKE_rigidbody_copy_constraint(Object *ob) { return NULL; }
void BKE_rigidbody_relink_constraint(RigidBodyCon *rbc) {}
void BKE_rigidbody_validate_sim_shape(Object *ob, short rebuild) {}
void BKE_rigidbody_validate_sim_object(RigidBodyWorld *rbw, Object *ob, short rebuild) {}
void BKE_rigidbody_validate_sim_constraint(RigidBodyWorld *rbw, Object *ob, short rebuild) {}
void BKE_rigidbody_validate_sim_world(Scene *scene, RigidBodyWorld *rbw, short rebuild) {}
struct RigidBodyWorld *BKE_rigidbody_create_world(Scene *scene) { return NULL; }
struct RigidBodyWorld *BKE_rigidbody_world_copy(RigidBodyWorld *rbw) { return NULL; }
void BKE_rigidbody_world_groups_relink(struct RigidBodyWorld *rbw) {}
struct RigidBodyOb *BKE_rigidbody_create_object(Scene *scene, Object *ob, short type) { return NULL; }
struct RigidBodyCon *BKE_rigidbody_create_constraint(Scene *scene, Object *ob, short type) { return NULL; }
struct RigidBodyWorld *BKE_rigidbody_get_world(Scene *scene) { return NULL; }
void BKE_rigidbody_remove_object(Scene *scene, Object *ob) {}
void BKE_rigidbody_remove_constraint(Scene *scene, Object *ob) {}
void BKE_rigidbody_sync_transforms(RigidBodyWorld *rbw, Object *ob, float ctime) {}
void BKE_rigidbody_aftertrans_update(Object *ob, float loc[3], float rot[3], float quat[4], float rotAxis[3], float rotAngle) {}
bool BKE_rigidbody_check_sim_running(RigidBodyWorld *rbw, float ctime) { return false; }
void BKE_rigidbody_cache_reset(RigidBodyWorld *rbw) {}
void BKE_rigidbody_rebuild_world(Scene *scene, float ctime) {}
void BKE_rigidbody_do_simulation(Scene *scene, float ctime) {}

#ifdef __GNUC__
#  pragma GCC diagnostic pop
#endif

#endif  /* WITH_BULLET */
