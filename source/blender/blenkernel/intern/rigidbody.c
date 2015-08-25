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
#include "BLI_math.h"
#include "BLI_kdtree.h"
#include "BLI_utildefines.h"

#ifdef WITH_BULLET
#  include "RBI_api.h"
#endif

#include "DNA_fracture_types.h"
#include "DNA_group_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_object_force.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_effect.h"
#include "BKE_fracture.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_library.h"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_pointcache.h"
#include "BKE_rigidbody.h"
#include "BKE_modifier.h"
#include "BKE_depsgraph.h"
#include "BKE_scene.h"

#ifdef WITH_BULLET

static void validateShard(Scene *scene, MeshIsland *mi, Object *ob, int rebuild, int transfer_speed);

static void activateRigidbody(RigidBodyShardOb* rbo, RigidBodyWorld *UNUSED(rbw), MeshIsland *UNUSED(mi), Object *ob)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	//FractureContainer *fc = rb->fracture_objects;

	// !rb can happen when being called from a constraints partner meshisland
	if ((!rb || (rb->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION)) && (rbo->flag & RBO_FLAG_KINEMATIC) && (rbo->type == RBO_TYPE_ACTIVE))
	{
		rbo->flag &= ~RBO_FLAG_KINEMATIC;
		//RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
		RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));
		RB_body_set_kinematic_state(rbo->physics_object, false);
		//RB_dworld_add_body(rbw->physics_world, rbo->physics_object, rb->col_groups, mi, ob, mi->linear_index);
		RB_body_activate(rbo->physics_object);
		rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
	}
}

static void calc_dist_angle(RigidBodyShardCon *con, float *dist, float *angle)
{
	float q1[4], q2[4], qdiff[4], axis[3];
	if ((con->mi1->rigidbody == NULL) || (con->mi2->rigidbody == NULL)) {
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
}

void BKE_rigidbody_start_dist_angle(RigidBodyShardCon *con)
{
	/* store starting angle and distance per constraint*/
	float dist, angle;
	calc_dist_angle(con, &dist, &angle);
	con->start_dist = dist;
	con->start_angle = angle;
}

float BKE_rigidbody_calc_max_con_mass(Object *ob)
{
	RigidBodyCon* rbc = ob->rigidbody_constraint;
	ConstraintContainer *cc = rbc->fracture_constraints;
	RigidBodyShardCon *con;
	float max_con_mass = 0, con_mass;

	for (con = cc->constraint_map.first; con; con = con->next) {
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

float BKE_rigidbody_calc_min_con_dist(Object *ob)
{
	RigidBodyCon *rbc = ob->rigidbody_constraint;
	ConstraintContainer* cc = rbc->fracture_constraints;
	RigidBodyShardCon *con;
	float min_con_dist = FLT_MAX, con_dist, con_vec[3];

	for (con = cc->constraint_map.first; con; con = con->next) {
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


void BKE_rigidbody_calc_threshold(float max_con_mass, Object *ob, RigidBodyShardCon *con) {

	RigidBodyCon* rbc = ob->rigidbody_constraint;
	ConstraintContainer *cc = rbc->fracture_constraints;

	float max_thresh, thresh = 0.0f, con_mass;
	if ((max_con_mass == 0) && (cc->flag & FMC_FLAG_USE_MASS_DEPENDENT_THRESHOLDS)) {
		return;
	}

	if ((con->mi1 == NULL) || (con->mi2 == NULL)) {
		return;
	}

	max_thresh = cc->breaking_threshold;
	if ((con->mi1->rigidbody != NULL) && (con->mi2->rigidbody != NULL)) {
		con_mass = con->mi1->rigidbody->mass + con->mi2->rigidbody->mass;

		if (cc->flag & FMC_FLAG_USE_MASS_DEPENDENT_THRESHOLDS)
		{
			thresh = (con_mass / max_con_mass) * max_thresh;
		}

		con->breaking_threshold = thresh;
	}
}

static int DM_mesh_minmax(DerivedMesh *dm, float r_min[3], float r_max[3])
{
	MVert *v;
	int i = 0;
	for (i = 0; i < dm->numVertData; i++) {
		v = CDDM_get_vert(dm, i);
		minmax_v3v3_v3(r_min, r_max, v->co);
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

/* helper function to calculate volume of rigidbody object */
float BKE_rigidbody_calc_volume(DerivedMesh *dm, RigidBodyOb *rbo)
{
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
	/* XXX: all dimensions are auto-determined now... later can add stored settings for this*/
	DM_mesh_boundbox(dm, loc, size);

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
		case RB_SHAPE_CONVEXH:
		case RB_SHAPE_TRIMESH:
			volume = size[0] * size[1] * size[2];
			if (size[0] == 0) {
				volume = size[1] * size[2];
			}
			else if (size[1] == 0) {
				volume = size[0] * size[2];
			}
			else if (size[2] == 0) {
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

void BKE_rigidbody_calc_shard_mass(Scene *scene, Object *ob, MeshIsland *mi)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	FractureContainer *fc = rb->fracture_objects;
	/*FM TODO, this might differ here*/
	DerivedMesh *dm_ob = NULL, *dm_mi;
	float vol_mi = 0, mass_mi = 0, vol_ob = 0, mass_ob = 0;
	bool skip = fc->flag & (FM_FLAG_REFRESH_SHAPE | FM_FLAG_SKIP_MASS_CALC);

	if (!skip)
	{
		if (fc->raw_mesh != NULL) {
			dm_ob = fc->raw_mesh;
		}
		//XXX TODO seems to be a bit problematic, because modifier eval stops here and flag might
		// not be reset properly... so take an potential inaccurate mass here as fallback
		/*else {
			fc->raw_mesh = BKE_fracture_ensure_mesh(scene, ob);
			fc->flag &= ~FM_FLAG_REFRESH;
		}*/

		if (dm_ob == NULL) {
			/* fallback method */

			if ((ob->type == OB_MESH)) {
				/* if we have a mesh, determine its volume */
				dm_ob = CDDM_from_mesh(ob->data);
				vol_ob = BKE_rigidbody_calc_volume(dm_ob, rb);

				dm_ob->needsFree = 1;
				dm_ob->release(dm_ob);
				dm_ob = NULL;
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
			vol_ob = BKE_rigidbody_calc_volume(dm_ob, rb);
		}
	}

	mass_ob = rb->mass;

	if (vol_ob > 0) {
		dm_mi = mi->physics_mesh;
		vol_mi = BKE_rigidbody_calc_volume(dm_mi, rb);
		mass_mi = (vol_mi / vol_ob) * mass_ob;
		mi->rigidbody->mass = mass_mi;
	}
	
	if (mi->rigidbody->type == RBO_TYPE_ACTIVE) {
		if (mi->rigidbody->mass == 0)
			mi->rigidbody->mass = 0.001;  /* set a minimum mass for active objects */
	}

	/* only active bodies need mass update */
	if ((mi->rigidbody->physics_object) && (mi->rigidbody->type == RBO_TYPE_ACTIVE)) {
		RB_body_set_mass(mi->rigidbody->physics_object, RBO_GET_MASS(mi->rigidbody));
	}
}
// TODO... move this to readfile or fracture, as BKE function
#if 0
static void initNormals(struct MeshIsland *mi, Object *ob)
{
	// LIMIT This to the according VGROUP !!! TODO
	/* hrm have to init Normals HERE, because we cant do this in readfile.c in case the file is loaded (have no access to the Object there) */
	if (mi->vertno == NULL && mi->vertices_cached != NULL) {
		KDTreeNearest n;
		int index = 0, i = 0;
		MVert mvrt;

		DerivedMesh *dm = ob->derivedFinal;
		if (dm == NULL) {
			dm = CDDM_from_mesh(ob->data);
		}

		if (fmd->fracture->nor_tree == NULL) {
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
			fmd->fracture->nor_tree = tree;
		}

		mi->vertno = MEM_callocN(sizeof(short) * 3 * mi->vertex_count, "mi->vertno");
		for (i = 0; i < mi->vertex_count; i++) {
			MVert *v = mi->vertices_cached[i];
			index = BLI_kdtree_find_nearest(fmd->fracture->nor_tree, v->co, &n);
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
#endif

void BKE_rigidbody_update_cell(struct MeshIsland *mi, Object *ob, float loc[3], float rot[4])
{
	float startco[3], centr[3], size[3];
	short startno[3];
	int j;
	bool invalidData;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

#if 0 // TODO MOVE init normals elsewhere !
	/* hrm have to init Normals HERE, because we cant do this in readfile.c in case the file is loaded (have no access to the Object there)*/
	if (mi->vertno == NULL && (rmd->fracture->flag & FM_FLAG_FIX_NORMALS)) {
		initNormals(mi, ob, rmd);
	}
#endif
	
	invalidData = (loc[0] == FLT_MIN) || (rot[0] == FLT_MIN);
	
	if (invalidData || !mi) {
		return;
	}

	invert_m4_m4(ob->imat, ob->obmat);
	mat4_to_size(size, ob->obmat);

#if 0 // TODO, utilize cache for this !!!
	if (fc->fracture_mode == MOD_FRACTURE_PREFRACTURED) {
		/*record only in prefracture case here, when you want to convert to keyframes*/
		n = frame - mi->start_frame + 1;

		if (n > mi->frame_count) {
			mi->locs = MEM_reallocN(mi->locs, sizeof(float) * 3 * (n+1));
			mi->rots = MEM_reallocN(mi->rots, sizeof(float) * 4 * (n+1));

			mi->locs[n*3] = loc[0];
			mi->locs[n*3+1] = loc[1];
			mi->locs[n*3+2] = loc[2];

			mi->rots[n*4] = rot[0];
			mi->rots[n*4+1] = rot[1];
			mi->rots[n*4+2] = rot[2];
			mi->rots[n*4+3] = rot[3];
			mi->frame_count = n;
		}
	}
#endif
	
	for (j = 0; j < mi->vertex_count; j++) {
		struct MVert *vert;
		float fno[3];
		
		if (!mi->vertices_cached) {
			return;
		}
		
		vert = mi->vertices_cached[j];
		if (vert == NULL) break;
		if (vert->co == NULL) break;
		if (mi->vertcos == NULL) break;
		//if (fc->flag & FM_FLAG_REFRESH) break;

		copy_v3_v3(startco, mi->vertcos[j]);

		if (fc->flag & FM_FLAG_FIX_NORMALS) {
			float irot[4], qrot[4];
			copy_v3_v3_short(startno, mi->vertnos[j]);

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

		//printf("Vertex Co: %d -> (%.2f, %.2f, %.2f) \n", j, loc[0], loc[1], loc[2]);
	}

	ob->recalc |= OB_RECALC_ALL;
}

/* ************************************** */
/* Memory Management */

/* Freeing Methods --------------------- */

/* Free rigidbody world */
void BKE_rigidbody_free_world(Scene* scene)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;

	/* sanity check */
	if (!rbw)
		return;

	if (rbw->physics_world) {
		/* free physics references, we assume that all physics objects in will have been added to the world */
		GroupObject *go;
		if (rbw->constraints) {
			for (go = rbw->constraints->gobject.first; go; go = go->next) {
				if (go->ob && go->ob->rigidbody_constraint) {
					BKE_rigidbody_free_constraint(go->ob);
				}
			}
		}
		if (rbw->group) {
			for (go = rbw->group->gobject.first; go; go = go->next) {
				if (go->ob && go->ob->rigidbody_object) {
					BKE_rigidbody_free_object(go->ob);
				}
			}
		}
		/* free dynamics world */
		if (rbw->physics_world != NULL)
			RB_dworld_delete(rbw->physics_world);
	}
	if (rbw->objects)
		MEM_freeN(rbw->objects);

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

	BKE_fracture_container_free(ob);

	/* free data itself */
	MEM_freeN(rbo);
	ob->rigidbody_object = NULL;
}

/* Free RigidBody constraint and sim instance */
void BKE_rigidbody_free_constraint(Object *ob)
{
	RigidBodyCon *rbc = (ob) ? (RigidBodyCon*)ob->rigidbody_constraint : NULL;

	/* sanity check */
	if (rbc == NULL)
		return;

	BKE_fracture_constraint_container_free(ob);

	/* free data itself */
	MEM_freeN(rbc);
	ob->rigidbody_constraint = NULL;
}

/* ************************************** */
/* Setup Utilities - Validate Sim Instances */

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

/* create collision shape of mesh - triangulated mesh
 * returns NULL if creation fails.
 */
static rbCollisionShape *rigidbody_get_shape_trimesh_from_mesh_shard(MeshIsland *mi, Object *ob)
{
	rbCollisionShape *shape = NULL;

	if (mi && mi->physics_mesh) {
		DerivedMesh *dm = NULL;
		MVert *mvert;
		MFace *mface;
		int totvert;
		int totface;
		int tottris = 0;
		int triangle_index = 0;

		dm = CDDM_copy(mi->physics_mesh);

		/* ensure mesh validity, then grab data */
		if (dm == NULL)
			return NULL;

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
			if (mi->rigidbody->type == RBO_TYPE_PASSIVE) {
				shape = RB_shape_new_trimesh(mdata);
			}
			else {
				shape = RB_shape_new_gimpact_mesh(mdata);
			}
		}

		/* cleanup temp data */
		if (dm /*&& ob->rigidbody_object->mesh_source == RBO_MESH_BASE*/) {
			dm->needsFree = 1;
		dm->release(dm);
			dm = NULL;
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
	RigidBodyShardOb *rbo = mi->rigidbody;
	RigidBodyOb *rb = ob->rigidbody_object;
	rbCollisionShape *new_shape = NULL;
	float size[3] = {1.0f, 1.0f, 1.0f}, loc[3] = {0.0f, 0.0f, 0.0f};
	float radius = 1.0f;
	float height = 1.0f;
	float capsule_height;
	float hull_margin = 0.0f;
	bool can_embed = true;
	bool has_volume;
	float min[3], max[3];
	
	/* sanity check */
	if (rbo == NULL)
		return;

	/* don't create a new shape if we already have one and don't want to rebuild it */
	if (rbo->physics_shape && !rebuild)
		return;

	if (mi->physics_mesh == NULL)
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

	if (ELEM(rb->shape, RB_SHAPE_CAPSULE, RB_SHAPE_CYLINDER, RB_SHAPE_CONE)) {
		/* take radius as largest x/y dimension, and height as z-dimension */
		radius = MAX2(size[0], size[1]);
		height = size[2];
	}
	else if (rb->shape == RB_SHAPE_SPHERE) {

		/* take radius to the largest dimension to try and encompass everything */
		radius = max_fff(size[0], size[1], size[2]) * 0.5f;
	}
	
	/* create new shape */
	switch (rb->shape) {
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

			if (!(rb->flag & RBO_FLAG_USE_MARGIN) && has_volume)
				hull_margin = 0.04f;
			new_shape = rigidbody_get_shape_convexhull_from_dm(mi->physics_mesh, hull_margin, &can_embed);
			if (!(rb->flag & RBO_FLAG_USE_MARGIN))
				rb->margin = (can_embed && has_volume) ? 0.04f : 0.0f;      /* RB_TODO ideally we shouldn't directly change the margin here */
			break;
		case RB_SHAPE_TRIMESH:
			new_shape = rigidbody_get_shape_trimesh_from_mesh_shard(mi, ob);
			break;
	}
	/* assign new collision shape if creation was successful */
	if (new_shape) {
		if (rbo->physics_shape)
			RB_shape_delete(rbo->physics_shape);
		rbo->physics_shape = new_shape;
		RB_shape_set_margin(rbo->physics_shape, RBO_GET_MARGIN(rb));
	}
	else { /* otherwise fall back to box shape */
		rb->shape = RB_SHAPE_BOX;
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
	}
}

#if 0 // XXX: not defined yet
		case RB_SHAPE_COMPOUND:
			volume = 0.0f;
			break;
#endif

/* --------------------- */
static bool flag_as_kinematic(void *object)
{
	bool is_kinematic = false;
	Object* ob = (Object*)object;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

	is_kinematic = (fc->flag & FM_FLAG_SKIP_STEPPING);

//	printf("Is Kinematic %d \n", is_kinematic);
	return is_kinematic;
	//return false;
}

/* Create physics sim representation of shard given RigidBody settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_shard(Scene* scene, MeshIsland *mi, Object *ob, short rebuild, int transfer_speeds)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyShardOb *rbo = (mi) ? mi->rigidbody : NULL;
	RigidBodyOb *rb = ob->rigidbody_object;

	float loc[3];
	float rot[4];

	/* sanity checks:
	 *	- object doesn't have RigidBody info already: then why is it here?
	 */
	if (rbo == NULL || rbw == NULL)
		return;

	/* make sure collision shape exists */
	/* FIXME we shouldn't always have to rebuild collision shapes when rebuilding objects, but it's needed for constraints to update correctly */
	if (rbo->physics_shape == NULL || rebuild)
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
	
	if (rbo->physics_object) {
		if ((rebuild == false) || (rb->flag & RBO_FLAG_KINEMATIC_REBUILD) ||
		        (rbw->flag & RBW_FLAG_OBJECT_CHANGED))
		{
			RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
		}
	}

	if (!rbo->physics_object || rebuild) {
		/* remove rigid body if it already exists before creating a new one */
		if (rbo->physics_object) {
			RB_body_delete(rbo->physics_object);
			rbo->physics_object = NULL;
		}

		copy_v3_v3(loc, rbo->pos);
		copy_v4_v4(rot, rbo->orn);
		
		if (!rbo->physics_object)
			rbo->physics_object = RB_body_new(rbo->physics_shape, loc, rot, NULL, ob);

		RB_body_set_friction(rbo->physics_object, rb->friction);
		RB_body_set_restitution(rbo->physics_object, rb->restitution);

		RB_body_set_damping(rbo->physics_object, rb->lin_damping, rb->ang_damping);
		RB_body_set_sleep_thresh(rbo->physics_object, rb->lin_sleep_thresh, rb->ang_sleep_thresh);
		RB_body_set_activation_state(rbo->physics_object, rb->flag & RBO_FLAG_USE_DEACTIVATION);

		if (rbo->type == RBO_TYPE_PASSIVE || rb->flag & RBO_FLAG_START_DEACTIVATED)
			RB_body_deactivate(rbo->physics_object);


		RB_body_set_linear_factor(rbo->physics_object,
		                          (ob->protectflag & OB_LOCK_LOCX) == 0,
		                          (ob->protectflag & OB_LOCK_LOCY) == 0,
		                          (ob->protectflag & OB_LOCK_LOCZ) == 0);
		RB_body_set_angular_factor(rbo->physics_object,
		                           (ob->protectflag & OB_LOCK_ROTX) == 0,
		                           (ob->protectflag & OB_LOCK_ROTY) == 0,
		                           (ob->protectflag & OB_LOCK_ROTZ) == 0);

		if (rbo->type == RBO_TYPE_ACTIVE)
			BKE_rigidbody_calc_shard_mass(scene, ob, mi);
		else
			RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo));

		RB_body_set_kinematic_state(rbo->physics_object, rb->flag & RBO_FLAG_KINEMATIC || rb->flag & RBO_FLAG_DISABLED);

		if (transfer_speeds)
		{
			if ((len_squared_v3(rbo->lin_vel) > (rb->lin_sleep_thresh * rb->lin_sleep_thresh)))
			{
				//printf("Setting linear velocity (%f, %f, %f)\n", rbo->lin_vel[0], rbo->lin_vel[1], rbo->lin_vel[2]);
				RB_body_set_linear_velocity(rbo->physics_object, rbo->lin_vel);
			}

			if ((len_squared_v3(rbo->ang_vel) > (rb->ang_sleep_thresh * rb->ang_sleep_thresh)))
			{
				//printf("Setting angular velocity (%f, %f, %f)\n", rbo->ang_vel[0], rbo->ang_vel[1], rbo->ang_vel[2]);
				RB_body_set_angular_velocity(rbo->physics_object, rbo->ang_vel);
			}
		}
	}

	if (rbw && rbw->physics_world && rbo->physics_object)
	{
		RB_dworld_add_body(rbw->physics_world, rbo->physics_object, rb->col_groups, mi, ob, mi->linear_index);
	}

	rbo->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
	rbo->flag &= ~RBO_FLAG_KINEMATIC_REBUILD;
}



#if 0 // XXX: not defined yet
		case RB_SHAPE_COMPOUND:
			volume = 0.0f;
			break;
#endif

/* --------------------- */


/* Create physics sim representation of constraint given rigid body constraint settings
 * < rebuild: even if an instance already exists, replace it
 */
void BKE_rigidbody_validate_sim_shard_constraint(RigidBodyWorld *rbw, Object *ob, RigidBodyShardCon *rbc, short rebuild)
{
	float loc[3];
	float rot[4];
	float lin_lower;
	float lin_upper;
	float ang_lower;
	float ang_upper;
	rbRigidBody *rb1;
	rbRigidBody *rb2;
	RigidBodyCon *con = (RigidBodyCon*)ob->rigidbody_constraint;

	/* sanity checks:
	 *	- object should have a rigid body constraint
	 *  - rigid body constraint should have at least one constrained object
	 */
	if (rbc == NULL) {
		return;
	}

	if (ELEM(NULL, rbc->mi1, rbc->mi1->rigidbody, rbc->mi2, rbc->mi2->rigidbody)) {
		if (rbc->physics_constraint) {
			RB_dworld_remove_constraint(rbw->physics_world, rbc->physics_constraint);
			RB_constraint_delete(rbc->physics_constraint);
			rbc->physics_constraint = NULL;
		}
		return;
	}
	
	if (rbc->mi1->rigidbody)
	{
		rb1 = rbc->mi1->rigidbody->physics_object;
	}
	
	if (rbc->mi2->rigidbody)
	{
		rb2 = rbc->mi2->rigidbody->physics_object;
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

		/* do this for all constraints */
		/* location for fixed constraints doesnt matter, so keep old setting */
		if (rbc->type == RBC_TYPE_FIXED) {
			copy_v3_v3(loc, rbc->mi1->rigidbody->pos);
		}
		else {
			/* else set location to center */
			add_v3_v3v3(loc, rbc->mi1->rigidbody->pos, rbc->mi2->rigidbody->pos);
			mul_v3_fl(loc, 0.5f);
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
						RB_constraint_set_limits_hinge(rbc->physics_constraint, con->limit_ang_z_lower, con->limit_ang_z_upper);
					}
					else
						RB_constraint_set_limits_hinge(rbc->physics_constraint, 0.0f, -1.0f);
					break;
				case RBC_TYPE_SLIDER:
					rbc->physics_constraint = RB_constraint_new_slider(loc, rot, rb1, rb2);
					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
						RB_constraint_set_limits_slider(rbc->physics_constraint, con->limit_lin_x_lower, con->limit_lin_x_upper);
					else
						RB_constraint_set_limits_slider(rbc->physics_constraint, 0.0f, -1.0f);
					break;
				case RBC_TYPE_PISTON:
					rbc->physics_constraint = RB_constraint_new_piston(loc, rot, rb1, rb2);
					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X) {
						lin_lower = con->limit_lin_x_lower;
						lin_upper = con->limit_lin_x_upper;
					}
					else {
						lin_lower = 0.0f;
						lin_upper = -1.0f;
					}
					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_X) {
						ang_lower = con->limit_ang_x_lower;
						ang_upper = con->limit_ang_x_upper;
					}
					else {
						ang_lower = 0.0f;
						ang_upper = -1.0f;
					}
					RB_constraint_set_limits_piston(rbc->physics_constraint, lin_lower, lin_upper, ang_lower, ang_upper);
					break;
				case RBC_TYPE_6DOF_SPRING:
					rbc->physics_constraint = RB_constraint_new_6dof_spring(loc, rot, rb1, rb2);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, con->flag & RBC_FLAG_USE_SPRING_X);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, con->spring_stiffness_x);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, con->spring_damping_x);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, con->flag & RBC_FLAG_USE_SPRING_Y);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, con->spring_stiffness_y);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, con->spring_damping_y);

					RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, con->flag & RBC_FLAG_USE_SPRING_Z);
					RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, con->spring_stiffness_z);
					RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, con->spring_damping_z);

					RB_constraint_set_equilibrium_6dof_spring(rbc->physics_constraint);
				/* fall through */
				case RBC_TYPE_6DOF:
					if (rbc->type == RBC_TYPE_6DOF)     /* a litte awkward but avoids duplicate code for limits */
						rbc->physics_constraint = RB_constraint_new_6dof(loc, rot, rb1, rb2);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, con->limit_lin_x_lower, con->limit_lin_x_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Y)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, con->limit_lin_y_lower, con->limit_lin_y_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Z)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, con->limit_lin_z_lower, con->limit_lin_z_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_X)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, con->limit_ang_x_lower, con->limit_ang_x_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Y)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, con->limit_ang_y_lower, con->limit_ang_y_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, 0.0f, -1.0f);

					if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Z)
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, con->limit_ang_z_lower, con->limit_ang_z_upper);
					else
						RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, 0.0f, -1.0f);
					break;
				case RBC_TYPE_MOTOR:
					rbc->physics_constraint = RB_constraint_new_motor(loc, rot, rb1, rb2);

					RB_constraint_set_enable_motor(rbc->physics_constraint, con->flag & RBC_FLAG_USE_MOTOR_LIN, con->flag & RBC_FLAG_USE_MOTOR_ANG);
					RB_constraint_set_max_impulse_motor(rbc->physics_constraint, con->motor_lin_max_impulse, con->motor_ang_max_impulse);
					RB_constraint_set_target_velocity_motor(rbc->physics_constraint, con->motor_lin_target_velocity, con->motor_ang_target_velocity);
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

static void do_activate(Object* ob, Object *ob2, MeshIsland *mi_compare, RigidBodyWorld *rbw)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	RigidBodyOb *rb2 = ob2->rigidbody_object;

	FractureContainer *fc = rb->fracture_objects;
	FractureContainer *fc2 = rb2->fracture_objects;

	bool valid = true;
	MeshIsland *mi;

	valid = valid && (fc != NULL);
	valid = valid && (fc2 != NULL);

	//TODO, add flag definitions to container as well, applies for all contained rigidbodies
	valid = valid && (rb->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION);
	valid = valid && (rb2->flag & RBO_FLAG_IS_TRIGGER);

	if (valid)
	{
		for (mi = fc->current->island_map.first; mi; mi = mi->next)
		{
			bool same_cluster = (mi->particle_index != -1) &&
								(mi->particle_index == mi_compare->particle_index);

			RigidBodyShardOb* rbo = mi->rigidbody;
			if ((rbo->flag & RBO_FLAG_KINEMATIC) && ((mi_compare == mi) || same_cluster))
			{
				if (rbo->physics_object) {
					activateRigidbody(rbo, rbw, mi, ob);
				}
			}
		}
	}
}

static int check_colgroup_ghost(Object* ob1, Object *ob2)
{
	int ret = 0;
	RigidBodyOb *rb1 = ob1->rigidbody_object;
	RigidBodyOb *rb2 = ob2->rigidbody_object;

	// TODO colgroups will be added to container as well, for convenience ?
	ret = colgroup_check(rb1->col_groups, rb2->col_groups);
	return ret && (!(rb1->flag & RBO_FLAG_IS_GHOST) && !(rb2->flag & RBO_FLAG_IS_GHOST));
}

/* this allows partial object activation, only some shards will be activated, called from bullet(!) */
static int filterCallback(void* world, void* island1, void* island2, void *blenderOb1, void* blenderOb2) {
	MeshIsland* mi1, *mi2;
	RigidBodyWorld *rbw = (RigidBodyWorld*)world;
	Object* ob1, *ob2;
	RigidBodyOb *rb1, *rb2;
	FractureContainer *fc1, *fc2;
	bool validOb = false;

	mi1 = (MeshIsland*)island1;
	mi2 = (MeshIsland*)island2;
	ob1 = blenderOb1;
	ob2 = blenderOb2;

	rb1 = ob1->rigidbody_object;
	rb2 = ob2->rigidbody_object;

	fc1 = rb1->fracture_objects;
	fc2 = rb2->fracture_objects;

	if (rbw == NULL || fc1->fracture_mode == MOD_FRACTURE_DYNAMIC || fc2->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		/* just check for ghost flags here, do not activate anything */
		return check_colgroup_ghost(ob1, ob2);
	}

	if ((mi1 != NULL) && (mi2 != NULL)) {
		validOb = (ob1 != ob2 && colgroup_check(rb1->col_groups, rb2->col_groups) &&
		          ((rb1->flag & RBO_FLAG_KINEMATIC) || (rb2->flag & RBO_FLAG_KINEMATIC)) &&
		          ((mi1->rigidbody->type == RBO_TYPE_ACTIVE) && (mi2->rigidbody->type == RBO_TYPE_ACTIVE)));
	}

	if (validOb)
	{
		if (rb1->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION)
		{
			do_activate(ob1, ob2, mi1, rbw);
		}

		if (rb2->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION)
		{
			do_activate(ob2, ob1, mi2, rbw);
		}
	}

	return check_colgroup_ghost(ob1, ob2);
}

static bool check_shard_size(Object* ob, int id, float impact_loc[3], Object* collider)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	float size = 0.1f;
	Shard *t = fs->frac_mesh->shard_map.first;
	Shard *s = NULL;
	float dim[3];

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

	if (s == NULL)
	{
		return false;
	}

	BKE_shard_calc_minmax(s);

	if ((fabsf(s->max[0] - s->min[0]) < size) ||
	   (fabsf(s->max[1] - s->min[1]) < size) ||
	   (fabsf(s->max[2] - s->min[2]) < size))
	{
		return false;
	}

	if (collider)
	{
		//simple calc, take just dimensions here.... will be refined later TODO
		BKE_object_dimensions_get(collider, dim);

		copy_v3_v3(s->impact_loc, impact_loc);
		copy_v3_v3(s->impact_size, dim);
	}

	printf("FRACTURE : %d\n", id);

	return true;
}

static void check_fracture(rbContactPoint* cp, Scene* scene)
{
	int linear_index1, linear_index2;
	FractureContainer *fc1, *fc2;
	float force;
	Object* ob1, *ob2;

	if (cp == NULL)
		return;

	force = cp->contact_force;

	linear_index1 = cp->contact_body_indexA; //this is only INSIDE object now !!! TODO
	linear_index2 = cp->contact_body_indexB;

	ob1 = cp->contact_obA;
	ob2 = cp->contact_obB;

	if (linear_index1 > -1 && ob1 && ob2)
	{
		fc1 = ob1->rigidbody_object->fracture_objects;
		if (fc1 && fc1->fracture_mode == MOD_FRACTURE_DYNAMIC) {
			if (force > fc1->dynamic_force) {
				if(check_shard_size(ob1, linear_index1, cp->contact_pos_world_onA, ob2))
				{
					BKE_dynamic_fracture_mesh(scene, ob1, linear_index1);
					fc1->flag |= FM_FLAG_UPDATE_DYNAMIC;
				}
			}
		}
	}

	if (linear_index2 > -1 && ob1 && ob2)
	{
		fc2 = ob2->rigidbody_object->fracture_objects;
		if (fc2 && fc2->fracture_mode == MOD_FRACTURE_DYNAMIC) {
			if (force > fc2->dynamic_force) {
				if(check_shard_size(ob2, linear_index2, cp->contact_pos_world_onB, ob1))
				{
					BKE_dynamic_fracture_mesh(scene, ob2, linear_index2);
					fc2->flag |= FM_FLAG_UPDATE_DYNAMIC;
				}
			}
		}
	}

	cp = NULL;
}

static void contactCallback(rbContactPoint* cp, void* sc)
{
	Scene *scene = (Scene*)sc;
	check_fracture(cp, scene);
}

static void cleanupWorld(RigidBodyWorld *rbw)
{
	GroupObject *go;

	/*attempt to remove constraint and object remainders... */
	if (rbw->constraints)
	{
		for (go = rbw->constraints->gobject.first; go; go = go->next)
		{
			Object *ob = go->ob;
			RigidBodyCon *rbc = ob->rigidbody_constraint;
			ConstraintContainer *cc = rbc->fracture_constraints;
			RigidBodyShardCon *con;

			for (con = cc->constraint_map.first; con; con = con->next)
			{
				if (con->physics_constraint)
				{
					RB_dworld_remove_constraint(rbw->physics_world, con->physics_constraint);
					RB_constraint_delete(con->physics_constraint);
					con->physics_constraint = NULL;
				}
			}
		}
	}

	if (rbw->group)
	{
		for (go = rbw->group->gobject.first; go; go = go->next)
		{
			Object *ob = go->ob;
			RigidBodyOb *rbo = ob->rigidbody_object;
			FractureContainer *fc = rbo->fracture_objects;
			FractureState *fs;

			if (fc->flag & FM_FLAG_EXECUTE_THREADED)
				continue;

			for (fs = fc->states.first; fs; fs = fs->next)
			{
				MeshIsland *mi;
				for (mi = fs->island_map.first; mi; mi = mi->next)
				{
					RigidBodyShardOb *rbo = mi->rigidbody;
					if (rbo->physics_object)
					{
						RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
						RB_body_delete(rbo->physics_object);
						rbo->physics_object = NULL;
					}
				}
			}
		}
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
		{
			cleanupWorld(rbw);
			RB_dworld_delete(rbw->physics_world);
		}
		rbw->physics_world = RB_dworld_new(scene->physics_settings.gravity, scene, filterCallback, contactCallback);
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

	//rbw->pointcache = BKE_ptcache_add(&(rbw->ptcaches));
	//rbw->pointcache->step = 1;
	rbw->flag &=~ RBW_FLAG_OBJECT_CHANGED;
	rbw->flag &=~ RBW_FLAG_REFRESH_MODIFIERS;

	rbw->objects = MEM_mallocN(sizeof(Object *), "objects");
	//rbw->cache_index_map = MEM_mallocN(sizeof(RigidBodyOb *), "cache_index_map");
	//rbw->cache_offset_map = MEM_mallocN(sizeof(int), "cache_offset_map");

	/* return this sim world */
	return rbw;
}

void BKE_rigidbody_set_initial_transform(Object *ob, MeshIsland *mi, RigidBodyShardOb *rbo)
{
	float size[3], centr[3];

	/* set initial transform */
	mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
	mat4_to_size(size, ob->obmat);

	//add initial "offset" (centroid), maybe subtract ob->obmat ?? (not sure)
	copy_v3_v3(centr, mi->centroid);
	mul_v3_v3(centr, size);
	mul_qt_v3(rbo->orn, centr);
	add_v3_v3(rbo->pos, centr);
}

/* Add rigid body settings to the specified shard */
RigidBodyShardOb *BKE_rigidbody_create_shard(Object *ob, MeshIsland *mi)
{
	RigidBodyShardOb *rbo;
	//RigidBodyWorld *rbw = BKE_rigidbody_get_world(scene);
	RigidBodyOb *rb = ob->rigidbody_object;

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
	
#if 0
	if ((ob->type == OB_MESH) && (((Mesh *)ob->data)->totvert == 0)) {
		return NULL;
	}
#endif

	/* since we are always member of an object, dupe its settings,
	 * create new settings data, and link it up */
	rbo = MEM_callocN(sizeof(RigidBodyShardOb), "RigidBodyShardOb create");
	rbo->type = rb->type;
	if (rbo->type == RBO_TYPE_ACTIVE)
		rbo->type = mi->ground_weight > 0.5f ? RBO_TYPE_PASSIVE : RBO_TYPE_ACTIVE;

	BKE_rigidbody_set_initial_transform(ob, mi, rbo);

	rbo->physics_object = NULL;
	rbo->physics_shape = NULL;
	rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;

	/* return this object */
	return rbo;
}

RigidBodyWorld *BKE_rigidbody_world_copy(RigidBodyWorld *rbw)
{
	RigidBodyWorld *rbwn = MEM_dupallocN(rbw);

	if (rbwn->group)
		id_us_plus(&rbwn->group->id);
	if (rbwn->constraints)
		id_us_plus(&rbwn->constraints->id);

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
}

/* Add rigid body settings to the specified object */
RigidBodyOb *BKE_rigidbody_create_object(Object *ob, short type)
{
	RigidBodyOb *rbo;

	if (ob == NULL)
		return NULL;

	if (ob->rigidbody_object)
		return ob->rigidbody_object;

	/* create new settings data, and link it up */
	rbo = MEM_callocN(sizeof(RigidBodyOb), "RigidBodyOb");

	/* set default settings */
	rbo->type = type;
	rbo->mass = 1.0f;

	rbo->fracture_objects = BKE_fracture_container_create(ob);

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

	rbo->mesh_source = RBO_MESH_FINAL;

	/* return this object */
	return rbo;
}

/* Add rigid body constraint to the specified object */
RigidBodyCon *BKE_rigidbody_create_constraint(Object *ob, short type)
{
	RigidBodyCon *rbc;

	/* sanity checks
	 *	- rigidbody world must exist
	 *	- object must exist
	 *	- cannot add constraint if it already exists
	 */
	if (ob == NULL)
		return NULL;

	if (ob->rigidbody_constraint)
		return ob->rigidbody_constraint;

	/* create new settings data, and link it up */
	rbc = MEM_callocN(sizeof(RigidBodyCon), "RigidBodyCon");

	rbc->fracture_constraints = BKE_fracture_constraint_container_create(ob);
	rbc->fracture_constraints->flag |= FM_FLAG_REFRESH_CONSTRAINTS;
	rbc->fracture_constraints->flag |= FMC_FLAG_USE_CONSTRAINTS;

	/* set default settings */
	rbc->type = type;

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

	/* return this object */
	return rbc;
}

/* Add rigid body constraint to the specified object */
RigidBodyShardCon *BKE_rigidbody_create_shard_constraint(short type)
{
	RigidBodyShardCon *rbc;

	/* create new settings data, and link it up */
	rbc = MEM_callocN(sizeof(RigidBodyShardCon), "RigidBodyCon");

	/* set default settings */
	rbc->type = type;

	rbc->mi1 = NULL;
	rbc->mi2 = NULL;

	rbc->flag |= RBC_FLAG_ENABLED;
	rbc->flag &= ~RBC_FLAG_DISABLE_COLLISIONS;
	rbc->flag |= RBC_FLAG_USE_BREAKING;

	rbc->breaking_threshold = 1.0f; /* no good default here, just use 10 for now */
	rbc->num_solver_iterations = 10; /* 10 is Bullet default */

	/* flag all caches as outdated */
	//BKE_rigidbody_cache_reset(rbw); //FM_TODO do elsewhere

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

void BKE_rigidbody_remove_shard_con(RigidBodyWorld *rbw, RigidBodyShardCon *con)
{
	if (rbw && rbw->physics_world && con->physics_constraint) {
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
			BKE_rigidbody_remove_shard_con(scene->rigidbody_world, con);
		}
		
		if (rbw->physics_world && mi->rigidbody->physics_object)
			RB_dworld_remove_body(rbw->physics_world, mi->rigidbody->physics_object);

		if (mi->rigidbody->physics_object) {
			RB_body_delete(mi->rigidbody->physics_object);
			mi->rigidbody->physics_object = NULL;
		}

		if (mi->rigidbody->physics_shape) {
			RB_shape_delete(mi->rigidbody->physics_shape);
			mi->rigidbody->physics_shape = NULL;
		}
	}
}

/* ************************************** */
/* Simulation Interface - Bullet */

/* Update object array and rigid body count so they're in sync with the rigid body group */
static void rigidbody_update_ob_array(RigidBodyWorld *rbw)
{
	GroupObject *go;
	int i = 0;
	
	if (rbw->objects != NULL) {
		MEM_freeN(rbw->objects);
		rbw->objects = NULL;
	}

	rbw->numbodies = BLI_listbase_count(&rbw->group->gobject);
	rbw->objects = MEM_mallocN(sizeof(Object *) * rbw->numbodies, "objects");

	for (go = rbw->group->gobject.first, i = 0; go; go = go->next, i++) {
		Object *ob = go->ob;
		rbw->objects[i] = ob;
	}
}

static void rigidbody_update_sim_world(Scene *scene, RigidBodyWorld *rbw)
{
	float adj_gravity[3];

	/* adjust gravity to take effector weights into account  do this somehow per object*/
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
	if (!(rbw->flag & RBW_FLAG_REFRESH_MODIFIERS))
		rigidbody_update_ob_array(rbw);
}

static void rigidbody_update_sim_ob(Scene *scene, RigidBodyWorld *rbw, Object *ob, RigidBodyShardOb *rbo, float centroid[3])
{
	float loc[3];
	float rot[4];
	float scale[3], centr[3], adj_gravity[3];

	RigidBodyOb *rb = ob->rigidbody_object;

	/* adjust gravity to take effector weights into account  do this somehow per object hrm maybe keep the gravity setting (effector weights
		to make quick global adjustments*/

	if (scene->physics_settings.flag & PHYS_GLOBAL_GRAVITY) {
		copy_v3_v3(adj_gravity, scene->physics_settings.gravity);
		mul_v3_fl(adj_gravity, rbw->effector_weights->global_gravity * rbw->effector_weights->weight[0]);
		//FM_TODO -> override with fc->effector_weights
	}
	else {
		zero_v3(adj_gravity);
	}

	/* only update if rigid body exists */
	if (rbo->physics_object == NULL)
		return;

	if (rb->shape == RB_SHAPE_TRIMESH && rb->flag & RBO_FLAG_USE_DEFORM) {
		DerivedMesh *dm = ob->derivedDeform;
		if (dm) {
			MVert *mvert = dm->getVertArray(dm);
			int totvert = dm->getNumVerts(dm);
			BoundBox *bb = BKE_object_boundbox_get(ob);

			RB_shape_trimesh_update(rbo->physics_shape, (float *)mvert, totvert, sizeof(MVert), bb->vec[0], bb->vec[6]);
		}
	}
	copy_v3_v3(centr, centroid);
	
	mat4_decompose(loc, rot, scale, ob->obmat);

	/* update scale for all objects */
	RB_body_set_scale(rbo->physics_object, scale);
	/* compensate for embedded convex hull collision margin */
	if (!(rb->flag & RBO_FLAG_USE_MARGIN) && rb->shape == RB_SHAPE_CONVEXH)
		RB_shape_set_margin(rbo->physics_shape, RBO_GET_MARGIN(rb) * MIN3(scale[0], scale[1], scale[2]));

	/* make transformed objects temporarily kinmatic so that they can be moved by the user during simulation */
	if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) {
		RB_body_set_kinematic_state(rbo->physics_object, true);
		RB_body_set_mass(rbo->physics_object, 0.0f);
	}

	/* update rigid body location and rotation for kinematic bodies */
	if (rb->flag & RBO_FLAG_KINEMATIC || (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)) {
		mul_v3_v3(centr, scale);
		mul_qt_v3(rot, centr);
		add_v3_v3(loc, centr);
		RB_body_activate(rbo->physics_object);
		RB_body_set_loc_rot(rbo->physics_object, loc, rot);
	}
	/* update influence of effectors - but don't do it on an effector */
	/* only dynamic bodies need effector update */
	else if (rbo->type == RBO_TYPE_ACTIVE && ((ob->pd == NULL) || (ob->pd->forcefield == PFIELD_NULL))) {
		EffectorWeights *effector_weights = rbw->effector_weights; //override with fc->effector_weights TODO;
		EffectedPoint epoint;
		ListBase *effectors;

		/* get effectors present in the group specified by effector_weights */
		effectors = pdInitEffectors(scene, ob, NULL, effector_weights, true);
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

static void validateShard(Scene *scene, MeshIsland *mi, Object *ob, int rebuild, int transfer_speed)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	RigidBodyWorld *rbw = scene->rigidbody_world;

	if (mi == NULL || mi->rigidbody == NULL || rbw == NULL) {
		return;
	}

	if (rebuild || (rb->flag & RBO_FLAG_KINEMATIC_REBUILD)) {
		/* World has been rebuilt so rebuild object */
		BKE_rigidbody_validate_sim_shard(scene, mi, ob, true, transfer_speed);
	}
	else if (rb->flag & RBO_FLAG_NEEDS_VALIDATE || mi->rigidbody->flag & RBO_FLAG_NEEDS_VALIDATE) {
		BKE_rigidbody_validate_sim_shard(scene, mi, ob, false, transfer_speed);
	}
	/* refresh shape... */
	if (rb->flag & RBO_FLAG_NEEDS_RESHAPE) {
		/* mesh/shape data changed, so force shape refresh */
		BKE_rigidbody_validate_sim_shard_shape(mi, ob, true);
		/* now tell RB sim about it */
		// XXX: we assume that this can only get applied for active/passive shapes that will be included as rigidbodies
		RB_body_set_collision_shape(mi->rigidbody->physics_object, mi->rigidbody->physics_shape);
	}
	mi->rigidbody->flag &= ~(RBO_FLAG_NEEDS_VALIDATE | RBO_FLAG_NEEDS_RESHAPE);
}

static void handle_breaking_percentage(Object *ob, MeshIsland *mi, RigidBodyWorld *rbw, int breaking_percentage)
{
	RigidBodyCon *rbc = ob->rigidbody_constraint;
	ConstraintContainer *cc = rbc->fracture_constraints;
	int broken_cons = 0, cons = 0, i = 0, cluster_cons = 0, broken_cluster_cons = 0;
	RigidBodyShardCon *con;

	cons = mi->participating_constraint_count;
	/* calc ratio of broken cons here, per MeshIsland and flag the rest to be broken too*/
	for (i = 0; i < cons; i++) {
		con = mi->participating_constraints[i];
		if (con && con->physics_constraint) {
			if (cc->cluster_breaking_percentage > 0)
			{
				/*only count as broken if between clusters!*/
				if (con->mi1->particle_index != con->mi2->particle_index)
				{
					cluster_cons++;

					if (!RB_constraint_is_enabled(con->physics_constraint)) {
						broken_cluster_cons++;
					}
				}
			}

			if (!RB_constraint_is_enabled(con->physics_constraint)) {
				broken_cons++;
			}
		}
	}

	if (cluster_cons > 0) {
		if ((float)broken_cluster_cons / (float)cluster_cons * 100 >= cc->cluster_breaking_percentage) {
			for (i = 0; i < cons; i++) {
				con = mi->participating_constraints[i];
				if (con && con->mi1->particle_index != con->mi2->particle_index) {
					if (cc->flag & FMC_FLAG_USE_BREAKING)
					{
						con->flag &= ~RBC_FLAG_ENABLED;
						con->flag |= RBC_FLAG_NEEDS_VALIDATE;

						if (con->physics_constraint) {
							RB_constraint_set_enabled(con->physics_constraint, false);
							activateRigidbody(con->mi1->rigidbody, rbw, con->mi1, ob);
							activateRigidbody(con->mi2->rigidbody, rbw, con->mi2, ob);
						}
					}
				}
			}
		}
	}


	if (cons > 0) {
		if ((float)broken_cons / (float)cons * 100 >= breaking_percentage) {
			/* break all cons if over percentage */
			for (i = 0; i < cons; i++) {
				con = mi->participating_constraints[i];
				if (con && (cc->flag & FMC_FLAG_USE_BREAKING))
				{
					con->flag &= ~RBC_FLAG_ENABLED;
					con->flag |= RBC_FLAG_NEEDS_VALIDATE;

					if (con->physics_constraint) {
						RB_constraint_set_enabled(con->physics_constraint, false);
						activateRigidbody(con->mi1->rigidbody, rbw, con->mi1, ob);
						activateRigidbody(con->mi2->rigidbody, rbw, con->mi2, ob);
					}
				}
			}
		}
	}
}

static void handle_breaking_angle(Object *ob, RigidBodyShardCon *rbsc, RigidBodyWorld *rbw,
                                  float anglediff, float weight, float breaking_angle)
{
	RigidBodyCon *rbc = ob->rigidbody_constraint;
	ConstraintContainer *cc = rbc->fracture_constraints;

	if ((cc->breaking_angle > 0 || ((cc->flag & FMC_FLAG_BREAKING_ANGLE_WEIGHTED) && weight > 0)) &&
		(anglediff > breaking_angle))
	{
		/* if we have cluster breaking angle, then only treat equal cluster indexes like the default, else all */
		if ((cc->cluster_breaking_angle > 0 && rbsc->mi1->particle_index == rbsc->mi2->particle_index) ||
			 cc->cluster_breaking_angle == 0)
		{
			if (cc->flag & FMC_FLAG_USE_BREAKING)
			{
				rbsc->flag &= ~RBC_FLAG_ENABLED;
				rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;

				if (rbsc->physics_constraint) {
					RB_constraint_set_enabled(rbsc->physics_constraint, false);
					activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
					activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
				}
			}
		}
	}

	if ((cc->cluster_breaking_angle > 0) && (rbsc->mi1->particle_index != rbsc->mi2->particle_index)
		&& anglediff > cc->cluster_breaking_angle)
	{
		if (cc->flag & FMC_FLAG_USE_BREAKING)
		{
			rbsc->flag &= ~RBC_FLAG_ENABLED;
			rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;

			if (rbsc->physics_constraint) {
				RB_constraint_set_enabled(rbsc->physics_constraint, false);
				activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
				activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
			}
		}
	}
}

static void handle_breaking_distance(Object *ob, RigidBodyShardCon *rbsc, RigidBodyWorld *rbw,
                                     float distdiff, float weight, float breaking_distance)
{
	RigidBodyCon *rbc = ob->rigidbody_constraint;
	ConstraintContainer *cc = rbc->fracture_constraints;

	if ((cc->breaking_distance > 0 || ((cc->flag & FMC_FLAG_BREAKING_DISTANCE_WEIGHTED) && weight > 0)) &&
		(distdiff > breaking_distance))
	{
		/* if we have cluster breaking distance, then only treat equal cluster indexes like the default, else all */
		if ((cc->cluster_breaking_distance > 0 && rbsc->mi1->particle_index == rbsc->mi2->particle_index) ||
			 cc->cluster_breaking_distance == 0)
		{
			if (cc->flag & FMC_FLAG_USE_BREAKING)
			{
				rbsc->flag &= ~RBC_FLAG_ENABLED;
				rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;

				if (rbsc->physics_constraint) {
					RB_constraint_set_enabled(rbsc->physics_constraint, false);
					activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
					activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
				}
			}
		}
	}

	if ((cc->cluster_breaking_distance > 0) && (rbsc->mi1->particle_index != rbsc->mi2->particle_index)
		&& distdiff > cc->cluster_breaking_distance)
	{
		if (cc->flag & FMC_FLAG_USE_BREAKING)
		{
			rbsc->flag &= ~RBC_FLAG_ENABLED;
			rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;

			if (rbsc->physics_constraint) {
				RB_constraint_set_enabled(rbsc->physics_constraint, false);
				activateRigidbody(rbsc->mi1->rigidbody, rbw, rbsc->mi1, ob);
				activateRigidbody(rbsc->mi2->rigidbody, rbw, rbsc->mi2, ob);
			}
		}
	}
}
static void do_update_constraint_container(Scene* scene, Object *ob, bool rebuild)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyCon *rbc = ob->rigidbody_constraint;
	ConstraintContainer *cc = rbc->fracture_constraints;
	RigidBodyShardCon *con;
	float max_con_mass = 0;
	RigidBodyShardCon *rbsc;

	if (cc->flag & FMC_FLAG_USE_MASS_DEPENDENT_THRESHOLDS) {
		max_con_mass = BKE_rigidbody_calc_max_con_mass(ob);
	}

#if 0
	rbsc = cc->constraint_map.first;
	while (rbsc) {
		if (rbsc->flag & RBC_FLAG_PURGE_ON_VALIDATE || rbsc->mi1 == NULL || rbsc->mi2 == NULL)
		{
			printf("Purging constraint...\n");
			BLI_remlink(&cc->constraint_map, rbsc);
			BKE_rigidbody_remove_shard_con(rbw, rbsc);
			MEM_freeN(rbsc);
			rbsc = NULL;
		}
		else
		{
			rbsc = rbsc->next;
		}
	}
#endif

	for (con = cc->constraint_map.first; con; con = con->next)
	{
		if (cc->flag & FMC_FLAG_USE_BREAKING && con->mi1 != NULL && con->mi2 != NULL)
		{
			int iterations;
			float breaking_angle, breaking_distance;
			float weight = (con->mi1->thresh_weight + con->mi2->thresh_weight) * 0.5f;
			int breaking_percentage = (cc->flag & FMC_FLAG_BREAKING_PERCENTAGE_WEIGHTED) ?
			                          (cc->breaking_percentage * weight) : cc->breaking_percentage;

			if (cc->breaking_percentage > 0 || ((cc->flag & FMC_FLAG_BREAKING_PERCENTAGE_WEIGHTED) && weight > 0)) {
				handle_breaking_percentage(ob, con->mi1, rbw, breaking_percentage);
			}

			weight = MIN2(con->mi1->thresh_weight, con->mi2->thresh_weight);
			breaking_angle = (cc->flag & FMC_FLAG_BREAKING_ANGLE_WEIGHTED) ?
			                  cc->breaking_angle * weight : cc->breaking_angle;

			breaking_distance = (cc->flag & FMC_FLAG_BREAKING_DISTANCE_WEIGHTED) ?
			                     cc->breaking_distance * weight : cc->breaking_distance;

			if (cc->solver_iterations_override == 0) {
				iterations = rbw->num_solver_iterations;
			}
			else {
				//FM_TODO, check whether we are in same object as well (index might be same, but in different objs)
				if ((con->mi1->particle_index != -1) && (con->mi1->particle_index == con->mi2->particle_index)) {
					iterations = cc->cluster_solver_iterations_override;
				}
				else {
					iterations = cc->solver_iterations_override;
				}
			}

			if (iterations > 0) {
				con->flag |= RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS;
				con->num_solver_iterations = iterations;
			}

			if ((cc->flag & FMC_FLAG_USE_MASS_DEPENDENT_THRESHOLDS)) {
				BKE_rigidbody_calc_threshold(max_con_mass, ob, con);
			}

			if (((cc->breaking_angle) > 0) || ((cc->flag & FMC_FLAG_BREAKING_ANGLE_WEIGHTED) && weight > 0) ||
				(((cc->breaking_distance > 0) || ((cc->flag & FMC_FLAG_BREAKING_DISTANCE_WEIGHTED) && weight > 0) ||
				 (cc->cluster_breaking_angle > 0 || cc->cluster_breaking_distance > 0)) && !rebuild ))
			{
				float dist, angle, distdiff, anglediff;
				calc_dist_angle(con, &dist, &angle);

				anglediff = fabs(angle - con->start_angle);
				distdiff = fabs(dist - con->start_dist);

				/* Treat angles here */
				handle_breaking_angle(ob, con, rbw, anglediff, weight, breaking_angle);

				/* Treat distances here */
				handle_breaking_distance(ob, con, rbw, distdiff, weight, breaking_distance);

			}

			if (rebuild || con->mi1->rigidbody->flag & RBO_FLAG_KINEMATIC_REBUILD ||
				con->mi2->rigidbody->flag & RBO_FLAG_KINEMATIC_REBUILD) {
				/* World has been rebuilt so rebuild constraint */
				BKE_rigidbody_validate_sim_shard_constraint(rbw, ob, con, true);
				BKE_rigidbody_start_dist_angle(con);
			}

			else if (con->flag & RBC_FLAG_NEEDS_VALIDATE) {
				BKE_rigidbody_validate_sim_shard_constraint(rbw, ob, con, false);
			}

			if (con->physics_constraint && rbw && (rbw->flag & RBW_FLAG_REBUILD_CONSTRAINTS)) {
				RB_constraint_set_enabled(con->physics_constraint, true);
			}

			con->flag &= ~RBC_FLAG_NEEDS_VALIDATE;
		}
	}
	rbc->flag &= ~RBC_FLAG_NEEDS_VALIDATE;
}

static void do_update_container(Scene* scene, Object* ob, RigidBodyWorld *rbw, bool rebuild)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	FractureContainer *fc = rb->fracture_objects;
	FractureState *fs = fc->current;
	MeshIsland *mi;

#if 0
			if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
			{
				int frame = (int)BKE_scene_frame_get(scene);
				if (BKE_lookup_mesh_state(fmd, frame, true))
				{
					rigidbody_update_ob_array(rbw);
				}
			}
#endif

	for (mi = fs->island_map.first; mi; mi = mi->next) {
		/* as usual, but for each shard now, and no constraints*/
		/* perform simulation data updates as tagged */
		/* refresh object... */
		int do_rebuild = rebuild;
		validateShard(scene, mi, ob, do_rebuild, fc->fracture_mode == MOD_FRACTURE_DYNAMIC);

		/* update simulation object... */
		rigidbody_update_sim_ob(scene, rbw, ob, mi->rigidbody, mi->centroid);
	}
	rbw->flag &= ~RBW_FLAG_OBJECT_CHANGED;
}

/* Updates and validates world, bodies and shapes.
 * < rebuild: rebuild entire simulation
 */
static void rigidbody_update_simulation_object(Scene *scene, Object* ob, RigidBodyWorld *rbw, bool rebuild)
{

	/* update world but only once !!! TODO*/
	if (rebuild && rbw->flag & RBW_FLAG_NEEDS_REBUILD) {
		BKE_rigidbody_validate_sim_world(scene, rbw, true);
		rigidbody_update_sim_world(scene, rbw);
		rbw->flag &= ~RBW_FLAG_NEEDS_REBUILD;
	}

	if (ob && (ob->type == OB_MESH || ob->type == OB_CURVE || ob->type == OB_SURF || ob->type == OB_FONT)) {
		if (ob->rigidbody_object)
			do_update_container(scene, ob, rbw, rebuild);
	}

	//here we have "inner" constraints... but should we make such a distinction at all.... ?
	//if (ob->rigidbody_constraint)
	//	do_update_constraint_container(scene, ob, rebuild);

	//perhaps do this if we only have 1 island, to mimic old behavior ? TODO (move object with it)
	rbw->flag &= ~RBW_FLAG_REFRESH_MODIFIERS;
}

static void rigidbody_update_simulation_post_step(Object *ob)
{
	//GroupObject *go;
	RigidBodyShardOb *rbo;
	RigidBodyOb *rb = ob->rigidbody_object;
	MeshIsland *mi;
	FractureContainer *fc = rb->fracture_objects;
	FractureState *fs = fc->current;

	for (mi = fs->island_map.first; mi; mi = mi->next) {
		rbo = mi->rigidbody;

		/* reset kinematic state for transformed objects */
		if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) {
			RB_body_set_kinematic_state(rbo->physics_object, rb->flag & RBO_FLAG_KINEMATIC || rb->flag & RBO_FLAG_DISABLED);
			RB_body_set_mass(rbo->physics_object, RBO_GET_MASS(rbo)); /*mass should be calculated already and stored in rbo */
			/* deactivate passive objects so they don't interfere with deactivation of active objects */
			if (rb->type == RBO_TYPE_PASSIVE)
				RB_body_deactivate(rbo->physics_object);
		}

		/* update stored velocities, can be set again after sim rebuild */
		if (fc->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			RB_body_get_linear_velocity(rbo->physics_object, rbo->lin_vel);
			RB_body_get_angular_velocity(rbo->physics_object, rbo->ang_vel);
		}
	}
}

bool BKE_rigidbody_check_sim_running(RigidBodyWorld *rbw, Object* ob, float ctime)
{
	//test pointcache of first participant, if any.... TODO
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

	return (rbw && (rbw->flag & RBW_FLAG_MUTED) == 0 && ctime > fc->pointcache->startframe);
}

static void do_sync_container(Object *ob, RigidBodyWorld *rbw, float ctime)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	FractureContainer *fc = rb->fracture_objects;
	FractureState *fs;
	MeshIsland *mi;
	float size[3] = {1, 1, 1};
	float centr[3];
	int i = 0;

	if (!fc || (fc && fc->flag & (FM_FLAG_REFRESH_SHAPE | FM_FLAG_SKIP_MASS_CALC)))
	{
		return;
	}

	fs = fc->current;

	for (mi = fs->island_map.first; mi; mi = mi->next)
	{
		RigidBodyShardOb *rbo = mi->rigidbody;
		if ((ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ) ||
			((rb) && (rb->flag & RBO_FLAG_KINEMATIC)))
		{
			if (ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ && rbw) {
				rbw->flag |= RBW_FLAG_OBJECT_CHANGED;
				BKE_rigidbody_cache_reset(rbw);

				/* re-enable all constraints as well, hmmm (participating ones ?) */
				for (i = 0; i < mi->participating_constraint_count; i++)
				{
					RigidBodyShardCon *con = mi->participating_constraints[i];
					con->flag |= RBC_FLAG_ENABLED;
					con->flag |= RBC_FLAG_NEEDS_VALIDATE;
				}
			}
		}

		/* use rigid body transform after cache start frame if objects is not being transformed */
		if (BKE_rigidbody_check_sim_running(rbw, ob, ctime) && !(ob->flag & SELECT && G.moving & G_TRANSFORM_OBJ)) {

			/* keep original transform when the simulation is muted */
			if (rbw->flag & RBW_FLAG_MUTED) {
				return;
			}
		}
		else
		{
			/* otherwise set rigid body transform to current obmat*/
			mat4_to_loc_quat(rbo->pos, rbo->orn, ob->obmat);
			mat4_to_size(size, ob->obmat);
			copy_v3_v3(centr, mi->centroid);
			mul_v3_v3(centr, size);
			mul_qt_v3(rbo->orn, centr);
			add_v3_v3(rbo->pos, centr);

			//if ((!(rb->flag & RBO_FLAG_KINEMATIC) && rb->type == RBO_TYPE_ACTIVE))
			rbw->flag |= RBW_FLAG_OBJECT_CHANGED;
		}
		BKE_rigidbody_update_cell(mi, ob, rbo->pos, rbo->orn);
	}
}

/* Sync rigid body and object transformations */
void BKE_rigidbody_sync_transforms(RigidBodyWorld *rbw, Object *ob, float ctime)
{
	if (rbw == NULL || ob->rigidbody_object == NULL)
		return;

	if (ob->rigidbody_object)
	{
		RigidBodyOb *rbo = ob->rigidbody_object;
		if (rbo && rbo->fracture_objects)
		{
			FracMesh *fm = rbo->fracture_objects->current->frac_mesh;
			if (fm && fm->running == 0)
			{
				do_sync_container(ob, rbw, ctime);
			}
		}
	}

	//do_sync_container(ob, rbw, ctime);

#if 0 //FM_TODO
	/* keep original transform for kinematic and passive objects */
	if (ELEM(NULL, rbw, rbo) || rbo->flag & RBO_FLAG_KINEMATIC || rbo->type == RBO_TYPE_PASSIVE)
		return;
#endif

}

static void do_reset_rigidbody(RigidBodyShardOb *rbo, Object *ob, MeshIsland* mi, float loc[3],
                              float rot[3], float quat[4], float rotAxis[3], float rotAngle)
{
	/* return rigid body and object to their initial states */
	copy_v3_v3(rbo->pos, ob->loc);
	if (mi)
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
			RB_body_set_kinematic_state(rbo->physics_object, true);
		RB_body_set_loc_rot(rbo->physics_object, rbo->pos, rbo->orn);
	}
}

/* Used when cancelling transforms - return rigidbody and object to initial states */
void BKE_rigidbody_aftertrans_update(Object *ob, float loc[3], float rot[3], float quat[4], float rotAxis[3], float rotAngle)
{
	RigidBodyShardOb *rbo;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	MeshIsland *mi;

	for (mi = fs->island_map.first; mi; mi = mi->next)
	{
		rbo = mi->rigidbody;
		do_reset_rigidbody(rbo, ob, mi, loc, rot, quat, rotAxis, rotAngle);
		// RB_TODO update rigid body physics object's loc/rot for dynamic objects here as well (needs to be done outside bullet's update loop)
	}
}

static void restoreKinematic(Object *ob)
{
	/*restore kinematic state of shards if object is kinematic, need to store old state in container now....FM_TODO */
	RigidBodyOb *rb = ob->rigidbody_object;
	if (rb && (rb->flag & RBO_FLAG_USE_KINEMATIC_DEACTIVATION)) {
		FractureContainer* fc = rb->fracture_objects;
		MeshIsland* mi;
		for (mi = fc->current->island_map.first; mi; mi = mi->next)
		{
			RigidBodyShardOb *rbo = mi->rigidbody;
			if (rbo)
			{
				if ((rb->flag & RBO_FLAG_KINEMATIC) != (rbo->flag & RBO_FLAG_KINEMATIC))
				{
					if (rb->flag & RBO_FLAG_KINEMATIC)
					{
						rbo->flag |= RBO_FLAG_KINEMATIC;
					}
					else
					{
						rbo->flag &= ~RBO_FLAG_KINEMATIC;
					}

					rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
				}
			}
		}
	}
}

void BKE_rigidbody_cache_reset(RigidBodyWorld *rbw)
{
	if (rbw) {
		GroupObject* go;
		for (go = rbw->group->gobject.first; go; go = go->next)
		{
			Object *ob = go->ob;
			RigidBodyOb *rb = ob->rigidbody_object;
			if (rb) {
				FractureContainer *fc = rb->fracture_objects;
				fc->pointcache->flag |= PTCACHE_OUTDATED; //FM TODO flag all caches in list (dynamic) as outdated
			}
		}
		//restoreKinematic(rbw);
	}
}

/* ------------------ */

/* Rebuild rigid body world */
/* NOTE: this needs to be called before frame update to work correctly */
void BKE_rigidbody_rebuild_world(Scene *scene, float ctime)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	GroupObject *go;

	PointCache *cache;
	PTCacheID pid;
	int startframe, endframe;

#if 0
	if (!(rbw->flag & RBW_FLAG_NEEDS_REBUILD))
	{
		return;
	}
#endif

#if 0
	//flag this once, so it doesnt get called every time in the loop
	rbw->flag |= RBW_FLAG_NEEDS_REBUILD;
	/* update world but only once !!! TODO*/
	if (rbw->flag & RBW_FLAG_NEEDS_REBUILD) {
		BKE_rigidbody_validate_sim_world(scene, rbw, true);
		rigidbody_update_sim_world(scene, rbw);
		rbw->flag &= ~RBW_FLAG_NEEDS_REBUILD;
	}
#endif

	for (go = rbw->group->gobject.first; go; go = go->next)
	{
		RigidBodyOb *rb = go->ob->rigidbody_object;
		FractureContainer *fc = rb->fracture_objects;

#if 0
		if (ctime == -1)
		{
			rigidbody_update_simulation_object(scene, go->ob, rbw, true);
			continue;
		}
#endif

		if (!fc)
			continue;

		BKE_ptcache_id_from_rigidbody(&pid, go->ob, fc);
		BKE_ptcache_id_time(&pid, scene, ctime, &startframe, &endframe, NULL);
		cache = fc->pointcache;

		/* flag cache as outdated if we don't have a world or number of objects in the simulation has changed */
		//rigidbody_group_count_items(&rbw->group->gobject, &shards, &objects);
		if (rbw->physics_world == NULL /*|| rbw->numbodies != objects*/) {
			cache->flag |= PTCACHE_OUTDATED;
		}

		if (ctime == startframe + 1 && rbw->ltime == startframe) {
			if (cache->flag & PTCACHE_OUTDATED) {
				BKE_ptcache_id_reset(scene, &pid, PTCACHE_RESET_OUTDATED);
				rigidbody_update_simulation_object(scene, go->ob, rbw, true);
				BKE_ptcache_validate(cache, (int)ctime);
				cache->last_exact = 0;
				cache->flag &= ~PTCACHE_REDO_NEEDED;
			}
		}
	}
}

/* Run RigidBody simulation for the specified physics world */
void BKE_rigidbody_do_simulation(Scene *scene, float ctime)
{
	float timestep;
	RigidBodyWorld *rbw = scene->rigidbody_world;
	GroupObject *go;
	bool is_cached = false;

	//flag this once, so it doesnt get called every time in the loop
	rbw->flag |= RBW_FLAG_NEEDS_REBUILD;

	//iterate over objects (rigidbodies), process caches
	for (go = rbw->group->gobject.first; go; go = go->next)
	{
		PointCache *cache;
		int startframe, endframe;
		PTCacheID pid;
		Object *ob = go->ob;
		RigidBodyOb *rb = ob->rigidbody_object;
		FractureContainer *fc = rb->fracture_objects;
		FractureState *fs = fc->current;

		if ((fs->frac_mesh->running == 1 || fs->frac_mesh->cancel == 1) && (fs->flag & FM_FLAG_EXECUTE_THREADED))
		{
			// do not even think of simulate when still fracturing....
			rbw->flag &= ~RBW_FLAG_NEEDS_REBUILD;
			return;
		}

		BKE_ptcache_id_from_rigidbody(&pid, ob, fc);
		BKE_ptcache_id_time(&pid, scene, ctime, &startframe, &endframe, NULL);
		cache = fc->pointcache;
		fc->flag &= ~FM_FLAG_SKIP_STEPPING;

		if ((ob->type == OB_MESH) && (fc->flag & FM_FLAG_REFRESH_SHAPE))
		{
			//this is meant for backwards compat, couldnt get shape in do_version yet (no ref to ob->data)
			//so do this here
			DerivedMesh *dm = CDDM_from_mesh(ob->data);
			BKE_fracture_container_initialize(ob, dm);
			fc->flag &= ~FM_FLAG_REFRESH_SHAPE;
		}

/*		if (rbw->ltime == -1) {
			rbw->ltime = startframe;
			rigidbody_update_simulation_object(scene, ob, rbw, true);
		}*/

		/*trigger dynamic update FM_TODO, use other flag*/
#if 0
		if ((rbw->flag & RBW_FLAG_OBJECT_CHANGED))
		{
			rbw->flag &= ~RBW_FLAG_OBJECT_CHANGED;
			rigidbody_update_simulation_object(scene, ob, rbw, true);
			rbw->flag &= ~RBW_FLAG_REFRESH_MODIFIERS;
		}
#endif
		if (ctime < startframe)
			ctime = startframe;

		if ((ctime == startframe) && (cache->flag == PTCACHE_OUTDATED)) {
			/* rebuild constraints */
			rbw->flag |= RBW_FLAG_REBUILD_CONSTRAINTS;

			rbw->ltime = startframe;
			if (rbw->flag & RBW_FLAG_OBJECT_CHANGED)
			{       /* flag modifier refresh at their next execution XXX TODO -> still used ? */
				rbw->flag |= RBW_FLAG_REFRESH_MODIFIERS;
				rigidbody_update_simulation_object(scene, ob, rbw, true);

				rbw->flag &= ~RBW_FLAG_OBJECT_CHANGED;
				continue;
			}
		}
		/* make sure we don't go out of cache frame range */
		else if (ctime > endframe) {
			ctime = endframe;
		}

		/* don't try to run the simulation if we don't have a world yet but allow reading baked cache */
		if (rbw->physics_world == NULL && !(cache->flag & PTCACHE_BAKED))
		{
			if (rbw->flag & RBW_FLAG_NEEDS_REBUILD)
			{
				BKE_rigidbody_validate_sim_world(scene, rbw, false);
				rbw->flag &= ~RBW_FLAG_NEEDS_REBUILD;
			}
			continue;
		}
		else if (rbw->objects == NULL)
			rigidbody_update_ob_array(rbw);

		/* try to read from caches */
		// RB_TODO deal with interpolated, old and baked results
		if (BKE_ptcache_read(&pid, ctime)) {
			//printf("Cache read:  %d\n", (int)ctime);
			BKE_ptcache_validate(cache, (int)ctime);

			rbw->ltime = ctime;

			//tag this container as to be skipped in step (kinematic, but cache driven ?)
			fc->flag |= FM_FLAG_SKIP_STEPPING;
			is_cached = true;
			continue;
		}
		else if (rbw->ltime == startframe /* || rb->flag & RBO_FLAG_NEEDS_VALIDATE*/)
		{
			//ensure flag if not set... hmmm FM_TODO
			//rb->flag |= RBO_FLAG_NEEDS_VALIDATE;
			restoreKinematic(ob);
			rigidbody_update_simulation_object(scene, ob, rbw, true);
			rb->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
		}

		/* advance simulation, we can only step one frame forward */
		if ((ctime == rbw->ltime + 1) && !(cache->flag & PTCACHE_BAKED)) {
			/* write cache for first frame when on second frame */
			if (rbw->ltime == startframe && (cache->flag & PTCACHE_OUTDATED || cache->last_exact == 0)) {
				BKE_ptcache_write(&pid, startframe);
			}

			if (rbw->ltime > startframe) {
				rbw->flag &= ~RBW_FLAG_REBUILD_CONSTRAINTS;
			}

			/* update and validate simulation */
			rigidbody_update_simulation_object(scene, ob, rbw, false);
		}
	}

	if (rbw->constraints && !is_cached)
	{
		for (go = rbw->constraints->gobject.first; go; go = go->next)
		{
			Object *ob = go->ob;
			//if ((ctime == rbw->ltime + 1) && !is_baked && ob->type == OB_EMPTY)
			{
				do_update_constraint_container(scene, ob, ob->rigidbody_constraint->flag & RBC_FLAG_NEEDS_VALIDATE);
			}
		}
	}

	/* calculate how much time elapsed since last step in seconds */
	timestep = 1.0f / (float)FPS * (ctime - rbw->ltime) * rbw->time_scale;
	/* step simulation by the requested timestep, steps per second are adjusted to take time scale into account */
	RB_dworld_step_simulation(rbw->physics_world, timestep, INT_MAX, 1.0f / (float)rbw->steps_per_second * min_ff(rbw->time_scale, 1.0f));

	for (go = rbw->group->gobject.first; go; go = go->next)
	{
		PointCache *cache;
		PTCacheID pid;
		Object *ob = go->ob;
		RigidBodyOb *rb = ob->rigidbody_object;
		FractureContainer *fc = rb->fracture_objects;
		int startframe, endframe;

		if (fc->flag & FM_FLAG_SKIP_STEPPING)
			continue;

		BKE_ptcache_id_from_rigidbody(&pid, ob, fc);
		BKE_ptcache_id_time(&pid, scene, ctime, &startframe, &endframe, NULL);
		cache = fc->pointcache;

		rigidbody_update_simulation_post_step(ob);

		/* write cache for current frame, but not when its baked already (here accidentally a write happened on startframe for 1 object */
		if ((ctime > rbw->ltime) && !(cache->flag & PTCACHE_BAKED))
		{
			//printf("Write cache: %s  %.2f\n", ob->id.name + 2, ctime);
			BKE_ptcache_validate(cache, (int)ctime);
			BKE_ptcache_write(&pid, (unsigned int)ctime);
		}
	}

	rbw->ltime = ctime;
}
/* ************************************** */
/* Copying Methods --------------------- */

/* These just copy the data, clearing out references to physics objects.
 * Anything that uses them MUST verify that the copied object will
 * be added to relevant groups later...
 */

RigidBodyOb *BKE_rigidbody_copy_object(Object *ob, Object* obN)
{
	RigidBodyOb *rboN = NULL;

	if (ob->rigidbody_object) {
		/* just duplicate the whole struct first (to catch all the settings) */
		rboN = MEM_dupallocN(ob->rigidbody_object);

		/* tag object as needing to be verified */
		rboN->flag |= RBO_FLAG_NEEDS_VALIDATE;

		/* clear out all the fields which need to be revalidated later */
		//rboN->physics_object = NULL;
		//rboN->physics_shape = NULL;

		rboN->fracture_objects = BKE_fracture_container_copy(ob, obN);
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
		//rbcN->physics_constraint = NULL;
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
void BKE_rigidbody_validate_sim_world(Scene *scene, RigidBodyWorld *rbw, bool rebuild) {}
void BKE_rigidbody_calc_volume(Object *ob, float *r_vol) { if (r_vol) *r_vol = 0.0f; }
void BKE_rigidbody_calc_center_of_mass(Object *ob, float r_com[3]) { zero_v3(r_com); }
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
