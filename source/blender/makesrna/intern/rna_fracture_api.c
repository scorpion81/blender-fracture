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
 * Copyright (C) 2017 by Martin Felke.
 * All rights reserved.
 *
 * The Original Code is: all of this file
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/makesrna/intern/rna_fracture_api.c
 *  \ingroup RNA
 */

#include <stdlib.h>

#include "DNA_mesh_types.h"
#include "DNA_meta_types.h"
#include "DNA_rigidbody_types.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "rna_internal.h"

#ifdef RNA_RUNTIME

static MeshIsland *rna_FractureModifier_mesh_island_new(ID* id, FractureModifierData *fmd, Object* ob)
{
	Object *owner = (Object*)id;
	if (ob != owner)
	{
		MeshIsland* mi = BKE_fracture_mesh_island_add(fmd, owner, ob);
		return mi;
	}

	return NULL;
}

static void rna_FractureModifier_mesh_island_remove(ID *UNUSED(id), FractureModifierData *fmd, ReportList *reports, MeshIsland* mi)
{
	if (BLI_findindex(&fmd->meshIslands, mi) == -1) {
		BKE_reportf(reports, RPT_ERROR, "MeshIsland '%s' not in this fracture modifier", mi->name);
		return;
	}

	BKE_fracture_mesh_island_remove(fmd, mi);
}

static void rna_FractureModifier_mesh_island_clear(ID *UNUSED(id), FractureModifierData *fmd)
{
	BKE_fracture_mesh_island_remove_all(fmd);
}

static RigidBodyShardCon *rna_FractureModifier_mesh_constraint_new(ID *UNUSED(id), FractureModifierData *fmd,
                                                                   MeshIsland* mi1, MeshIsland* mi2, int type)
{
	RigidBodyShardCon* con = BKE_fracture_mesh_islands_connect(fmd, mi1, mi2, type);
	return con;
}

static void rna_FractureModifier_mesh_constraint_remove(ID *UNUSED(id), FractureModifierData *fmd, ReportList *reports, RigidBodyShardCon *con)
{
	if (con && BLI_findindex(&fmd->meshConstraints, con) == -1) {
		BKE_reportf(reports, RPT_ERROR, "MeshConstraint '%s' not in this fracture modifier", con->name);
		return;
	}

	if (con)
		BKE_fracture_mesh_constraint_remove(fmd, con);
}

static void rna_FractureModifier_mesh_constraint_clear(ID *UNUSED(id), FractureModifierData *fmd)
{
	BKE_fracture_mesh_constraint_remove_all(fmd);
}

static float rna_MeshCon_get_applied_impulse(RigidBodyShardCon *con)
{
#ifdef WITH_BULLET
	if (con && con->physics_constraint)
		return RB_constraint_get_applied_impulse(con->physics_constraint);
#endif
	return 0.0f;
}

static int rna_MeshCon_is_intact(RigidBodyShardCon *con)
{
#ifdef WITH_BULLET
	if (con && con->physics_constraint)
		return RB_constraint_is_enabled(con->physics_constraint);
#endif
	return 0;
}

#define RB_FLAG_SET(dest, value, flag) { \
	if (value) \
		dest |= flag; \
	else \
		dest &= ~flag; \
}

static void rna_MeshCon_enabled_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_ENABLED);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		RB_constraint_set_enabled(rbc->physics_constraint, value);
	}
#endif
}

static void rna_MeshCon_disable_collisions_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_DISABLE_COLLISIONS);

	rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
}

static void rna_MeshCon_use_breaking_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	if (value) {
		rbc->flag |= RBC_FLAG_USE_BREAKING;
#ifdef WITH_BULLET
		if (rbc->physics_constraint) {
			RB_constraint_set_breaking_threshold(rbc->physics_constraint, rbc->breaking_threshold);
		}
#endif
	}
	else {
		rbc->flag &= ~RBC_FLAG_USE_BREAKING;
#ifdef WITH_BULLET
		if (rbc->physics_constraint) {
			RB_constraint_set_breaking_threshold(rbc->physics_constraint, FLT_MAX);
		}
#endif
	}
}

static void rna_MeshCon_breaking_threshold_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->breaking_threshold = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && (rbc->flag & RBC_FLAG_USE_BREAKING)) {
		RB_constraint_set_breaking_threshold(rbc->physics_constraint, value);
	}
#endif
}

static void rna_MeshCon_position_set(PointerRNA *ptr, const float value[3])
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;
	copy_v3_v3(rbc->pos, value);
	rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
}

static void rna_MeshCon_orientation_set(PointerRNA *ptr, const float value[4])
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;
	copy_qt_qt(rbc->orn, value);
	rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
}

static void rna_MeshCon_plastic_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_PLASTIC);

	rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
}

static void rna_MeshCon_override_solver_iterations_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	if (value) {
		rbc->flag |= RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS;
#ifdef WITH_BULLET
		if (rbc->physics_constraint) {
			RB_constraint_set_solver_iterations(rbc->physics_constraint, rbc->num_solver_iterations);
		}
#endif
	}
	else {
		rbc->flag &= ~RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS;
#ifdef WITH_BULLET
		if (rbc->physics_constraint) {
			RB_constraint_set_solver_iterations(rbc->physics_constraint, -1);
		}
#endif
	}
}

static void rna_MeshCon_num_solver_iterations_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->num_solver_iterations = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && (rbc->flag & RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS)) {
		RB_constraint_set_solver_iterations(rbc->physics_constraint, value);
	}
#endif
}

static void rna_MeshCon_spring_stiffness_x_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_stiffness_x = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_X)) {
		RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, value);
	}
#endif
}

static void rna_MeshCon_spring_stiffness_y_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_stiffness_y = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_Y)) {
		RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, value);
	}
#endif
}

static void rna_MeshCon_spring_stiffness_z_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_stiffness_z = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_Z)) {
		RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, value);
	}
#endif
}

static void rna_MeshCon_spring_stiffness_ang_x_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_stiffness_ang_x = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_ANG_X)) {
		RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, value);
	}
#endif
}

static void rna_MeshCon_spring_stiffness_ang_y_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_stiffness_ang_y = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_ANG_Y)) {
		RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, value);
	}
#endif
}

static void rna_MeshCon_spring_stiffness_ang_z_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_stiffness_ang_z = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_ANG_Z)) {
		RB_constraint_set_stiffness_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, value);
	}
#endif
}

static void rna_MeshCon_spring_damping_x_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_damping_x = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_X)) {
		RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, value);
	}
#endif
}

static void rna_MeshCon_spring_damping_y_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_damping_y = value;
#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_Y)) {
		RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, value);
	}
#endif
}

static void rna_MeshCon_spring_damping_z_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_damping_z = value;
#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_Z)) {
		RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, value);
	}
#endif
}

static void rna_MeshCon_spring_damping_ang_x_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_damping_ang_x = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_ANG_X)) {
		RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, value);
	}
#endif
}

static void rna_MeshCon_spring_damping_ang_y_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_damping_ang_y = value;
#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_ANG_Y)) {
		RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, value);
	}
#endif
}

static void rna_MeshCon_spring_damping_ang_z_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->spring_damping_ang_z = value;
#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_6DOF_SPRING && (rbc->flag & RBC_FLAG_USE_SPRING_ANG_Z)) {
		RB_constraint_set_damping_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, value);
	}
#endif
}

static void rna_MeshCon_motor_lin_max_impulse_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->motor_lin_max_impulse = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_MOTOR) {
		RB_constraint_set_max_impulse_motor(rbc->physics_constraint, value, rbc->motor_ang_max_impulse);
	}
#endif
}

static void rna_MeshCon_use_motor_lin_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_MOTOR_LIN);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		RB_constraint_set_enable_motor(rbc->physics_constraint, rbc->flag & RBC_FLAG_USE_MOTOR_LIN, rbc->flag & RBC_FLAG_USE_MOTOR_ANG);
	}
#endif
}

static void rna_MeshCon_use_motor_ang_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_MOTOR_ANG);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		RB_constraint_set_enable_motor(rbc->physics_constraint, rbc->flag & RBC_FLAG_USE_MOTOR_LIN, rbc->flag & RBC_FLAG_USE_MOTOR_ANG);
	}
#endif
}

static void rna_MeshCon_motor_lin_target_velocity_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->motor_lin_target_velocity = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_MOTOR) {
		RB_constraint_set_target_velocity_motor(rbc->physics_constraint, value, rbc->motor_ang_target_velocity);
	}
#endif
}

static void rna_MeshCon_motor_ang_max_impulse_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->motor_ang_max_impulse = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_MOTOR) {
		RB_constraint_set_max_impulse_motor(rbc->physics_constraint, rbc->motor_lin_max_impulse, value);
	}
#endif
}

static void rna_MeshCon_motor_ang_target_velocity_set(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->motor_ang_target_velocity = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint && rbc->type == RBC_TYPE_MOTOR) {
		RB_constraint_set_target_velocity_motor(rbc->physics_constraint, rbc->motor_lin_target_velocity, value);
	}
#endif
}

static char *rna_MeshIsland_path(PointerRNA *ptr)
{
	MeshIsland* mi = ptr->data;
	Object* ob = ptr->id.data;
	FractureModifierData *fmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);
	if (fmd)
	{
		ModifierData *md = (ModifierData*)fmd;
		//int index = mi->id;
		char name_esc[sizeof(md->name) * 2];
		BLI_strescape(name_esc, md->name, sizeof(name_esc));
#if 0
		if (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL)
		{
			/* in regular fracture modes, meshisland index is 1-based */
			index--;
			return BLI_sprintfN("modifiers[\"%s\"].mesh_islands[%d]", name_esc, index);
		}
#endif

		return BLI_sprintfN("modifiers[\'%s\'].mesh_islands[\'%s\']", name_esc, mi->name);
	}
	else
	{	/*should not happen yet, meshislands only exist in modifier*/
		return BLI_sprintfN("mesh_islands[%d]", mi->id);
	}
}

static char *rna_MeshIslandVertex_path(PointerRNA *ptr)
{
	Object* ob = ptr->id.data;
	MVert* mv = (MVert*)ptr->data;
	FractureModifierData* fmd = (FractureModifierData*) modifiers_findByType(ob, eModifierType_Fracture);
	int index = 0;
	int v = 0;
	MeshIsland *mi;

	if (fmd)
	{
		ModifierData *md = (ModifierData*)fmd;
		char name_esc[sizeof(md->name) * 2];
		BLI_strescape(name_esc, md->name, sizeof(name_esc));
		bool found = false;

		/* a looong search perhaps */
		for (mi = fmd->meshIslands.first; mi; mi = mi->next)
		{
			int i = 0;
			for (i = 0; i < mi->vertex_count; i++)
			{
				//printf("%p %p\n", mv, mi->vertices_cached[i]);
				if (mv == mi->vertices_cached[i])
				{
					found = true;
					v = i;
					break;
				}
			}

			if (found)
				break;

			index++;
		}

		return BLI_sprintfN("modifiers[\"%s\"].mesh_islands[%d].vertices[%d]", name_esc, index, v);
	}
	else
	{	/*should not happen yet, meshislands only exist in modifier*/
		index = -1;
		v = -1;
		return BLI_sprintfN("mesh_islands[%d].vertices[%d]", index, v);
	}
}

static char *rna_MeshConstraint_path(PointerRNA *ptr)
{
	RigidBodyShardCon* con = ptr->data;
	Object* ob = ptr->id.data;
	ModifierData *md = modifiers_findByType(ob, eModifierType_Fracture);
	if (md)
	{
		char name_esc[sizeof(md->name) * 2];
		BLI_strescape(name_esc, md->name, sizeof(name_esc));
		return BLI_sprintfN("modifiers[\'%s\'].mesh_constraints[\'%s\']", name_esc, con->name);
	}
	else
	{	/*should not happen yet, meshconstraints only exist in modifier*/
		return BLI_sprintfN("mesh_constraints[%s]", con->name);
	}
}

static void rna_MeshIsland_cluster_index_set(PointerRNA *ptr, int value)
{
	MeshIsland *mi = (MeshIsland *)ptr->data;
	Object* ob = ptr->id.data;
	FractureModifierData *fmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);
	int i = 0;

	mi->particle_index = value;

#ifdef WITH_BULLET
	for (i = 0; i < mi->participating_constraint_count; i++)
	{
		RigidBodyShardCon* con = mi->participating_constraints[i];
		float thresh = fmd->breaking_threshold;

		if (con->mi1 == mi)
		{
			if (con->mi2->particle_index == mi->particle_index)
			{
				con->breaking_threshold = fmd->cluster_breaking_threshold; //TODO check against original constraint fn
			}
			else {
				if (fmd->thresh_defgrp_name[0]) {
					/* modify maximum threshold by minimum weight */
					con->breaking_threshold = thresh * MIN2(con->mi1->thresh_weight, con->mi2->thresh_weight);
				}
				else {
					con->breaking_threshold = thresh;
				}
			}
		}
		else if (con->mi2 == mi)
		{
			if (con->mi1->particle_index == mi->particle_index)
			{
				con->breaking_threshold = fmd->cluster_breaking_threshold; //TODO check against original constraint fn
			}
			else {
				if (fmd->thresh_defgrp_name[0]) {
					/* modify maximum threshold by minimum weight */
					con->breaking_threshold = thresh * MIN2(con->mi1->thresh_weight, con->mi2->thresh_weight);
				}
				else {
					con->breaking_threshold = thresh;
				}
			}
		}
	}
#endif
}

static void rna_MeshIslandVertexGroup_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
	FractureModifierData *fmd = (FractureModifierData*)ptr->data;
	DerivedMesh* dm = fmd->visible_mesh_cached;

	if (dm)
	{
		MDeformVert *dvert = dm->getVertDataArray(dm, CD_MDEFORMVERT);
		int totvert = dm->getNumVerts(dm);
		rna_iterator_array_begin(iter, (void *)dvert, sizeof(MDeformVert), totvert, 0, NULL);
	}
	else {
		rna_iterator_array_begin(iter, NULL, 0, 0, 0, NULL);
	}
}

static void rna_MeshIslandVertexGroupElement_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
	MDeformVert* dvert = (MDeformVert*)ptr->data;
	rna_iterator_array_begin(iter, (void *)dvert->dw, sizeof(MDeformWeight), dvert->totweight, 0, NULL);
}

static int rna_MeshIslandVertex_index_get(PointerRNA *ptr)
{
	Object* ob = (Object*)ptr->id.data;
	FractureModifierData *fmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);
	DerivedMesh* dm = fmd->visible_mesh_cached;
	if (dm)
	{
		MVert *vert = (MVert *)ptr->data;
		MVert *mv = dm->getVertArray(dm);
		return (int)(vert - mv);
	}
	else {
		return -1;
	}
}

static void rna_MeshCon_use_limit_lin_x(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_LIMIT_LIN_X);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:

				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
				{
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, 0.0f, -1.0f);
				}
				break;

			case RBC_TYPE_SLIDER:
				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_X)
				{
					RB_constraint_set_limits_slider(rbc->physics_constraint, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_slider(rbc->physics_constraint, 0.0f, -1.0f);
				}
				break;
		}
	}
#endif
}


static void rna_MeshCon_limit_lin_x_lower(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_lin_x_lower = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				break;
			case RBC_TYPE_SLIDER:
				RB_constraint_set_limits_slider(rbc->physics_constraint, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				break;
		}
	}
#endif
}

static void rna_MeshCon_limit_lin_x_upper(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_lin_x_upper = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				break;
			case RBC_TYPE_SLIDER:
				RB_constraint_set_limits_slider(rbc->physics_constraint, rbc->limit_lin_x_lower, rbc->limit_lin_x_upper);
				break;
		}
	}
#endif
}


static void rna_MeshCon_use_limit_lin_y(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_LIMIT_LIN_Y);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:

				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Y)
				{
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->limit_lin_y_lower, rbc->limit_lin_y_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, 0.0f, -1.0f);
				}
				break;
		}
	}
#endif
}

static void rna_MeshCon_limit_lin_y_lower(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_lin_y_lower = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->limit_lin_y_lower, rbc->limit_lin_y_upper);
				break;
		}
	}
#endif
}

static void rna_MeshCon_limit_lin_y_upper(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_lin_y_upper = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->limit_lin_y_lower, rbc->limit_lin_y_upper);
				break;
		}
	}
#endif
}


static void rna_MeshCon_use_limit_lin_z(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_LIMIT_LIN_Z);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:

				if (rbc->flag & RBC_FLAG_USE_LIMIT_LIN_Z)
				{
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->limit_lin_z_lower, rbc->limit_lin_z_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, 0.0f, -1.0f);
				}
				break;
		}
	}
#endif
}

static void rna_MeshCon_limit_lin_z_lower(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_lin_z_lower = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->limit_lin_z_lower, rbc->limit_lin_z_upper);
				break;
		}
	}
#endif
}

static void rna_MeshCon_limit_lin_z_upper(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_lin_z_upper = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->limit_lin_z_lower, rbc->limit_lin_z_upper);
				break;
		}
	}
#endif
}


static void rna_MeshCon_use_limit_ang_x(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_LIMIT_ANG_X);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:

				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_X)
				{
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->limit_ang_x_lower, rbc->limit_ang_x_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, 0.0f, -1.0f);
				}
				break;
		}
	}
#endif
}



static void rna_MeshCon_limit_ang_x_lower(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_ang_x_lower = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->limit_ang_x_lower, rbc->limit_ang_x_upper);
				break;
		}
	}
#endif
}

static void rna_MeshCon_limit_ang_x_upper(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_ang_x_upper = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->limit_ang_x_lower, rbc->limit_ang_x_upper);
				break;
		}
	}
#endif
}


static void rna_MeshCon_use_limit_ang_y(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_LIMIT_ANG_Y);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:

				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Y)
				{
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->limit_ang_y_lower, rbc->limit_ang_y_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, 0.0f, -1.0f);
				}
				break;
		}
	}
#endif
}


static void rna_MeshCon_limit_ang_y_lower(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_ang_y_lower = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->limit_ang_y_lower, rbc->limit_ang_y_upper);
				break;
		}
	}
#endif
}

static void rna_MeshCon_limit_ang_y_upper(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_ang_y_upper = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->limit_ang_y_lower, rbc->limit_ang_y_upper);
				break;
		}
	}
#endif
}


static void rna_MeshCon_use_limit_ang_z(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_LIMIT_ANG_Z);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:

				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Z)
				{
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, 0.0f, -1.0f);
				}
				break;

			case RBC_TYPE_HINGE:
				if (rbc->flag & RBC_FLAG_USE_LIMIT_ANG_Z)
				{
					RB_constraint_set_limits_hinge(rbc->physics_constraint, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
				}
				else {
					//validator in rigidbody.c will change properties.... but here ensure physics constraint is updated at once
					RB_constraint_set_limits_hinge(rbc->physics_constraint, 0.0f, -1.0f);
				}
				break;
		}
	}
#endif
}


static void rna_MeshCon_limit_ang_z_lower(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_ang_z_lower = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
				break;
			case RBC_TYPE_HINGE:
				RB_constraint_set_limits_hinge(rbc->physics_constraint, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
		}
	}
#endif
}

static void rna_MeshCon_limit_ang_z_upper(PointerRNA *ptr, float value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->limit_ang_z_upper = value;

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF:
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_limits_6dof(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
				break;
			case RBC_TYPE_HINGE:
				RB_constraint_set_limits_hinge(rbc->physics_constraint, rbc->limit_ang_z_lower, rbc->limit_ang_z_upper);
		}
	}
#endif
}



static void rna_MeshCon_use_spring_x(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_SPRING_X);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_X, rbc->flag & RBC_FLAG_USE_SPRING_X);
				break;
		}
	}
#endif
}

static void rna_MeshCon_use_spring_y(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_SPRING_Y);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Y, rbc->flag & RBC_FLAG_USE_SPRING_Y);
				break;
		}
	}
#endif
}

static void rna_MeshCon_use_spring_z(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_SPRING_Z);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_LIN_Z, rbc->flag & RBC_FLAG_USE_SPRING_Z);
				break;
		}
	}
#endif
}


static void rna_MeshCon_use_spring_ang_x(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_SPRING_ANG_X);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_X, rbc->flag & RBC_FLAG_USE_SPRING_ANG_X);
				break;
		}
	}
#endif
}

static void rna_MeshCon_use_spring_ang_y(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_SPRING_ANG_Y);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Y, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Y);
				break;
		}
	}
#endif
}

static void rna_MeshCon_use_spring_ang_z(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	RB_FLAG_SET(rbc->flag, value, RBC_FLAG_USE_SPRING_ANG_Z);

#ifdef WITH_BULLET
	if (rbc->physics_constraint) {
		switch (rbc->type)
		{
			case RBC_TYPE_6DOF_SPRING:
				RB_constraint_set_spring_6dof_spring(rbc->physics_constraint, RB_LIMIT_ANG_Z, rbc->flag & RBC_FLAG_USE_SPRING_ANG_Z);
				break;
		}
	}
#endif
}


#endif

static void rna_def_mesh_island_vertex(BlenderRNA* brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	srna = RNA_def_struct(brna, "MeshIslandVertex", NULL);
	RNA_def_struct_sdna(srna, "MVert");
	RNA_def_struct_ui_text(srna, "Mesh Island Vertex", "Vertex in a Mesh Island");
	RNA_def_struct_path_func(srna, "rna_MeshIslandVertex_path");

	prop = RNA_def_property(srna, "co", PROP_FLOAT, PROP_TRANSLATION);
	RNA_def_property_ui_text(prop, "Location", "");
	//RNA_def_property_update(prop, 0, "rna_Mesh_update_data"); XXX TODO allow updates, how ?

	/*prop = RNA_def_property(srna, "normal", PROP_FLOAT, PROP_DIRECTION);
	RNA_def_property_float_sdna(prop, NULL, "no");
	RNA_def_property_array(prop, 3);
	RNA_def_property_range(prop, -1.0f, 1.0f);
	RNA_def_property_float_funcs(prop, "rna_MeshVertex_normal_get", "rna_MeshVertex_normal_set", NULL);
	RNA_def_property_ui_text(prop, "Normal", "Vertex Normal");*/

	prop = RNA_def_property(srna, "index", PROP_INT, PROP_NONE);
	RNA_def_property_int_funcs(prop, "rna_MeshIslandVertex_index_get", NULL, NULL);
	RNA_def_property_ui_text(prop, "index", "Index of this vertex in global fracture modifier derived mesh");
	RNA_def_property_clear_flag(prop, PROP_EDITABLE);
}

static void rna_def_mesh_island_vertices(BlenderRNA* brna, PropertyRNA* cprop)
{
	StructRNA *srna;

	RNA_def_property_srna(cprop, "MeshIslandVertices");
	srna = RNA_def_struct(brna, "MeshIslandVertices", NULL);
	RNA_def_struct_sdna(srna, "MeshIsland");
	RNA_def_struct_ui_text(srna, "Mesh Island Vertices", "Collection of mesh island vertices");
}

static void rna_def_mesh_island(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	srna = RNA_def_struct(brna, "MeshIsland", NULL);
	RNA_def_struct_sdna(srna, "MeshIsland");
	RNA_def_struct_ui_text(srna, "Mesh Island", "A set of connected vertices and faces, represents a single shard entity in Fracture Modifier");
	RNA_def_struct_path_func(srna, "rna_MeshIsland_path");

	prop = RNA_def_property(srna, "rigidbody", PROP_POINTER, PROP_NONE);
	RNA_def_property_pointer_sdna(prop, NULL, "rigidbody");
	RNA_def_property_ui_text(prop, "Rigid Body", "Rigidbody object of this mesh island");
	RNA_def_property_clear_flag(prop, PROP_EDITABLE);

	prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "name");
	RNA_def_property_ui_text(prop, "MeshIsland Name", "A name or id of this mesh island");
	RNA_def_struct_name_property(srna, prop);
	//RNA_def_property_clear_flag(prop, PROP_EDITABLE);

	prop = RNA_def_property(srna, "centroid", PROP_FLOAT, PROP_XYZ);
	RNA_def_property_float_sdna(prop, NULL, "centroid");
	RNA_def_property_array(prop, 3);
	RNA_def_property_ui_text(prop, "Centroid", "Mesh Island Centroid");
	RNA_def_property_clear_flag(prop, PROP_EDITABLE);

	prop = RNA_def_property(srna, "vertices", PROP_COLLECTION, PROP_NONE);
	RNA_def_property_collection_sdna(prop, NULL, "vertices_cached", "vertex_count");
	RNA_def_property_struct_type(prop, "MeshIslandVertex");
	RNA_def_property_ui_text(prop, "Vertices", "Vertices of the mesh island");

	rna_def_mesh_island_vertices(brna, prop);
	rna_def_mesh_island_vertex(brna);

	prop = RNA_def_property(srna, "constraints", PROP_COLLECTION, PROP_NONE);
	RNA_def_property_collection_sdna(prop, NULL, "participating_constraints", "participating_constraint_count");
	RNA_def_property_struct_type(prop, "MeshConstraint");
	RNA_def_property_ui_text(prop, "Constraints", "Constraints where this mesh island participates in");
	RNA_def_property_clear_flag(prop, PROP_EDITABLE);

	prop = RNA_def_property(srna, "cluster_index", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "particle_index");
	RNA_def_property_int_funcs(prop, NULL, "rna_MeshIsland_cluster_index_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Index", "To which cluster this mesh island belongs.");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");
}

static void rna_def_mesh_constraint(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop, *parm;
	FunctionRNA *func;

	srna = RNA_def_struct(brna, "MeshConstraint", NULL);
	RNA_def_struct_sdna(srna, "RigidBodyShardCon");
	RNA_def_struct_ui_text(srna, "Mesh Constraint", "A connection between two mesh islands in Fracture Modifier");
	RNA_def_struct_path_func(srna, "rna_MeshConstraint_path");

	prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "name");
	RNA_def_property_ui_text(prop, "Constraint Name", "A name or id of this mesh constraint");
	RNA_def_struct_name_property(srna, prop);

	prop = RNA_def_property(srna, "location", PROP_FLOAT, PROP_TRANSLATION);
	RNA_def_property_float_sdna(prop, NULL, "pos");
	RNA_def_property_array(prop, 3);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_position_set", NULL);
	RNA_def_property_ui_text(prop, "Location", "Position of the mesh constraint");

	prop = RNA_def_property(srna, "rotation", PROP_FLOAT, PROP_QUATERNION);
	RNA_def_property_float_sdna(prop, NULL, "orn");
	RNA_def_property_array(prop, 4);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_orientation_set", NULL);
	RNA_def_property_ui_text(prop, "Rotation", "Quaternion rotation of the mesh constraint");

	prop = RNA_def_property(srna, "plastic", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_PLASTIC);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_plastic_set");
	RNA_def_property_ui_text(prop, "Plastic", "This constraint belongs to a plastic connection");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "type");
	RNA_def_property_enum_items(prop, rna_enum_rigidbody_constraint_type_items);
	//RNA_def_property_enum_funcs(prop, NULL, "rna_MeshCon_type_set", NULL);
	RNA_def_property_ui_text(prop, "Type", "Type of Rigid Body Constraint");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "island1", PROP_POINTER, PROP_NONE);
	RNA_def_property_pointer_sdna(prop, NULL, "mi1");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_ui_text(prop, "Mesh Island 1", "First Mesh Island to be constrained");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "island2", PROP_POINTER, PROP_NONE);
	RNA_def_property_pointer_sdna(prop, NULL, "mi2");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_ui_text(prop, "Mesh Island 2", "Second MeshIsland to be constrained");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "enabled", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_ENABLED);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_enabled_set");
	RNA_def_property_ui_text(prop, "Enabled", "Enable this constraint");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "disable_collisions", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_DISABLE_COLLISIONS);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_disable_collisions_set");
	RNA_def_property_ui_text(prop, "Disable Collisions", "Disable collisions between constrained rigid bodies");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	/* Breaking Threshold */
	prop = RNA_def_property(srna, "use_breaking", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_BREAKING);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_breaking_set");
	RNA_def_property_ui_text(prop, "Breakable",
	                         "Constraint can be broken if it receives an impulse above the threshold");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_threshold", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_threshold");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 1000.0f, 100.0, 2);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_breaking_threshold_set", NULL);
	RNA_def_property_ui_text(prop, "Breaking Threshold",
	                         "Impulse threshold that must be reached for the constraint to break");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	/* Solver Iterations */
	prop = RNA_def_property(srna, "use_override_solver_iterations", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_override_solver_iterations_set");
	RNA_def_property_ui_text(prop, "Override Solver Iterations",
	                         "Override the number of solver iterations for this constraint");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "solver_iterations", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "num_solver_iterations");
	RNA_def_property_range(prop, 1, 1000);
	RNA_def_property_ui_range(prop, 1, 100, 1, 0);
	RNA_def_property_int_default(prop, 10);
	RNA_def_property_int_funcs(prop, NULL, "rna_MeshCon_num_solver_iterations_set", NULL);
	RNA_def_property_ui_text(prop, "Solver Iterations",
	                         "Number of constraint solver iterations made per simulation step (higher values are more "
	                         "accurate but slower)");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	/* Limits */
	prop = RNA_def_property(srna, "use_limit_lin_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_X);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_limit_lin_x");
	RNA_def_property_ui_text(prop, "X Axis", "Limit translation on X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_lin_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_Y);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_limit_lin_y");
	RNA_def_property_ui_text(prop, "Y Axis", "Limit translation on Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_lin_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_Z);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_limit_lin_z");
	RNA_def_property_ui_text(prop, "Z Axis", "Limit translation on Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_ang_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_X);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_limit_ang_x");
	RNA_def_property_ui_text(prop, "X Angle", "Limit rotation around X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_ang_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_Y);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_limit_ang_y");
	RNA_def_property_ui_text(prop, "Y Angle", "Limit rotation around Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_ang_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_Z);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_limit_ang_z");
	RNA_def_property_ui_text(prop, "Z Angle", "Limit rotation around Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_spring_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_X);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_spring_x");
	RNA_def_property_ui_text(prop, "X Spring", "Enable spring on X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_spring_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_Y);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_spring_y");
	RNA_def_property_ui_text(prop, "Y Spring", "Enable spring on Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_spring_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_Z);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_spring_z");
	RNA_def_property_ui_text(prop, "Z Spring", "Enable spring on Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_spring_ang_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_ANG_X);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_spring_ang_x");
	RNA_def_property_ui_text(prop, "X Angle Spring", "Enable spring on X rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "use_spring_ang_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_ANG_Y);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_spring_ang_y");
	RNA_def_property_ui_text(prop, "Y Angle Spring", "Enable spring on Y rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "use_spring_ang_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_ANG_Z);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_spring_ang_z");
	RNA_def_property_ui_text(prop, "Z Angle Spring", "Enable spring on Z rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "use_motor_lin", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_MOTOR_LIN);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_motor_lin_set");
	RNA_def_property_ui_text(prop, "Linear Motor", "Enable linear motor");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_motor_ang", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_MOTOR_ANG);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_MeshCon_use_motor_ang_set");
	RNA_def_property_ui_text(prop, "Angular Motor", "Enable angular motor");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_x_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_x_lower");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_lin_x_lower", NULL);
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower X Limit", "Lower limit of X axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_x_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_x_upper");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_lin_x_upper", NULL);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper X Limit", "Upper limit of X axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_y_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_y_lower");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_lin_y_lower", NULL);
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower Y Limit", "Lower limit of Y axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_y_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_y_upper");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_lin_y_upper", NULL);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper Y Limit", "Upper limit of Y axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_z_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_z_lower");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_lin_z_lower", NULL);
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower Z Limit", "Lower limit of Z axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_z_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_z_upper");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_lin_z_upper", NULL);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper Z Limit", "Upper limit of Z axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_x_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_x_lower");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_ang_x_lower", NULL);
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower X Angle Limit", "Lower limit of X axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_x_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_x_upper");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_ang_x_upper", NULL);
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper X Angle Limit", "Upper limit of X axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_y_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_y_lower");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_ang_y_lower", NULL);
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower Y Angle Limit", "Lower limit of Y axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_y_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_y_upper");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_ang_y_upper", NULL);
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper Y Angle Limit", "Upper limit of Y axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_z_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_z_lower");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_ang_z_lower", NULL);
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower Z Angle Limit", "Lower limit of Z axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_z_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_z_upper");
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_limit_ang_z_upper", NULL);
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper Z Angle Limit", "Upper limit of Z axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "spring_stiffness_x", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_x");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_stiffness_x_set", NULL);
	RNA_def_property_ui_text(prop, "X Axis Stiffness", "Stiffness on the X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "spring_stiffness_y", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_y");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_stiffness_y_set", NULL);
	RNA_def_property_ui_text(prop, "Y Axis Stiffness", "Stiffness on the Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "spring_stiffness_z", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_z");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_stiffness_z_set", NULL);
	RNA_def_property_ui_text(prop, "Z Axis Stiffness", "Stiffness on the Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "spring_stiffness_ang_x", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_ang_x");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_stiffness_ang_x_set", NULL);
	RNA_def_property_ui_text(prop, "X Angle Stiffness", "Stiffness on the X rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "spring_stiffness_ang_y", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_ang_y");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_stiffness_ang_y_set", NULL);
	RNA_def_property_ui_text(prop, "Y Angle Stiffness", "Stiffness on the Y rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "spring_stiffness_ang_z", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_ang_z");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_stiffness_ang_z_set", NULL);
	RNA_def_property_ui_text(prop, "Z Angle Stiffness", "Stiffness on the Z rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "spring_damping_x", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_x");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_damping_x_set", NULL);
	RNA_def_property_ui_text(prop, "Damping X", "Damping on the X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "spring_damping_y", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_y");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_damping_y_set", NULL);
	RNA_def_property_ui_text(prop, "Damping Y", "Damping on the Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "spring_damping_z", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_z");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_damping_z_set", NULL);
	RNA_def_property_ui_text(prop, "Damping Z", "Damping on the Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "spring_damping_ang_x", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_ang_x");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_damping_ang_x_set", NULL);
	RNA_def_property_ui_text(prop, "Damping X Angle", "Damping on the X rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "spring_damping_ang_y", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_ang_y");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_damping_ang_y_set", NULL);
	RNA_def_property_ui_text(prop, "Damping Y Angle", "Damping on the Y rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "spring_damping_ang_z", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_ang_z");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_spring_damping_ang_z_set", NULL);
	RNA_def_property_ui_text(prop, "Damping Z Angle", "Damping on the Z rotational axis");
	//RNA_def_property_update(prop, NC_OBJECT, "rna_RigidBodyOb_reset");

	prop = RNA_def_property(srna, "motor_lin_target_velocity", PROP_FLOAT, PROP_UNIT_VELOCITY);
	RNA_def_property_float_sdna(prop, NULL, "motor_lin_target_velocity");
	RNA_def_property_range(prop, -FLT_MAX, FLT_MAX);
	RNA_def_property_ui_range(prop, -100.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_motor_lin_target_velocity_set", NULL);
	RNA_def_property_ui_text(prop, "Target Velocity", "Target linear motor velocity");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "motor_lin_max_impulse", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "motor_lin_max_impulse");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_motor_lin_max_impulse_set", NULL);
	RNA_def_property_ui_text(prop, "Max Impulse", "Maximum linear motor impulse");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "motor_ang_target_velocity", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "motor_ang_target_velocity");
	RNA_def_property_range(prop, -FLT_MAX, FLT_MAX);
	RNA_def_property_ui_range(prop, -100.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_motor_ang_target_velocity_set", NULL);
	RNA_def_property_ui_text(prop, "Target Velocity", "Target angular motor velocity");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "motor_ang_max_impulse", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "motor_ang_max_impulse");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_motor_ang_max_impulse_set", NULL);
	RNA_def_property_ui_text(prop, "Max Impulse", "Maximum angular motor impulse");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_distance", PROP_FLOAT, PROP_DISTANCE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_dist");
	//RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_breaking_distance_set");
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Breaking Distance", "Breaking Distance Tolerance of this constraint, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_angle");
	//RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_breaking_angle_set");
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, DEG2RADF(360.0));
	RNA_def_property_ui_text(prop, "Breaking Angle", "Breaking Angle Tolerance of this constraint, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "plastic_distance", PROP_FLOAT, PROP_DISTANCE);
	RNA_def_property_float_sdna(prop, NULL, "plastic_dist");
	//RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_set_plastic_dist");
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Plastic Distance", "Distance Tolerance of this constraint, when exceeded enter plastic mode, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "plastic_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "plastic_angle");
	//RNA_def_property_float_funcs(prop, NULL, "rna_MeshCon_set_plastic_angle");
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, DEG2RADF(360.0));
	RNA_def_property_ui_text(prop, "Plastic Angle", "Angle Tolerance of this constraint, when exceeded enter plastic mode, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	//do as function, dont need an dna value for storage, instead query from bullet directly
	func = RNA_def_function(srna, "appliedImpulse", "rna_MeshCon_get_applied_impulse");
	parm = RNA_def_float(func, "impulse", 0, -FLT_MAX, FLT_MAX, "Applied Impulse", "The currently applied impulse on this constraint",
	                     -FLT_MIN, FLT_MAX);
	RNA_def_function_return(func, parm);

	func = RNA_def_function(srna, "isIntact", "rna_MeshCon_is_intact");
	parm = RNA_def_boolean(func, "intactness", 0, "Is Intact", "Whether this constraint is still intact or already broken");
	RNA_def_function_return(func, parm);
}

static void rna_def_fracture_meshislands(BlenderRNA *brna, PropertyRNA *cprop)
{
	StructRNA *srna;

	FunctionRNA *func;
	PropertyRNA *parm;

	RNA_def_property_srna(cprop, "MeshIslands");
	srna = RNA_def_struct(brna, "MeshIslands", NULL);
	RNA_def_struct_sdna(srna, "FractureModifierData");
	RNA_def_struct_ui_text(srna, "Mesh Islands", "Collection of mesh islands");

	func = RNA_def_function(srna, "new", "rna_FractureModifier_mesh_island_new");
	RNA_def_function_ui_description(func, "Add mesh island to Fracture Modifier");
	RNA_def_function_flag(func, FUNC_USE_SELF_ID);

	parm = RNA_def_pointer(func, "source_object", "Object", "Object", "Source Mesh Object for this mesh island");
	RNA_def_property_flag(parm, PARM_REQUIRED | PROP_NEVER_NULL | PARM_RNAPTR);

	//RNA_def_int(func, "index", -1, -1, INT_MAX, "Index", "Optional index for mesh island, -1 for automatic", -1, INT_MAX);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");
	parm = RNA_def_pointer(func, "mesh_island", "MeshIsland", "", "New mesh island");
	RNA_def_function_return(func, parm);

	func = RNA_def_function(srna, "remove", "rna_FractureModifier_mesh_island_remove");
	RNA_def_function_flag(func, FUNC_USE_REPORTS | FUNC_USE_SELF_ID);
	RNA_def_function_ui_description(func, "Delete mesh island from fracture modifier");
	parm = RNA_def_pointer(func, "mesh_island", "MeshIsland", "", "Mesh Island to remove");
	RNA_def_property_flag(parm, PARM_REQUIRED | PROP_NEVER_NULL | PARM_RNAPTR);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");
	RNA_def_property_clear_flag(parm, PROP_THICK_WRAP);

	func = RNA_def_function(srna, "clear", "rna_FractureModifier_mesh_island_clear");
	RNA_def_function_flag(func, FUNC_USE_SELF_ID);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");
	RNA_def_function_ui_description(func, "Delete all mesh islands from fracture modifier");
}

static void rna_def_fracture_meshconstraints(BlenderRNA *brna, PropertyRNA *cprop)
{
	StructRNA *srna;

	FunctionRNA *func;
	PropertyRNA *parm;

	RNA_def_property_srna(cprop, "MeshConstraints");
	srna = RNA_def_struct(brna, "MeshConstraints", NULL);
	RNA_def_struct_sdna(srna, "FractureModifierData");
	RNA_def_struct_ui_text(srna, "Mesh Constraints", "Collection of mesh constraints");

	func = RNA_def_function(srna, "new", "rna_FractureModifier_mesh_constraint_new");
	RNA_def_function_ui_description(func, "Add mesh island to Fracture Modifier");
	RNA_def_function_flag(func, FUNC_USE_SELF_ID);
	parm = RNA_def_pointer(func, "mi_first", "MeshIsland", "", "First mesh island");
	RNA_def_property_flag(parm, PARM_REQUIRED);
	parm = RNA_def_pointer(func, "mi_second", "MeshIsland", "", "Second mesh island");
	RNA_def_property_flag(parm, PARM_REQUIRED);
	parm = RNA_def_enum(func, "type", rna_enum_rigidbody_constraint_type_items, RBC_TYPE_FIXED, "Constraint Type", "Type of constraint");
	RNA_def_property_flag(parm, PARM_REQUIRED);

	//RNA_def_int(func, "index", -1, -1, INT_MAX, "Index", "Optional index for mesh constraint, -1 for automatic", -1, INT_MAX);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");

	parm = RNA_def_pointer(func, "mesh_constraint", "MeshConstraint", "", "New mesh constraint");
	RNA_def_function_return(func, parm);

	func = RNA_def_function(srna, "remove", "rna_FractureModifier_mesh_constraint_remove");
	RNA_def_function_flag(func, FUNC_USE_REPORTS | FUNC_USE_SELF_ID);
	RNA_def_function_ui_description(func, "Delete mesh constraint from fracture modifier");
	parm = RNA_def_pointer(func, "mesh_constraint", "MeshConstraint", "", "Mesh Constraint to remove");
	RNA_def_property_flag(parm, PARM_REQUIRED | PROP_NEVER_NULL | PARM_RNAPTR);
	RNA_def_property_clear_flag(parm, PROP_THICK_WRAP);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");

	func = RNA_def_function(srna, "clear", "rna_FractureModifier_mesh_constraint_clear");
	RNA_def_function_flag(func, FUNC_USE_SELF_ID);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");
	RNA_def_function_ui_description(func, "Delete all mesh constraints from fracture modifier");
}



static void rna_def_mesh_vertex_group_element(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	srna = RNA_def_struct(brna, "MeshIslandVertexGroupElement", NULL);
	RNA_def_struct_sdna(srna, "MDeformWeight");
	//RNA_def_struct_path_func(srna, "rna_MeshVertexGroupElement_path");
	RNA_def_struct_ui_text(srna, "Vertex Group Element", "Weight value of a vertex in a vertex group");
	//RNA_def_struct_ui_icon(srna, ICON_GROUP_VERTEX);

	prop = RNA_def_property(srna, "group", PROP_INT, PROP_UNSIGNED);
	RNA_def_property_int_sdna(prop, NULL, "def_nr");
	RNA_def_property_clear_flag(prop, PROP_EDITABLE);
	RNA_def_property_ui_text(prop, "Group Index", "");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "weight", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_ui_text(prop, "Weight", "Vertex Weight");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");
}

static void rna_def_mesh_vertex_group(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	srna = RNA_def_struct(brna, "MeshIslandVertexGroup", NULL);
	RNA_def_struct_sdna(srna, "MDeformVert");
	//RNA_def_struct_path_func(srna, "rna_MeshVertexGroup_path");
	RNA_def_struct_ui_text(srna, "Vertex Group", "Weights of all vertex groups this deform vert is in");

	prop = RNA_def_property(srna, "weights", PROP_COLLECTION, PROP_NONE);
	rna_def_mesh_vertex_group_element(brna);

	RNA_def_property_collection_funcs(prop, "rna_MeshIslandVertexGroupElement_begin", "rna_iterator_array_next",
	                                  "rna_iterator_array_end", "rna_iterator_array_get", NULL, NULL, NULL, NULL);
	RNA_def_property_struct_type(prop, "MeshIslandVertexGroupElement");
	RNA_def_property_ui_text(prop, "weights", "Array of weights");
}

void RNA_api_fracture(BlenderRNA *brna, StructRNA *srna)
{
	PropertyRNA *prop;

	/*Fracture Modifiers own python / RNA API */
	rna_def_mesh_island(brna);
	prop = RNA_def_property(srna, "mesh_islands", PROP_COLLECTION, PROP_NONE);
	RNA_def_property_struct_type(prop, "MeshIsland");
	RNA_def_property_collection_sdna(prop, NULL, "meshIslands", NULL);
	//RNA_def_property_collection_funcs(prop, NULL, NULL, NULL, NULL, NULL, "rna_FractureModifier_meshIsland_get_int", NULL, NULL);
	RNA_def_property_ui_text(prop, "Mesh Islands", "A single entity inside the modifier, representing a single shard");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");
	rna_def_fracture_meshislands(brna, prop);

	rna_def_mesh_constraint(brna);
	prop = RNA_def_property(srna, "mesh_constraints", PROP_COLLECTION, PROP_NONE);
	RNA_def_property_struct_type(prop, "MeshConstraint");
	RNA_def_property_collection_sdna(prop, NULL, "meshConstraints", NULL);
	//RNA_def_property_collection_funcs(prop, NULL, NULL, NULL, NULL, NULL, "rna_FractureModifier_meshConstraint_get_int", NULL, NULL);
	RNA_def_property_ui_text(prop, "Mesh Constraints", "A connection between two Mesh Islands inside the modifier");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");
	rna_def_fracture_meshconstraints(brna, prop);

	rna_def_mesh_vertex_group(brna);
	prop = RNA_def_property(srna, "vertex_groups", PROP_COLLECTION, PROP_NONE);
	RNA_def_property_collection_funcs(prop, "rna_MeshIslandVertexGroup_begin", "rna_iterator_array_next",
	                                  "rna_iterator_array_end", "rna_iterator_array_get", NULL, NULL, NULL, NULL);
	RNA_def_property_struct_type(prop, "MeshIslandVertexGroup");
	RNA_def_property_ui_text(prop, "vertex_groups", "Global fracture modifier vertex group array");
}
