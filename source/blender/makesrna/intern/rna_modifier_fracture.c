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
 * Contributor(s): Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/makesrna/intern/rna_modifier_fracture.c
 *  \ingroup RNA
 */

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"
#include "DNA_rigidbody_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_math.h"

#include "BKE_DerivedMesh.h"
#include "BKE_rigidbody.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "rna_internal.h"

#include "WM_api.h"
#include "WM_types.h"


#ifdef RNA_RUNTIME

#ifdef WITH_BULLET
#  include "RBI_api.h"
#endif

#include "BKE_modifier.h"
#include "BKE_DerivedMesh.h"
#include "BKE_fracture.h"


static void rna_FractureModifier_thresh_defgrp_name_set(PointerRNA *ptr, const char *value)
{
	FractureModifierData *tmd = (FractureModifierData *)ptr->data;
	rna_object_vgroup_name_set(ptr, value, tmd->thresh_defgrp_name, sizeof(tmd->thresh_defgrp_name));
	tmd->refresh_constraints = true;
	tmd->reset_shards = true;
}

static void rna_FractureModifier_ground_defgrp_name_set(PointerRNA *ptr, const char *value)
{
	FractureModifierData *tmd = (FractureModifierData *)ptr->data;
	rna_object_vgroup_name_set(ptr, value, tmd->ground_defgrp_name, sizeof(tmd->ground_defgrp_name));
	tmd->refresh_constraints = true;
	tmd->reset_shards = true;
}

static void rna_FractureModifier_inner_defgrp_name_set(PointerRNA *ptr, const char *value)
{
	FractureModifierData *tmd = (FractureModifierData *)ptr->data;
	rna_object_vgroup_name_set(ptr, value, tmd->inner_defgrp_name, sizeof(tmd->inner_defgrp_name));
	tmd->refresh_constraints = true;
	tmd->reset_shards = true;
}

static void rna_FractureModifier_threshold_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->breaking_threshold = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_contact_dist_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->contact_dist = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_use_constraints_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData *)ptr->data;
	rmd->use_constraints = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_use_compounds_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData *)ptr->data;
	rmd->use_compounds = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_mass_dependent_thresholds_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData *)ptr->data;
	rmd->use_mass_dependent_thresholds = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_constraint_limit_set(PointerRNA *ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->constraint_limit = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_breaking_percentage_set(PointerRNA *ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->breaking_percentage = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_breaking_angle_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->breaking_angle = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_breaking_distance_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->breaking_distance = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_cluster_threshold_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_breaking_threshold = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_solver_iterations_override_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->solver_iterations_override = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_cluster_solver_iterations_override_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_solver_iterations_override = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_autohide_dist_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->autohide_dist = value;
	rmd->refresh_autohide = true;
}

static void rna_FractureModifier_automerge_dist_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->automerge_dist = value;
	rmd->refresh_autohide = true;
}

static void rna_FractureModifier_cluster_breaking_angle_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_breaking_angle = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_cluster_breaking_distance_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_breaking_distance = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_cluster_breaking_percentage_set(PointerRNA *ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_breaking_percentage = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_use_breaking_set(PointerRNA *ptr, bool value)
{
	RigidBodyShardCon* rbsc;
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->use_breaking = value;
	//rmd->refresh_constraints = true;

	for (rbsc = rmd->meshConstraints.first; rbsc; rbsc = rbsc->next)
	{
		if (value == true){
			rbsc->flag |= RBC_FLAG_USE_BREAKING;
		}
		else {
			rbsc->flag &= ~RBC_FLAG_USE_BREAKING;
		}

		rbsc->flag |= RBC_FLAG_NEEDS_VALIDATE;
	}
}

static void rna_FractureModifier_cluster_constraint_type_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_constraint_type = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_constraint_target_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->constraint_target = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_frac_algorithm_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->frac_algorithm = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_point_source_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->point_source = value;
	printf("PointSource\n");
	rmd->reset_shards = true;
}

static void rna_FractureModifier_point_seed_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->point_seed = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_percentage_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->percentage = value;
	rmd->reset_shards = true;
}


static void rna_FractureModifier_extra_group_set(PointerRNA* ptr, PointerRNA value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->extra_group = value.data;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_shards_to_islands_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->shards_to_islands = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_fix_normals_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->fix_normals = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_inner_material_set(PointerRNA* ptr, PointerRNA value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->inner_material = value.data;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_use_particle_birth_coordinates_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->use_particle_birth_coordinates = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_splinter_length_set(PointerRNA* ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->splinter_length = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_splinter_axis_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->splinter_axis = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_cutter_axis_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cutter_axis = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_nor_range_set(PointerRNA* ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->nor_range = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_use_smooth_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->use_smooth = value;
	rmd->reset_shards = true;
}


static void rna_FractureModifier_fractal_cuts_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->fractal_cuts = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_fractal_amount_set(PointerRNA* ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->fractal_amount = value;
	rmd->reset_shards = true;
}


static void rna_FractureModifier_physics_mesh_scale_set(PointerRNA* ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->physics_mesh_scale = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_fractal_iterations_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->fractal_iterations = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_cutter_group_set(PointerRNA* ptr, PointerRNA value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cutter_group = value.data;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_use_greasepencil_edges_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->use_greasepencil_edges = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_grease_offset_set(PointerRNA* ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->grease_offset = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_grease_decimate_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->grease_decimate = value;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_dm_group_set(PointerRNA* ptr, PointerRNA value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->dm_group = value.data;
	rmd->reset_shards = true;
}

static void rna_FractureModifier_impulse_dampening_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->impulse_dampening = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_directional_factor_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->directional_factor = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_minimum_impulse_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->minimum_impulse = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_mass_threshold_factor_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->mass_threshold_factor = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_autohide_filter_group_set(PointerRNA* ptr, PointerRNA value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->autohide_filter_group = value.data;
	//rmd->reset_shards = true;
}


static MeshIsland *rna_FractureModifier_mesh_island_new(ID* id, FractureModifierData *fmd, PointerRNA* source_ptr)
{
	Object *ob = source_ptr->data;
	Object *owner = (Object*)id;
	if (ob != owner)
	{
		MeshIsland* mi = BKE_fracture_mesh_island_add(fmd, owner, ob);
		return mi;
	}

	return NULL;
}

static void rna_FractureModifier_mesh_island_remove(ID *id, FractureModifierData *fmd, ReportList *reports, PointerRNA *mi_ptr)
{
	MeshIsland *mi = mi_ptr->data;

	if (BLI_findindex(&fmd->meshIslands, mi) == -1) {
		BKE_reportf(reports, RPT_ERROR, "MeshIsland '%s' not in this fracture modifier", mi->id);
		return;
	}

	BKE_fracture_mesh_island_remove(fmd, mi);
	RNA_POINTER_INVALIDATE(mi_ptr);
}

static void rna_FractureModifier_mesh_island_clear(ID *id, FractureModifierData *fmd)
{
	BKE_fracture_mesh_island_remove_all(fmd);
}

static RigidBodyShardCon *rna_FractureModifier_mesh_constraint_new(ID* id, FractureModifierData *fmd,
                                                                   MeshIsland* mi1, MeshIsland* mi2, int type)
{
	RigidBodyShardCon* con = BKE_fracture_mesh_islands_connect(fmd, mi1, mi2, type);
	return con;
}

static void rna_FractureModifier_mesh_constraint_remove(ID *id, FractureModifierData *fmd, ReportList *reports, PointerRNA *con_ptr)
{
	RigidBodyShardCon *con = con_ptr->data;

	if (con && BLI_findindex(&fmd->meshConstraints, con) == -1) {
		BKE_reportf(reports, RPT_ERROR, "MeshConstraint '%s' not in this fracture modifier", con->name);
		return;
	}

	if (con)
		BKE_fracture_mesh_constraint_remove(fmd, con);

	RNA_POINTER_INVALIDATE(con_ptr);
}

static void rna_FractureModifier_mesh_constraint_clear(ID *id, FractureModifierData *fmd)
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

#define RB_FLAG_SET(dest, value, flag) { \
	if (value) \
		dest |= flag; \
	else \
		dest &= ~flag; \
}

static void rna_MeshCon_type_set(PointerRNA *ptr, int value)
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;

	rbc->type = value;
	rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
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

static void rna_MeshCon_position_set(PointerRNA *ptr, float value[3])
{
	RigidBodyShardCon *rbc = (RigidBodyShardCon *)ptr->data;
	copy_v3_v3(rbc->pos, value);
	rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
}

static void rna_MeshCon_orientation_set(PointerRNA *ptr, float value[4])
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

#if 0
int rna_FractureModifier_meshIsland_get_int(PointerRNA *ptr, int index, PointerRNA *r_ptr)
{

}

int rna_FractureModifier_meshConstraint_get_int(PointerRNA *ptr, int index, PointerRNA *r_ptr)
{

}
#endif

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
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

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
	RNA_def_property_ui_text(prop, "X Axis", "Limit translation on X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_lin_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_Y);
	RNA_def_property_ui_text(prop, "Y Axis", "Limit translation on Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_lin_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_Z);
	RNA_def_property_ui_text(prop, "Z Axis", "Limit translation on Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_ang_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_X);
	RNA_def_property_ui_text(prop, "X Angle", "Limit rotation around X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_ang_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_Y);
	RNA_def_property_ui_text(prop, "Y Angle", "Limit rotation around Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_limit_ang_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_Z);
	RNA_def_property_ui_text(prop, "Z Angle", "Limit rotation around Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_spring_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_X);
	RNA_def_property_ui_text(prop, "X Spring", "Enable spring on X axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_spring_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_Y);
	RNA_def_property_ui_text(prop, "Y Spring", "Enable spring on Y axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_spring_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_Z);
	RNA_def_property_ui_text(prop, "Z Spring", "Enable spring on Z axis");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

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
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower X Limit", "Lower limit of X axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_x_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_x_upper");
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper X Limit", "Upper limit of X axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_y_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_y_lower");
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower Y Limit", "Lower limit of Y axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_y_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_y_upper");
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper Y Limit", "Upper limit of Y axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_z_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_z_lower");
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower Z Limit", "Lower limit of Z axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_lin_z_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_z_upper");
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper Z Limit", "Upper limit of Z axis translation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_x_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_x_lower");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower X Angle Limit", "Lower limit of X axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_x_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_x_upper");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper X Angle Limit", "Upper limit of X axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_y_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_y_lower");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower Y Angle Limit", "Lower limit of Y axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_y_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_y_upper");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper Y Angle Limit", "Upper limit of Y axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_z_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_z_lower");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower Z Angle Limit", "Lower limit of Z axis rotation");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "limit_ang_z_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_z_upper");
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
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Breaking Distance", "Breaking Distance Tolerance of this constraint, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_angle");
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, DEG2RADF(360.0));
	RNA_def_property_ui_text(prop, "Breaking Angle", "Breaking Angle Tolerance of this constraint, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "plastic_distance", PROP_FLOAT, PROP_DISTANCE);
	RNA_def_property_float_sdna(prop, NULL, "plastic_dist");
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Plastic Distance", "Distance Tolerance of this constraint, when exceeded enter plastic mode, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "plastic_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "plastic_angle");
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_range(prop, -1.0f, DEG2RADF(360.0));
	RNA_def_property_ui_text(prop, "Plastic Angle", "Angle Tolerance of this constraint, when exceeded enter plastic mode, -1 disables");
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	//do as function, dont need an dna value for storage, instead query from bullet directly
	func = RNA_def_function(srna, "appliedImpulse", "rna_MeshCon_get_applied_impulse");
	parm = RNA_def_float(func, "impulse", 0, -FLT_MAX, FLT_MAX, "Applied Impulse", "The currently applied impulse on this constraint", -FLT_MIN, FLT_MAX);
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
	RNA_def_property_flag(parm, PROP_REQUIRED | PROP_NEVER_NULL | PROP_RNAPTR);

	//RNA_def_int(func, "index", -1, -1, INT_MAX, "Index", "Optional index for mesh island, -1 for automatic", -1, INT_MAX);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");
	parm = RNA_def_pointer(func, "mesh_island", "MeshIsland", "", "New mesh island");
	RNA_def_function_return(func, parm);

	func = RNA_def_function(srna, "remove", "rna_FractureModifier_mesh_island_remove");
	RNA_def_function_flag(func, FUNC_USE_REPORTS | FUNC_USE_SELF_ID);
	RNA_def_function_ui_description(func, "Delete mesh island from fracture modifier");
	parm = RNA_def_pointer(func, "mesh_island", "MeshIsland", "", "Mesh Island to remove");
	RNA_def_property_flag(parm, PROP_REQUIRED | PROP_NEVER_NULL | PROP_RNAPTR);
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
	RNA_def_property_flag(parm, PROP_REQUIRED);
	parm = RNA_def_pointer(func, "mi_second", "MeshIsland", "", "Second mesh island");
	RNA_def_property_flag(parm, PROP_REQUIRED);
	parm = RNA_def_enum(func, "type", rna_enum_rigidbody_constraint_type_items, RBC_TYPE_FIXED, "Constraint Type", "Type of constraint");
	RNA_def_property_flag(parm, PROP_REQUIRED);

	//RNA_def_int(func, "index", -1, -1, INT_MAX, "Index", "Optional index for mesh constraint, -1 for automatic", -1, INT_MAX);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");

	parm = RNA_def_pointer(func, "mesh_constraint", "MeshConstraint", "", "New mesh constraint");
	RNA_def_function_return(func, parm);

	func = RNA_def_function(srna, "remove", "rna_FractureModifier_mesh_constraint_remove");
	RNA_def_function_flag(func, FUNC_USE_REPORTS | FUNC_USE_SELF_ID);
	RNA_def_function_ui_description(func, "Delete mesh constraint from fracture modifier");
	parm = RNA_def_pointer(func, "mesh_constraint", "MeshConstraint", "", "Mesh Constraint to remove");
	RNA_def_property_flag(parm, PROP_REQUIRED | PROP_NEVER_NULL | PROP_RNAPTR);
	RNA_def_property_clear_flag(parm, PROP_THICK_WRAP);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");

	func = RNA_def_function(srna, "clear", "rna_FractureModifier_mesh_constraint_clear");
	RNA_def_function_flag(func, FUNC_USE_SELF_ID);
	//RNA_def_boolean(func, "update", false, "update", "Update immediately");
	RNA_def_function_ui_description(func, "Delete all mesh constraints from fracture modifier");
}

static void rna_def_modifier_fracture(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	static EnumPropertyItem prop_fracture_algorithm[] = {
		{MOD_FRACTURE_BISECT_FAST, "BISECT_FAST", 0, "Fast Bisect", "Use a faster but more inaccurate bisection algorithm, also creates uglier shards."},
		{MOD_FRACTURE_BISECT_FAST_FILL, "BISECT_FAST_FILL", 0, "Fast Bisect + Fill ", "Use the faster but different bisection algorithm and fill cut faces"},
		{MOD_FRACTURE_BOOLEAN, "BOOLEAN", 0, "Voronoi + Boolean", "Use voronoi and boolean intersection as fracture algorithm"},
		{MOD_FRACTURE_BISECT_FILL, "BISECT_FILL", 0, "Voronoi + Bisect + Fill", "Use voronoi and mesh bisect as fracture algorithm, fill cut faces"},
		{MOD_FRACTURE_BISECT, "BISECT", 0, "Voronoi + Bisect", "Use voronoi and mesh bisect as fracture algorithm, don't fill cut faces"},
		{MOD_FRACTURE_BOOLEAN_FRACTAL, "BOOLEAN_FRACTAL", 0, "Voronoi + Fractal Boolean", "Use voronoi and boolean intersection with fractally subdivided cells" },
		{0, NULL, 0, NULL, NULL}
	};

	static EnumPropertyItem prop_point_source_items[] = {
		{MOD_FRACTURE_OWN_PARTICLES, "OWN_PARTICLES", 0, "Own Particles", "Use own particles as point cloud"},
		{MOD_FRACTURE_OWN_VERTS, "OWN_VERTS", 0, "Own Vertices", "Use own vertices as point cloud"},
		{MOD_FRACTURE_EXTRA_PARTICLES, "EXTRA_PARTICLES", 0, "Extra Particles", "Use particles of group objects as point cloud"},
		{MOD_FRACTURE_EXTRA_VERTS, "EXTRA_VERTS", 0, "Extra Vertices", "Use vertices of group objects as point cloud"},
		{MOD_FRACTURE_GREASEPENCIL, "GREASE_PENCIL", 0, "Grease Pencil", "Use grease pencil points as point cloud"},
		{MOD_FRACTURE_UNIFORM, "UNIFORM", 0, "Uniform", "Use a random uniform pointcloud generated over the bounding box"},
		{0, NULL, 0, NULL, NULL}
	};

	static EnumPropertyItem prop_splinter_axises[] = {
		{MOD_FRACTURE_SPLINTER_X, "SPLINTER_X", 0, "Splinter X", "Splinters in X Direction"},
		{MOD_FRACTURE_SPLINTER_Y, "SPLINTER_Y", 0, "Splinter Y", "Splinters in Y Direction"},
		{MOD_FRACTURE_SPLINTER_Z, "SPLINTER_Z", 0, "Splinter Z", "Splinters in Z Direction"},
		{0, NULL, 0, NULL, NULL}
	};

	static EnumPropertyItem prop_cutter_axises[] = {
		{MOD_FRACTURE_CUTTER_X, "CUTTER_X", 0, "Cutter X", "Cut in X Direction"},
		{MOD_FRACTURE_CUTTER_Y, "CUTTER_Y", 0, "Cutter Y", "Cut in Y Direction"},
		{MOD_FRACTURE_CUTTER_Z, "CUTTER_Z", 0, "Cutter Z", "Cut in Z Direction"},
		{0, NULL, 0, NULL, NULL}
	};

	static EnumPropertyItem prop_constraint_targets[] = {
		{MOD_FRACTURE_CENTROID, "CENTROID", 0, "Centroid", "Build constraints based on distances between centroids"},
		{MOD_FRACTURE_VERTEX, "VERTEX", 0, "Vertex", "Build constraints based on distances between vertices (use lower values here)"},
		{0, NULL, 0, NULL, NULL}
	};

	static EnumPropertyItem prop_fracture_modes[] = {
		{MOD_FRACTURE_PREFRACTURED, "PREFRACTURED", 0, "Prefractured", "Fracture the mesh once prior to the simulation"},
		{MOD_FRACTURE_DYNAMIC, "DYNAMIC", 0, "Dynamic (WIP)", "Fracture the mesh dynamically during the simulation"},
		{MOD_FRACTURE_EXTERNAL, "EXTERNAL", 0, "External (WIP)", "Pack external mesh objects as islands and constraints too into the modifier"},
		{0, NULL, 0, NULL, NULL}
	};

	srna = RNA_def_struct(brna, "FractureModifier", "Modifier");
	RNA_def_struct_ui_text(srna, "Fracture Modifier", "Add a fracture container to this object");
	RNA_def_struct_sdna(srna, "FractureModifierData");
	RNA_def_struct_ui_icon(srna, ICON_MOD_EXPLODE);

	prop = RNA_def_property(srna, "cluster_count", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 0, 100000);
	RNA_def_property_ui_text(prop, "Cluster Count", "Amount of clusters built from existing shards, 0 for none");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	//simulation stuff...
	prop = RNA_def_property(srna, "breaking_threshold", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_threshold");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_threshold_set", NULL);
	RNA_def_property_ui_text(prop, "Inner Breaking threshold", "Threshold to break constraints between shards in the same object");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_constraints", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_use_constraints_set");
	RNA_def_property_ui_text(prop, "Use Constraints", "Create constraints between all shards");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "contact_dist", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "contact_dist");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_contact_dist_set", NULL);
	RNA_def_property_ui_text(prop, "Search Radius", "Limit search radius up to which two mesh islands are being connected, 0 for entire boundingbox");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_mass_dependent_thresholds", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_mass_dependent_thresholds_set");
	RNA_def_property_ui_text(prop, "Use Mass Dependent Thresholds", "Match the breaking threshold according to the masses of the constrained shards");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "constraint_limit", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "constraint_limit");
	RNA_def_property_range(prop, 0, INT_MAX);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_constraint_limit_set", NULL);
	RNA_def_property_ui_text(prop, "Constraint Search Limit", "Maximum number of neighbors being searched per mesh island during constraint creation, 0 for unlimited");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_percentage", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "breaking_percentage");
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_breaking_percentage_set", NULL);
	RNA_def_property_ui_text(prop, "Breaking Percentage", "Percentage of broken constraints per island which leads to breaking of all others");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_angle");
	RNA_def_property_range(prop, 0, DEG2RADF(360.0));
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_breaking_angle_set", NULL);
	RNA_def_property_ui_text(prop, "Breaking Angle", "Angle in degrees above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_distance", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_distance");
	RNA_def_property_range(prop, 0, FLT_MAX);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_breaking_distance_set", NULL);
	RNA_def_property_ui_text(prop, "Breaking Distance", "Distance above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_experimental", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "use_experimental", false);
	RNA_def_property_ui_text(prop, "Use Experimental", "Experimental features, work in progress. Use at own risk!");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

	prop = RNA_def_property(srna, "cluster_breaking_threshold", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_breaking_threshold");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_cluster_threshold_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Breaking threshold", "Threshold to break constraints INSIDE a cluster of shards");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "solver_iterations_override", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "solver_iterations_override");
	RNA_def_property_range(prop, 0, INT_MAX);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_solver_iterations_override_set", NULL);
	RNA_def_property_ui_text(prop, "Solver Iterations Override", "Override the world constraint solver iteration value with this value, 0 means no override");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "frac_algorithm", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_fracture_algorithm);
	RNA_def_property_enum_funcs(prop, NULL, "rna_FractureModifier_frac_algorithm_set", NULL);
	RNA_def_property_ui_text(prop, "Fracture Algorithm", "Select type of fracture algorithm");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "shard_count", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 1, 100000);
	RNA_def_property_int_default(prop, 10);
	RNA_def_property_ui_text(prop, "Shard Count", "How many sub-shards should be generated from the current shard");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "point_source", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_point_source_items);
	RNA_def_property_flag(prop, PROP_ENUM_FLAG);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_UNIFORM);
	RNA_def_property_enum_funcs(prop, NULL, "rna_FractureModifier_point_source_set", NULL);
	RNA_def_property_ui_text(prop, "Point Source", "Source of point cloud");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "point_seed", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 0, 100000);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_point_seed_set", NULL);
	RNA_def_property_ui_text(prop, "Seed", "Seed for uniform pointcloud");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "percentage", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_percentage_set", NULL);
	RNA_def_property_ui_text(prop, "Percentage", "Percentage of the sum of points of all selected pointsources to actually use for fracture");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "extra_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Extra Group", "");
	RNA_def_property_pointer_funcs(prop, NULL, "rna_FractureModifier_extra_group_set", NULL, NULL);
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "shards_to_islands", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "shards_to_islands", false);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_shards_to_islands_set");
	RNA_def_property_ui_text(prop, "Split Shards to Islands", "Split each shard to separate mesh islands");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "execute_threaded", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "execute_threaded", false);
	RNA_def_property_ui_text(prop, "Execute as threaded job (WIP)", "Execute the fracture as threaded job, Warning: WIP, still may crash");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");


	//expose this to RNA to be able to let py checkbox disappear while job is running, otherwise crash
	prop = RNA_def_property(srna, "refresh", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "refresh", false);
	RNA_def_property_ui_text(prop, "Refresh", "Refresh");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

	prop = RNA_def_property(srna, "thresh_vertex_group", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "thresh_defgrp_name");
	RNA_def_property_ui_text(prop, "Threshold Vertex Group", "Vertex group name for defining weighted thresholds on different mesh parts");
	RNA_def_property_string_funcs(prop, NULL, NULL, "rna_FractureModifier_thresh_defgrp_name_set");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "ground_vertex_group", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "ground_defgrp_name");
	RNA_def_property_ui_text(prop, "Passive Vertex Group", "Vertex group name for defining passive mesh parts (will remain static during rigidbody simulation");
	RNA_def_property_string_funcs(prop, NULL, NULL, "rna_FractureModifier_ground_defgrp_name_set");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "fix_normals", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "fix_normals", false);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_fix_normals_set");
	RNA_def_property_ui_text(prop, "Fix normals (WIP)", "Fix normals of fractured smooth objects, to let cracks nearly disappear");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "inner_material", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Inner Material", "");
	RNA_def_property_pointer_funcs(prop, NULL, "rna_FractureModifier_inner_material_set", NULL, NULL);
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "inner_vertex_group", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "inner_defgrp_name");
	RNA_def_property_ui_text(prop, "Inner Vertex Group", "Vertex group name for defining inner vertices (will contain vertices of inner faces (Boolean, Bisect + Fill only) ");
	RNA_def_property_string_funcs(prop, NULL, NULL, "rna_FractureModifier_inner_defgrp_name_set");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "auto_execute", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "auto_execute", false);
	RNA_def_property_ui_text(prop, "Auto Execute", "Automatic execution of fracturing, CAUTION: this can be slow and buggy");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "dm_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Sub Object Group", "");
	RNA_def_property_pointer_funcs(prop, NULL, "rna_FractureModifier_dm_group_set", NULL, NULL);
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "autohide_dist", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "autohide_dist");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_autohide_dist_set", NULL);
	RNA_def_property_range(prop, 0.0f, 10.0f);
	RNA_def_property_ui_text(prop, "Autohide Distance", "Distance between faces below which both faces should be hidden");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "automerge_dist", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "automerge_dist");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_automerge_dist_set", NULL);
	RNA_def_property_range(prop, 0.0f, 10.0f);
	RNA_def_property_ui_text(prop, "Automerge Distance",
 "Distance between faces below which vertices of both faces should be merged; (costs performance, use with smooth objects and fix normals to better hide cracks)");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_percentage_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "breaking_percentage_weighted", false);
	RNA_def_property_ui_text(prop, "Weighted Percentage", "Modify breaking percentage by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_angle_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "breaking_angle_weighted", false);
	RNA_def_property_ui_text(prop, "Weighted Angle", "Modify breaking angle by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "breaking_distance_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "breaking_distance_weighted", false);
	RNA_def_property_ui_text(prop, "Weighted Distance", "Modify breaking distance by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_particle_birth_coordinates", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "use_particle_birth_coordinates", false);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_use_particle_birth_coordinates_set");
	RNA_def_property_ui_text(prop, "Use Particle Birth Coordinates", "Use birth or simulated state particle coordinates");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "splinter_axis", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_splinter_axises);
	RNA_def_property_flag(prop, PROP_ENUM_FLAG);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_SPLINTER_Z);
	RNA_def_property_enum_funcs(prop, NULL, "rna_FractureModifier_splinter_axis_set", NULL);
	RNA_def_property_ui_text(prop, "Splinter Axis", "Global direction of splinters");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "splinter_length", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 1.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Splinter length", "Length of splinters");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_splinter_length_set", NULL);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_solver_iterations_override", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "cluster_solver_iterations_override");
	RNA_def_property_range(prop, 0, INT_MAX);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_cluster_solver_iterations_override_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Solver Iterations Override", "Override the world constraint solver iteration value for INSIDE clusters with this value, 0 means no override");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "nor_range", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Normal Search Radius", "Radius in which to search for valid normals");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_nor_range_set", NULL);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_breaking_percentage", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "cluster_breaking_percentage");
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_cluster_breaking_percentage_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Breaking Percentage", "Percentage of broken constraints per cluster which leads to breaking of all others");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_breaking_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_breaking_angle");
	RNA_def_property_range(prop, 0, DEG2RADF(360.0));
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_cluster_breaking_angle_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Breaking Angle", "Angle in degrees above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_breaking_distance", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_breaking_distance");
	RNA_def_property_range(prop, 0, FLT_MAX);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_cluster_breaking_distance_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Breaking Distance", "Distance above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	/*Breakable constraints*/
	prop = RNA_def_property(srna, "use_breaking", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_use_breaking_set");
	RNA_def_property_ui_text(prop, "Breakable",
	                         "Constraints can be broken if it receives an impulse above the threshold");
	//RNA_def_property_update(prop, /*NC_OBJECT | ND_POINTCACHE*/ 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_smooth", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "use_smooth", false);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_use_smooth_set");
	RNA_def_property_ui_text(prop, "Smooth Inner Faces", "Set Inner Faces to Smooth Shading (needs refracture)");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "fractal_cuts", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "fractal_cuts");
	RNA_def_property_range(prop, 1, 10);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_fractal_cuts_set", NULL);
	RNA_def_property_ui_text(prop, "Fractal Grid Cuts", "Number of fractal cuts on each cell");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "fractal_amount", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "fractal_amount");
	RNA_def_property_range(prop, 0, 20);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_fractal_amount_set", NULL);
	RNA_def_property_ui_text(prop, "Fractal Displacement", "Amount of fractal displacement on each cell");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "physics_mesh_scale", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "physics_mesh_scale");
	RNA_def_property_range(prop, 0.1f , 1.0f);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_physics_mesh_scale_set", NULL);
	RNA_def_property_ui_text(prop, "Physics Mesh Scale", "Scale factor of physics mesh, reduce this to avoid explosion of the mesh (MESH SHAPE ONLY)");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "fractal_iterations", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "fractal_iterations");
	RNA_def_property_range(prop, 1, 10);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_fractal_iterations_set", NULL);
	RNA_def_property_ui_text(prop, "Fractal Iterations", "Number of times the number of cuts will be made to the grid, with the given fractal amount");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Cluster Group", "Centroids of objects in this group determine where cluster centers will be");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cutter_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Cutter Group", "A set of objects to make boolean cuts against");
	RNA_def_property_pointer_funcs(prop, NULL, "rna_FractureModifier_cutter_group_set", NULL, NULL);
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_greasepencil_edges", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "use_greasepencil_edges", false);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_use_greasepencil_edges_set");
	RNA_def_property_ui_text(prop, "Use Greasepencil Edges", "Use edges instead of points from Greasepencil strokes");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "grease_offset", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "grease_offset");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_grease_offset_set", NULL);
	RNA_def_property_ui_text(prop, "Greasepencil Offset", "Extrusion offset of greasepencil stroke, to create a mesh from it");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "grease_decimate", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "grease_decimate");
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_grease_decimate_set", NULL);
	RNA_def_property_ui_text(prop, "Greasepencil Decimate", "Decimate Factor in percent for greasepencil strokes");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cutter_axis", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_cutter_axises);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_CUTTER_Z);
	RNA_def_property_enum_funcs(prop, NULL, "rna_FractureModifier_cutter_axis_set", NULL);
	RNA_def_property_ui_text(prop, "Cutter Axis", "Global direction of cutters");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_constraint_type", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "cluster_constraint_type");
	RNA_def_property_enum_items(prop, rna_enum_rigidbody_constraint_type_items);
	RNA_def_property_enum_funcs(prop, NULL, "rna_FractureModifier_cluster_constraint_type_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Constraint Type", "Type of Rigid Body Constraint between clusters");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "constraint_target", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_constraint_targets);
	RNA_def_property_enum_funcs(prop, NULL, "rna_FractureModifier_constraint_target_set", NULL);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_CENTROID);
	RNA_def_property_ui_text(prop, "Constraint Method", "Method to build constraints");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "fracture_mode", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_fracture_modes);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_PREFRACTURED);
	RNA_def_property_ui_text(prop, "Fracture Mode", "Determines how to fracture the mesh");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "dynamic_force", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Dynamic force threshold", "Only break dynamically when force is above this threshold");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update_and_keep");

	prop = RNA_def_property(srna, "limit_impact", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "limit_impact", false);
	RNA_def_property_ui_text(prop, "Limit Impact", "Activates only shards within the impact object size approximately");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update_and_keep");

#if 0
	prop = RNA_def_property(srna, "fracture_settings", PROP_COLLECTION, PROP_NONE);
	RNA_def_property_struct_type(prop, "FractureSettings");
	RNA_def_property_collection_sdna(prop, NULL, "fracture_settings", NULL);
	RNA_def_property_ui_text(prop, "Fracture Settings", "Settings defining the fracture and constraint parameters per island vertex group");
	rna_def_fracture_settings(brna, prop);

	prop = RNA_def_property(srna, "active_setting", PROP_INT, PROP_UNSIGNED);
	RNA_def_property_int_sdna(prop, NULL, "active_setting");
	RNA_def_property_ui_text(prop, "Active Fracture Setting", "Index of active fracture setting");
	RNA_def_property_update(prop, 0, "rna_Modifier_update_index");
#endif

	prop = RNA_def_property(srna, "use_compounds", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_use_compounds_set");
	RNA_def_property_ui_text(prop, "Use Compounds", "Use compounds instead of fixed constraints (supposed to be faster and not wobbling)");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

#if 0
	prop = RNA_def_property(srna, "impulse_dampening", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "impulse_dampening");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_impulse_dampening_set", NULL);
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_ui_text(prop, "Impulse Dampening", "Determines how strong the impulse is dampened during damage propagation steps");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "directional_factor", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "directional_factor");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_directional_factor_set", NULL);
	RNA_def_property_range(prop, -1.0f, 1.0f);
	RNA_def_property_ui_text(prop, "Directional Factor",
	                         "Determines how much the damage propagation depends on impact direction; -1 means not at all, 1 fully (dot product)");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");
#endif

	prop = RNA_def_property(srna, "minimum_impulse", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "minimum_impulse");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_minimum_impulse_set", NULL);
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Minimum Impulse",
	                         "Determines how strong the remaining impulse must be for continuing damage propagation");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "mass_threshold_factor", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "mass_threshold_factor");
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_mass_threshold_factor_set", NULL);
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_ui_text(prop, "Stability Factor",
	                         "Determines how 'stable' an object is 0 means min_mass / min_mass + max_mass, 1 means maximal threshold everywhere");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "uv_layer", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "uvlayer_name");
	RNA_def_property_ui_text(prop, "Inner UV Map", "Name of UV map for inner faces");
	RNA_def_property_string_funcs(prop, NULL, NULL, "rna_FractureModifier_uvlayer_name_set");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "autohide_filter_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Autohide Filter Group", "");
	RNA_def_property_pointer_funcs(prop, NULL, "rna_FractureModifier_autohide_filter_group_set", NULL, NULL);
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

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

	/* types */
	rna_def_modifier_fracture(brna);
}

#endif
