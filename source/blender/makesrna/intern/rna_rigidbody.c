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
 * Contributor(s): Blender Foundation 2013, Joshua Leung, Sergej Reich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file rna_rigidbody.c
 *  \ingroup rna
 *  \brief RNA property definitions for Rigid Body datatypes
 */

#include <stdlib.h>
#include <string.h>

#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "rna_internal.h"

#include "DNA_fracture_types.h"
#include "DNA_group_types.h"
#include "DNA_object_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"
#include "DNA_modifier_types.h"

#include "BLI_utildefines.h"
#include "BLI_math.h"

#include "WM_types.h"

/* roles of objects in RigidBody Sims */
EnumPropertyItem rigidbody_object_type_items[] = {
	{RBO_TYPE_ACTIVE, "ACTIVE", 0, "Active", "Object is directly controlled by simulation results"},
	{RBO_TYPE_PASSIVE, "PASSIVE", 0, "Passive", "Object is directly controlled by animation system"},
	{0, NULL, 0, NULL, NULL}};

/* collision shapes of objects in rigid body sim */
EnumPropertyItem rigidbody_object_shape_items[] = {
	{RB_SHAPE_BOX, "BOX", ICON_MESH_CUBE, "Box", "Box-like shapes (i.e. cubes), including planes (i.e. ground planes)"},
	{RB_SHAPE_SPHERE, "SPHERE", ICON_MESH_UVSPHERE, "Sphere", ""},
	{RB_SHAPE_CAPSULE, "CAPSULE", ICON_OUTLINER_OB_META, "Capsule", ""},
	{RB_SHAPE_CYLINDER, "CYLINDER", ICON_MESH_CYLINDER, "Cylinder", ""},
	{RB_SHAPE_CONE, "CONE", ICON_MESH_CONE, "Cone", ""},
	{RB_SHAPE_CONVEXH, "CONVEX_HULL", ICON_MESH_ICOSPHERE, "Convex Hull",
	                   "A mesh-like surface encompassing (i.e. shrinkwrap over) all vertices (best results with "
	                   "fewer vertices)"},
	{RB_SHAPE_TRIMESH, "MESH", ICON_MESH_MONKEY, "Mesh",
	                   "Mesh consisting of triangles only, allowing for more detailed interactions than convex hulls"},
	{0, NULL, 0, NULL, NULL}};

/* collision shapes of constraints in rigid body sim */
EnumPropertyItem rigidbody_constraint_type_items[] = {
	{RBC_TYPE_FIXED, "FIXED", ICON_NONE, "Fixed", "Glue rigid bodies together"},
	{RBC_TYPE_POINT, "POINT", ICON_NONE, "Point", "Constrain rigid bodies to move around common pivot point"},
	{RBC_TYPE_HINGE, "HINGE", ICON_NONE, "Hinge", "Restrict rigid body rotation to one axis"},
	{RBC_TYPE_SLIDER, "SLIDER", ICON_NONE, "Slider", "Restrict rigid body translation to one axis"},
	{RBC_TYPE_PISTON, "PISTON", ICON_NONE, "Piston", "Restrict rigid body translation and rotation to one axis"},
	{RBC_TYPE_6DOF, "GENERIC", ICON_NONE, "Generic", "Restrict translation and rotation to specified axes"},
	{RBC_TYPE_6DOF_SPRING, "GENERIC_SPRING", ICON_NONE, "Generic Spring",
	                       "Restrict translation and rotation to specified axes with springs"},
	{RBC_TYPE_MOTOR, "MOTOR", ICON_NONE, "Motor", "Drive rigid body around or along an axis"},
	{0, NULL, 0, NULL, NULL}};

#ifndef RNA_RUNTIME
/* mesh source for collision shape creation */
static EnumPropertyItem rigidbody_mesh_source_items[] = {
	{RBO_MESH_BASE, "BASE", 0, "Base", "Base mesh"},
	{RBO_MESH_DEFORM, "DEFORM", 0, "Deform", "Deformations (shape keys, deform modifiers)"},
	{RBO_MESH_FINAL, "FINAL", 0, "Final", "All modifiers"},
	{0, NULL, 0, NULL, NULL}};
#endif

#ifdef RNA_RUNTIME

#ifdef WITH_BULLET
#  include "RBI_api.h"
#endif

#include "BKE_depsgraph.h"
#include "BKE_rigidbody.h"
#include "BKE_fracture.h"

#include "WM_api.h"

#define RB_FLAG_SET(dest, value, flag) { \
	if (value) \
		dest |= flag; \
	else \
		dest &= ~flag; \
}


/* ******************************** */

static void rna_RigidBodyWorld_reset(Main *UNUSED(bmain), Scene *UNUSED(scene), PointerRNA *ptr)
{
	RigidBodyWorld *rbw = (RigidBodyWorld *)ptr->data;
	
	BKE_rigidbody_cache_reset(rbw);
}

static char *rna_RigidBodyWorld_path(PointerRNA *UNUSED(ptr))
{	
	return BLI_sprintfN("rigidbody_world");
}

static void rna_RigidBodyWorld_num_solver_iterations_set(PointerRNA *ptr, int value)
{
	RigidBodyWorld *rbw = (RigidBodyWorld *)ptr->data;
	
	rbw->num_solver_iterations = value;

#ifdef WITH_BULLET
	if (rbw->physics_world) {
		RB_dworld_set_solver_iterations(rbw->physics_world, value);
	}
#endif
}

static void rna_RigidBodyWorld_split_impulse_set(PointerRNA *ptr, int value)
{
	RigidBodyWorld *rbw = (RigidBodyWorld *)ptr->data;
	
	RB_FLAG_SET(rbw->flag, value, RBW_FLAG_USE_SPLIT_IMPULSE);

#ifdef WITH_BULLET
	if (rbw->physics_world) {
		RB_dworld_set_split_impulse(rbw->physics_world, value);
	}
#endif
}

/* ******************************** */


/* ------------------------------------------ */
static void rna_RigidBodyOb_kinematic_set(PointerRNA *ptr, int value)
{
	RigidBodyOb *rbo = ptr->data;
	RB_FLAG_SET(rbo->flag, value, RBO_FLAG_KINEMATIC);
	rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
}


static void rna_RigidBodyOb_triggered_set(PointerRNA *ptr, int value)
{
	RigidBodyOb *rbo = ptr->data;
	RB_FLAG_SET(rbo->flag, value, RBO_FLAG_KINEMATIC_REBUILD);
	rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
}

static void rna_RigidBodyOb_trigger_set(PointerRNA *ptr, int value)
{
	RigidBodyOb *rbo = ptr->data;
	RB_FLAG_SET(rbo->flag, value, RBO_FLAG_IS_TRIGGER);
	rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
}

static void rna_RigidBodyOb_ghost_set(PointerRNA *ptr, int value)
{
	RigidBodyOb *rbo = ptr->data;
	RB_FLAG_SET(rbo->flag, value, RBO_FLAG_IS_GHOST);
	rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
}

static void fracture_container_reset(FractureContainer *fc, RigidBodyWorld *rbw)
{
	//this caused the modifier to abort execution when animation was enabled and disabled again
	//fc->flag |= (FM_FLAG_REFRESH | FM_FLAG_RESET_SHARDS);
	fc->flag |= FM_FLAG_RESET_SHARDS;
	BKE_rigidbody_cache_reset(rbw);
}

static void rna_FractureContainer_reset(Main *UNUSED(bmain), Scene *scene, PointerRNA *ptr)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	FractureContainer *fc = ptr->data;
	fracture_container_reset(fc, rbw);
//	BKE_fracture_synchronize_caches(scene);
}

static void rna_FractureContainer_rigidbody_reset(Main *bmain, Scene *scene, PointerRNA *ptr)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyOb *rbo = ptr->data;
	FractureContainer *fc = rbo->fracture_objects;
	rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;

	fracture_container_reset(fc, rbw);
//	BKE_fracture_synchronize_caches(scene);
}

static void constraint_container_reset(ConstraintContainer *cc, RigidBodyWorld *rbw, Object* ob)
{
	cc->flag |= FM_FLAG_REFRESH_CONSTRAINTS;
	BKE_fracture_constraint_container_update(ob);
	cc->flag &= ~FM_FLAG_REFRESH_CONSTRAINTS;
	BKE_rigidbody_cache_reset(rbw);
}

static void rna_ConstraintContainer_reset(Main *UNUSED(bmain), Scene *scene, PointerRNA *ptr)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	ConstraintContainer *cc = ptr->data;
	Object *ob = ptr->id.data;

	constraint_container_reset(cc, rbw, ob);
	rbw->pointcache = NULL;
	BKE_fracture_synchronize_caches(scene);
}

static void rna_ConstraintContainer_constraint_reset(Main *UNUSED(bmain), Scene *scene, PointerRNA *ptr)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyCon *con = ptr->data;
	Object *ob = ptr->id.data;
	ConstraintContainer *cc = con->fracture_constraints;

	constraint_container_reset(cc, rbw, ob);
	rbw->pointcache = NULL;
	BKE_fracture_synchronize_caches(scene);
}

static void rna_FractureContainer_rigidbody_shape_update(Main *bmain, Scene *scene, PointerRNA *ptr)
{
	Object *ob = ptr->id.data;

	rna_FractureContainer_rigidbody_reset(bmain, scene, ptr);

	WM_main_add_notifier(NC_OBJECT | ND_DRAW, ob);
}

static void rna_RigidBodyOb_shape_reset(Main *UNUSED(bmain), Scene *scene, PointerRNA *ptr)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;
	RigidBodyOb *rbo = ptr->data;
	rbo->flag |= RBO_FLAG_NEEDS_RESHAPE;

	BKE_rigidbody_cache_reset(rbw);
}

static void rna_FractureContainer_autohide_update(Main *UNUSED(bmain), Scene *scene, PointerRNA *ptr)
{
	FractureContainer *fc = ptr->data;
	fc->flag |= FM_FLAG_UPDATE_AUTOHIDE;
	DAG_id_tag_update(ptr->id.data, OB_RECALC_DATA);
	WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, ptr->id.data);
}

static char *rna_FractureContainer_path(PointerRNA *UNUSED(ptr))
{
	/* NOTE: this hardcoded path should work as long as only Objects have this */
	return BLI_sprintfN("rigidbody_object.fracture_container");
}

static char *rna_ConstraintContainer_path(PointerRNA *UNUSED(ptr))
{
	/* NOTE: this hardcoded path should work as long as only Objects have this */
	return BLI_sprintfN("rigidbody_constraint.constraint_container");
}

static char *rna_RigidBodyOb_path(PointerRNA *UNUSED(ptr))
{
	/* NOTE: this hardcoded path should work as long as only Objects have this */
	return BLI_sprintfN("rigidbody_object");
}

void set_collision_groups(RigidBodyOb* rbo, const int *values)
{
	int i;

	for (i = 0; i < 20; i++) {
		if (values[i])
			rbo->col_groups |= (1 << i);
		else
			rbo->col_groups &= ~(1 << i);
	}
	rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;
}

static void rna_RigidBodyOb_collision_groups_set(PointerRNA *ptr, const int *values)
{
	RigidBodyOb *rbo = (RigidBodyOb *)ptr->data;
	set_collision_groups(rbo, values);
}

static char *rna_RigidBodyCon_path(PointerRNA *UNUSED(ptr))
{
	/* NOTE: this hardcoded path should work as long as only Objects have this */
	return BLI_sprintfN("rigidbody_constraint");
}

/* Sweep test, take 1st island only */
static void rna_RigidBodyWorld_convex_sweep_test(
        RigidBodyWorld *rbw, ReportList *reports,
        Object *object, float ray_start[3], float ray_end[3],
        float r_location[3], float r_hitpoint[3], float r_normal[3], int *r_hit)
{
#ifdef WITH_BULLET
	MeshIsland *mi = object->rigidbody_object->fracture_objects->current->island_map.first;
	RigidBodyShardOb *rob = mi->rigidbody;
	if (rbw->physics_world != NULL && rob->physics_object != NULL) {
		RB_world_convex_sweep_test(rbw->physics_world, rob->physics_object, ray_start, ray_end,
		                           r_location, r_hitpoint, r_normal, r_hit);
		if (*r_hit == -2) {
			BKE_report(reports, RPT_ERROR,
			           "A non convex collision shape was passed to the function, use only convex collision shapes");
		}
	}
	else {
		*r_hit = -1;
		BKE_report(reports, RPT_ERROR, "Rigidbody world was not properly initialized, need to step the simulation first");
	}
#else
	UNUSED_VARS(rbw, reports, object, ray_start, ray_end, r_location, r_hitpoint, r_normal, r_hit);
#endif
}

#else

static void rna_def_rigidbody_world(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;
	FunctionRNA *func;
	
	srna = RNA_def_struct(brna, "RigidBodyWorld", NULL);
	RNA_def_struct_sdna(srna, "RigidBodyWorld");
	RNA_def_struct_ui_text(srna, "Rigid Body World", "Self-contained rigid body simulation environment and settings");
	RNA_def_struct_path_func(srna, "rna_RigidBodyWorld_path");
	
	/* groups */
	prop = RNA_def_property(srna, "group", PROP_POINTER, PROP_NONE);
	RNA_def_property_struct_type(prop, "Group");
	RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_SELF_CHECK);
	RNA_def_property_ui_text(prop, "Group", "Group containing objects participating in this simulation");
	RNA_def_property_update(prop, NC_SCENE, "rna_RigidBodyWorld_reset");

	prop = RNA_def_property(srna, "constraints", PROP_POINTER, PROP_NONE);
	RNA_def_property_struct_type(prop, "Group");
	RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_SELF_CHECK);
	RNA_def_property_ui_text(prop, "Constraints", "Group containing rigid body constraint objects");
	RNA_def_property_update(prop, NC_SCENE, "rna_RigidBodyWorld_reset");
	
	/* booleans */
	prop = RNA_def_property(srna, "enabled", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_negative_sdna(prop, NULL, "flag", RBW_FLAG_MUTED);
	RNA_def_property_ui_text(prop, "Enabled", "Simulation will be evaluated");
	RNA_def_property_update(prop, NC_SCENE, NULL);
	
	/* time scale */
	prop = RNA_def_property(srna, "time_scale", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "time_scale");
	RNA_def_property_range(prop, 0.0f, 100.0f);
	RNA_def_property_ui_range(prop, 0.0f, 10.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Time Scale", "Change the speed of the simulation");
	RNA_def_property_update(prop, NC_SCENE, "rna_RigidBodyWorld_reset");
	
	/* timestep */
	prop = RNA_def_property(srna, "steps_per_second", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "steps_per_second");
	RNA_def_property_range(prop, 1, SHRT_MAX);
	RNA_def_property_ui_range(prop, 60, 1000, 1, 0);
	RNA_def_property_int_default(prop, 60);
	RNA_def_property_ui_text(prop, "Steps Per Second",
	                         "Number of simulation steps taken per second (higher values are more accurate "
	                         "but slower)");
	RNA_def_property_update(prop, NC_SCENE, "rna_RigidBodyWorld_reset");
	
	/* constraint solver iterations */
	prop = RNA_def_property(srna, "solver_iterations", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "num_solver_iterations");
	RNA_def_property_range(prop, 1, 1000);
	RNA_def_property_ui_range(prop, 10, 100, 1, 0);
	RNA_def_property_int_default(prop, 10);
	RNA_def_property_int_funcs(prop, NULL, "rna_RigidBodyWorld_num_solver_iterations_set", NULL);
	RNA_def_property_ui_text(prop, "Solver Iterations",
	                         "Number of constraint solver iterations made per simulation step (higher values are more "
	                         "accurate but slower)");
	RNA_def_property_update(prop, NC_SCENE, "rna_RigidBodyWorld_reset");
	
	/* split impulse */
	prop = RNA_def_property(srna, "use_split_impulse", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBW_FLAG_USE_SPLIT_IMPULSE);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_RigidBodyWorld_split_impulse_set");
	RNA_def_property_ui_text(prop, "Split Impulse",
	                         "Reduce extra velocity that can build up when objects collide (lowers simulation "
	                         "stability a little so use only when necessary)");
	RNA_def_property_update(prop, NC_SCENE, "rna_RigidBodyWorld_reset");

	/* cache */
	prop = RNA_def_property(srna, "point_cache", PROP_POINTER, PROP_NONE);
	RNA_def_property_flag(prop, PROP_NEVER_NULL);
	RNA_def_property_pointer_sdna(prop, NULL, "pointcache");
	RNA_def_property_ui_text(prop, "Point Cache", "");

	/* effector weights */
	prop = RNA_def_property(srna, "effector_weights", PROP_POINTER, PROP_NONE);
	RNA_def_property_struct_type(prop, "EffectorWeights");
	RNA_def_property_clear_flag(prop, PROP_EDITABLE);
	RNA_def_property_ui_text(prop, "Effector Weights", "");

	/* Sweep test */
	func = RNA_def_function(srna, "convex_sweep_test", "rna_RigidBodyWorld_convex_sweep_test");
	RNA_def_function_ui_description(func, "Sweep test convex rigidbody against the current rigidbody world");
	RNA_def_function_flag(func, FUNC_USE_REPORTS);

	prop = RNA_def_pointer(func, "object", "Object", "", "Rigidbody object with a convex collision shape");
	RNA_def_property_flag(prop, PROP_REQUIRED | PROP_NEVER_NULL);
	RNA_def_property_clear_flag(prop, PROP_THICK_WRAP);

	/* ray start and end */
	prop = RNA_def_float_vector(func, "start", 3, NULL, -FLT_MAX, FLT_MAX, "", "", -1e4, 1e4);
	RNA_def_property_flag(prop, PROP_REQUIRED);
	prop = RNA_def_float_vector(func, "end", 3, NULL, -FLT_MAX, FLT_MAX, "", "", -1e4, 1e4);
	RNA_def_property_flag(prop, PROP_REQUIRED);

	prop = RNA_def_float_vector(func, "object_location", 3, NULL, -FLT_MAX, FLT_MAX, "Location",
	                            "The hit location of this sweep test", -1e4, 1e4);
	RNA_def_property_flag(prop, PROP_THICK_WRAP);
	RNA_def_function_output(func, prop);

	prop = RNA_def_float_vector(func, "hitpoint", 3, NULL, -FLT_MAX, FLT_MAX, "Hitpoint",
	                            "The hit location of this sweep test", -1e4, 1e4);
	RNA_def_property_flag(prop, PROP_THICK_WRAP);
	RNA_def_function_output(func, prop);

	prop = RNA_def_float_vector(func, "normal", 3, NULL, -FLT_MAX, FLT_MAX, "Normal",
	                            "The face normal at the sweep test hit location", -1e4, 1e4);
	RNA_def_property_flag(prop, PROP_THICK_WRAP);
	RNA_def_function_output(func, prop);

	prop = RNA_def_int(func, "has_hit", 0, 0, 0, "", "If the function has found collision point, value is 1, otherwise 0", 0, 0);
	RNA_def_function_output(func, prop);
}

static void rna_def_rigidbody_object(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;
	
	srna = RNA_def_struct(brna, "RigidBodyObject", NULL);
	RNA_def_struct_sdna(srna, "RigidBodyOb");
	RNA_def_struct_ui_text(srna, "Rigid Body Object", "Settings for object participating in Rigid Body Simulation");
	RNA_def_struct_path_func(srna, "rna_RigidBodyOb_path");

	prop = RNA_def_property(srna, "fracture_container", PROP_POINTER, PROP_NONE);
	RNA_def_property_pointer_sdna(prop, NULL, "fracture_objects");
	RNA_def_property_struct_type(prop, "FractureContainer");
	RNA_def_property_ui_text(prop, "Fracture Container", "Container for fractured rigid bodies");
	
	/* Enums */
	prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "type");
	RNA_def_property_enum_items(prop, rigidbody_object_type_items);
	RNA_def_property_ui_text(prop, "Type", "Role of object in Rigid Body Simulations");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "mesh_source", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "mesh_source");
	RNA_def_property_enum_items(prop, rigidbody_mesh_source_items);
	RNA_def_property_ui_text(prop, "Mesh Source", "Source of the mesh used to create collision shape");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	/* booleans */
	prop = RNA_def_property(srna, "enabled", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_negative_sdna(prop, NULL, "flag", RBO_FLAG_DISABLED);
	RNA_def_property_ui_text(prop, "Enabled", "Rigid Body actively participates to the simulation");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "collision_shape", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "shape");
	RNA_def_property_enum_items(prop, rigidbody_object_shape_items);
	RNA_def_property_ui_text(prop, "Collision Shape", "Collision Shape of object in Rigid Body Simulations");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_shape_update");
	
	prop = RNA_def_property(srna, "kinematic", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_KINEMATIC);
	//RNA_def_property_boolean_funcs(prop, NULL, "rna_RigidBodyOb_kinematic_set");
	RNA_def_property_ui_text(prop, "Kinematic", "Allow rigid body to be controlled by the animation system");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "use_deform", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_USE_DEFORM);
	RNA_def_property_ui_text(prop, "Deforming", "Rigid body deforms during simulation");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");

	prop = RNA_def_property(srna, "use_kinematic_deactivation", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_USE_KINEMATIC_DEACTIVATION);
	//RNA_def_property_boolean_funcs(prop, NULL, "rna_RigidBodyOb_triggered_set");
	RNA_def_property_ui_text(prop, "Kinematic Deactivation", "Allow kinematic state being reset by collisions");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");

	prop = RNA_def_property(srna, "is_ghost", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_IS_GHOST);
	//RNA_def_property_boolean_funcs(prop, NULL, "rna_RigidBodyOb_ghost_set");
	RNA_def_property_ui_text(prop, "Ghost", "Do not collide with object, but can activate other animated objects");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");

	prop = RNA_def_property(srna, "is_trigger", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_IS_TRIGGER);
	//RNA_def_property_boolean_funcs(prop, NULL, "rna_RigidBodyOb_trigger_set");
	RNA_def_property_ui_text(prop, "Trigger", "Can trigger activation of other animated objects, which are set up to be triggered");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	/* Physics Parameters */
	prop = RNA_def_property(srna, "mass", PROP_FLOAT, PROP_UNIT_MASS);
	RNA_def_property_float_sdna(prop, NULL, "mass");
	RNA_def_property_range(prop, 0.001f, FLT_MAX); // range must always be positive (and non-zero)
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Mass", "How much the object 'weighs' irrespective of gravity");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	/* Dynamics Parameters - Activation */
	// TODO: define and figure out how to implement these
	
	/* Dynamics Parameters - Deactivation */
	prop = RNA_def_property(srna, "use_deactivation", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_USE_DEACTIVATION);
	RNA_def_property_boolean_default(prop, true);
	RNA_def_property_ui_text(prop, "Enable Deactivation",
	                         "Enable deactivation of resting rigid bodies (increases performance and stability "
	                         "but can cause glitches)");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "use_start_deactivated", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_START_DEACTIVATED);
	RNA_def_property_ui_text(prop, "Start Deactivated", "Deactivate rigid body at the start of the simulation");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "deactivate_linear_velocity", PROP_FLOAT, PROP_UNIT_VELOCITY);
	RNA_def_property_float_sdna(prop, NULL, "lin_sleep_thresh");
	RNA_def_property_range(prop, FLT_MIN, FLT_MAX); // range must always be positive (and non-zero)
	RNA_def_property_float_default(prop, 0.4f);
	RNA_def_property_ui_text(prop, "Linear Velocity Deactivation Threshold",
	                         "Linear Velocity below which simulation stops simulating object");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "deactivate_angular_velocity", PROP_FLOAT, PROP_UNIT_VELOCITY);
	RNA_def_property_float_sdna(prop, NULL, "ang_sleep_thresh");
	RNA_def_property_range(prop, FLT_MIN, FLT_MAX); // range must always be positive (and non-zero)
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_ui_text(prop, "Angular Velocity Deactivation Threshold",
	                         "Angular Velocity below which simulation stops simulating object");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	/* Dynamics Parameters - Damping Parameters */
	prop = RNA_def_property(srna, "linear_damping", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "lin_damping");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.04f);
	RNA_def_property_ui_text(prop, "Linear Damping", "Amount of linear velocity that is lost over time");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "angular_damping", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "ang_damping");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.1f);
	RNA_def_property_ui_text(prop, "Angular Damping", "Amount of angular velocity that is lost over time");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	/* Collision Parameters - Surface Parameters */
	prop = RNA_def_property(srna, "friction", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "friction");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1, 3);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_ui_text(prop, "Friction", "Resistance of object to movement");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	prop = RNA_def_property(srna, "restitution", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "restitution");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1, 3);
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_ui_text(prop, "Restitution",
	                         "Tendency of object to bounce after colliding with another "
	                         "(0 = stays still, 1 = perfectly elastic)");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	
	/* Collision Parameters - Sensitivity */
	prop = RNA_def_property(srna, "use_margin", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBO_FLAG_USE_MARGIN);
	RNA_def_property_boolean_default(prop, false);
	RNA_def_property_ui_text(prop, "Collision Margin",
	                         "Use custom collision margin (some shapes will have a visible gap around them)");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_RigidBodyOb_shape_reset");
	
	prop = RNA_def_property(srna, "collision_margin", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "margin");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_ui_range(prop, 0.0f, 1.0f, 0.01, 3);
	RNA_def_property_float_default(prop, 0.04f);
	RNA_def_property_ui_text(prop, "Collision Margin",
	                         "Threshold of distance near surface where collisions are still considered "
	                         "(best results when non-zero)");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_RigidBodyOb_shape_reset");
	
	prop = RNA_def_property(srna, "collision_groups", PROP_BOOLEAN, PROP_LAYER_MEMBER);
	RNA_def_property_boolean_sdna(prop, NULL, "col_groups", 1);
	RNA_def_property_array(prop, 20);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_RigidBodyOb_collision_groups_set");
	RNA_def_property_ui_text(prop, "Collision Groups", "Collision Groups Rigid Body belongs to");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_rigidbody_reset");
	RNA_def_property_flag(prop, PROP_LIB_EXCEPTION);
}

static void rna_def_rigidbody_constraint(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	srna = RNA_def_struct(brna, "RigidBodyConstraint", NULL);
	RNA_def_struct_sdna(srna, "RigidBodyCon");
	RNA_def_struct_ui_text(srna, "Rigid Body Constraint",
	                       "Constraint influencing Objects inside Rigid Body Simulation");
	RNA_def_struct_path_func(srna, "rna_RigidBodyCon_path");

	prop = RNA_def_property(srna, "constraint_container", PROP_POINTER, PROP_NONE);
	RNA_def_property_pointer_sdna(prop, NULL, "fracture_constraints");
	RNA_def_property_struct_type(prop, "ConstraintContainer");
	RNA_def_property_ui_text(prop, "Constraint Container", "Container for fracture constraints");

	prop = RNA_def_property(srna, "object1", PROP_POINTER, PROP_NONE);
	RNA_def_property_pointer_sdna(prop, NULL, "ob1");
	//RNA_def_property_struct_type(prop, "Object");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_ui_text(prop, "Object 1", "First constraint object");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "object2", PROP_POINTER, PROP_NONE);
	RNA_def_property_pointer_sdna(prop, NULL, "ob2");
	//RNA_def_property_struct_type(prop, "Object");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_ui_text(prop, "Object 2", "Second constraint object");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");

	/* Enums */
	prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "type");
	RNA_def_property_enum_items(prop, rigidbody_constraint_type_items);
	RNA_def_property_ui_text(prop, "Type", "Type of Rigid Body Constraint");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");

	//TODO do not use anymore, deprecate
	prop = RNA_def_property(srna, "enabled", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_ENABLED);
	RNA_def_property_ui_text(prop, "Enabled", "Enable this constraint");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "disable_collisions", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_DISABLE_COLLISIONS);
	RNA_def_property_ui_text(prop, "Disable Collisions", "Disable collisions between constrained rigid bodies");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");


	/* Breaking Threshold */
	//TODO do not use anymore, deprecate
	prop = RNA_def_property(srna, "use_breaking", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_BREAKING);
	RNA_def_property_ui_text(prop, "Breakable",
	                         "Constraint can be broken if it receives an impulse above the threshold");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");

	//TODO do not use anymore, deprecate
	prop = RNA_def_property(srna, "breaking_threshold", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_threshold");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 1000.0f, 100.0, 2);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_ui_text(prop, "Breaking Threshold",
	                         "Impulse threshold that must be reached for the constraint to break");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");

	/* Solver Iterations */
	//TODO do not use anymore, deprecate
	prop = RNA_def_property(srna, "use_override_solver_iterations", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS);
	RNA_def_property_ui_text(prop, "Override Solver Iterations",
	                         "Override the number of solver iterations for this constraint");
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_constraint_reset");

	//TODO do not use anymore, deprecate
	prop = RNA_def_property(srna, "solver_iterations", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "num_solver_iterations");
	RNA_def_property_range(prop, 1, 1000);
	RNA_def_property_ui_range(prop, 1, 100, 1, 0);
	RNA_def_property_int_default(prop, 10);
	RNA_def_property_ui_text(prop, "Solver Iterations",
	                         "Number of constraint solver iterations made per simulation step (higher values are more "
	                         "accurate but slower)");
	RNA_def_property_update(prop, NC_OBJECT, "rna_FractureContainer_rigidbody_reset");

	/* Limits */
	prop = RNA_def_property(srna, "use_limit_lin_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_X);
	RNA_def_property_ui_text(prop, "X Axis", "Limit translation on X axis");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_limit_lin_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_Y);
	RNA_def_property_ui_text(prop, "Y Axis", "Limit translation on Y axis");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_limit_lin_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_LIN_Z);
	RNA_def_property_ui_text(prop, "Z Axis", "Limit translation on Z axis");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_limit_ang_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_X);
	RNA_def_property_ui_text(prop, "X Angle", "Limit rotation around X axis");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_limit_ang_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_Y);
	RNA_def_property_ui_text(prop, "Y Angle", "Limit rotation around Y axis");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_limit_ang_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_LIMIT_ANG_Z);
	RNA_def_property_ui_text(prop, "Z Angle", "Limit rotation around Z axis");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_spring_x", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_X);
	RNA_def_property_ui_text(prop, "X Spring", "Enable spring on X axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_spring_y", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_Y);
	RNA_def_property_ui_text(prop, "Y Spring", "Enable spring on Y axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_spring_z", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_SPRING_Z);
	RNA_def_property_ui_text(prop, "Z Spring", "Enable spring on Z axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_FractureContainer_rigidbody_reset");

	prop = RNA_def_property(srna, "use_motor_lin", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_MOTOR_LIN);
	RNA_def_property_ui_text(prop, "Linear Motor", "Enable linear motor");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "use_motor_ang", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", RBC_FLAG_USE_MOTOR_ANG);
	RNA_def_property_ui_text(prop, "Angular Motor", "Enable angular motor");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_lin_x_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_x_lower");
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower X Limit", "Lower limit of X axis translation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_lin_x_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_x_upper");
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper X Limit", "Upper limit of X axis translation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_lin_y_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_y_lower");
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower Y Limit", "Lower limit of Y axis translation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_lin_y_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_y_upper");
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper Y Limit", "Upper limit of Y axis translation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_lin_z_lower", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_z_lower");
	RNA_def_property_float_default(prop, -1.0f);
	RNA_def_property_ui_text(prop, "Lower Z Limit", "Lower limit of Z axis translation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_lin_z_upper", PROP_FLOAT, PROP_UNIT_LENGTH);
	RNA_def_property_float_sdna(prop, NULL, "limit_lin_z_upper");
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Upper Z Limit", "Upper limit of Z axis translation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_ang_x_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_x_lower");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower X Angle Limit", "Lower limit of X axis rotation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_ang_x_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_x_upper");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper X Angle Limit", "Upper limit of X axis rotation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_ang_y_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_y_lower");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower Y Angle Limit", "Lower limit of Y axis rotation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_ang_y_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_y_upper");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper Y Angle Limit", "Upper limit of Y axis rotation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_ang_z_lower", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_z_lower");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, -M_PI_4);
	RNA_def_property_ui_text(prop, "Lower Z Angle Limit", "Lower limit of Z axis rotation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "limit_ang_z_upper", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "limit_ang_z_upper");
	RNA_def_property_range(prop, -M_PI * 2, M_PI * 2);
	RNA_def_property_float_default(prop, M_PI_4);
	RNA_def_property_ui_text(prop, "Upper Z Angle Limit", "Upper limit of Z axis rotation");
	RNA_def_property_update(prop, NC_OBJECT | ND_DRAW, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "spring_stiffness_x", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_x");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_ui_text(prop, "X Axis Stiffness", "Stiffness on the X axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "spring_stiffness_y", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_y");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_ui_text(prop, "Y Axis Stiffness", "Stiffness on the Y axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "spring_stiffness_z", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "spring_stiffness_z");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 10.0f);
	RNA_def_property_ui_text(prop, "Z Axis Stiffness", "Stiffness on the Z axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "spring_damping_x", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_x");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_ui_text(prop, "Damping X", "Damping on the X axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "spring_damping_y", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_y");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_ui_text(prop, "Damping Y", "Damping on the Y axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "spring_damping_z", PROP_FLOAT, PROP_FACTOR);
	RNA_def_property_float_sdna(prop, NULL, "spring_damping_z");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.5f);
	RNA_def_property_ui_text(prop, "Damping Z", "Damping on the Z axis");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "motor_lin_target_velocity", PROP_FLOAT, PROP_UNIT_VELOCITY);
	RNA_def_property_float_sdna(prop, NULL, "motor_lin_target_velocity");
	RNA_def_property_range(prop, -FLT_MAX, FLT_MAX);
	RNA_def_property_ui_range(prop, -100.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Target Velocity", "Target linear motor velocity");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "motor_lin_max_impulse", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "motor_lin_max_impulse");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Max Impulse", "Maximum linear motor impulse");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "motor_ang_target_velocity", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "motor_ang_target_velocity");
	RNA_def_property_range(prop, -FLT_MAX, FLT_MAX);
	RNA_def_property_ui_range(prop, -100.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Target Velocity", "Target angular motor velocity");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");

	prop = RNA_def_property(srna, "motor_ang_max_impulse", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "motor_ang_max_impulse");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_range(prop, 0.0f, 100.0f, 1, 3);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Max Impulse", "Maximum angular motor impulse");
	RNA_def_property_update(prop, NC_OBJECT, "rna_ConstraintContainer_constraint_reset");
}

static void rna_def_rigidbody_constraint_container(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	static EnumPropertyItem prop_constraint_targets[] = {
		{MOD_FRACTURE_CENTROID, "CENTROID", 0, "Centroid", "Build constraints based on distances between centroids"},
		{MOD_FRACTURE_VERTEX, "VERTEX", 0, "Vertex", "Build constraints based on distances between vertices (use lower values here)"},
		{0, NULL, 0, NULL, NULL}
	};

	srna = RNA_def_struct(brna, "ConstraintContainer", NULL);
	RNA_def_struct_ui_text(srna, "Constraint Container", "Container of rigidbody constraints");
	RNA_def_struct_sdna(srna, "ConstraintContainer");
	RNA_def_struct_path_func(srna, "rna_ConstraintContainer_path");


	prop = RNA_def_property(srna, "breaking_threshold", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_threshold");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Inner Breaking threshold", "Threshold to break constraints between shards in the same object");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "use_constraints", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FMC_FLAG_USE_CONSTRAINTS);
	RNA_def_property_ui_text(prop, "Use Constraints", "Create constraints between all shards");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "contact_dist", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Search Radius", "Limit search radius up to which two mesh islands are being connected, 0 for entire boundingbox");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "use_mass_dependent_thresholds", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FMC_FLAG_USE_MASS_DEPENDENT_THRESHOLDS);
	RNA_def_property_ui_text(prop, "Use Mass Dependent Thresholds", "Match the breaking threshold according to the masses of the constrained shards");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

	prop = RNA_def_property(srna, "constraint_limit", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "constraint_limit");
	RNA_def_property_range(prop, 0, INT_MAX);
	RNA_def_property_ui_text(prop, "Constraint Search Limit", "Maximum number of neighbors being searched per mesh island during constraint creation, 0 for unlimited");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "breaking_percentage", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "breaking_percentage");
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_ui_text(prop, "Breaking Percentage", "Percentage of broken constraints per island which leads to breaking of all others");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "breaking_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_angle");
	RNA_def_property_range(prop, 0, DEG2RADF(360.0));
	RNA_def_property_ui_text(prop, "Breaking Angle", "Angle in degrees above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "breaking_distance", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "breaking_distance");
	RNA_def_property_range(prop, 0, FLT_MAX);
	RNA_def_property_ui_text(prop, "Breaking Distance", "Distance above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "cluster_breaking_threshold", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_breaking_threshold");
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Cluster Breaking threshold", "Threshold to break constraints INSIDE a cluster of shards");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "solver_iterations_override", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "solver_iterations_override");
	RNA_def_property_range(prop, 0, INT_MAX);
	RNA_def_property_ui_text(prop, "Solver Iterations Override", "Override the world constraint solver iteration value with this value, 0 means no override");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "breaking_percentage_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FMC_FLAG_BREAKING_PERCENTAGE_WEIGHTED);
	RNA_def_property_ui_text(prop, "Weighted Percentage", "Modify breaking percentage by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "breaking_angle_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FMC_FLAG_BREAKING_ANGLE_WEIGHTED);
	RNA_def_property_ui_text(prop, "Weighted Angle", "Modify breaking angle by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "breaking_distance_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FMC_FLAG_BREAKING_DISTANCE_WEIGHTED);
	RNA_def_property_ui_text(prop, "Weighted Distance", "Modify breaking distance by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "cluster_solver_iterations_override", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "cluster_solver_iterations_override");
	RNA_def_property_range(prop, 0, INT_MAX);
	RNA_def_property_ui_text(prop, "Cluster Solver Iterations Override",
	                         "Override the world constraint solver iteration value for INSIDE clusters with this value, 0 means no override");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "cluster_breaking_percentage", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "cluster_breaking_percentage");
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_ui_text(prop, "Cluster Breaking Percentage", "Percentage of broken constraints per cluster which leads to breaking of all others");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "cluster_breaking_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_breaking_angle");
	RNA_def_property_range(prop, 0, DEG2RADF(360.0));
	RNA_def_property_ui_text(prop, "Cluster Breaking Angle", "Angle in degrees above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "cluster_breaking_distance", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_breaking_distance");
	RNA_def_property_range(prop, 0, FLT_MAX);
	RNA_def_property_ui_text(prop, "Cluster Breaking Distance", "Distance above which constraint should break");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	/*Breakable constraints*/
	prop = RNA_def_property(srna, "use_breaking", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FMC_FLAG_USE_BREAKING);
	RNA_def_property_ui_text(prop, "Breakable",
	                         "Constraints can be broken if it receives an impulse above the threshold");
	//RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "cluster_constraint_type", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "cluster_constraint_type");
	RNA_def_property_enum_items(prop, rigidbody_constraint_type_items);
	RNA_def_property_ui_text(prop, "Cluster Constraint Type", "Type of Rigid Body Constraint between clusters");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");

	prop = RNA_def_property(srna, "constraint_target", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_constraint_targets);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_CENTROID);
	RNA_def_property_ui_text(prop, "Constraint Method", "Method to build constraints");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_ConstraintContainer_reset");
}

static void rna_def_rigidbody_fracture_container(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;

	static EnumPropertyItem prop_fracture_modes[] = {
		{MOD_FRACTURE_PREFRACTURED, "PREFRACTURED", 0, "Prefractured", "Fracture the mesh once prior to the simulation"},
		{MOD_FRACTURE_DYNAMIC, "DYNAMIC", 0, "Dynamic", "Fracture the mesh dynamically during the simulation"},
		{0, NULL, 0, NULL, NULL}
	};

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

	srna = RNA_def_struct(brna, "FractureContainer", NULL);
	RNA_def_struct_ui_text(srna, "Fracture Container", "Add a fracture container to this object");
	RNA_def_struct_sdna(srna, "FractureContainer");
	RNA_def_struct_path_func(srna, "rna_FractureContainer_path");

	prop = RNA_def_property(srna, "use_experimental", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_USE_EXPERIMENTAL);
	RNA_def_property_ui_text(prop, "Use Experimental", "Experimental features, work in progress. Use at own risk!");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

	prop = RNA_def_property(srna, "shards_to_islands", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_SHARDS_TO_ISLANDS);
	RNA_def_property_ui_text(prop, "Split Shards to Islands", "Split each shard to separate mesh islands");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	//TODO deprecated, wont probably be used any more...
	prop = RNA_def_property(srna, "execute_threaded", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_EXECUTE_THREADED);
	RNA_def_property_ui_text(prop, "Execute as threaded job (WIP)", "Execute the fracture as threaded job, Warning: WIP, still may crash");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "fracture_mode", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_fracture_modes);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_PREFRACTURED);
	RNA_def_property_ui_text(prop, "Fracture Mode", "Determines how to fracture the mesh");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "frac_algorithm", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_fracture_algorithm);
	RNA_def_property_ui_text(prop, "Fracture Algorithm", "Select type of fracture algorithm");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "shard_count", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 1, 100000);
	RNA_def_property_int_default(prop, 10);
	RNA_def_property_ui_text(prop, "Shard Count", "How many sub-shards should be generated from the current shard");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "cluster_count", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 0, 100000);
	RNA_def_property_ui_text(prop, "Cluster Count", "Amount of clusters built from existing shards, 0 for none");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "point_source", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_point_source_items);
	RNA_def_property_flag(prop, PROP_ENUM_FLAG);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_UNIFORM);
	RNA_def_property_ui_text(prop, "Point Source", "Source of point cloud");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "point_seed", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 0, 100000);
	RNA_def_property_ui_text(prop, "Seed", "Seed for uniform pointcloud");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "percentage", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_ui_text(prop, "Percentage", "Percentage of the sum of points of all selected pointsources to actually use for fracture");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "extra_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Extra Group", "");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "cluster_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Cluster Group", "Centroids of objects in this group determine where cluster centers will be");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

	prop = RNA_def_property(srna, "thresh_vertex_group", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "thresh_defgrp_name");
	RNA_def_property_ui_text(prop, "Threshold Vertex Group", "Vertex group name for defining weighted thresholds on different mesh parts");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "ground_vertex_group", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "ground_defgrp_name");
	RNA_def_property_ui_text(prop, "Passive Vertex Group", "Vertex group name for defining passive mesh parts (will remain static during rigidbody simulation");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "fix_normals", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_FIX_NORMALS);
	RNA_def_property_ui_text(prop, "Fix normals (WIP)", "Fix normals of fractured smooth objects, to let cracks nearly disappear");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "inner_material", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Inner Material", "");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "inner_vertex_group", PROP_STRING, PROP_NONE);
	RNA_def_property_string_sdna(prop, NULL, "inner_defgrp_name");
	RNA_def_property_ui_text(prop, "Inner Vertex Group",
	                         "Vertex group name for defining inner vertices (will contain vertices of inner faces (Boolean, Bisect + Fill only) ");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "autohide_dist", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "autohide_dist");
	RNA_def_property_range(prop, 0.0f, 10.0f);
	RNA_def_property_ui_text(prop, "Autohide Distance", "Distance between faces below which both faces should be hidden");
	RNA_def_property_update(prop, 0, "rna_FractureContainer_autohide_update");

	prop = RNA_def_property(srna, "use_particle_birth_coordinates", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_USE_PARTICLE_BIRTH_COORDS);
	RNA_def_property_ui_text(prop, "Use Particle Birth Coordinates", "Use birth or simulated state particle coordinates");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "splinter_axis", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_splinter_axises);
	RNA_def_property_flag(prop, PROP_ENUM_FLAG);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_SPLINTER_Z);
	RNA_def_property_ui_text(prop, "Splinter Axis", "Global direction of splinters");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "splinter_length", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 1.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Splinter length", "Length of splinters");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "nor_range", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Normal Search Radius", "Radius in which to search for valid normals");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "use_smooth", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_USE_SMOOTH);
	RNA_def_property_ui_text(prop, "Smooth Inner Faces", "Set Inner Faces to Smooth Shading (needs refracture)");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "fractal_cuts", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 1, 10);
	RNA_def_property_ui_text(prop, "Fractal Grid Cuts", "Number of fractal cuts on each cell");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "fractal_amount", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0, 20);
	RNA_def_property_ui_text(prop, "Fractal Displacement", "Amount of fractal displacement on each cell");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "fractal_iterations", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 1, 10);
	RNA_def_property_ui_text(prop, "Fractal Iterations", "Number of times the number of cuts will be made to the grid, with the given fractal amount");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "cutter_group", PROP_POINTER, PROP_NONE);
	RNA_def_property_ui_text(prop, "Cutter Group", "A set of objects to make boolean cuts against");
	RNA_def_property_flag(prop, PROP_EDITABLE);
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "use_greasepencil_edges", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_USE_GREASEPENCIL_EDGES);
	RNA_def_property_ui_text(prop, "Use Greasepencil Edges", "Use edges instead of points from Greasepencil strokes");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	//TODO do not expose; automatically determine (longest bbox dimension ?)
	prop = RNA_def_property(srna, "grease_offset", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Greasepencil Offset", "Extrusion offset of greasepencil stroke, to create a mesh from it");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "grease_decimate", PROP_INT, PROP_NONE);
	RNA_def_property_range(prop, 0, 100);
	RNA_def_property_ui_text(prop, "Greasepencil Decimate", "Decimate Factor in percent for greasepencil strokes");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "cutter_axis", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_cutter_axises);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_CUTTER_Z);
	RNA_def_property_ui_text(prop, "Cutter Axis", "Global direction of cutters");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "dynamic_force", PROP_FLOAT, PROP_NONE);
	RNA_def_property_range(prop, 0.0f, FLT_MAX);
	RNA_def_property_ui_text(prop, "Dynamic force threshold", "Only break dynamically when force is above this threshold");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

	prop = RNA_def_property(srna, "limit_impact", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "flag", FM_FLAG_LIMIT_IMPACT);
	RNA_def_property_ui_text(prop, "Limit Impact", "Activates only shards within the impact object size approximately");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, NC_OBJECT | ND_POINTCACHE, "rna_FractureContainer_reset");

}

void RNA_def_rigidbody(BlenderRNA *brna)
{
	rna_def_rigidbody_world(brna);
	rna_def_rigidbody_fracture_container(brna);
	rna_def_rigidbody_constraint_container(brna);
	rna_def_rigidbody_object(brna);
	rna_def_rigidbody_constraint(brna);
}


#endif
