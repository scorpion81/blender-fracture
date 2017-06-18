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
 * Contributor(s): Blender Foundation (2008), Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/makesrna/intern/rna_fracture.c
 *  \ingroup RNA
 */

#include <stdlib.h>

#include "DNA_mesh_types.h"
#include "DNA_meta_types.h"
#include "DNA_fracture_types.h"
#include "DNA_modifier_types.h"
#include "DNA_rigidbody_types.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"
#include "BKE_rigidbody.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "rna_internal.h"

#include "WM_api.h"
#include "WM_types.h"

#ifdef RNA_RUNTIME

#include "BKE_depsgraph.h"
#include "BKE_fracture.h"
#include "BKE_DerivedMesh.h"
#include "BKE_modifier.h"

#ifdef WITH_BULLET
#  include "RBI_api.h"
#endif

static char *rna_Modifier_path(PointerRNA *ptr)
{
	ModifierData *md = ptr->data;
	char name_esc[sizeof(md->name) * 2];

	BLI_strescape(name_esc, md->name, sizeof(name_esc));
	return BLI_sprintfN("modifiers[\"%s\"]", name_esc);
}

static StructRNA *rna_Modifier_refine(struct PointerRNA *ptr)
{
	ModifierData *md = (ModifierData *)ptr->data;

	switch ((ModifierType)md->type) {
		case eModifierType_Subsurf:
			return &RNA_SubsurfModifier;
		case eModifierType_Lattice:
			return &RNA_LatticeModifier;
		case eModifierType_Curve:
			return &RNA_CurveModifier;
		case eModifierType_Build:
			return &RNA_BuildModifier;
		case eModifierType_Mirror:
			return &RNA_MirrorModifier;
		case eModifierType_Decimate:
			return &RNA_DecimateModifier;
		case eModifierType_Wave:
			return &RNA_WaveModifier;
		case eModifierType_Armature:
			return &RNA_ArmatureModifier;
		case eModifierType_Hook:
			return &RNA_HookModifier;
		case eModifierType_Softbody:
			return &RNA_SoftBodyModifier;
		case eModifierType_Boolean:
			return &RNA_BooleanModifier;
		case eModifierType_Array:
			return &RNA_ArrayModifier;
		case eModifierType_EdgeSplit:
			return &RNA_EdgeSplitModifier;
		case eModifierType_Displace:
			return &RNA_DisplaceModifier;
		case eModifierType_UVProject:
			return &RNA_UVProjectModifier;
		case eModifierType_Smooth:
			return &RNA_SmoothModifier;
		case eModifierType_Cast:
			return &RNA_CastModifier;
		case eModifierType_MeshDeform:
			return &RNA_MeshDeformModifier;
		case eModifierType_ParticleSystem:
			return &RNA_ParticleSystemModifier;
		case eModifierType_ParticleInstance:
			return &RNA_ParticleInstanceModifier;
		case eModifierType_Explode:
			return &RNA_ExplodeModifier;
		case eModifierType_Cloth:
			return &RNA_ClothModifier;
		case eModifierType_Collision:
			return &RNA_CollisionModifier;
		case eModifierType_Bevel:
			return &RNA_BevelModifier;
		case eModifierType_Shrinkwrap:
			return &RNA_ShrinkwrapModifier;
		case eModifierType_Fluidsim:
			return &RNA_FluidSimulationModifier;
		case eModifierType_Mask:
			return &RNA_MaskModifier;
		case eModifierType_SimpleDeform:
			return &RNA_SimpleDeformModifier;
		case eModifierType_Multires:
			return &RNA_MultiresModifier;
		case eModifierType_Surface:
			return &RNA_SurfaceModifier;
		case eModifierType_Smoke:
			return &RNA_SmokeModifier;
		case eModifierType_Solidify:
			return &RNA_SolidifyModifier;
		case eModifierType_Screw:
			return &RNA_ScrewModifier;
		case eModifierType_Ocean:
			return &RNA_OceanModifier;
		case eModifierType_Warp:
			return &RNA_WarpModifier;
		case eModifierType_WeightVGEdit:
			return &RNA_VertexWeightEditModifier;
		case eModifierType_WeightVGMix:
			return &RNA_VertexWeightMixModifier;
		case eModifierType_WeightVGProximity:
			return &RNA_VertexWeightProximityModifier;
		case eModifierType_DynamicPaint:
			return &RNA_DynamicPaintModifier;
		case eModifierType_Remesh:
			return &RNA_RemeshModifier;
		case eModifierType_Skin:
			return &RNA_SkinModifier;
		case eModifierType_LaplacianSmooth:
			return &RNA_LaplacianSmoothModifier;
		case eModifierType_Triangulate:
			return &RNA_TriangulateModifier;
		case eModifierType_UVWarp:
			return &RNA_UVWarpModifier;
		case eModifierType_MeshCache:
			return &RNA_MeshCacheModifier;
		case eModifierType_LaplacianDeform:
			return &RNA_LaplacianDeformModifier;
		case eModifierType_Wireframe:
			return &RNA_WireframeModifier;
		case eModifierType_DataTransfer:
			return &RNA_DataTransferModifier;
		case eModifierType_NormalEdit:
			return &RNA_NormalEditModifier;
		case eModifierType_CorrectiveSmooth:
			return &RNA_CorrectiveSmoothModifier;
		case eModifierType_MeshSequenceCache:
			return &RNA_MeshSequenceCacheModifier;
		case eModifierType_Fracture:
			return &RNA_FractureModifier;
		/* Default */
		case eModifierType_None:
		case eModifierType_ShapeKey:
		case NUM_MODIFIER_TYPES:
			return &RNA_Modifier;
	}

	return &RNA_Modifier;
}

/* Vertex Groups */

#define RNA_MOD_VGROUP_NAME_SET(_type, _prop)                                               \
static void rna_##_type##Modifier_##_prop##_set(PointerRNA *ptr, const char *value)         \
{                                                                                           \
	_type##ModifierData *tmd = (_type##ModifierData *)ptr->data;                            \
	rna_object_vgroup_name_set(ptr, value, tmd->_prop, sizeof(tmd->_prop));                 \
}

RNA_MOD_VGROUP_NAME_SET(Fracture, thresh_defgrp_name);
RNA_MOD_VGROUP_NAME_SET(Fracture, ground_defgrp_name);
RNA_MOD_VGROUP_NAME_SET(Fracture, inner_defgrp_name);

#undef RNA_MOD_VGROUP_NAME_SET

/* UV layers */

#define RNA_MOD_UVLAYER_NAME_SET(_type, _prop)                                              \
static void rna_##_type##Modifier_##_prop##_set(PointerRNA *ptr, const char *value)         \
{                                                                                           \
	_type##ModifierData *tmd = (_type##ModifierData *)ptr->data;                            \
	rna_object_uvlayer_name_set(ptr, value, tmd->_prop, sizeof(tmd->_prop));                \
}

RNA_MOD_UVLAYER_NAME_SET(Fracture, uvlayer_name);

#undef RNA_MOD_UVLAYER_NAME_SET


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

static void rna_FractureModifier_use_constraint_collision_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData *)ptr->data;
	rmd->use_constraint_collision = value;
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

static void rna_FractureModifier_deform_angle_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->deform_angle = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_deform_distance_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->deform_distance = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_cluster_deform_angle_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_deform_angle = value;
	rmd->refresh_constraints = true;
}

static void rna_FractureModifier_cluster_deform_distance_set(PointerRNA *ptr, float value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->cluster_deform_distance = value;
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

static void rna_FractureModifier_constraint_type_set(PointerRNA* ptr, int value)
{
	FractureModifierData *rmd = (FractureModifierData*)ptr->data;
	rmd->constraint_type = value;
	rmd->refresh_constraints = true;
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

static void rna_Modifier_update(Main *UNUSED(bmain), Scene *UNUSED(scene), PointerRNA *ptr)
{
	ModifierData* md = ptr->data;

	if (md && md->type == eModifierType_Fracture)
	{
		FractureModifierData *fmd = (FractureModifierData*)md;
		if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
		{
			FractureSetting* fs = BLI_findlink(&fmd->fracture_settings, fmd->active_setting);
			BKE_fracture_store_settings(fmd, fs);

			if (fmd->refresh)
			{
				return;
			}
		}
		else if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			// do purge
			fmd->last_frame = INT_MAX;
			fmd->refresh = true;
		}
	}

	DAG_id_tag_update(ptr->id.data, OB_RECALC_DATA);
	WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, ptr->id.data);
}

static void rna_Modifier_update_and_keep(Main *UNUSED(bmain), Scene *UNUSED(scene), PointerRNA *ptr)
{
	ModifierData* md = ptr->data;

	if (md && md->type == eModifierType_Fracture)
	{
		FractureModifierData *fmd = (FractureModifierData*)md;
		if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED || fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
		{
			FractureSetting* fs = BLI_findlink(&fmd->fracture_settings, fmd->active_setting);
			BKE_fracture_store_settings(fmd, fs);

			if (fmd->refresh)
			{
				return;
			}
		}

		DAG_id_tag_update(ptr->id.data, OB_RECALC_DATA);
		WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, ptr->id.data);
	}
}

#endif

void RNA_def_fracture(BlenderRNA *brna)
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

	static EnumPropertyItem prop_keep_cutter_shards[] = {
		{MOD_FRACTURE_KEEP_BOTH, "KEEP_BOTH", 0, "Both", "Keep both shards(intersect and difference)"},
		{MOD_FRACTURE_KEEP_INTERSECT, "KEEP_INTERSECT", 0, "Intersect Only", "Keep intersected shards only"},
		{MOD_FRACTURE_KEEP_DIFFERENCE, "KEEP_DIFFERENCE", 0, "Difference Only", "Keep difference shards only"},
		{0, NULL, 0, NULL, NULL}
	};

	static EnumPropertyItem prop_solver_items[] = {
		{eBooleanModifierSolver_BMesh, "BMESH", 0, "BMesh", "Use the BMesh boolean solver"},
		{eBooleanModifierSolver_Carve, "CARVE", 0, "Carve", "Use the Carve boolean solver"},
		{0, NULL, 0, NULL, NULL}
	};

	static EnumPropertyItem prop_dynamic_constraints[] = {
		{MOD_FRACTURE_NO_DYNAMIC_CONSTRAINTS, "NO_CONSTRAINTS", 0, "None", "Build no new constraints"},
		{MOD_FRACTURE_MIXED_DYNAMIC_CONSTRAINTS, "MIXED_CONSTRAINTS", 0, "Mixed", "Build constraints between new and old shards"},
		{MOD_FRACTURE_ALL_DYNAMIC_CONSTRAINTS, "ALL_CONSTRAINTS", 0, "All", "Build all new constraints"},
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
	RNA_def_property_ui_text(prop, "Execute multithreaded (WIP)", "Execute the fracture with multiple threads, Warning: Only use on complex geometry, may produce errors on simple geometry!");
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
	//RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_automerge_dist_set", NULL);
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

	prop = RNA_def_property(srna, "constraint_type", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "constraint_type");
	RNA_def_property_enum_items(prop, rna_enum_rigidbody_constraint_type_items);
	RNA_def_property_enum_funcs(prop, NULL, "rna_FractureModifier_constraint_type_set", NULL);
	RNA_def_property_ui_text(prop, "Constraint Type", "Type of Rigid Body Constraint between shards and inside clusters");
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

	prop = RNA_def_property(srna, "keep_cutter_shards", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_keep_cutter_shards);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_KEEP_BOTH);
	RNA_def_property_ui_text(prop, "Cutter Mode", "Determines which shards to keep from cutting process");
	//RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "boolean_solver", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_sdna(prop, NULL, "boolean_solver");
	RNA_def_property_enum_items(prop, prop_solver_items);
	RNA_def_property_ui_text(prop, "Boolean Solver", "Whether to use carve or bmesh for the boolean operations");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "boolean_double_threshold", PROP_FLOAT, PROP_DISTANCE);
	RNA_def_property_float_sdna(prop, NULL, "boolean_double_threshold");
	RNA_def_property_range(prop, 0, 1.0f);
	RNA_def_property_ui_range(prop, 0, 1, 0.0001, 7);
	RNA_def_property_ui_text(prop, "Overlap Threshold",  "Threshold for checking overlapping geometry");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "dynamic_percentage", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "dynamic_percentage");
	RNA_def_property_range(prop, 0, 100);
	//RNA_def_property_int_funcs(prop, NULL, "rna_FractureModifier_breaking_percentage_set", NULL);
	RNA_def_property_ui_text(prop, "Constraint Percentage", "Percentage of broken constraints per island which allows dynamic fracturing of this island");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "dynamic_new_constraints", PROP_ENUM, PROP_NONE);
	RNA_def_property_enum_items(prop, prop_dynamic_constraints);
	RNA_def_property_enum_default(prop, MOD_FRACTURE_NO_DYNAMIC_CONSTRAINTS);
	RNA_def_property_ui_text(prop, "New Constraints", "Which constraints are created while dynamically fracturing");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "dynamic_min_size", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "dynamic_min_size");
	RNA_def_property_range(prop, 0.001f, 10000.0f);
	RNA_def_property_float_default(prop, 1.0f);
	RNA_def_property_ui_text(prop, "Minimum Size",  "Minimum shard size in blenderunits");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "use_constraint_collision", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_funcs(prop, NULL, "rna_FractureModifier_use_constraint_collision_set");
	RNA_def_property_ui_text(prop, "Constrained Collision", "Let constrained shards collide with each other");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "inner_crease", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "inner_crease");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_ui_text(prop, "Inner Crease",  "Crease at edges of inner faces");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "is_dynamic_external", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "is_dynamic_external", false);
	RNA_def_property_ui_text(prop, "Dynamic External", "Indicator whether the data for dynamic fracture was loaded externally");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	//RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "material_offset_intersect", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "mat_ofs_intersect");
	RNA_def_property_range(prop, 0, SHRT_MAX);
	RNA_def_property_ui_text(prop, "Intersect Material Offset", "Offset material index of intersected shards");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "material_offset_difference", PROP_INT, PROP_NONE);
	RNA_def_property_int_sdna(prop, NULL, "mat_ofs_difference");
	RNA_def_property_range(prop, 0, SHRT_MAX);
	RNA_def_property_ui_text(prop, "Difference Material Offset", "Offset material index of difference shards");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "orthogonality_factor", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "orthogonality_factor");
	RNA_def_property_range(prop, 0.0f, 1.0f);
	RNA_def_property_float_default(prop, 0.0f);
	RNA_def_property_ui_text(prop, "Orthogonality Factor",  "1 means only orthogonal cuts, move down to 0 to get more diagonal-ish cuts");
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "keep_distort", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "keep_distort", false);
	RNA_def_property_ui_text(prop, "Keep Distortion", "Whether or not to make the distortion on torn merged shards persistent.");
	RNA_def_property_update(prop, 0, "rna_Modifier_update_and_keep");

	prop = RNA_def_property(srna, "do_merge", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "do_merge", false);
	RNA_def_property_ui_text(prop, "Perform Merge", "Whether or not to actually weld the prepared automerge geometry.");
	RNA_def_property_update(prop, 0, "rna_Modifier_update_and_keep");

	prop = RNA_def_property(srna, "deform_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "deform_angle");
	RNA_def_property_range(prop, 0, DEG2RADF(360.0));
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_deform_angle_set", NULL);
	RNA_def_property_ui_text(prop, "Deforming Angle", "Angle in degrees above which constraint should keep its deform");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "deform_distance", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "deform_distance");
	RNA_def_property_range(prop, 0, FLT_MAX);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_deform_distance_set", NULL);
	RNA_def_property_ui_text(prop, "Deforming Distance", "Distance above which constraint should keep its deform");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_deform_angle", PROP_FLOAT, PROP_ANGLE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_deform_angle");
	RNA_def_property_range(prop, 0, DEG2RADF(360.0));
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_cluster_deform_angle_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Deforming Angle", "Angle in degrees above which constraint should keep its deform");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "cluster_deform_distance", PROP_FLOAT, PROP_NONE);
	RNA_def_property_float_sdna(prop, NULL, "cluster_deform_distance");
	RNA_def_property_range(prop, 0, FLT_MAX);
	RNA_def_property_float_funcs(prop, NULL, "rna_FractureModifier_cluster_deform_distance_set", NULL);
	RNA_def_property_ui_text(prop, "Cluster Deforming Distance", "Distance above which constraint should keep its deform");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "deform_angle_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "deform_angle_weighted", false);
	RNA_def_property_ui_text(prop, "Weighted Deforming Angle", "Modify deform angle by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	prop = RNA_def_property(srna, "deform_distance_weighted", PROP_BOOLEAN, PROP_NONE);
	RNA_def_property_boolean_sdna(prop, NULL, "deform_distance_weighted", false);
	RNA_def_property_ui_text(prop, "Weighted Deforming Distance", "Modify deform distance by threshold weights");
	RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
	RNA_def_property_update(prop, 0, "rna_Modifier_update");

	RNA_api_fracture(brna, srna);
}
