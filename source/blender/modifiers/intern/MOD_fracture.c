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
 * along with this program; if not, write to the Free Software  Foundation,
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

/** \file blender/modifiers/intern/MOD_fracture.c
 *  \ingroup modifiers
 */

#include <stdlib.h>
#include <limits.h>

#include "BLI_listbase.h"
#include "BLI_sys_types.h"
#include "BLI_string.h"
#include "BLI_math.h"
#include "MEM_guardedalloc.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_DerivedMesh.h"
#include "BKE_fracture.h"

#include "DNA_group_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_object_types.h"

#include "MOD_util.h"
#include "depsgraph_private.h" /* for depgraph updates */

static void initData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	const char *init = "Default";

	//if we have already vgroups, init all settings to default !!!
	fmd->fracture = MEM_callocN(sizeof(FractureSetting), "fracture_setting");
	BLI_strncpy(fmd->fracture->name, init, sizeof(fmd->fracture->name));
	BLI_addtail(&fmd->fracture_settings, fmd->fracture);

	fmd->constraint = MEM_callocN(sizeof(ConstraintSetting), "constraint_setting");
	BLI_strncpy(fmd->constraint->name, init, sizeof(fmd->constraint->name));
	BLI_addtail(&fmd->constraint_settings, fmd->constraint);

	fmd->fracture->extra_group = NULL;
	fmd->fracture->frac_algorithm = MOD_FRACTURE_BOOLEAN;
	fmd->fracture->point_source = MOD_FRACTURE_UNIFORM;
	fmd->fracture->shard_count = 10;
	fmd->fracture->percentage = 100;;

	fmd->fracture->visible_mesh = NULL;
	fmd->fracture->visible_mesh_cached = NULL;
	fmd->fracture->flag &= ~FM_FLAG_REFRESH;
	zero_m4(fmd->origmat);

	fmd->constraint->cluster_count = 0;
	fmd->constraint->breaking_threshold = 10.0f;
	fmd->constraint->flag &= ~FMC_FLAG_USE_CONSTRAINTS;
	fmd->constraint->contact_dist = 1.0f;
	fmd->constraint->flag &= ~FMC_FLAG_USE_MASS_DEPENDENT_THRESHOLDS;
	fmd->fracture->flag &= FM_FLAG_USE_FRACMESH; // use fracmesh... is this global or per setting ? TODO...
	fmd->constraint->constraint_limit = 50;
	fmd->constraint->breaking_distance = 0;
	fmd->constraint->breaking_angle = 0;
	fmd->constraint->breaking_percentage = 0;     /* disable by default*/
	fmd->fracture->max_vol = 0;
	fmd->fracture->flag &= ~FM_FLAG_REFRESH_CONSTRAINTS;

	fmd->constraint->cluster_breaking_threshold = 1000.0f;
	fmd->constraint->solver_iterations_override = 0;
	fmd->constraint->cluster_solver_iterations_override = 0;
	fmd->fracture->flag &= ~FM_FLAG_SHARDS_TO_ISLANDS;
	fmd->flag &= ~FMG_FLAG_EXECUTE_THREADED;
	fmd->fracture->nor_tree = NULL;
	fmd->fracture->flag &= ~FM_FLAG_FIX_NORMALS;
	fmd->fracture->flag &= ~FM_FLAG_AUTO_EXECUTE; // is this global or per setting, TODO....
	fmd->fracture->face_pairs = NULL;
	fmd->fracture->autohide_dist = 0.0f;

	fmd->constraint->flag &= ~FMC_FLAG_BREAKING_PERCENTAGE_WEIGHTED;
	fmd->constraint->flag &= ~FMC_FLAG_BREAKING_ANGLE_WEIGHTED;
	fmd->constraint->flag &= ~FMC_FLAG_BREAKING_DISTANCE_WEIGHTED;

	/* XXX needed because of messy particle cache, shows incorrect positions when start/end on frame 1
	 * default use case is with this flag being enabled, disable at own risk */
	fmd->fracture->flag |= FM_FLAG_USE_PARTICLE_BIRTH_COORDS;
	fmd->fracture->splinter_length = 1.0f;
	fmd->fracture->nor_range = 1.0f;

	fmd->constraint->cluster_breaking_angle = 0;
	fmd->constraint->cluster_breaking_distance = 0;
	fmd->constraint->cluster_breaking_percentage = 0;

	/* used for advanced fracture settings now, XXX needs rename perhaps*/
	fmd->flag &= ~FMG_FLAG_USE_EXPERIMENTAL;
	fmd->constraint->flag |= FMC_FLAG_USE_BREAKING;
	fmd->fracture->flag &= ~FM_FLAG_USE_SMOOTH;

	fmd->fracture->fractal_cuts = 1;
	fmd->fracture->fractal_amount = 1.0f;
	fmd->fracture->physics_mesh_scale = 1.0f; //almost useless....
	fmd->fracture->fractal_iterations = 5;

	fmd->constraint->cluster_group = NULL;
	fmd->fracture->cutter_group = NULL;

	fmd->fracture->grease_decimate = 100.0f;
	fmd->fracture->grease_offset = 0.5f;
	fmd->fracture->flag |= FM_FLAG_USE_GREASEPENCIL_EDGES;

	fmd->fracture->cutter_axis = MOD_FRACTURE_CUTTER_Z;
	fmd->constraint->cluster_constraint_type = RBC_TYPE_FIXED; //this is maybe not necessary any more....
	fmd->vert_index_map = NULL;
	fmd->constraint->constraint_target = MOD_FRACTURE_CENTROID;
	fmd->fracture->vertex_island_map = NULL;

	fmd->fracture->meshIslands.first = NULL;
	fmd->fracture->meshIslands.last = NULL;
	fmd->constraint->meshConstraints.first = NULL;
	fmd->constraint->meshConstraints.last = NULL;

	fmd->fracture_mode = MOD_FRACTURE_PREFRACTURED;
	fmd->last_frame = FLT_MIN;
	fmd->fracture->dynamic_force = 10.0f;
	fmd->fracture->flag &= ~FM_FLAG_UPDATE_DYNAMIC;
	fmd->fracture->flag &=~ FM_FLAG_LIMIT_IMPACT;
	fmd->fracture->flag &= ~FM_FLAG_RESET_SHARDS;
}

static void freeData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	BKE_free_fracture_modifier(fmd, true);
}


static void copyData(ModifierData *md, ModifierData *target)
{
	FractureModifierData *rmd  = (FractureModifierData *)md;
	FractureModifierData *trmd = (FractureModifierData *)target;

	/*todo -> copy fracture stuff as well, and dont forget readfile / writefile...*/
	zero_m4(trmd->origmat);

	/* vgroups  XXX TODO non ascii strings ?*/
	strncpy(trmd->fracture->thresh_defgrp_name, rmd->fracture->thresh_defgrp_name, strlen(rmd->fracture->thresh_defgrp_name));
	strncpy(trmd->fracture->ground_defgrp_name, rmd->fracture->ground_defgrp_name, strlen(rmd->fracture->ground_defgrp_name));
	strncpy(trmd->fracture->inner_defgrp_name, rmd->fracture->inner_defgrp_name, strlen(rmd->fracture->inner_defgrp_name));

	trmd->fracture->visible_mesh = NULL;
	trmd->fracture->visible_mesh_cached = NULL;
	trmd->fracture->meshIslands.first = NULL;
	trmd->fracture->meshIslands.last = NULL;
	trmd->constraint->meshConstraints.first = NULL;
	trmd->constraint->meshConstraints.last = NULL;
	trmd->fracture->face_pairs = NULL;
	trmd->vert_index_map = NULL;
	trmd->fracture->vertex_island_map = NULL;

	trmd->constraint->breaking_threshold = rmd->constraint->breaking_threshold;
	trmd->constraint->flag = rmd->constraint->flag;
	trmd->constraint->contact_dist = rmd->constraint->contact_dist;
	trmd->flag = rmd->flag;
	trmd->fracture->flag = rmd->fracture->flag;

	trmd->fracture->flag &= ~FM_FLAG_REFRESH;
	trmd->constraint->constraint_limit = rmd->constraint->constraint_limit;
	trmd->constraint->breaking_angle = rmd->constraint->breaking_angle;
	trmd->constraint->breaking_distance = rmd->constraint->breaking_distance;
	trmd->constraint->breaking_percentage = rmd->constraint->breaking_percentage;
	trmd->fracture->flag &= ~FM_FLAG_REFRESH_CONSTRAINTS;

	trmd->constraint->cluster_count = rmd->constraint->cluster_count;
	trmd->constraint->cluster_breaking_threshold = rmd->constraint->cluster_breaking_threshold;
	trmd->constraint->solver_iterations_override = rmd->constraint->solver_iterations_override;

	trmd->fracture->shard_count = rmd->fracture->shard_count;
	trmd->fracture->frac_algorithm = rmd->fracture->frac_algorithm;

	trmd->fracture->autohide_dist = rmd->fracture->autohide_dist;
	trmd->fracture->point_seed = rmd->fracture->point_seed;
	trmd->fracture->point_source = rmd->fracture->point_source;

	/*id refs ?*/
	trmd->fracture->inner_material = rmd->fracture->inner_material;
	trmd->fracture->extra_group = rmd->fracture->extra_group;

	/* sub object group  XXX Do we keep this ?*/
	trmd->dm_group = rmd->dm_group;

	trmd->constraint->cluster_group = rmd->constraint->cluster_group;
	trmd->fracture->cutter_group = rmd->fracture->cutter_group;

	trmd->fracture->splinter_length = rmd->fracture->splinter_length;
	trmd->constraint->cluster_solver_iterations_override = rmd->constraint->cluster_solver_iterations_override;

	trmd->constraint->cluster_breaking_angle = rmd->constraint->cluster_breaking_angle;
	trmd->constraint->cluster_breaking_distance = rmd->constraint->cluster_breaking_distance;
	trmd->constraint->cluster_breaking_percentage = rmd->constraint->cluster_breaking_percentage;

	trmd->fracture->fractal_cuts = rmd->fracture->fractal_cuts;
	trmd->fracture->fractal_amount = rmd->fracture->fractal_amount;

	trmd->fracture->grease_decimate = rmd->fracture->grease_decimate;
	trmd->fracture->grease_offset = rmd->fracture->grease_offset;
	trmd->fracture->cutter_axis = rmd->fracture->cutter_axis;

	trmd->constraint->cluster_constraint_type = rmd->constraint->cluster_constraint_type;
	trmd->constraint->constraint_target = rmd->constraint->constraint_target;

	trmd->fracture_mode = rmd->fracture_mode;
	trmd->last_frame = rmd->last_frame;
	trmd->fracture->dynamic_force = rmd->fracture->dynamic_force;

	trmd->fracture->flag &= ~FM_FLAG_UPDATE_DYNAMIC;
	trmd->fracture->flag &= ~FM_FLAG_RESET_SHARDS;
}

static bool dependsOnTime(ModifierData *UNUSED(md))
{
	return true;
}

static bool dependsOnNormals(ModifierData *UNUSED(md))
{
	return true;
}

static void foreachIDLink(ModifierData *md, Object *ob,
                          IDWalkFunc walk, void *userData)
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	FractureSetting *fs = NULL;
	ConstraintSetting *cs = NULL;

	walk(userData, ob, (ID **)&fmd->dm_group);

	for (fs = fmd->fracture_settings.first; fs; fs = fs->next)
	{
		walk(userData, ob, (ID **)&fs->inner_material);
		walk(userData, ob, (ID **)&fs->extra_group);
		walk(userData, ob, (ID **)&fs->cutter_group);
	}

	for (cs = fmd->constraint_settings.first; cs; cs = cs->next)
	{
		walk(userData, ob, (ID **)&cs->cluster_group);
	}
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md))
{
	CustomDataMask dataMask = 0;
	dataMask |= CD_MASK_MDEFORMVERT;
	return dataMask;
}

static void updateDepgraph(ModifierData *md, DagForest *forest,
                           struct Scene *UNUSED(scene),
                           Object *UNUSED(ob),
                           DagNode *obNode)
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	FractureSetting *fs = NULL;

	for (fs = fmd->fracture_settings.first; fs; fs = fs->next)
	{
		if (fs->extra_group) {
			GroupObject *go;
			for (go = fs->extra_group->gobject.first; go; go = go->next) {
				if (go->ob)
				{
					DagNode *curNode = dag_get_node(forest, go->ob);
					dag_add_relation(forest, curNode, obNode,
									 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Fracture Modifier Setting");
				}
			}
		}
	}
}

static void foreachObjectLink(
    ModifierData *md, Object *ob,
    void (*walk)(void *userData, Object *ob, Object **obpoin),
    void *userData)
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	FractureSetting *fs = NULL;

	for (fs = fmd->fracture_settings.first; fs; fs = fs->next)
	{
		if (fs->extra_group) {
			GroupObject *go;
			for (go = fs->extra_group->gobject.first; go; go = go->next) {
				if (go->ob) {
					walk(userData, ob, &go->ob);
				}
			}
		}

		if (fs->cutter_group) {
			GroupObject *go;
			for (go = fs->cutter_group->gobject.first; go; go = go->next) {
				if (go->ob) {
					walk(userData, ob, &go->ob);
				}
			}
		}
	}
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	DerivedMesh *final_dm = derivedData;

	if (ob->rigidbody_object == NULL) {
		//initialize FM here once
		FractureSetting* fs = NULL;
		for (fs = fmd->fracture_settings.first; fs; fs = fs->next)
		{
			fmd->fracture = fs;
			fmd->fracture->flag |= FM_FLAG_REFRESH;
		}

		//need a global flag here too !!!
		BKE_initialize_from_vertex_groups(fmd, ob);
	}

	if (fmd->fracture->start_mesh == NULL)
	{
		fmd->fracture->start_mesh = CDDM_copy(derivedData);
	}

	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
		final_dm = BKE_prefracture_mesh(fmd, ob, derivedData);
	}
	else if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		final_dm = BKE_dynamic_fracture_mesh(fmd, ob, derivedData);
	}

	return final_dm;
}

static DerivedMesh *applyModifierEM(ModifierData *md, Object *ob,
                                    struct BMEditMesh *UNUSED(editData),
                                    DerivedMesh *derivedData,
                                    ModifierApplyFlag flag)
{
	return applyModifier(md, ob, derivedData, flag);
}

ModifierTypeInfo modifierType_Fracture = {
	/* name */ "Fracture",
	/* structName */ "FractureModifierData",
	/* structSize */ sizeof(FractureModifierData),
	/* type */  eModifierTypeType_Constructive,
	/* flags */ eModifierTypeFlag_AcceptsMesh |
	eModifierTypeFlag_AcceptsCVs |
	eModifierTypeFlag_Single |
	eModifierTypeFlag_SupportsEditmode |
	eModifierTypeFlag_SupportsMapping |
	eModifierTypeFlag_UsesPreview,
	/* copyData */ copyData,
	/* deformVerts */ NULL,
	/* deformMatrices */ NULL,
	/* deformVertsEM */ NULL,
	/* deformMatricesEM */ NULL,
	/* applyModifier */ applyModifier,
	/* applyModifierEM */ applyModifierEM,
	/* initData */ initData,
	/* requiredDataMask */ requiredDataMask,
	/* freeData */ freeData,
	/* isDisabled */ NULL,
	/* updateDepgraph */ updateDepgraph,
	/* dependsOnTime */ dependsOnTime,
	/* dependsOnNormals */ dependsOnNormals,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */ foreachIDLink,
};
