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
#include "DNA_fracture_types.h"

#include "MOD_util.h"
#include "depsgraph_private.h" /* for depgraph updates */

static bool dependsOnTime(ModifierData *UNUSED(md))
{
	return true;
}

static bool dependsOnNormals(ModifierData *UNUSED(md))
{
	return true;
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md))
{
	CustomDataMask dataMask = 0;
	dataMask |= CD_MASK_MDEFORMVERT;
	return dataMask;
}

#if 0
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
#endif

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	if (ob->fracture_objects)
	{
		return BKE_autohide_inner(ob);
	}

	return derivedData;
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
	/* copyData */ NULL,
	/* deformVerts */ NULL,
	/* deformMatrices */ NULL,
	/* deformVertsEM */ NULL,
	/* deformMatricesEM */ NULL,
	/* applyModifier */ applyModifier,
	/* applyModifierEM */ applyModifierEM,
	/* initData */ NULL,
	/* requiredDataMask */ requiredDataMask,
	/* freeData */ NULL,
	/* isDisabled */ NULL,
	/* updateDepgraph */ NULL,
	/* dependsOnTime */ dependsOnTime,
	/* dependsOnNormals */ dependsOnNormals,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */ NULL,
};
