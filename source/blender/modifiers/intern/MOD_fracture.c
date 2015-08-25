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
#include "BLI_threads.h"
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

static CustomDataMask requiredDataMask(Object* ob, ModifierData *UNUSED(md))
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	CustomDataMask dataMask = 0;
	if (fc && (fc->flag & FM_FLAG_REFRESH))
	{
		/* indicate modifier evaluation stop, yuck, just because we need an object ref here,
		 * we have to workaround by returning an old unused CustomdataMask, cant use isDisabled here
		 * because we dont get an Object passed into it, and changing the function pointer would mean changing it globally
		 */
		dataMask |= CD_MASK_MSTICKY;
	}

	return dataMask;
}

static ThreadMutex fracture_modifier_lock = BLI_MUTEX_INITIALIZER;

static DerivedMesh *applyModifier(ModifierData *UNUSED(md), Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	RigidBodyOb *rb = ob->rigidbody_object;
	if (rb && rb->fracture_objects)
	{
		FractureState *fs = rb->fracture_objects->current;
		rb->fracture_objects->flag &= ~FM_FLAG_UPDATE_AUTOHIDE;
		if (fs->visual_mesh)
		{
			DerivedMesh *dm = NULL;

			//argh, this happens in a threaded environment too, and may interfere with threaded fracture, so lock here to prevent
			//crashes on freed memory access
			BLI_mutex_lock(&fracture_modifier_lock);

			BKE_fracture_prepare_autohide(ob); /*in case after loading rebuild facepairs,
												* this GHash is a runtime struct ONLY*/
			dm = BKE_fracture_autohide(ob);

			BLI_mutex_unlock(&fracture_modifier_lock);
			return dm;
		}
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
	eModifierTypeFlag_UsesPreview |
	eModifierTypeFlag_StopWhenDisabled, //this is a new flag; if set the modifier stack will stop evaluation right before the 1st modifier which has this flag set
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
