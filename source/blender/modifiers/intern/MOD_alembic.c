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
 * Contributor(s): Cedric Paille
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

/** \file blender/modifiers/intern/MOD_alembic.c
 *  \ingroup modifiers
 */

#include "DNA_object_types.h"
#include "DNA_mesh_types.h"
#include "DNA_scene_types.h"
#include "DNA_meshdata_types.h"
#include "BLI_utildefines.h"
#include "BKE_DerivedMesh.h"
#include "BKE_modifier.h"
#include "BKE_mesh.h"
#include "BKE_scene.h"
#include "BKE_global.h"
#include "BKE_main.h"

#include "BLI_string.h"
#include "BLI_path_util.h"
#include "MEM_guardedalloc.h"
#include <string.h>

#ifdef WITH_ALEMBIC
#include "ABC_alembic.h"
#endif

static void initData(ModifierData *md)
{
	AlembicModifierData *amd = (AlembicModifierData *)md;
	amd->cache_filename[0] 	= 0;
	amd->sub_object[0] 		= 0;
	amd->flag 				= 0;
	amd->frame_start 		= 0;
}

static void freeData(ModifierData *md)
{
	AlembicModifierData *amd = (AlembicModifierData *)md;
#ifdef WITH_ALEMBIC
	abcDestroyMeshData(md);
#endif
	amd->cache_filename[0] 	= 0;
	amd->sub_object[0] 		= 0;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	modifier_copyData_generic(md, target);
}

static void deformVerts(ModifierData *md, Object *ob,
                        DerivedMesh *derivedData,
                        float (*vertexCos)[3],
                        int numVerts,
                        ModifierApplyFlag flag)
{
#ifdef WITH_ALEMBIC
	AlembicModifierData *amd = (AlembicModifierData *)md;
	char 				abs_path[1024];
	float 				ctime = BKE_scene_frame_get(md->scene) - amd->frame_start;
    float				time  = ctime / (float)md->scene->r.frs_sec;

	BLI_strncpy(abs_path, amd->cache_filename, sizeof(abs_path));
	BLI_path_abs(abs_path, ID_BLEND_PATH(G.main, (ID *)ob));

    abcGetVertexCache(abs_path, time, (void*)md, vertexCos, numVerts, amd->sub_object, 0);

#else
	UNUSED_VARS(md, ob, vertexCos, numVerts);
#endif

	UNUSED_VARS(derivedData, flag);
}

static bool dependsOnTime(ModifierData *UNUSED(md))
{
	return true;
}

static bool modifierDisabled(ModifierData *md, int useRenderParams)
{
#ifdef WITH_ALEMBIC
	AlembicModifierData *amd = (AlembicModifierData *)md;
	bool ok;
	if (amd->cache_filename[0] == '\0')
		return true;

	if (amd->sub_object[0] == '\0')
		return true;

	ok = checkSubobjectValid(amd->cache_filename, amd->sub_object);
	return !ok;
#else
	UNUSED_VARS(md);
#endif

	UNUSED_VARS(useRenderParams);
	return true;
}

ModifierTypeInfo modifierType_Alembic = {
	/* name */              "Alembic",
	/* structName */        "AlembicModifierData",
	/* structSize */        sizeof(AlembicModifierData),
	/* type */              eModifierTypeType_OnlyDeform,
	                        eModifierTypeFlag_AcceptsCVs,

	/* copyData */          copyData,
	/* deformVerts */       deformVerts,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     NULL,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          freeData,
	/* isDisabled */        modifierDisabled,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     dependsOnTime,
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};

