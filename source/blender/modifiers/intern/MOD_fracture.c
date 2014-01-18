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
* ***** END GPL LICENSE BLOCK *****
*
*/

/** \file blender/modifiers/intern/MOD_fracture.c
*  \ingroup modifiers
*/

#include "DNA_fracture_types.h"
#include "BKE_cdderivedmesh.h"
#include "MOD_util.h"

static void initData(ModifierData *md)
{
        FractureModifierData *fmd = (FractureModifierData*) md;
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
		return derivedData;		//need to pass rendermesh out... created outside !!
		//also show single islands via selected index... hmm but how to apply to it ?
		//apply to selected shards; this must be in kernel as well (via index) but there prepare
		//an array of indexes; for multiselection
		//outliner ? hmm
		//all not so good...
}

ModifierTypeInfo modifierType_Fracture = {
        /* name */ "Fracture",
        /* structName */ "FractureModifierData",
        /* structSize */ sizeof(FractureModifierData),
        /* type */ eModifierTypeType_Constructive,
        /* flags */ eModifierTypeFlag_AcceptsMesh | eModifierTypeFlag_Single,
        /* copyData */ NULL,//copyData,
        /* deformVerts */ NULL, // deformVerts,
        /* deformMatrices */ NULL,
        /* deformVertsEM */ NULL,
        /* deformMatricesEM */ NULL,
        /* applyModifier */ applyModifier,
        /* applyModifierEM */ NULL,
        /* initData */ initData,
        /* requiredDataMask */ NULL,
        /* freeData */ NULL,
        /* isDisabled */ NULL,
        /* updateDepgraph */ NULL,
        /* dependsOnTime */ NULL,//dependsOnTime,
        /* dependsOnNormals */ NULL,
        /* foreachObjectLink */ NULL,
        /* foreachIDLink */ NULL,
};
