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
#include "BKE_fracture.h"
#include "BKE_cdderivedmesh.h"
#include "BLI_rand.h"
#include "MOD_util.h"
#include "MEM_guardedalloc.h"

static void do_fracture(FractureModifierData *fracmd, ShardID id);

static void initData(ModifierData *md)
{
		FractureModifierData *fmd = (FractureModifierData*) md;
		fmd->frac_algorithm = MOD_FRACTURE_VORONOI;
		fmd->shard_count = 10;
		fmd->shard_id = 0;
}

static void freeData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData*) md;
	
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
	
	BKE_fracmesh_free(fmd->frac_mesh);
	MEM_freeN(fmd->frac_mesh);
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	//also show single islands via selected index... hmm but how to apply to it ?
	//apply to selected shards; this must be in kernel as well (via index) but there prepare
	//an array of indexes; for multiselection
	//outliner ? hmm
	//all not so good...

	FractureModifierData *fmd = (FractureModifierData*) md;
	DerivedMesh *final_dm;

	if (fmd->frac_mesh && fmd->shard_id == 0)
	{	//recreate container, if shard 0 is to be fractured
		BKE_fracmesh_free(fmd->frac_mesh);
		MEM_freeN(fmd->frac_mesh);

		if (fmd->dm) {
			fmd->dm->needsFree = 1;
			fmd->dm->release(fmd->dm);
			fmd->dm = NULL;
		}

		fmd->frac_mesh = BKE_create_fracture_container(derivedData);
	}
	
	if (fmd->frac_mesh == NULL) {
		fmd->frac_mesh = BKE_create_fracture_container(derivedData);
	}
	
	do_fracture(fmd, fmd->shard_id);

	if (fmd->dm)
		final_dm = CDDM_copy(fmd->dm);
	else
		final_dm = derivedData;
	
	return final_dm;
}

static void do_fracture(FractureModifierData *fracmd, ShardID id)
{
	float min[3], max[3];
	/* dummy point cloud, random */
	FracPointCloud points;
	int i;

	INIT_MINMAX(min, max);
	BKE_get_shard_minmax(fracmd->frac_mesh, id, min, max);
	points.totpoints = fracmd->shard_count;

	points.points = MEM_mallocN(sizeof(FracPoint) * points.totpoints, "random points");
	BLI_srandom(12345);
	for (i = 0; i < points.totpoints; ++i) {
		float *co = points.points[i].co;
		co[0] = min[0] + (max[0] - min[0]) * BLI_frand();
		co[1] = min[1] + (max[1] - min[1]) * BLI_frand();
		co[2] = min[2] + (max[2] - min[2]) * BLI_frand();
	}

	BKE_fracture_shard_by_points(fracmd->frac_mesh, id, &points, fracmd->frac_algorithm);
	MEM_freeN(points.points);
	BKE_fracture_create_dm(fracmd, false);
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
        /* freeData */ freeData,
        /* isDisabled */ NULL,
        /* updateDepgraph */ NULL,
        /* dependsOnTime */ NULL,//dependsOnTime,
        /* dependsOnNormals */ NULL,
        /* foreachObjectLink */ NULL,
        /* foreachIDLink */ NULL,
};
