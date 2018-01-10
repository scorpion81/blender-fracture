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
 * The Original Code is Copyright (C) 2011 by Nicholas Bishop.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/modifiers/intern/MOD_remesh.c
 *  \ingroup modifiers
 */

#include "MEM_guardedalloc.h"

#include "BLI_math_base.h"
#include "BLI_math_vector.h"
#include "BLI_utildefines.h"
#include "BLI_listbase.h"
#include "BLI_memarena.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_DerivedMesh.h"
#include "BKE_mball_tessellate.h"

#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_particle_types.h"

#include "MOD_modifiertypes.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#ifdef WITH_MOD_REMESH
#  include "dualcon.h"
#endif

static void initData(ModifierData *md)
{
	RemeshModifierData *rmd = (RemeshModifierData *) md;

	rmd->scale = 0.9;
	rmd->depth = 4;
	rmd->hermite_num = 1;
	rmd->flag = MOD_REMESH_FLOOD_FILL;
	rmd->mode = MOD_REMESH_SHARP_FEATURES;
	rmd->threshold = 1;

	rmd->basesize[0] = rmd->basesize[1] = rmd->basesize[2] = 1.0f;
	rmd->thresh = 0.6f;
	rmd->wiresize = 0.4f;
	rmd->rendersize = 0.2f;

	rmd->input = 0;
	rmd->pflag = 1;
	rmd->psys = 1;
}

static void copyData(ModifierData *md, ModifierData *target)
{
#if 0
	RemeshModifierData *rmd = (RemeshModifierData *) md;
	RemeshModifierData *trmd = (RemeshModifierData *) target;
#endif
	modifier_copyData_generic(md, target);
}

#ifdef WITH_MOD_REMESH

static void init_dualcon_mesh(DualConInput *mesh, DerivedMesh *dm)
{
	memset(mesh, 0, sizeof(DualConInput));

	mesh->co = (void *)dm->getVertArray(dm);
	mesh->co_stride = sizeof(MVert);
	mesh->totco = dm->getNumVerts(dm);

	mesh->mloop = (void *)dm->getLoopArray(dm);
	mesh->loop_stride = sizeof(MLoop);
	mesh->looptri = (void *)dm->getLoopTriArray(dm);
	mesh->tri_stride = sizeof(MLoopTri);
	mesh->tottri = dm->getNumLoopTri(dm);

	INIT_MINMAX(mesh->min, mesh->max);
	dm->getMinMax(dm, mesh->min, mesh->max);
}

/* simple structure to hold the output: a CDDM and two counters to
 * keep track of the current elements */
typedef struct {
	DerivedMesh *dm;
	int curvert, curface;
} DualConOutput;

/* allocate and initialize a DualConOutput */
static void *dualcon_alloc_output(int totvert, int totquad)
{
	DualConOutput *output;

	if (!(output = MEM_callocN(sizeof(DualConOutput),
	                           "DualConOutput")))
	{
		return NULL;
	}
	
	output->dm = CDDM_new(totvert, 0, 0, 4 * totquad, totquad);
	return output;
}

static void dualcon_add_vert(void *output_v, const float co[3])
{
	DualConOutput *output = output_v;
	DerivedMesh *dm = output->dm;
	
	assert(output->curvert < dm->getNumVerts(dm));
	
	copy_v3_v3(CDDM_get_verts(dm)[output->curvert].co, co);
	output->curvert++;
}

static void dualcon_add_quad(void *output_v, const int vert_indices[4])
{
	DualConOutput *output = output_v;
	DerivedMesh *dm = output->dm;
	MLoop *mloop;
	MPoly *cur_poly;
	int i;
	
	assert(output->curface < dm->getNumPolys(dm));

	mloop = CDDM_get_loops(dm);
	cur_poly = CDDM_get_poly(dm, output->curface);
	
	cur_poly->loopstart = output->curface * 4;
	cur_poly->totloop = 4;
	for (i = 0; i < 4; i++)
		mloop[output->curface * 4 + i].v = vert_indices[i];
	
	output->curface++;
}

static int get_particle_positions_sizes_vel(RemeshModifierData *rmd, ParticleSystem *psys, Object *ob,
                                        float (**pos)[3], float **size, float (**vel)[3], MemArena *pardata)
{
	//take alive, for now
	ParticleData *pa;
	int i = 0, j = 0;
	float imat[4][4];
	invert_m4_m4(imat, ob->obmat);

	(*pos) = BLI_memarena_alloc(pardata, sizeof(float) * 3 * psys->totpart);
	(*size) = BLI_memarena_alloc(pardata, sizeof(float) * psys->totpart);
	(*vel) = BLI_memarena_calloc(pardata, sizeof(float) * 3 * psys->totpart);

	for (i = 0; i < psys->totpart; i++)
	{
		float co[3];
		pa = psys->particles + i;
		if (pa->alive == PARS_UNBORN && (rmd->pflag & eRemeshFlag_Unborn) == 0) continue;
		if (pa->alive == PARS_ALIVE && (rmd->pflag & eRemeshFlag_Alive) == 0) continue;
		if (pa->alive == PARS_DEAD && (rmd->pflag & eRemeshFlag_Dead) == 0) continue;

		mul_v3_m4v3(co, imat, pa->state.co);
		copy_v3_v3((*pos)[j], co);
		(*size)[j] = pa->size;
		copy_v3_v3((*vel)[j], pa->state.vel);
		j++;
	}

	return j;
}

static ParticleSystem *get_psys(RemeshModifierData *rmd, Object *ob, bool render)
{
	ParticleSystem *psys;
	ModifierData *ob_md = (ModifierData*)rmd;

	psys = BLI_findlink(&ob->particlesystem, rmd->psys - 1);
	if (psys == NULL)
		return NULL;

	/* If the psys modifier is disabled we cannot use its data.
	 * First look up the psys modifier from the object, then check if it is enabled.
	 */

	for (ob_md = ob->modifiers.first; ob_md; ob_md = ob_md->next) {
		if (ob_md->type == eModifierType_ParticleSystem) {
			ParticleSystemModifierData *psmd = (ParticleSystemModifierData *)ob_md;
			if (psmd->psys == psys) {
				int required_mode;

				if (render) required_mode = eModifierMode_Render;
				else required_mode = eModifierMode_Realtime;

				if (modifier_isEnabled(ob_md->scene, ob_md, required_mode))
				{
					return psys;
				}

				return NULL;
			}
		}
	}

	return NULL;
}

static DerivedMesh *repolygonize(RemeshModifierData *rmd, Object* ob, DerivedMesh* derived, ParticleSystem *psys, bool render)
{
	DerivedMesh *dm = NULL, *result = NULL;
	MVert *mv = NULL, *mv2 = NULL;
	float (*pos)[3] = NULL, (*vel)[3] = NULL;
	float *size = NULL, *psize = NULL, *velX = NULL, *velY = NULL, *velZ = NULL;
	int i = 0, n = 0;
	bool override_size = rmd->pflag & eRemeshFlag_Size;
	bool verts_only = rmd->pflag & eRemeshFlag_Verts;

	if (((rmd->input & MOD_REMESH_VERTICES)==0) && (rmd->input & MOD_REMESH_PARTICLES))
	{
		//particles only
		MemArena *pardata = NULL;
		if (psys == NULL)
			return derived;

		pardata = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "pardata");

		n = get_particle_positions_sizes_vel(rmd, psys, ob, &pos, &size, &vel, pardata);
		dm = CDDM_new(n, 0, 0, 0, 0);
		mv = dm->getVertArray(dm);
		psize = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n, "psize");
		velX = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n, "velX");
		velY = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n, "velY");
		velZ = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n, "velZ");

#pragma omp parallel for
		for (i = 0; i < n; i++)
		{
			copy_v3_v3(mv[i].co, pos[i]);
			psize[i] = size[i];
			velX[i] = vel[i][0];
			velY[i] = vel[i][1];
			velZ[i] = vel[i][2];
		}

		if (verts_only)
		{
			BLI_memarena_free(pardata);
			return dm;
		}
		else {
			BLI_memarena_free(pardata);
//#pragma omp parallel num_threads(4)
			result = BKE_repolygonize_dm(dm, rmd->thresh, rmd->basesize, rmd->wiresize, rmd->rendersize, render, override_size);
			dm->release(dm);
			return result;
		}
	}
	else if ((rmd->input & MOD_REMESH_VERTICES) && ((rmd->input & MOD_REMESH_PARTICLES) == 0))
	{
		//verts only
		DerivedMesh *result = NULL;
//#pragma omp parallel
		result = BKE_repolygonize_dm(derived, rmd->thresh, rmd->basesize, rmd->wiresize, rmd->rendersize, render, override_size);
		return result;
	}
	else if ((rmd->input & MOD_REMESH_VERTICES) && (rmd->input & MOD_REMESH_PARTICLES))
	{
		//both, for simplicity only use vert data here
		float* ovX, *ovY, *ovZ;
		n = 0;
		MemArena *pardata = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "pardata");

		if (psys)
			n = get_particle_positions_sizes_vel(rmd, psys, ob, &pos, &size, &vel, pardata);

		dm = CDDM_new(n + derived->numVertData, 0, 0, 0, 0);
		psize = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n + derived->numVertData, "psize");
		velX = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n + derived->numVertData, "velX");
		velY = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n + derived->numVertData, "velY");
		velZ = CustomData_add_layer_named(&dm->vertData, CD_PROP_FLT, CD_CALLOC, NULL, n + derived->numVertData, "velZ");

		mv = dm->getVertArray(dm);
		mv2 = derived->getVertArray(derived);

		ovX = CustomData_get_layer_named(&derived->vertData, CD_PROP_FLT, "velX");
		ovY = CustomData_get_layer_named(&derived->vertData, CD_PROP_FLT, "velY");
		ovZ = CustomData_get_layer_named(&derived->vertData, CD_PROP_FLT, "velZ");

#pragma omp parallel for
		for (i = 0; i < n; i++)
		{
			copy_v3_v3(mv[i].co, pos[i]);
			psize[i] = size[i];
			velX[i] = vel[i][0];
			velY[i] = vel[i][1];
			velZ[i] = vel[i][2];
		}

#pragma omp parallel for
		for (i = n; i < n + derived->numVertData; i++)
		{
			copy_v3_v3(mv[i].co, mv2[i-n].co);
			psize[i] = -1.0f; //use mball sizep
			velX[i] = ovX ? ovX[i-n] : 0.0f;
			velY[i] = ovY ? ovY[i-n] : 0.0f;
			velZ[i] = ovZ ? ovZ[i-n] : 0.0f;
		}

		if (verts_only)
		{
			BLI_memarena_free(pardata);
			return dm;
		}
		else {
			BLI_memarena_free(pardata);
//#pragma omp parallel
			result = BKE_repolygonize_dm(dm, rmd->thresh, rmd->basesize, rmd->wiresize, rmd->rendersize, render, override_size);
			dm->release(dm);
			return result;
		}
	}

	return NULL;
}

static DerivedMesh *applyModifier(ModifierData *md,
                                  Object *ob,
                                  DerivedMesh *dm,
                                  ModifierApplyFlag flag)
{
	RemeshModifierData *rmd;
	DualConOutput *output;
	DualConInput input;
	DerivedMesh *result;
	DualConFlags flags = 0;
	DualConMode mode = 0;

	rmd = (RemeshModifierData *)md;
	
	if (rmd->mode != MOD_REMESH_MBALL)
	{
		init_dualcon_mesh(&input, dm);

		if (rmd->flag & MOD_REMESH_FLOOD_FILL)
			flags |= DUALCON_FLOOD_FILL;

		switch (rmd->mode) {
			case MOD_REMESH_CENTROID:
				mode = DUALCON_CENTROID;
				break;
			case MOD_REMESH_MASS_POINT:
				mode = DUALCON_MASS_POINT;
				break;
			case MOD_REMESH_SHARP_FEATURES:
				mode = DUALCON_SHARP_FEATURES;
				break;
		}

		output = dualcon(&input,
						 dualcon_alloc_output,
						 dualcon_add_vert,
						 dualcon_add_quad,
						 flags,
						 mode,
						 rmd->threshold,
						 rmd->hermite_num,
						 rmd->scale,
						 rmd->depth);
		result = output->dm;
		MEM_freeN(output);

		CDDM_calc_edges(result);
		result->dirty |= DM_DIRTY_NORMALS;
	}
	else {
		ParticleSystem* psys = NULL;
		bool render = flag & MOD_APPLY_RENDER;
		psys = get_psys(rmd, ob, render);
		result = repolygonize(rmd, ob, dm, psys, render);
	}

	if (result && (rmd->flag & MOD_REMESH_SMOOTH_SHADING)) {
		MPoly *mpoly = CDDM_get_polys(result);
		int i, totpoly = result->getNumPolys(result);

		/* Apply smooth shading to output faces */
		for (i = 0; i < totpoly; i++) {
			mpoly[i].flag |= ME_SMOOTH;
		}
	}

	return result;
}

#else /* !WITH_MOD_REMESH */

static DerivedMesh *applyModifier(ModifierData *UNUSED(md), Object *UNUSED(ob),
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	return derivedData;
}

#endif /* !WITH_MOD_REMESH */

ModifierTypeInfo modifierType_Remesh = {
	/* name */              "Remesh",
	/* structName */        "RemeshModifierData",
	/* structSize */        sizeof(RemeshModifierData),
	/* type */              eModifierTypeType_Nonconstructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_AcceptsCVs |
                            eModifierTypeFlag_SupportsMapping |
	                        eModifierTypeFlag_SupportsEditmode,
	/* copyData */          copyData,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          NULL,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
};
