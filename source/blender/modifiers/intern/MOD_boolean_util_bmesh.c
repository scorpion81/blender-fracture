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
 * The Original Code is Copyright (C) Blender Foundation
 * All rights reserved.
 *
 * Contributor(s): Sergey Sharybin.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/modifiers/intern/MOD_boolean_util_bmesh.c
 *  \ingroup modifiers
 */

#include "BLI_alloca.h"
#include "BLI_math_geom.h"
#include "BKE_material.h"
#include "BKE_global.h"  /* only to check G.debug */
#include "MEM_guardedalloc.h"

#include "bmesh.h"
#include "bmesh_tools.h"
#include "tools/bmesh_intersect.h"
#include "BKE_cdderivedmesh.h"

#ifdef DEBUG_TIME
#include "PIL_time.h"
#include "PIL_time_utildefines.h"
#endif

#include "DNA_material_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_modifier_types.h"
#include "BLI_utildefines.h"

#include "MOD_boolean_util.h"

/* has no meaning for faces, do this so we can tell which face is which */
#define BM_FACE_TAG BM_ELEM_DRAW

/**
 * Compare selected/unselected.
 */
static int bm_face_isect_pair(BMFace *f, void *UNUSED(user_data))
{
	return BM_elem_flag_test(f, BM_FACE_TAG) ? 1 : 0;
}

DerivedMesh *NewBooleanDerivedMeshBMesh(DerivedMesh *dm, struct Object *ob,
                                   DerivedMesh *dm_other, struct Object *ob_other, int op_type,
                                   float double_threshold, struct BooleanModifierData *bmd)
{
	BMesh *bm;
	const BMAllocTemplate allocsize = BMALLOC_TEMPLATE_FROM_DM(dm, dm_other);
	const bool is_flip = (is_negative_m4(ob->obmat) != is_negative_m4(ob_other->obmat));
	DerivedMesh *result;

	#ifdef DEBUG_TIME
	TIMEIT_START(boolean_bmesh);
	#endif
	bm = BM_mesh_create(
			 &allocsize,
			 &((struct BMeshCreateParams){.use_toolflags = false,}));

	DM_to_bmesh_ex(dm_other, bm, true);

	if (UNLIKELY(is_flip)) {
		const int cd_loop_mdisp_offset = CustomData_get_offset(&bm->ldata, CD_MDISPS);
		BMIter iter;
		BMFace *efa;
		BM_ITER_MESH (efa, &iter, bm, BM_FACES_OF_MESH) {
			BM_face_normal_flip_ex(bm, efa, cd_loop_mdisp_offset, true);
		}
	}
	
	DM_to_bmesh_ex(dm, bm, true);

	/* main bmesh intersection setup */
	{
		/* create tessface & intersect */
		const int looptris_tot = poly_to_tri_count(bm->totface, bm->totloop);
		int tottri;
		BMLoop *(*looptris)[3];

		looptris = MEM_mallocN(sizeof(*looptris) * looptris_tot, __func__);

		BM_mesh_calc_tessellation(bm, looptris, &tottri);

		/* postpone this until after tessellating
		 * so we can use the original normals before the vertex are moved */
		{
			BMIter iter;
			int i;
			const int i_verts_end = dm_other->getNumVerts(dm_other);
			const int i_faces_end = dm_other->getNumPolys(dm_other);

			float imat[4][4];
			float omat[4][4];

			invert_m4_m4(imat, ob->obmat);
			mul_m4_m4m4(omat, imat, ob_other->obmat);


			BMVert *eve;
			i = 0;
			BM_ITER_MESH (eve, &iter, bm, BM_VERTS_OF_MESH) {
				mul_m4_v3(omat, eve->co);
				if (++i == i_verts_end) {
					break;
				}
			}

			/* we need face normals because of 'BM_face_split_edgenet'
			 * we could calculate on the fly too (before calling split). */
			{
				float nmat[4][4];
				invert_m4_m4(nmat, omat);

				const short ob_src_totcol = ob_other->totcol;
				short *material_remap = BLI_array_alloca(material_remap, ob_src_totcol ? ob_src_totcol : 1);

				BKE_material_remap_object_calc(ob, ob_other, material_remap);

				BMFace *efa;
				i = 0;
				BM_ITER_MESH (efa, &iter, bm, BM_FACES_OF_MESH) {
					mul_transposed_mat3_m4_v3(nmat, efa->no);
					normalize_v3(efa->no);
					BM_elem_flag_enable(efa, BM_FACE_TAG);  /* temp tag to test which side split faces are from */

					/* remap material */
					if (LIKELY(efa->mat_nr < ob_src_totcol)) {
						efa->mat_nr = material_remap[efa->mat_nr];
					}

					if (++i == i_faces_end) {
						break;
					}
				}
			}
		}

		/* not needed, but normals for 'dm' will be invalid,
		 * currently this is ok for 'BM_mesh_intersect' */
		// BM_mesh_normals_update(bm);

		bool use_separate = false;
		bool use_dissolve = true;
		bool use_island_connect = true;

		/* change for testing */
		if (G.debug & G_DEBUG) {
			use_separate = (bmd->bm_flag & eBooleanModifierBMeshFlag_BMesh_Separate) != 0;
			use_dissolve = (bmd->bm_flag & eBooleanModifierBMeshFlag_BMesh_NoDissolve) == 0;
			use_island_connect = (bmd->bm_flag & eBooleanModifierBMeshFlag_BMesh_NoConnectRegions) == 0;
		}

		BM_mesh_intersect(
				bm,
				looptris, tottri,
				bm_face_isect_pair, NULL,
				false,
				use_separate,
				use_dissolve,
				use_island_connect,
				false,
				op_type,
				double_threshold);

		MEM_freeN(looptris);
	}

	result = CDDM_from_bmesh(bm, true);

	BM_mesh_free(bm);

	result->dirty |= DM_DIRTY_NORMALS;

	#ifdef DEBUG_TIME
	TIMEIT_END(boolean_bmesh);
	#endif

	return result;
}
