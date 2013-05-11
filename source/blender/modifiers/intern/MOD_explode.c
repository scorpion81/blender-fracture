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
 * The Original Code is Copyright (C) 2005 by the Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Daniel Dunbar
 *                 Ton Roosendaal,
 *                 Ben Batt,
 *                 Brecht Van Lommel,
 *                 Campbell Barton,
 *				   Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

/** \file blender/modifiers/intern/MOD_explode.c
 *  \ingroup modifiers
 */


#include "DNA_meshdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_scene_types.h"
#include "DNA_object_types.h"
#include "DNA_group_types.h"

#include "BLI_kdtree.h"
#include "BLI_rand.h"
#include "BLI_math.h"
#include "BLI_edgehash.h"
#include "BLI_utildefines.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_deform.h"
#include "BKE_lattice.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_particle.h"
#include "BKE_scene.h"
#include "BKE_group.h"


#include "MEM_guardedalloc.h"

#include "MOD_util.h"

#include "MOD_boolean_util.h"
#include "bmesh.h"
#include "BLI_path_util.h"
#include <ctype.h>
#include "DNA_material_types.h"
#include "BKE_material.h"
#include "DNA_gpencil_types.h"
#include "BKE_global.h"
#include "BKE_main.h"
#include "BKE_library.h"

#include "BKE_submesh.h"

#ifdef WITH_MOD_VORONOI
#  include "../../../../extern/voro++/src/c_interface.hh"
#endif

//void updateMesh(VoronoiCell* cell, Object* ob);

static void initData(ModifierData *md)
{
	ExplodeModifierData *emd = (ExplodeModifierData *) md;

	emd->mode = eFractureMode_Faces;
	emd->use_boolean = FALSE;
	emd->use_cache = MOD_VORONOI_USECACHE;
	//emd->refracture = FALSE;
	emd->fracMesh = NULL;
	emd->tempOb = NULL;
	emd->cells = NULL;
	//emd->flip_normal = FALSE;

	emd->last_part = 0;
	emd->last_bool = FALSE;
//	emd->last_flip = FALSE;

	emd->facepa = NULL;
	emd->emit_continuously = FALSE;
	emd->flag |= eExplodeFlag_Unborn + eExplodeFlag_Alive + eExplodeFlag_Dead;
	emd->patree = NULL;
	emd->map_delay = 1;
	emd->last_map_delay = 1;
	emd->inner_material = NULL;
	emd->point_source = eOwnParticles;
	emd->last_point_source = eOwnParticles;
	emd->use_animation = FALSE;
	emd->noise = 0.0f;
	emd->percentage = 100;
	emd->storage = NULL;
}

static void freeCells(ExplodeModifierData* emd)
{
	int c = 0, v = 0;
	
	if ((emd->cells) && (emd->mode == eFractureMode_Cells)) {
		if (emd->cells->data) {
			for (c = 0; c < emd->cells->count; c++) {
				MEM_freeN(emd->cells->data[c].vertco);
				emd->cells->data[c].vertco = NULL;
				MEM_freeN(emd->cells->data[c].vertices);
				for (v = 0; v < emd->cells->data[c].vertex_count; v++) {
					emd->cells->data[c].vertices[v] = NULL;
				}
				emd->cells->data[c].vertices = NULL;
				if (emd->cells->data[c].cell_mesh != NULL) {
					DM_release(emd->cells->data[c].cell_mesh);
					MEM_freeN(emd->cells->data[c].cell_mesh);
					emd->cells->data[c].cell_mesh = NULL;
				}
				if (emd->cells->data[c].storage!= NULL)
				{
					BKE_submesh_free(emd->cells->data[c].storage);
					emd->cells->data[c].storage = NULL;
				}

				if (emd->cells->data[c].vert_indexes != NULL)
				{
					MEM_freeN(emd->cells->data[c].vert_indexes);
					emd->cells->data[c].vert_indexes = NULL;
				}

				if (emd->cells->data[c].neighbor_ids != NULL)
				{
					MEM_freeN(emd->cells->data[c].neighbor_ids);
					emd->cells->data[c].neighbor_ids = NULL;
				}

				if (emd->cells->data[c].global_face_map != NULL)
				{
					MEM_freeN(emd->cells->data[c].global_face_map);
					emd->cells->data[c].global_face_map = NULL;
				}
			}

			MEM_freeN(emd->cells->data);
			emd->cells->data = NULL;
		}

		MEM_freeN(emd->cells);
		emd->cells = NULL;
	}
}

#ifdef WITH_MOD_VORONOI

static void freeData(ModifierData *md)
{
	ExplodeModifierData *emd = (ExplodeModifierData *) md;

	freeCells(emd);

	if ((emd->fracMesh) && (emd->mode == eFractureMode_Cells)) {
		BM_mesh_free(emd->fracMesh);
		emd->fracMesh = NULL;
	}

	if ((emd->tempOb) && (emd->tempOb->data) && (emd->mode == eFractureMode_Cells)) {
		BKE_libblock_free_us(&(G.main->object), emd->tempOb);
		BKE_object_unlink(emd->tempOb);
		BKE_object_free(emd->tempOb);
		emd->tempOb = NULL;
	}

	//if (emd->mode == eFractureMode_Faces)
	{
		if (emd->facepa) MEM_freeN(emd->facepa);
	}
	
	if (emd->patree) {
		BLI_kdtree_free(emd->patree);
		emd->patree = NULL;
	}
	
	if (emd->inner_material) {
		//will be freed by walk/foreachIDLink ?
		emd->inner_material = NULL;
	}

	if (emd->storage != NULL)
	{
		BKE_submesh_free(emd->storage);
		emd->storage = NULL;
	}
}

#else

static void freeData(ModifierData *md)
{
	ExplodeModifierData *emd = (ExplodeModifierData *) md;
	if (emd->mode == eFractureMode_Faces)
	{
		if (emd->facepa) MEM_freeN(emd->facepa);
	}
}

#endif

static void copy_voronoicell(ExplodeModifierData* emd, VoronoiCell* dst, VoronoiCell src)
{
	int i = 0;
	BMesh* bmtemp;
	(*dst).vertco = MEM_dupallocN(src.vertco);
	(*dst).vert_indexes = MEM_dupallocN(src.vert_indexes);
	(*dst).vertices = MEM_mallocN(sizeof(BMVert*) * src.vertex_count, "voronoicell->dstvertices");
	for (i = 0; i < src.vertex_count; i++)
	{
		(*dst).vertices[i] = BM_vert_at_index(emd->fracMesh, src.vert_indexes[i]);
	}

	bmtemp = DM_to_bmesh(src.cell_mesh);
	(*dst).cell_mesh = CDDM_from_bmesh(bmtemp, TRUE);
	//(*dst).storage = BKE_bmesh_to_submesh(bmtemp);
	(*dst).storage = NULL;

	BM_mesh_free(bmtemp);
	bmtemp = NULL;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	ExplodeModifierData *emd = (ExplodeModifierData *) md;
	ExplodeModifierData *temd = (ExplodeModifierData *) target;
	int i;

	temd->facepa = NULL;
	temd->flag = emd->flag;
	temd->protect = emd->protect;
	temd->vgroup = emd->vgroup;
	temd->mode = emd->mode;
	temd->use_boolean = emd->use_boolean;

	temd->fracMesh = BM_mesh_copy(emd->fracMesh);// better regenerate this ?
	//temd->storage = BKE_bmesh_to_submesh(temd->fracMesh);
	temd->storage = NULL;

	temd->use_cache = emd->use_cache;
	temd->tempOb = emd->tempOb;
	temd->cells = MEM_dupallocN(emd->cells);
	temd->cells->data = MEM_dupallocN(emd->cells->data);

	for (i = 0; i < emd->cells->count; i++) {
		copy_voronoicell(temd, &temd->cells->data[i], emd->cells->data[i]);
	}

	temd->last_part = emd->last_part;
	temd->last_bool = emd->last_bool;
	temd->emit_continuously = emd->emit_continuously;
	temd->map_delay = emd->map_delay;
	temd->last_map_delay = emd->last_map_delay;
	temd->inner_material = emd->inner_material;
	temd->point_source = emd->point_source;
	temd->last_point_source = emd->last_point_source;
	temd->use_animation = emd->use_animation;
	temd->noise = emd->noise;
	temd->percentage = emd->percentage;

}

static int dependsOnTime(ModifierData *UNUSED(md)) 
{
	return 1;
}
static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *md)
{
	ExplodeModifierData *emd = (ExplodeModifierData *) md;
	CustomDataMask dataMask = 0;

	if (emd->vgroup)
		dataMask |= CD_MASK_MDEFORMVERT;

	return dataMask;
}

static void createFacepa(ExplodeModifierData *emd,
						 ParticleSystemModifierData *psmd,
						 DerivedMesh *dm)
{
	ParticleSystem *psys = psmd->psys;
	MFace *fa = NULL, *mface = NULL;
	MVert *mvert = NULL;
	ParticleData *pa;
	KDTree *tree;
	float center[3], co[3];
	int *facepa = NULL, *vertpa = NULL, totvert = 0, totface = 0, totpart = 0;
	int i, p, v1, v2, v3, v4 = 0;

	mvert = dm->getVertArray(dm);
	mface = dm->getTessFaceArray(dm);
	totface = dm->getNumTessFaces(dm);
	totvert = dm->getNumVerts(dm);
	totpart = psmd->psys->totpart;

	BLI_srandom(psys->seed);

	if (emd->facepa)
		MEM_freeN(emd->facepa);

	facepa = emd->facepa = MEM_callocN(sizeof(int) * totface, "explode_facepa");

	vertpa = MEM_callocN(sizeof(int) * totvert, "explode_vertpa");

	/* initialize all faces & verts to no particle */
	for (i = 0; i < totface; i++)
		facepa[i] = totpart;

	for (i = 0; i < totvert; i++)
		vertpa[i] = totpart;

	/* set protected verts */
	if (emd->vgroup) {
		MDeformVert *dvert = dm->getVertDataArray(dm, CD_MDEFORMVERT);
		if (dvert) {
			const int defgrp_index = emd->vgroup - 1;
			for (i = 0; i < totvert; i++, dvert++) {
				float val = BLI_frand();
				val = (1.0f - emd->protect) * val + emd->protect * 0.5f;
				if (val < defvert_find_weight(dvert, defgrp_index))
					vertpa[i] = -1;
			}
		}
	}

	/* make tree of emitter locations */
	tree = BLI_kdtree_new(totpart);
	for (p = 0, pa = psys->particles; p < totpart; p++, pa++) {
		psys_particle_on_emitter(psmd, psys->part->from, pa->num, pa->num_dmcache, pa->fuv, pa->foffset, co, NULL, NULL, NULL, NULL, NULL);
		BLI_kdtree_insert(tree, p, co, NULL);
	}
	BLI_kdtree_balance(tree);

	/* set face-particle-indexes to nearest particle to face center */
	for (i = 0, fa = mface; i < totface; i++, fa++) {
		add_v3_v3v3(center, mvert[fa->v1].co, mvert[fa->v2].co);
		add_v3_v3(center, mvert[fa->v3].co);
		if (fa->v4) {
			add_v3_v3(center, mvert[fa->v4].co);
			mul_v3_fl(center, 0.25);
		}
		else
			mul_v3_fl(center, 1.0f / 3.0f);

		p = BLI_kdtree_find_nearest(tree, center, NULL, NULL);

		v1 = vertpa[fa->v1];
		v2 = vertpa[fa->v2];
		v3 = vertpa[fa->v3];
		if (fa->v4)
			v4 = vertpa[fa->v4];

		if (v1 >= 0 && v2 >= 0 && v3 >= 0 && (fa->v4 == 0 || v4 >= 0))
			facepa[i] = p;

		if (v1 >= 0) vertpa[fa->v1] = p;
		if (v2 >= 0) vertpa[fa->v2] = p;
		if (v3 >= 0) vertpa[fa->v3] = p;
		if (fa->v4 && v4 >= 0) vertpa[fa->v4] = p;
	}

	if (vertpa) MEM_freeN(vertpa);
	BLI_kdtree_free(tree);
}

static int edgecut_get(EdgeHash *edgehash, unsigned int v1, unsigned int v2)
{
	return GET_INT_FROM_POINTER(BLI_edgehash_lookup(edgehash, v1, v2));
}

 
static const short add_faces[24] = {
	0,
	0, 0, 2, 0, 1, 2, 2, 0, 2, 1,
	2, 2, 2, 2, 3, 0, 0, 0, 1, 0,
	1, 1, 2
};

static MFace *get_dface(DerivedMesh *dm, DerivedMesh *split, int cur, int i, MFace *mf)
{
	MFace *df = CDDM_get_tessface(split, cur);
	DM_copy_tessface_data(dm, split, i, cur, 1);
	*df = *mf;
	return df;
}

#define SET_VERTS(a, b, c, d)           \
	{                                   \
		v[0] = mf->v##a; uv[0] = a - 1; \
		v[1] = mf->v##b; uv[1] = b - 1; \
		v[2] = mf->v##c; uv[2] = c - 1; \
		v[3] = mf->v##d; uv[3] = d - 1; \
	} (void)0

#define GET_ES(v1, v2) edgecut_get(eh, v1, v2)
#define INT_UV(uvf, c0, c1) interp_v2_v2v2(uvf, mf->uv[c0], mf->uv[c1], 0.5f)

static void remap_faces_3_6_9_12(DerivedMesh *dm, DerivedMesh *split, MFace *mf, int *facepa, int *vertpa, int i, EdgeHash *eh, int cur, int v1, int v2, int v3, int v4)
{
	MFace *df1 = get_dface(dm, split, cur, i, mf);
	MFace *df2 = get_dface(dm, split, cur + 1, i, mf);
	MFace *df3 = get_dface(dm, split, cur + 2, i, mf);

	facepa[cur] = vertpa[v1];
	df1->v1 = v1;
	df1->v2 = GET_ES(v1, v2);
	df1->v3 = GET_ES(v2, v3);
	df1->v4 = v3;
	df1->flag |= ME_FACE_SEL;

	facepa[cur + 1] = vertpa[v2];
	df2->v1 = GET_ES(v1, v2);
	df2->v2 = v2;
	df2->v3 = GET_ES(v2, v3);
	df2->v4 = 0;
	df2->flag &= ~ME_FACE_SEL;

	facepa[cur + 2] = vertpa[v1];
	df3->v1 = v1;
	df3->v2 = v3;
	df3->v3 = v4;
	df3->v4 = 0;
	df3->flag &= ~ME_FACE_SEL;
}

static void remap_uvs_3_6_9_12(DerivedMesh *dm, DerivedMesh *split, int numlayer, int i, int cur, int c0, int c1, int c2, int c3)
{
	MTFace *mf, *df1, *df2, *df3;
	int l;

	for (l = 0; l < numlayer; l++) {
		mf = CustomData_get_layer_n(&split->faceData, CD_MTFACE, l);
		df1 = mf + cur;
		df2 = df1 + 1;
		df3 = df1 + 2;
		mf = CustomData_get_layer_n(&dm->faceData, CD_MTFACE, l);
		mf += i;

		copy_v2_v2(df1->uv[0], mf->uv[c0]);
		INT_UV(df1->uv[1], c0, c1);
		INT_UV(df1->uv[2], c1, c2);
		copy_v2_v2(df1->uv[3], mf->uv[c2]);

		INT_UV(df2->uv[0], c0, c1);
		copy_v2_v2(df2->uv[1], mf->uv[c1]);
		INT_UV(df2->uv[2], c1, c2);

		copy_v2_v2(df3->uv[0], mf->uv[c0]);
		copy_v2_v2(df3->uv[1], mf->uv[c2]);
		copy_v2_v2(df3->uv[2], mf->uv[c3]);
	}
}

static void remap_faces_5_10(DerivedMesh *dm, DerivedMesh *split, MFace *mf, int *facepa, int *vertpa, int i, EdgeHash *eh, int cur, int v1, int v2, int v3, int v4)
{
	MFace *df1 = get_dface(dm, split, cur, i, mf);
	MFace *df2 = get_dface(dm, split, cur + 1, i, mf);

	facepa[cur] = vertpa[v1];
	df1->v1 = v1;
	df1->v2 = v2;
	df1->v3 = GET_ES(v2, v3);
	df1->v4 = GET_ES(v1, v4);
	df1->flag |= ME_FACE_SEL;

	facepa[cur + 1] = vertpa[v3];
	df2->v1 = GET_ES(v1, v4);
	df2->v2 = GET_ES(v2, v3);
	df2->v3 = v3;
	df2->v4 = v4;
	df2->flag |= ME_FACE_SEL;
}

static void remap_uvs_5_10(DerivedMesh *dm, DerivedMesh *split, int numlayer, int i, int cur, int c0, int c1, int c2, int c3)
{
	MTFace *mf, *df1, *df2;
	int l;

	for (l = 0; l < numlayer; l++) {
		mf = CustomData_get_layer_n(&split->faceData, CD_MTFACE, l);
		df1 = mf + cur;
		df2 = df1 + 1;
		mf = CustomData_get_layer_n(&dm->faceData, CD_MTFACE, l);
		mf += i;

		copy_v2_v2(df1->uv[0], mf->uv[c0]);
		copy_v2_v2(df1->uv[1], mf->uv[c1]);
		INT_UV(df1->uv[2], c1, c2);
		INT_UV(df1->uv[3], c0, c3);

		INT_UV(df2->uv[0], c0, c3);
		INT_UV(df2->uv[1], c1, c2);
		copy_v2_v2(df2->uv[2], mf->uv[c2]);
		copy_v2_v2(df2->uv[3], mf->uv[c3]);

	}
}

static void remap_faces_15(DerivedMesh *dm, DerivedMesh *split, MFace *mf, int *facepa, int *vertpa, int i, EdgeHash *eh, int cur, int v1, int v2, int v3, int v4)
{
	MFace *df1 = get_dface(dm, split, cur, i, mf);
	MFace *df2 = get_dface(dm, split, cur + 1, i, mf);
	MFace *df3 = get_dface(dm, split, cur + 2, i, mf);
	MFace *df4 = get_dface(dm, split, cur + 3, i, mf);

	facepa[cur] = vertpa[v1];
	df1->v1 = v1;
	df1->v2 = GET_ES(v1, v2);
	df1->v3 = GET_ES(v1, v3);
	df1->v4 = GET_ES(v1, v4);
	df1->flag |= ME_FACE_SEL;

	facepa[cur + 1] = vertpa[v2];
	df2->v1 = GET_ES(v1, v2);
	df2->v2 = v2;
	df2->v3 = GET_ES(v2, v3);
	df2->v4 = GET_ES(v1, v3);
	df2->flag |= ME_FACE_SEL;

	facepa[cur + 2] = vertpa[v3];
	df3->v1 = GET_ES(v1, v3);
	df3->v2 = GET_ES(v2, v3);
	df3->v3 = v3;
	df3->v4 = GET_ES(v3, v4);
	df3->flag |= ME_FACE_SEL;

	facepa[cur + 3] = vertpa[v4];
	df4->v1 = GET_ES(v1, v4);
	df4->v2 = GET_ES(v1, v3);
	df4->v3 = GET_ES(v3, v4);
	df4->v4 = v4;
	df4->flag |= ME_FACE_SEL;
}

static void remap_uvs_15(DerivedMesh *dm, DerivedMesh *split, int numlayer, int i, int cur, int c0, int c1, int c2, int c3)
{
	MTFace *mf, *df1, *df2, *df3, *df4;
	int l;

	for (l = 0; l < numlayer; l++) {
		mf = CustomData_get_layer_n(&split->faceData, CD_MTFACE, l);
		df1 = mf + cur;
		df2 = df1 + 1;
		df3 = df1 + 2;
		df4 = df1 + 3;
		mf = CustomData_get_layer_n(&dm->faceData, CD_MTFACE, l);
		mf += i;

		copy_v2_v2(df1->uv[0], mf->uv[c0]);
		INT_UV(df1->uv[1], c0, c1);
		INT_UV(df1->uv[2], c0, c2);
		INT_UV(df1->uv[3], c0, c3);

		INT_UV(df2->uv[0], c0, c1);
		copy_v2_v2(df2->uv[1], mf->uv[c1]);
		INT_UV(df2->uv[2], c1, c2);
		INT_UV(df2->uv[3], c0, c2);

		INT_UV(df3->uv[0], c0, c2);
		INT_UV(df3->uv[1], c1, c2);
		copy_v2_v2(df3->uv[2], mf->uv[c2]);
		INT_UV(df3->uv[3], c2, c3);

		INT_UV(df4->uv[0], c0, c3);
		INT_UV(df4->uv[1], c0, c2);
		INT_UV(df4->uv[2], c2, c3);
		copy_v2_v2(df4->uv[3], mf->uv[c3]);
	}
}

static void remap_faces_7_11_13_14(DerivedMesh *dm, DerivedMesh *split, MFace *mf, int *facepa, int *vertpa, int i, EdgeHash *eh, int cur, int v1, int v2, int v3, int v4)
{
	MFace *df1 = get_dface(dm, split, cur, i, mf);
	MFace *df2 = get_dface(dm, split, cur + 1, i, mf);
	MFace *df3 = get_dface(dm, split, cur + 2, i, mf);

	facepa[cur] = vertpa[v1];
	df1->v1 = v1;
	df1->v2 = GET_ES(v1, v2);
	df1->v3 = GET_ES(v2, v3);
	df1->v4 = GET_ES(v1, v4);
	df1->flag |= ME_FACE_SEL;

	facepa[cur + 1] = vertpa[v2];
	df2->v1 = GET_ES(v1, v2);
	df2->v2 = v2;
	df2->v3 = GET_ES(v2, v3);
	df2->v4 = 0;
	df2->flag &= ~ME_FACE_SEL;

	facepa[cur + 2] = vertpa[v4];
	df3->v1 = GET_ES(v1, v4);
	df3->v2 = GET_ES(v2, v3);
	df3->v3 = v3;
	df3->v4 = v4;
	df3->flag |= ME_FACE_SEL;
}

static void remap_uvs_7_11_13_14(DerivedMesh *dm, DerivedMesh *split, int numlayer, int i, int cur, int c0, int c1, int c2, int c3)
{
	MTFace *mf, *df1, *df2, *df3;
	int l;

	for (l = 0; l < numlayer; l++) {
		mf = CustomData_get_layer_n(&split->faceData, CD_MTFACE, l);
		df1 = mf + cur;
		df2 = df1 + 1;
		df3 = df1 + 2;
		mf = CustomData_get_layer_n(&dm->faceData, CD_MTFACE, l);
		mf += i;

		copy_v2_v2(df1->uv[0], mf->uv[c0]);
		INT_UV(df1->uv[1], c0, c1);
		INT_UV(df1->uv[2], c1, c2);
		INT_UV(df1->uv[3], c0, c3);

		INT_UV(df2->uv[0], c0, c1);
		copy_v2_v2(df2->uv[1], mf->uv[c1]);
		INT_UV(df2->uv[2], c1, c2);

		INT_UV(df3->uv[0], c0, c3);
		INT_UV(df3->uv[1], c1, c2);
		copy_v2_v2(df3->uv[2], mf->uv[c2]);
		copy_v2_v2(df3->uv[3], mf->uv[c3]);
	}
}

static void remap_faces_19_21_22(DerivedMesh *dm, DerivedMesh *split, MFace *mf, int *facepa, int *vertpa, int i, EdgeHash *eh, int cur, int v1, int v2, int v3)
{
	MFace *df1 = get_dface(dm, split, cur, i, mf);
	MFace *df2 = get_dface(dm, split, cur + 1, i, mf);

	facepa[cur] = vertpa[v1];
	df1->v1 = v1;
	df1->v2 = GET_ES(v1, v2);
	df1->v3 = GET_ES(v1, v3);
	df1->v4 = 0;
	df1->flag &= ~ME_FACE_SEL;

	facepa[cur + 1] = vertpa[v2];
	df2->v1 = GET_ES(v1, v2);
	df2->v2 = v2;
	df2->v3 = v3;
	df2->v4 = GET_ES(v1, v3);
	df2->flag |= ME_FACE_SEL;
}

static void remap_uvs_19_21_22(DerivedMesh *dm, DerivedMesh *split, int numlayer, int i, int cur, int c0, int c1, int c2)
{
	MTFace *mf, *df1, *df2;
	int l;

	for (l = 0; l < numlayer; l++) {
		mf = CustomData_get_layer_n(&split->faceData, CD_MTFACE, l);
		df1 = mf + cur;
		df2 = df1 + 1;
		mf = CustomData_get_layer_n(&dm->faceData, CD_MTFACE, l);
		mf += i;

		copy_v2_v2(df1->uv[0], mf->uv[c0]);
		INT_UV(df1->uv[1], c0, c1);
		INT_UV(df1->uv[2], c0, c2);

		INT_UV(df2->uv[0], c0, c1);
		copy_v2_v2(df2->uv[1], mf->uv[c1]);
		copy_v2_v2(df2->uv[2], mf->uv[c2]);
		INT_UV(df2->uv[3], c0, c2);
	}
}

static void remap_faces_23(DerivedMesh *dm, DerivedMesh *split, MFace *mf, int *facepa, int *vertpa, int i, EdgeHash *eh, int cur, int v1, int v2, int v3)
{
	MFace *df1 = get_dface(dm, split, cur, i, mf);
	MFace *df2 = get_dface(dm, split, cur + 1, i, mf);
	MFace *df3 = get_dface(dm, split, cur + 2, i, mf);

	facepa[cur] = vertpa[v1];
	df1->v1 = v1;
	df1->v2 = GET_ES(v1, v2);
	df1->v3 = GET_ES(v2, v3);
	df1->v4 = GET_ES(v1, v3);
	df1->flag |= ME_FACE_SEL;

	facepa[cur + 1] = vertpa[v2];
	df2->v1 = GET_ES(v1, v2);
	df2->v2 = v2;
	df2->v3 = GET_ES(v2, v3);
	df2->v4 = 0;
	df2->flag &= ~ME_FACE_SEL;

	facepa[cur + 2] = vertpa[v3];
	df3->v1 = GET_ES(v1, v3);
	df3->v2 = GET_ES(v2, v3);
	df3->v3 = v3;
	df3->v4 = 0;
	df3->flag &= ~ME_FACE_SEL;
}

static void remap_uvs_23(DerivedMesh *dm, DerivedMesh *split, int numlayer, int i, int cur, int c0, int c1, int c2)
{
	MTFace *mf, *df1, *df2;
	int l;

	for (l = 0; l < numlayer; l++) {
		mf = CustomData_get_layer_n(&split->faceData, CD_MTFACE, l);
		df1 = mf + cur;
		df2 = df1 + 1;
		mf = CustomData_get_layer_n(&dm->faceData, CD_MTFACE, l);
		mf += i;

		copy_v2_v2(df1->uv[0], mf->uv[c0]);
		INT_UV(df1->uv[1], c0, c1);
		INT_UV(df1->uv[2], c1, c2);
		INT_UV(df1->uv[3], c0, c2);

		INT_UV(df2->uv[0], c0, c1);
		copy_v2_v2(df2->uv[1], mf->uv[c1]);
		INT_UV(df2->uv[2], c1, c2);

		INT_UV(df2->uv[0], c0, c2);
		INT_UV(df2->uv[1], c1, c2);
		copy_v2_v2(df2->uv[2], mf->uv[c2]);
	}
}

static DerivedMesh *cutEdges(ExplodeModifierData *emd, DerivedMesh *dm)
{
	DerivedMesh *splitdm;
	MFace *mf = NULL, *df1 = NULL;
	MFace *mface = dm->getTessFaceArray(dm);
	MVert *dupve, *mv;
	EdgeHash *edgehash;
	EdgeHashIterator *ehi;
	int totvert = dm->getNumVerts(dm);
	int totface = dm->getNumTessFaces(dm);

	int *facesplit = MEM_callocN(sizeof(int) * totface, "explode_facesplit");
	int *vertpa = MEM_callocN(sizeof(int) * totvert, "explode_vertpa2");
	int *facepa = emd->facepa;
	int *fs, totesplit = 0, totfsplit = 0, curdupface = 0;
	int i, v1, v2, v3, v4, esplit,
	    v[4]  = {0, 0, 0, 0}, /* To quite gcc barking... */
	    uv[4] = {0, 0, 0, 0}; /* To quite gcc barking... */
	int numlayer;
	unsigned int ed_v1, ed_v2;

	edgehash = BLI_edgehash_new();

	/* recreate vertpa from facepa calculation */
	for (i = 0, mf = mface; i < totface; i++, mf++) {
		vertpa[mf->v1] = facepa[i];
		vertpa[mf->v2] = facepa[i];
		vertpa[mf->v3] = facepa[i];
		if (mf->v4)
			vertpa[mf->v4] = facepa[i];
	}

	/* mark edges for splitting and how to split faces */
	for (i = 0, mf = mface, fs = facesplit; i < totface; i++, mf++, fs++) {
		v1 = vertpa[mf->v1];
		v2 = vertpa[mf->v2];
		v3 = vertpa[mf->v3];

		if (v1 != v2) {
			BLI_edgehash_insert(edgehash, mf->v1, mf->v2, NULL);
			(*fs) |= 1;
		}

		if (v2 != v3) {
			BLI_edgehash_insert(edgehash, mf->v2, mf->v3, NULL);
			(*fs) |= 2;
		}

		if (mf->v4) {
			v4 = vertpa[mf->v4];

			if (v3 != v4) {
				BLI_edgehash_insert(edgehash, mf->v3, mf->v4, NULL);
				(*fs) |= 4;
			}

			if (v1 != v4) {
				BLI_edgehash_insert(edgehash, mf->v1, mf->v4, NULL);
				(*fs) |= 8;
			}

			/* mark center vertex as a fake edge split */
			if (*fs == 15)
				BLI_edgehash_insert(edgehash, mf->v1, mf->v3, NULL);
		}
		else {
			(*fs) |= 16; /* mark face as tri */

			if (v1 != v3) {
				BLI_edgehash_insert(edgehash, mf->v1, mf->v3, NULL);
				(*fs) |= 4;
			}
		}
	}

	/* count splits & create indexes for new verts */
	ehi = BLI_edgehashIterator_new(edgehash);
	totesplit = totvert;
	for (; !BLI_edgehashIterator_isDone(ehi); BLI_edgehashIterator_step(ehi)) {
		BLI_edgehashIterator_setValue(ehi, SET_INT_IN_POINTER(totesplit));
		totesplit++;
	}
	BLI_edgehashIterator_free(ehi);

	/* count new faces due to splitting */
	for (i = 0, fs = facesplit; i < totface; i++, fs++)
		totfsplit += add_faces[*fs];
	
	splitdm = CDDM_from_template(dm, totesplit, 0, totface + totfsplit, 0, 0);
	numlayer = CustomData_number_of_layers(&splitdm->faceData, CD_MTFACE);

	/* copy new faces & verts (is it really this painful with custom data??) */
	for (i = 0; i < totvert; i++) {
		MVert source;
		MVert *dest;
		dm->getVert(dm, i, &source);
		dest = CDDM_get_vert(splitdm, i);

		DM_copy_vert_data(dm, splitdm, i, i, 1);
		*dest = source;
	}

	/* override original facepa (original pointer is saved in caller function) */

	/* BMESH_TODO, (totfsplit * 2) over allocation is used since the quads are
	 * later interpreted as tri's, for this to work right I think we probably
	 * have to stop using tessface - campbell */

	facepa = MEM_callocN(sizeof(int) * (totface + (totfsplit * 2)), "explode_facepa");
	//memcpy(facepa, emd->facepa, totface*sizeof(int));
	emd->facepa = facepa;

	/* create new verts */
	ehi = BLI_edgehashIterator_new(edgehash);
	for (; !BLI_edgehashIterator_isDone(ehi); BLI_edgehashIterator_step(ehi)) {
		BLI_edgehashIterator_getKey(ehi, &ed_v1, &ed_v2);
		esplit = GET_INT_FROM_POINTER(BLI_edgehashIterator_getValue(ehi));
		mv = CDDM_get_vert(splitdm, ed_v2);
		dupve = CDDM_get_vert(splitdm, esplit);

		DM_copy_vert_data(splitdm, splitdm, ed_v2, esplit, 1);

		*dupve = *mv;

		mv = CDDM_get_vert(splitdm, ed_v1);

		mid_v3_v3v3(dupve->co, dupve->co, mv->co);
	}
	BLI_edgehashIterator_free(ehi);

	/* create new faces */
	curdupface = 0; //=totface;
	//curdupin=totesplit;
	for (i = 0, fs = facesplit; i < totface; i++, fs++) {
		mf = dm->getTessFaceData(dm, i, CD_MFACE);

		switch (*fs) {
			case 3:
			case 10:
			case 11:
			case 15:
				SET_VERTS(1, 2, 3, 4);
				break;
			case 5:
			case 6:
			case 7:
				SET_VERTS(2, 3, 4, 1);
				break;
			case 9:
			case 13:
				SET_VERTS(4, 1, 2, 3);
				break;
			case 12:
			case 14:
				SET_VERTS(3, 4, 1, 2);
				break;
			case 21:
			case 23:
				SET_VERTS(1, 2, 3, 4);
				break;
			case 19:
				SET_VERTS(2, 3, 1, 4);
				break;
			case 22:
				SET_VERTS(3, 1, 2, 4);
				break;
		}

		switch (*fs) {
			case 3:
			case 6:
			case 9:
			case 12:
				remap_faces_3_6_9_12(dm, splitdm, mf, facepa, vertpa, i, edgehash, curdupface, v[0], v[1], v[2], v[3]);
				if (numlayer)
					remap_uvs_3_6_9_12(dm, splitdm, numlayer, i, curdupface, uv[0], uv[1], uv[2], uv[3]);
				break;
			case 5:
			case 10:
				remap_faces_5_10(dm, splitdm, mf, facepa, vertpa, i, edgehash, curdupface, v[0], v[1], v[2], v[3]);
				if (numlayer)
					remap_uvs_5_10(dm, splitdm, numlayer, i, curdupface, uv[0], uv[1], uv[2], uv[3]);
				break;
			case 15:
				remap_faces_15(dm, splitdm, mf, facepa, vertpa, i, edgehash, curdupface, v[0], v[1], v[2], v[3]);
				if (numlayer)
					remap_uvs_15(dm, splitdm, numlayer, i, curdupface, uv[0], uv[1], uv[2], uv[3]);
				break;
			case 7:
			case 11:
			case 13:
			case 14:
				remap_faces_7_11_13_14(dm, splitdm, mf, facepa, vertpa, i, edgehash, curdupface, v[0], v[1], v[2], v[3]);
				if (numlayer)
					remap_uvs_7_11_13_14(dm, splitdm, numlayer, i, curdupface, uv[0], uv[1], uv[2], uv[3]);
				break;
			case 19:
			case 21:
			case 22:
				remap_faces_19_21_22(dm, splitdm, mf, facepa, vertpa, i, edgehash, curdupface, v[0], v[1], v[2]);
				if (numlayer)
					remap_uvs_19_21_22(dm, splitdm, numlayer, i, curdupface, uv[0], uv[1], uv[2]);
				break;
			case 23:
				remap_faces_23(dm, splitdm, mf, facepa, vertpa, i, edgehash, curdupface, v[0], v[1], v[2]);
				if (numlayer)
					remap_uvs_23(dm, splitdm, numlayer, i, curdupface, uv[0], uv[1], uv[2]);
				break;
			case 0:
			case 16:
				df1 = get_dface(dm, splitdm, curdupface, i, mf);
				facepa[curdupface] = vertpa[mf->v1];

				if (df1->v4)
					df1->flag |= ME_FACE_SEL;
				else
					df1->flag &= ~ME_FACE_SEL;
				break;
		}

		curdupface += add_faces[*fs] + 1;
	}

	for (i = 0; i < curdupface; i++) {
		mf = CDDM_get_tessface(splitdm, i);
		test_index_face(mf, &splitdm->faceData, i, (mf->flag & ME_FACE_SEL ? 4 : 3));
	}

	BLI_edgehash_free(edgehash, NULL);
	MEM_freeN(facesplit);
	MEM_freeN(vertpa);

	CDDM_calc_edges_tessface(splitdm);
	CDDM_tessfaces_to_faces(splitdm); /*builds ngon faces from tess (mface) faces*/

	return splitdm;
}
static DerivedMesh *explodeMesh(ExplodeModifierData *emd,
                                ParticleSystemModifierData *psmd, Scene *scene, Object *ob,
                                DerivedMesh *to_explode)
{
	DerivedMesh *explode, *dm = to_explode;
	MFace *mf = NULL, *mface;
	/* ParticleSettings *part=psmd->psys->part; */ /* UNUSED */
	ParticleSimulationData sim = {NULL};
	ParticleData *pa = NULL, *pars = psmd->psys->particles;
	ParticleKey state, birth;
	EdgeHash *vertpahash;
	EdgeHashIterator *ehi;
	float *vertco = NULL, imat[4][4];
	float rot[4];
	float cfra;
	/* float timestep; */
	int *facepa = emd->facepa;
	int totdup = 0, totvert = 0, totface = 0, totpart = 0, delface = 0;
	int i, v, u;
	unsigned int ed_v1, ed_v2, mindex = 0;
	MTFace *mtface = NULL, *mtf;

	totface = dm->getNumTessFaces(dm);
	totvert = dm->getNumVerts(dm);
	mface = dm->getTessFaceArray(dm);
	totpart = psmd->psys->totpart;

	sim.scene = scene;
	sim.ob = ob;
	sim.psys = psmd->psys;
	sim.psmd = psmd;

	/* timestep = psys_get_timestep(&sim); */

	cfra = BKE_scene_frame_get(scene);

	/* hash table for vertice <-> particle relations */
	vertpahash = BLI_edgehash_new();

	for (i = 0; i < totface; i++) {
		if (facepa[i] != totpart) {
			pa = pars + facepa[i];

			if ((pa->alive == PARS_UNBORN && (emd->flag & eExplodeFlag_Unborn) == 0) ||
			    (pa->alive == PARS_ALIVE && (emd->flag & eExplodeFlag_Alive) == 0) ||
			    (pa->alive == PARS_DEAD && (emd->flag & eExplodeFlag_Dead) == 0))
			{
				delface++;
				continue;
			}
		}

		/* do mindex + totvert to ensure the vertex index to be the first
		 * with BLI_edgehashIterator_getKey */
		if (facepa[i] == totpart || cfra < (pars + facepa[i])->time)
			mindex = totvert + totpart;
		else 
			mindex = totvert + facepa[i];

		mf = &mface[i];

		/* set face vertices to exist in particle group */
		BLI_edgehash_insert(vertpahash, mf->v1, mindex, NULL);
		BLI_edgehash_insert(vertpahash, mf->v2, mindex, NULL);
		BLI_edgehash_insert(vertpahash, mf->v3, mindex, NULL);
		if (mf->v4)
			BLI_edgehash_insert(vertpahash, mf->v4, mindex, NULL);
	}

	/* make new vertice indexes & count total vertices after duplication */
	ehi = BLI_edgehashIterator_new(vertpahash);
	for (; !BLI_edgehashIterator_isDone(ehi); BLI_edgehashIterator_step(ehi)) {
		BLI_edgehashIterator_setValue(ehi, SET_INT_IN_POINTER(totdup));
		totdup++;
	}
	BLI_edgehashIterator_free(ehi);

	/* the final duplicated vertices */
	explode = CDDM_from_template(dm, totdup, 0, totface - delface, 0, 0);
	mtface = CustomData_get_layer_named(&explode->faceData, CD_MTFACE, emd->uvname);
	/*dupvert = CDDM_get_verts(explode);*/

	/* getting back to object space */
	invert_m4_m4(imat, ob->obmat);

	psmd->psys->lattice = psys_get_lattice(&sim);

	/* duplicate & displace vertices */
	ehi = BLI_edgehashIterator_new(vertpahash);
	for (; !BLI_edgehashIterator_isDone(ehi); BLI_edgehashIterator_step(ehi)) {
		MVert source;
		MVert *dest;

		/* get particle + vertex from hash */
		BLI_edgehashIterator_getKey(ehi, &ed_v1, &ed_v2);
		ed_v2 -= totvert;
		v = GET_INT_FROM_POINTER(BLI_edgehashIterator_getValue(ehi));

		dm->getVert(dm, ed_v1, &source);
		dest = CDDM_get_vert(explode, v);

		DM_copy_vert_data(dm, explode, ed_v1, v, 1);
		*dest = source;

		if (ed_v2 != totpart) {
			/* get particle */
			pa = pars + ed_v2;

			psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);

			state.time = cfra;
			psys_get_particle_state(&sim, ed_v2, &state, 1);

			vertco = CDDM_get_vert(explode, v)->co;
			mul_m4_v3(ob->obmat, vertco);

			sub_v3_v3(vertco, birth.co);

			/* apply rotation, size & location */
			sub_qt_qtqt(rot, state.rot, birth.rot);
			mul_qt_v3(rot, vertco);

			if (emd->flag & eExplodeFlag_PaSize)
				mul_v3_fl(vertco, pa->size);

			add_v3_v3(vertco, state.co);

			mul_m4_v3(imat, vertco);
		}
	}
	BLI_edgehashIterator_free(ehi);

	/*map new vertices to faces*/
	for (i = 0, u = 0; i < totface; i++) {
		MFace source;
		int orig_v4;

		if (facepa[i] != totpart) {
			pa = pars + facepa[i];

			if (pa->alive == PARS_UNBORN && (emd->flag & eExplodeFlag_Unborn) == 0) continue;
			if (pa->alive == PARS_ALIVE && (emd->flag & eExplodeFlag_Alive) == 0) continue;
			if (pa->alive == PARS_DEAD && (emd->flag & eExplodeFlag_Dead) == 0) continue;
		}

		dm->getTessFace(dm, i, &source);
		mf = CDDM_get_tessface(explode, u);
		
		orig_v4 = source.v4;

		if (facepa[i] != totpart && cfra < pa->time)
			mindex = totvert + totpart;
		else 
			mindex = totvert + facepa[i];

		source.v1 = edgecut_get(vertpahash, source.v1, mindex);
		source.v2 = edgecut_get(vertpahash, source.v2, mindex);
		source.v3 = edgecut_get(vertpahash, source.v3, mindex);
		if (source.v4)
			source.v4 = edgecut_get(vertpahash, source.v4, mindex);

		DM_copy_tessface_data(dm, explode, i, u, 1);

		*mf = source;

		/* override uv channel for particle age */
		if (mtface) {
			float age = (cfra - pa->time) / pa->lifetime;
			/* Clamp to this range to avoid flipping to the other side of the coordinates. */
			CLAMP(age, 0.001f, 0.999f);

			mtf = mtface + u;

			mtf->uv[0][0] = mtf->uv[1][0] = mtf->uv[2][0] = mtf->uv[3][0] = age;
			mtf->uv[0][1] = mtf->uv[1][1] = mtf->uv[2][1] = mtf->uv[3][1] = 0.5f;
		}

		test_index_face(mf, &explode->faceData, u, (orig_v4 ? 4 : 3));
		u++;
	}

	/* cleanup */
	BLI_edgehash_free(vertpahash, NULL);

	/* finalization */
	CDDM_calc_edges_tessface(explode);
	CDDM_tessfaces_to_faces(explode);
	CDDM_calc_normals(explode);

	if (psmd->psys->lattice) {
		end_latt_deform(psmd->psys->lattice);
		psmd->psys->lattice = NULL;
	}

	return explode;
}

static ParticleSystemModifierData *findPrecedingParticlesystem(Object *ob, ModifierData *emd)
{
	ModifierData *md;
	ParticleSystemModifierData *psmd = NULL;

	for (md = ob->modifiers.first; emd != md; md = md->next) {
		if (md->type == eModifierType_ParticleSystem)
			psmd = (ParticleSystemModifierData *) md;
	}
	return psmd;
}

static RigidBodyModifierData *findFollowingRigidBodyModifier(Object *ob, ExplodeModifierData *emd)
{
	ModifierData *md;
	RigidBodyModifierData *rmd = NULL;

	md = &emd->modifier;
	md = md->next;

	if (md && md->type == eModifierType_RigidBody) {
		rmd = (RigidBodyModifierData*)md;
	}

	return rmd;
}

static int dm_minmax(DerivedMesh* dm, float min[3], float max[3])
{

	int verts = dm->getNumVerts(dm);
	MVert *mverts = dm->getVertArray(dm);
	MVert *mvert;
	int i = 0;

	INIT_MINMAX(min, max);
	for (i = 0; i < verts; i++) {
		mvert = &mverts[i];
		minmax_v3v3_v3(min, max, mvert->co);
	}
	
	return (verts != 0);
}

static int points_from_verts(Object** ob, int totobj, float*** points, int p_exist, float mat[4][4], float thresh, ExplodeModifierData *emd, DerivedMesh* dm, Object* obj)
{
	int v, o, pt = p_exist;
	float co[3];
	
	for (o = 0; o < totobj; o++)
	{
		if (ob[o]->type == OB_MESH)
		{
			float imat[4][4];
			//Mesh* me = (Mesh*)ob[o]->data;
			DerivedMesh *d;
			MVert* vert;

			if (ob[o] == obj)
			{
				d = dm;
			}
			else
			{
				d = mesh_get_derived_final(emd->modifier.scene, ob[o], 0);
			}

			invert_m4_m4(imat, mat);
			vert = d->getVertArray(d);

			for (v = 0; v < d->getNumVerts(d); v++)
			{
				if (BLI_frand() < thresh) {
					*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
					(*points)[pt] = MEM_callocN(3 * sizeof(float), "points[pt]");
				
					//copy_v3_v3(co, me->mvert[v].co);
					copy_v3_v3(co, vert[v].co);

					if ((o > 0) ||
					   ((emd->point_source & eExtraVerts) &&
					   (!(emd->point_source & eOwnVerts)) && (o == 0)))
					{
						mul_m4_v3(ob[o]->obmat, co);
						mul_m4_v3(imat, co);
					}

					copy_v3_v3((*points)[pt], co);
					pt++;
				}
			}
		}
	}
	
	return pt;
}

static int points_from_particles(Object** ob, int totobj, Scene* scene, float*** points, int p_exist, float mat[4][4],
								 float thresh, ExplodeModifierData* emd)
{
	int o, p, pt = p_exist;
	ParticleSystemModifierData* psmd;
	ParticleData* pa;
	ParticleSimulationData sim = {NULL};
	ParticleKey birth;
	ModifierData* mod;
	
	for (o = 0; o < totobj; o++)
	{
		for (mod = ob[o]->modifiers.first; mod; mod = mod->next)
		{
			if (mod->type == eModifierType_ParticleSystem)
			{
				float imat[4][4];
				psmd = (ParticleSystemModifierData*)mod;
				sim.scene = scene;
				sim.ob = ob[o];
				sim.psys = psmd->psys;
				sim.psmd = psmd;
				invert_m4_m4(imat, mat);

				for (p = 0, pa = psmd->psys->particles; p < psmd->psys->totpart; p++, pa++)
				{
					if (BLI_frand() < thresh) {
						float co[3];
						psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);
						*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
						(*points)[pt] = MEM_callocN(3*sizeof(float), "points[pt]");
						copy_v3_v3(co, birth.co);

						if ((o > 0) ||
						   ((emd->point_source & eExtraParticles) &&
						   (!(emd->point_source & eOwnParticles)) && (o == 0)))
						{
							//mul_m4_v3(ob[o]->obmat, co);
							mul_m4_v3(imat, co);
						}
						copy_v3_v3((*points)[pt], co);
						pt++;
					}
				}
			}
		}
	}
	
	return pt;
}

static int points_from_greasepencil(Object** ob, int totobj, float*** points, int p_exist, float mat[4][4], float thresh)
{
	bGPDlayer* gpl;
	bGPDframe* gpf;
	bGPDstroke* gps;
	int pt = p_exist, p, o;
	
	for (o = 0; o < totobj; o++)
	{
		if ((ob[o]->gpd) && (ob[o]->gpd->layers.first))
		{
			float imat[4][4];
			invert_m4_m4(imat, mat);
			for (gpl = ob[o]->gpd->layers.first; gpl; gpl = gpl->next)
			{
				for (gpf = gpl->frames.first; gpf; gpf = gpf->next) {
				//gpf = gpl->actframe;
					for (gps = gpf->strokes.first; gps; gps = gps->next)
					{
						for (p = 0; p < gps->totpoints; p++)
						{
							if (BLI_frand() < thresh)
							{
								float point[3] = {0, 0, 0};
								*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
								(*points)[pt] = MEM_callocN(3*sizeof(float), "points[pt]");

								point[0] = gps->points[p].x;
								point[1] = gps->points[p].y;
								point[2] = gps->points[p].z;

								mul_m4_v3(imat, point);
								copy_v3_v3((*points)[pt], point);
								pt++;
							}
						}
					}
				}
			}
		}
	}
	
	return pt;
}

static int getGroupObjects(Group *gr, Object ***obs, int g_exist)
{
	int ctr = g_exist;
	GroupObject *go;
	if (gr == NULL) return ctr;

	for (go = gr->gobject.first; go; go = go->next) {

		*obs = MEM_reallocN(*obs, sizeof(Object*) * (ctr+1));
		(*obs)[ctr] = go->ob;
		ctr++;
	}

	return ctr;
}

static int get_points(ExplodeModifierData *emd, Scene *scene, Object *ob, float ***points, float mat[4][4], DerivedMesh *derivedData)
{
	int totpoint = 0, totgroup = 0, t = 0;
	Object** go = MEM_mallocN(sizeof(Object*), "groupobjects");
	float thresh = (float)emd->percentage / 100.0f;
	BoundBox* bb;

	if (emd->point_source & (eExtraParticles | eExtraVerts ))
	{
		if (((emd->point_source & eOwnParticles) && (emd->point_source & eExtraParticles)) ||
			((emd->point_source & eOwnVerts) && (emd->point_source & eExtraVerts)) ||
			((emd->point_source & eGreasePencil) && (emd->point_source & eExtraParticles)) ||
			((emd->point_source & eGreasePencil) && (emd->point_source & eExtraVerts)))
		{
			go = MEM_reallocN(go, sizeof(Object*)*(totgroup+1));
			go[totgroup] = ob;
			totgroup++;
		}

		totgroup = getGroupObjects(emd->extra_group, &go, totgroup);
	}
	else
	{
		totgroup = 1;
		go[0] = ob;
	}
	
	if (emd->point_source & (eOwnParticles | eExtraParticles))
	{
		totpoint = points_from_particles(go, totgroup, scene, points, totpoint, mat, thresh, emd);
	}
	
	if (emd->point_source & (eOwnVerts | eExtraVerts))
	{
		totpoint = points_from_verts(go, totgroup, points, totpoint, mat, thresh, emd, derivedData, ob);
	}
	
	if (emd->point_source & eGreasePencil)
	{
		totpoint = points_from_greasepencil(go, totgroup, points, totpoint, mat, thresh);
	}

	//apply noise

	bb = BKE_object_boundbox_get(ob);
	for (t = 0; t < totpoint; t++) {
		float bbox_min[3], bbox_max[3];

		if (emd->noise > 0.0f) {
			float scalar, size[3], rand[3] = {0, 0, 0}, temp_x[3], temp_y[3], temp_z[3], rand_x[3], rand_y[3], rand_z[3];
			mul_v3_m4v3(bbox_min, ob->obmat, bb->vec[0]);
			mul_v3_m4v3(bbox_max, ob->obmat, bb->vec[6]);
			sub_v3_v3v3(size, bbox_max, bbox_min);

			scalar = emd->noise * len_v3(size) / 2.0f;
			rand[0] = 2.0f * (BLI_frand() - 0.5f);
			rand[1] = 2.0f * (BLI_frand() - 0.5f);
			rand[2] = 2.0f * (BLI_frand() - 0.5f);

			rand[0] *= (scalar * BLI_frand());
			rand[1] *= (scalar * BLI_frand());
			rand[2] *= (scalar * BLI_frand());

			//test each component separately
			zero_v3(rand_x);
			rand_x[0] = rand[0];

			zero_v3(rand_y);
			rand_y[1] = rand[1];

			zero_v3(rand_z);
			rand_z[2] = rand[2];

			add_v3_v3v3(temp_x, (*points)[t], rand_x);
			add_v3_v3v3(temp_y, (*points)[t], rand_y);
			add_v3_v3v3(temp_z, (*points)[t], rand_z);

			//stay inside bounds !!
			if ((temp_x[0] >= bb->vec[0][0]) && (temp_x[0] <= bb->vec[6][0]))
				add_v3_v3((*points)[t], rand_x);

			if ((temp_y[1] >= bb->vec[0][1]) && (temp_y[1] <= bb->vec[6][1]))
				add_v3_v3((*points)[t], rand_y);

			if ((temp_z[2] >= bb->vec[0][2]) && (temp_z[2] <= bb->vec[6][2]))
				add_v3_v3((*points)[t], rand_z);
		}
	}

	//MEM_freeN(bb);
	MEM_freeN(go);
	return totpoint;
}

// create the voronoi cell faces inside the existing mesh
static BMesh* fractureToCells(Object *ob, DerivedMesh* derivedData, ExplodeModifierData* emd, float mat[4][4])
{
	void* container = NULL;
	void* particle_order = NULL;
	float min[3], max[3];
	int p = 0;
	float co[3], vco[3];
	BMesh *bm = NULL, *bmtemp = NULL;

	FILE *fp = NULL;
	int vert_index = 0, edg_index = 0, fac_index = 0;
	int read = 0;
	BMVert **faceverts = NULL, **tempvert = NULL, *vert = NULL, **localverts = NULL;
	BMEdge **faceedges = NULL, *edge = NULL, **localedges = NULL;
	//int *facevert_indexes = NULL;
	int face_index = 0;
	int edge_index = 0;
	char c;
	int facevert_index = 0;
	BMFace *face = NULL;

	DerivedMesh *dm = NULL, *boolresult = NULL;
	MEdge* ed = NULL;
	MVert* ve_old = NULL, *ve_new = NULL;
	int totvert_old = 0, totvert_new = 0;

	int totvert, totedge, totpoly;
	int v, e;
	int tempvert_index;
	const char *file;
	char *path, *fullpath;
	float imat[4][4];
	float theta = 0.0f;
	int n_size = 8;

	float** points = NULL;
	int totpoint = 0;
//	int degenerate = FALSE;
	int neighbor_index = 0;
	int global_face_index = 0;

	INIT_MINMAX(min, max);

	/*if (emd->use_boolean) {
		//theta = -0.01f;
		//make container bigger for boolean case,so cube and container dont have equal size which can lead to boolean errors
		//if (emd->flip_normal)
		{	//cubes usually need flip_normal, so enable theta only here, otherwise it will be subtracted
			//theta = 0.0001f;
		}
		BKE_mesh_minmax(ob->data, min, max);
	}
	else {
		//con = voronoi.domain(xmin-theta,xmax+theta,ymin-theta,ymax+theta,zmin-theta,zmax+theta,nx,ny,nz,False, False, False, particles)
		//dm_minmax(derivedData, min, max);
		BKE_mesh_minmax(ob->data, min, max);
	}*/

	if (emd->use_boolean) {
		theta = 0.1f;
	}

	//BKE_mesh_minmax(ob->data, min, max);
	dm_minmax(derivedData, min, max);
	//use global coordinates for container
	mul_v3_m4v3(min, ob->obmat, min);
	mul_v3_m4v3(max, ob->obmat, max);


	// printf("Container: %f;%f;%f;%f;%f;%f \n", min[0], max[0], min[1], max[1], min[2], max[2]);
	//TODO: maybe support simple shapes without boolean, but eh...
	//container = container_new(min[0]-theta, max[0]+theta, min[1]-theta, max[1]+theta, min[2]-theta, max[2]+theta,
	//						  n_size, n_size, n_size, FALSE, FALSE, FALSE, psmd->psys->totpart); //add number of parts here!
	// particle_order = particle_order_new();

	
	//choose from point sources here
	//if (!emd->refracture)

	points = MEM_mallocN(sizeof(float*), "points");
	totpoint = get_points(emd, emd->modifier.scene, ob, &points, mat, derivedData);

	//no points, cant do anything
	if (totpoint == 0) {
		MEM_freeN(points);
		points = NULL;
		return NULL;
	}

	if (emd->point_source & eOwnVerts)
	{
		//make container a little bigger ? or some noise ?
		if (!emd->use_boolean) {
			theta = 0.0001f;
		}
	}
	container = container_new(min[0]-theta, max[0]+theta, min[1]-theta, max[1]+theta, min[2]-theta, max[2]+theta,
							  n_size, n_size, n_size, FALSE, FALSE, FALSE, totpoint);

	for (p = 0; p < totpoint; p++)
	{
		copy_v3_v3(co, points[p]);
		container_put(container, particle_order, p, co[0], co[1], co[2]);
	}

	//TODO: write results to temp file, ensure using the systems temp dir...
	// this prints out vertex positions and face indexes
	file = "test.out";
	path = MEM_mallocN(((strlen(BLI_temporary_dir()) + strlen(file) + 2) * sizeof(char)), "path");
	path = strcpy(path, BLI_temporary_dir());
	fullpath = strcat(path, file);
	fp = fopen(fullpath, "w+");


	//this triggers computation of the voronoi cells and produces the desired data:
	//see voro++ homepage for detailed description
	//TODO: insert link here

	// %P global vertex coordinates of voronoi vertices
	// v  the vertex -> face delimiter
	// %t the indexes to the cell vertices, describes which vertices build each face
	// f  the face -> centroid section delimiter
	// %C the centroid of the voronoi cell, used to find nearest particle in createCellpa

	//%i the particle index
	container_print_custom(container, "%i %P v %t f %C %n n", fp);
	fflush(fp);
	rewind(fp);
	
	if (points)
	{
		int t;

		for (t = 0; t < totpoint; t++) {
			MEM_freeN(points[t]);
			points[t] = NULL;
		}
		MEM_freeN(points);
		points = NULL;
	}
	
	if (totpoint == 0)
		return NULL;

	bm = DM_to_bmesh(derivedData);
	//empty the mesh
	BM_mesh_clear(bm);

	//TODO: maybe put variable declarations at top of function ? C90 ?
	//TODO: check for setting in psys to volume
	//vert_index = 0;
	
	if (emd->cells)
	{
		freeCells(emd);
	}

	emd->cells = MEM_mallocN(sizeof(VoronoiCells), "emd->cells");
	emd->cells->data = MEM_mallocN(sizeof(VoronoiCell), "emd->cells->data");
	emd->cells->count = 0;

	while(feof(fp) == 0)
	{
		//printf("Reading line...\n");
		int *real_indexes = MEM_mallocN(sizeof(int*), "real_indexes");
		int len_real_indexes = 0;
		BMesh* bmsub;

		//store cell data: centroid and associated vertex coords
		emd->cells->data = MEM_reallocN(emd->cells->data, sizeof(VoronoiCell) * (emd->cells->count + 1));
		emd->cells->data[emd->cells->count].vertices = MEM_mallocN(sizeof(BMVert*), "vertices");// (float*)malloc(3 * sizeof(float));
		emd->cells->data[emd->cells->count].vertco = MEM_mallocN(sizeof(float), "vertco");
		emd->cells->data[emd->cells->count].vertex_count = 0;
		emd->cells->data[emd->cells->count].particle_index = -1;
//		emd->cells->data[emd->cells->count].rigidbody = NULL;
		emd->cells->data[emd->cells->count].vert_indexes = MEM_mallocN(sizeof(int), "fractureToCells->vert_indexes");
		emd->cells->data[emd->cells->count].storage = NULL;
		emd->cells->data[emd->cells->count].neighbor_ids = MEM_mallocN(sizeof(int), "neighbor_ids");
		emd->cells->data[emd->cells->count].neighbor_count = 0;
		emd->cells->data[emd->cells->count].is_at_boundary = FALSE;
		emd->cells->data[emd->cells->count].global_face_map = MEM_mallocN(sizeof(int), "global_face_map");
		

		bmtemp = BM_mesh_create(&bm_mesh_chunksize_default);
		tempvert = MEM_mallocN(sizeof(BMVert*), "tempvert");
		tempvert_index = 0;
//		degenerate = FALSE;

		// Read in the cell data, each line in the output file represents a voronoi cell
		while (1)
		{
			fscanf(fp, "%d ", &emd->cells->data[emd->cells->count].pid);
			// Read in the vertex coordinates of the cell vertices
			read = fscanf(fp, "(%f,%f,%f) ", &vco[0], &vco[1], &vco[2]);
			if (read < 3) break;

			//back to object space

			invert_m4_m4(imat, ob->obmat);
			mul_v3_m4v3(vco, imat, vco);


			tempvert = MEM_reallocN(tempvert, sizeof(BMVert*) * (tempvert_index + 1));
			vert = BM_vert_create(bmtemp, vco, NULL, 0);
			tempvert[tempvert_index] = vert;
			real_indexes = MEM_reallocN(real_indexes, sizeof(int*) * (len_real_indexes+1));
			real_indexes[len_real_indexes] = tempvert_index;

			if (tempvert_index > 0){
				int i;
				//compare all verts of array
				for (i = 0; i < tempvert_index; i++) {
					BMVert* last = tempvert[i];
					real_indexes = MEM_reallocN(real_indexes, sizeof(int*) * (len_real_indexes+1));

					if (compare_v3v3(last->co, vert->co, 1.0e-09)) {
						printf("Verts: (%f, %f, %f), (%f, %f, %f) %d %d\n",
							   last->co[0], last->co[1], last->co[2],
							   vert->co[0], vert->co[1], vert->co[2], i, tempvert_index);
						BM_vert_kill(bmtemp, vert);
						//MEM_freeN(vert);
						tempvert = MEM_reallocN(tempvert, sizeof(BMVert*) * (tempvert_index));
						real_indexes[len_real_indexes] = i;
						tempvert_index--;
						break;
					}
				}
			}

			len_real_indexes++;
			tempvert_index++;
		}

		faceverts = MEM_mallocN(sizeof(BMVert*), "faceverts");
		faceedges = MEM_mallocN(sizeof(BMEdge*), "faceedges");
		//facevert_indexes = MEM_mallocN(sizeof(int), "facevert_indexes");

		face_index = 0;
		edge_index = 0;

		while(1)
		{
			//printf ("Reading faces...\n");
			c = fgetc(fp);
			if (isdigit(c)) //maybe atoi !! (ascii value to int ?)
			{
				int double_vert = FALSE;

				//put/seek back and better do fscanf !
				fseek(fp, -sizeof(char), SEEK_CUR);
				facevert_index = 0;
				fscanf(fp, "%d", &facevert_index);
				faceverts = MEM_reallocN(faceverts, (face_index + 1) * sizeof(BMVert*));
				// find vertices for each face, store indexes here
				faceverts[face_index] = tempvert[real_indexes[facevert_index]];//emd->cells->data[emd->cells->count].vertices[facevert_index];

				if (face_index > 0) {
					//argh, need to determine edges manually...
					int i;
					for (i = 0; i < face_index; i++) {
						if (faceverts[face_index] == faceverts[i])
						{
							//avoid 2 equal verts in an edge or a face
							faceverts = MEM_reallocN(faceverts, (face_index) * sizeof(BMVert*));
							face_index--;
							double_vert = TRUE;
							break;
						}
					}
					if (!double_vert)
					{
						faceedges = MEM_reallocN(faceedges, (edge_index + 1) * sizeof(BMEdge*));
						//printf("Indexes: %d %d %d %p \n", face_index, facevert_index, real_indexes[facevert_index], faceverts[face_index]);
						edge = BM_edge_create(bmtemp, faceverts[face_index], faceverts[face_index-1], NULL, 0);
						faceedges[edge_index] = edge;
						edge_index++;
					}
				}

				face_index++;

			}
			else if ( c == ')') {
				float area;
				Mesh *me = (Mesh*)ob->data;
				//end of face tuple, can create face now, but before create last edge to close the circle
				if (faceverts[0] != faceverts[face_index-1]) {
					faceedges = MEM_reallocN(faceedges, (edge_index + 1) * sizeof(BMEdge*));
					edge = BM_edge_create(bmtemp, faceverts[0], faceverts[face_index-1], NULL, 0);
					faceedges[edge_index] = edge;
				}
				else
				{
					face_index--;
				}

				face = BM_face_create(bmtemp, faceverts, faceedges, face_index, 0);

				if (face != NULL)
					area = BM_face_calc_area(face);
				//printf("Face Area: %f\n", area);

				if ((face != NULL) && ((area < 0.000005f) || (face_index < 3))) {
					//remove degenerate faces
					int i;
					BM_face_kill(bmtemp, face);
					face = NULL;

					for (i = 0; i < edge_index; i++) {
						BM_edge_kill(bmtemp, faceedges[i]);
					}

					for (i = 0; i < vert_index; i++) {
						BM_vert_kill(bmtemp, faceverts[i]);
					}
				}

				if (face != NULL) { //&& (emd->flip_normal))
					BM_face_normal_flip(bmtemp, face);
					if (me->mpoly[0].flag & ME_SMOOTH) {
						BM_elem_flag_enable(face, BM_ELEM_SMOOTH);
					}
				}

				edge_index = 0;
				face_index = 0;
			}
			else if ((c == 'f') || (feof(fp) != 0))
			{
				if (c == 'f')
				{
					//Intersection, use elements from temporary per-cell bmeshes and write to global bmesh, which
					//is passed around and whose vertices are manipulated directly.
					int mat_index = 0;
					MPoly* mp;
					MLoop* ml;
					boolresult = NULL;
					
					dm = CDDM_from_bmesh(bmtemp, TRUE);
					//printf(" %d Faces missing \n", (bmtemp->totface - dm->numPolyData));
					BM_mesh_free(bmtemp);
					bmtemp = NULL;

					if (emd->use_boolean)
					{
						DM_ensure_tessface(derivedData);
						DM_ensure_tessface(dm);
						CDDM_calc_edges_tessface(dm);
						CDDM_tessfaces_to_faces(dm);
						CDDM_calc_normals(dm);

						//put temp bmesh to temp object ? Necessary ? Seems so.
						//TODO: maybe get along without temp object ?
						if (!emd->tempOb)
						{
							emd->tempOb = BKE_object_add_only_object(G.main, OB_MESH, "Intersect");
							//emd->tempOb = BKE_object_add(emd->modifier.scene, OB_MESH);
						}

						if (!emd->tempOb->data)
						{
							emd->tempOb->data = BKE_object_obdata_add_from_type(G.main, OB_MESH);
							//object_add_material_slot(emd->tempOb);
						}


						//assign inner material to temp Object
						if (emd->inner_material)
						{
							//assign inner material as secondary mat to ob if not there already
							mat_index = find_material_index(ob, emd->inner_material);
							if (mat_index == 0)
							{
								object_add_material_slot(ob);
								assign_material(ob, emd->inner_material, ob->totcol, BKE_MAT_ASSIGN_OBDATA);
							}
							
							//shard gets inner material, maybe assign to all faces as well (in case this does not happen automatically
							assign_material(emd->tempOb, emd->inner_material, 1, BKE_MAT_ASSIGN_OBDATA);
						}
						
						DM_to_mesh(dm, emd->tempOb->data, emd->tempOb, 0);
						copy_m4_m4(emd->tempOb->obmat, ob->obmat);
						
						boolresult = NewBooleanDerivedMesh(dm, emd->tempOb, derivedData, ob, eBooleanModifierOp_Intersect);
						
						//if boolean fails, return original mesh, emit a warning
						if (!boolresult)
						{
							boolresult = dm;
							printf("Boolean Operation failed, using original mesh !\n");
						}
						else
						{

							DM_ensure_tessface(boolresult);
							CDDM_calc_edges_tessface(boolresult);
							CDDM_tessfaces_to_faces(boolresult);
							CDDM_calc_normals(boolresult);


							/*DM_release(dm);
							MEM_freeN(dm);
							dm = NULL;*/
						}
					}
					else
					{
						boolresult = dm;
						DM_ensure_tessface(boolresult);
						CDDM_calc_edges_tessface(boolresult);
						CDDM_tessfaces_to_faces(boolresult);
						CDDM_calc_normals(boolresult);
					}
					
					emd->cells->data[emd->cells->count].cell_mesh = boolresult;
					//bmsub = DM_to_bmesh(boolresult);

					/*CustomData_bmesh_init_pool(&bmsub->vdata, bm_mesh_allocsize_default.totvert, BM_VERT);
					CustomData_bmesh_init_pool(&bmsub->edata, bm_mesh_allocsize_default.totedge, BM_EDGE);
					CustomData_bmesh_init_pool(&bmsub->ldata, bm_mesh_allocsize_default.totloop, BM_LOOP);
					CustomData_bmesh_init_pool(&bmsub->pdata, bm_mesh_allocsize_default.totface, BM_FACE);*/

					BKE_submesh_free(emd->cells->data[emd->cells->count].storage);
					emd->cells->data[emd->cells->count].storage = NULL;
					//emd->cells->data[emd->cells->count].storage = BKE_bmesh_to_submesh(bmsub);
					//BM_mesh_free(bmsub);
					
					totvert = boolresult->getNumVerts(boolresult);
					totedge = boolresult->getNumEdges(boolresult);
				//	totface = boolresult->getNumTessFaces(boolresult);
					totpoly = boolresult->getNumPolys(boolresult);

					localverts = MEM_mallocN(sizeof(BMVert*) * totvert, "localverts");
					localedges = MEM_mallocN(sizeof(BMEdge*) * totedge, "localedges");
					ed = boolresult->getEdgeArray(boolresult);

					if (dm != boolresult) {
						ve_new = boolresult->getVertArray(boolresult);
						ve_old = dm->getVertArray(dm);
						totvert_old = dm->getNumVerts(dm);
						totvert_new = boolresult->getNumVerts(boolresult);

						if (totvert_new == totvert_old)
						{
							int v;
							for (v = 0; v < totvert_old; v++)
							{
								//if vertex coords differ, we had a bool op, at must be at boundary of object now
								if (compare_v3v3(ve_old[v].co, ve_new[v].co, 0.000001f)) {
									emd->cells->data[emd->cells->count].is_at_boundary = TRUE;
									break;
								}
							}
						}
						else
						{
							emd->cells->data[emd->cells->count].is_at_boundary = TRUE;
						}

					/*	MEM_freeN(ve_old);
						ve_old = NULL;
						MEM_freeN(ve_new);
						ve_new = NULL;*/

						DM_release(dm);
						MEM_freeN(dm);
						dm = NULL;
					}
					
					
					CustomData_bmesh_merge(&boolresult->vertData, &bm->vdata, CD_MASK_DERIVEDMESH,
									 CD_CALLOC, bm, BM_VERT);
					CustomData_bmesh_merge(&boolresult->edgeData, &bm->edata, CD_MASK_DERIVEDMESH,
									 CD_CALLOC, bm, BM_EDGE);
					CustomData_bmesh_merge(&boolresult->loopData, &bm->ldata, CD_MASK_DERIVEDMESH,
									 CD_CALLOC, bm, BM_LOOP);
					CustomData_bmesh_merge(&boolresult->polyData, &bm->pdata, CD_MASK_DERIVEDMESH,
									 CD_CALLOC, bm, BM_FACE);
					
					for (v = 0; v < totvert; v++)
					{
						boolresult->getVertCo(boolresult, v, co);

						emd->cells->data[emd->cells->count].vertices =
								MEM_reallocN(emd->cells->data[emd->cells->count].vertices, (vert_index + 1)* sizeof(BMVert));

						emd->cells->data[emd->cells->count].vertex_count++;


						vert = BM_vert_create(bm, co, NULL, 0);
					/*	if (BM_elem_index_get(vert) == -1) {
							BM_elem_index_set(vert, vert_index);
						}*/

						localverts[v] = vert;
						//vert = BM_vert_at_index(bm, vert_index);

						emd->cells->data[emd->cells->count].vertices[vert_index] = vert;

						//store original coordinates for later re-use

						emd->cells->data[emd->cells->count].vertco =
								MEM_reallocN(emd->cells->data[emd->cells->count].vertco, (vert_index + 1)* (3*sizeof(float)));

						emd->cells->data[emd->cells->count].vertco[3*vert_index] = vert->co[0];
						emd->cells->data[emd->cells->count].vertco[3*vert_index+1] = vert->co[1];
						emd->cells->data[emd->cells->count].vertco[3*vert_index+2] = vert->co[2];

						emd->cells->data[emd->cells->count].vert_indexes =
								MEM_reallocN(emd->cells->data[emd->cells->count].vert_indexes, sizeof(int) * (vert_index+1));
						emd->cells->data[emd->cells->count].vert_indexes[vert_index] = vert_index;

						CustomData_to_bmesh_block(&boolresult->vertData, &bm->vdata, v, &vert->head.data , 0);
						
						vert_index++;
						//vert_index_global++;
					}

					for (e = 0; e < totedge; e++)
					{
						BMEdge* edg;
						edg = BM_edge_create(bm, localverts[ed[e].v1], localverts[ed[e].v2], NULL, 0);
						/*if (BM_elem_index_get(edg) == -1) {
							BM_elem_index_set(edg, edg_index);
						}*/
						CustomData_to_bmesh_block(&boolresult->edgeData, &bm->edata, e, &edg->head.data , 0);
						localedges[e] = edg;
						edg_index++;
					}

					mp = boolresult->getPolyArray(boolresult);
					ml = boolresult->getLoopArray(boolresult);
					for (p = 0; p < totpoly; p++)
					{
						MLoop* lo;
						BMVert** ve = NULL;
						BMEdge** ed = NULL;
						BMLoop* loop;
						BMIter liter;
						int k = 0, l = (mp+p)->loopstart, t = (mp+p)->totloop;
						ve = MEM_mallocN(sizeof(BMVert*)*t, "poly->verts");
						ed = MEM_mallocN(sizeof(BMEdge*)*t, "poly->edges");

						for (k = 0; k < t; k++) {
							lo = ml+l+k;
							ve[k] = localverts[lo->v];
							ed[k] = localedges[lo->e];
						}

						face = BM_face_create(bm, ve, ed, t, 0);
						face->mat_nr = (mp+p)->mat_nr;
						/*if (BM_elem_index_get(face) == -1) {
							BM_elem_index_set(face, fac_index);
						}*/
						emd->cells->data[emd->cells->count].global_face_map = MEM_reallocN(emd->cells->data[emd->cells->count].global_face_map, sizeof(int) * (fac_index+1));
						emd->cells->data[emd->cells->count].global_face_map[fac_index] = global_face_index;
						emd->cells->data[emd->cells->count].face_count = fac_index+1;
						fac_index++;
						global_face_index++;

						if ((mp+p)->flag & ME_SMOOTH)
							BM_elem_flag_enable(face, BM_ELEM_SMOOTH);
						
						CustomData_to_bmesh_block(&boolresult->polyData, &bm->pdata, p, &face->head.data , 0);
						
						loop = BM_iter_new(&liter, bm, BM_LOOPS_OF_FACE, face);
						
						for (k = (mp+p)->loopstart; loop; loop = BM_iter_step(&liter), k++) {
							CustomData_to_bmesh_block(&boolresult->loopData, &bm->ldata, k, &loop->head.data, 0);
						}

						MEM_freeN(ve);
						MEM_freeN(ed);
					}

					MEM_freeN(localverts);
					MEM_freeN(localedges);
				}

				edge_index = 0;
				face_index = 0;
				break;
			}
		}

		//read centroid
		fscanf(fp, " %f %f %f ", &emd->cells->data[emd->cells->count].centroid[0],
				&emd->cells->data[emd->cells->count].centroid[1],
				&emd->cells->data[emd->cells->count].centroid[2]);
		
		invert_m4_m4(imat, ob->obmat);
		mul_m4_v3(imat, emd->cells->data[emd->cells->count].centroid);

		//read neighbor id list (its PER FACE !!!) -> store in face customdata ?
		// no its per cell (i hope)
		neighbor_index = 0;
		while (feof(fp) == 0) {
			c = fgetc(fp);
			if (c != 'n') // end mark, cant use isdigit because negative numbers possible -> -
			{
				int n_index;
				//seek back one char,
				fseek(fp, -sizeof(char), SEEK_CUR);
				emd->cells->data[emd->cells->count].neighbor_ids = MEM_reallocN(emd->cells->data[emd->cells->count].neighbor_ids, sizeof(int)*(neighbor_index+1));
				fscanf(fp, "%d ", &n_index);

				if (n_index < 0)//cell is at the boundary of object
				{
					emd->cells->data[emd->cells->count].is_at_boundary = TRUE;
				}
				emd->cells->data[emd->cells->count].neighbor_ids[neighbor_index] = n_index;
				neighbor_index++;
				emd->cells->data[emd->cells->count].neighbor_count++;
			}
			else
			{
				//fseek(fp, -sizeof(char), SEEK_CUR);
				break;
			}
		}
		
		//skip newline
		if (feof(fp) == 0)
		{

#ifdef _WIN32
			//skip \r\n
			fseek(fp, 2*sizeof(char), SEEK_CUR);
#else
			//skip \n
			fseek(fp, sizeof(char), SEEK_CUR);
#endif
			emd->cells->count++;
		}
		else
		{
			//EOF reached, free last vertco/vertices
			MEM_freeN(emd->cells->data[emd->cells->count].vertco);
			emd->cells->data[emd->cells->count].vertco = NULL;
			MEM_freeN(emd->cells->data[emd->cells->count].vertices);
			emd->cells->data[emd->cells->count].vertices = NULL;
			MEM_freeN(emd->cells->data[emd->cells->count].vert_indexes);
			emd->cells->data[emd->cells->count].vert_indexes = NULL;
			MEM_freeN(emd->cells->data[emd->cells->count].global_face_map);
			emd->cells->data[emd->cells->count].global_face_map = NULL;
			MEM_freeN(emd->cells->data[emd->cells->count].neighbor_ids);
			emd->cells->data[emd->cells->count].neighbor_ids = NULL;
		}

		vert_index = 0;
		edg_index = 0;
		fac_index = 0;

		MEM_freeN(tempvert);
		if (bmtemp) BM_mesh_free(bmtemp);
		MEM_freeN(faceverts);
	//	MEM_freeN(facevert_indexes);
		MEM_freeN(faceedges);

		MEM_freeN(real_indexes);
		len_real_indexes = 0;
	}

	fclose(fp);
	MEM_freeN(path);
	
	printf("%d cells missing\n", totpoint - emd->cells->count); //use totpoint here
	
	return bm;
}

static void createParticleTree(ExplodeModifierData *emd, ParticleSystemModifierData *psmd, Scene* scene, Object* ob)
{
	ParticleSimulationData sim = {NULL};
	ParticleSystem *psys = psmd->psys;
	ParticleData *pa;
	ParticleKey birth;
	int p = 0, totpart = 0;
	
	totpart = psys->totpart;
	sim.scene = scene;
	sim.ob = ob;
	sim.psys = psmd->psys;
	sim.psmd = psmd;

	/* make tree of emitter locations */
	if (emd->patree)
	{
		BLI_kdtree_free(emd->patree);
		emd->patree = NULL;
	}
	
	emd->patree = BLI_kdtree_new(totpart);
	for (p = 0, pa = psys->particles; p < totpart; p++, pa++)
	{
		if (emd->emit_continuously)
		{
			psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);
			BLI_kdtree_insert(emd->patree, p, birth.co, NULL);
		}
		else if (ELEM3(pa->alive, PARS_ALIVE, PARS_DYING, PARS_DEAD))
		{
			//psys_particle_on_emitter(psmd, psys->part->from, pa->num, pa->num_dmcache, pa->fuv, pa->foffset, co, NULL, NULL, NULL, NULL, NULL);
			psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);
			BLI_kdtree_insert(emd->patree, p, birth.co, NULL);
		}
	}
	
	BLI_kdtree_balance(emd->patree);
}


static void mapCellsToParticles(ExplodeModifierData *emd, ParticleSystemModifierData *psmd, Scene* scene, Object* ob)
{
	ParticleSystem *psys = psmd->psys;
	float center[3];
	int p = 0, c;
	// if voronoi: need to set centroids of cells to nearest particle, apply same(?) matrix to a group of verts
	float cfra;
	cfra = BKE_scene_frame_get(scene);

	for(c = 0; c < emd->cells->count; c++) {
		center[0] = emd->cells->data[c].centroid[0];
		center[1] = emd->cells->data[c].centroid[1];
		center[2] = emd->cells->data[c].centroid[2];

		//centroids were stored in object space, go to global space (particles are in global space)
		mul_m4_v3(ob->obmat, center);
		p = BLI_kdtree_find_nearest(emd->patree, center, NULL, NULL);

		if (emd->emit_continuously) {
			if (ELEM3(psys->particles[p].alive, PARS_ALIVE, PARS_DYING, PARS_DEAD)) {
				emd->cells->data[c].particle_index = p;
			}
			else {
				emd->cells->data[c].particle_index = -1;
			}
		}
		else {
			if ((emd->cells->data[c].particle_index == -1) && (cfra > (psys->part->sta + emd->map_delay))) {
				//map once, with delay, the larger the delay, the more smaller chunks !
				emd->cells->data[c].particle_index = p;
			}
		}
	}
}

static void explodeCells(ExplodeModifierData *emd,
						 ParticleSystemModifierData *psmd, Scene *scene, Object *ob)
{
	ParticleSimulationData sim = {NULL};
	ParticleData *pa = NULL, *pars = psmd->psys->particles;
	ParticleKey state, birth;
	float imat[4][4];
	float rot[4];
	int totpart = 0;
	int i, j, p;

	totpart = psmd->psys->totpart;

	sim.scene = scene;
	sim.ob = ob;
	sim.psys = psmd->psys;
	sim.psmd = psmd;

	if (emd->cells == NULL)
	{
		return;
	}

	/* getting back to object space */ // do i need this ?
	invert_m4_m4(imat, ob->obmat);
	psmd->psys->lattice = psys_get_lattice(&sim);

	//create a new mesh aka "explode" from new vertex positions. use bmesh for that now.

	//since we dont have real access to tessfaces we can deal with from our cells, need to recreate the edges
	//and faces now the bmesh way. Or ... simply move the verts ? and recreate the DerivedMesh.

	//but... need the Verts objects... hopefully global indexes are the same as in DerivedMesh, if not, need to deal with BMesh there too.

	for (i = 0; i < emd->cells->count; i++)
	{
		p = emd->cells->data[i].particle_index;
		if ((p >= 0) && (p < totpart))
		{
			pa = pars + p;
		}

		for (j = 0; j < emd->cells->data[i].vertex_count; j++)
		{
			BMVert* vert = NULL;

			if (emd->cells->data[i].vertices == NULL) return;

			vert = emd->cells->data[i].vertices[j];

			//reset to original coords // stored at fracture time
			vert->co[0] = emd->cells->data[i].vertco[j*3];
			vert->co[1] = emd->cells->data[i].vertco[j*3+1];
			vert->co[2] = emd->cells->data[i].vertco[j*3+2];
			
			if ((p < 0) || (p > totpart-1) || ((!emd->emit_continuously) && (pa->alive == PARS_UNBORN)))
			{
				continue;
			}
			
			//do recalc here ! what about uv maps and such.... ? let boolean (initially) handle this, but
			//i HOPE vertex movement will update the uvs as well... or bust

			//particle CACHE causes lots of problems with this kind of calculation.
			psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);
			// psys_particle_on_emitter(psmd, psmd->psys->part->from, pa->num, pa->num_dmcache, pa->fuv, pa->foffset, co, NULL, NULL, NULL, NULL, NULL);

			//birth.time = cfra;
			//psys_get_particle_state(&sim, p, &birth, 1);
			//birth = pa->prev_state;
			state = pa->state;

			//vertco = vert->co;
			mul_m4_v3(ob->obmat, vert->co);
			sub_v3_v3(vert->co, birth.co);

			/* apply rotation, size & location */
			//only if defined, means if checked in psys
			if (psmd->psys->part->flag & PART_ROTATIONS)
			{
				sub_qt_qtqt(rot, state.rot, birth.rot);
				mul_qt_v3(rot, vert->co);
			}

			//TODO: maybe apply size flag, alive / unborn / dead flags
			//  if (emd->flag & eExplodeFlag_PaSize)
			//      mul_v3_fl(vert->co, pa->size);

			add_v3_v3(vert->co, state.co);

			mul_m4_v3(imat, vert->co);

		}
	}

	if (psmd->psys->lattice) {
		end_latt_deform(psmd->psys->lattice);
		psmd->psys->lattice = NULL;
	}

	//return bm;
}

static void resetCells(ExplodeModifierData *emd)
{
	int c;
	
	for (c = 0; c < emd->cells->count; c++)
	{
		emd->cells->data[c].particle_index = -1;
	}
}


static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
									DerivedMesh *derivedData,
									ModifierApplyFlag UNUSED(flag))
{
	DerivedMesh *dm = derivedData;
	ExplodeModifierData *emd = (ExplodeModifierData *) md;
	ParticleSystemModifierData *psmd = findPrecedingParticlesystem(ob, md);
	RigidBodyModifierData* rmd = findFollowingRigidBodyModifier(ob, md);

	DerivedMesh *result = NULL, *d = NULL;
	int i = 0, j = 0, f, f_index = 0, ml_index = 0;
	MTFace* mtface = NULL;
	MTFace* mtf = NULL;
	MTFace tf;
	//MTFaces not used in BMesh/further modifier processing , so need to split to MTexPoly/MLoopUV
	MTexPoly* mtps,  mtp;
	MLoopUV* mluvs, mluv;
	MLoop* mla = NULL, *ml;
	MPoly* mpa = NULL, *mp;
	float imat[4][4], oldobmat[4][4];

	//if (psmd)
	{
		if (emd->mode == eFractureMode_Cells)
		{
#ifdef WITH_MOD_VORONOI

			if ((emd->cells == NULL) ||
					(psmd && (emd->last_part != psmd->psys->totpart)) ||
					(emd->last_bool != emd->use_boolean) ||
					//(emd->last_flip != emd->flip_normal) ||
					(emd->last_point_source != emd->point_source) ||
					(emd->use_cache == FALSE))
			{
				invert_m4_m4(imat, ob->obmat);
				copy_m4_m4(oldobmat, ob->obmat);
				mult_m4_m4m4(ob->obmat, imat, ob->obmat); //neutralize obmat due to rotation problem with container
				
				if ((emd->cells) && (emd->fracMesh)) {
					BM_mesh_free(emd->fracMesh);
					emd->fracMesh = NULL;
				}
				emd->fracMesh = fractureToCells(ob, derivedData, emd, oldobmat);
				BKE_submesh_free(emd->storage); // in case this is not the first call;
				emd->storage = NULL;

				if (emd->fracMesh != NULL) {
					BMO_op_callf(emd->fracMesh,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "recalc_face_normals faces=%af use_face_tag=%b", BM_FACES_OF_MESH, false);
					BM_mesh_normals_update(emd->fracMesh);
					BMO_op_callf(emd->fracMesh,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "dissolve_limit edges=%ae verts=%av angle_limit=%f use_dissolve_boundaries=%b",
				                   BM_EDGES_OF_MESH, BM_VERTS_OF_MESH, 0.087f, false);
				//	emd->storage = BKE_bmesh_to_submesh(emd->fracMesh);
				}
				
				copy_m4_m4(ob->obmat, oldobmat); // restore obmat

				if (psmd != NULL)
					emd->last_part = psmd->psys->totpart;
				emd->last_bool = emd->use_boolean;
				emd->last_point_source = emd->point_source;

				//trigger refresh of possible following rmd too
				if (rmd)
					rmd->refresh = TRUE;
			}

			if (emd->cells == NULL)
				return derivedData;

			if (emd->use_animation && psmd != NULL)
			{
				if (emd->map_delay != emd->last_map_delay) resetCells(emd);
				emd->last_map_delay = emd->map_delay;
				createParticleTree(emd, psmd, md->scene, ob);
				mapCellsToParticles(emd, psmd, md->scene, ob);
				explodeCells(emd, psmd, md->scene, ob);
			}

			if (emd->fracMesh) {
				result = CDDM_from_bmesh(emd->fracMesh, TRUE);
			}
			else {
				result = derivedData;
				return result;
			}

			if (emd->use_boolean && !emd->use_cache)
			{
				DM_ensure_tessface(result);
				CDDM_calc_edges_tessface(result);
				CDDM_tessfaces_to_faces(result);
				CDDM_calc_normals(result);

				mtface = MEM_mallocN(sizeof(MTFace), "mtface");
				mtps = MEM_mallocN(sizeof(MTexPoly), "mtps");
				mluvs = MEM_mallocN(sizeof(MLoopUV), "mluvs");

				f_index = 0;
				ml_index = 0;

				for (i = 0; i < emd->cells->count; i++)
				{
					d = emd->cells->data[i].cell_mesh;

					DM_ensure_tessface(d);
					CDDM_calc_edges_tessface(d);
					CDDM_tessfaces_to_faces(d);
					CDDM_calc_normals(d);

					//hope this merges the MTFace data... successfully...
					mtf = DM_get_tessface_data_layer(d, CD_MTFACE);
					if (!mtf)
					{
						//argh, something went wrong, data will be missing...
						MEM_freeN(mtface);
						MEM_freeN(mtps);
						MEM_freeN(mluvs);
						mtface = NULL;
						break;
					}

					mpa = d->getPolyArray(d);
					mla = d->getLoopArray(d);

					for (f = 0; f < d->numTessFaceData; f++)
					{
						mtface = MEM_reallocN(mtface, sizeof(MTFace) * (f_index+1));
						tf = mtf[f];

						mtps = MEM_reallocN(mtps, sizeof(MTexPoly) * (f_index+1));
						mtp.tpage = tf.tpage;
						mtp.flag = tf.flag;
						mtp.mode = tf.mode;
						mtp.tile = tf.tile;
						mtp.transp = tf.transp;
						mtps[f_index] = mtp;

						//assume facecount = polycount and faces = polys since only tris and quads available
						//because of boolean

						mp = mpa + f; // get Polygon according to face index
						mluvs = MEM_reallocN(mluvs, sizeof(MLoopUV)* (mp->totloop + ml_index));

						for (j = mp->loopstart; j < mp->loopstart + mp->totloop; j++)
						{
							//assume vertindex = uvindex, sigh, is that so ?, and no more than 4 loops !
							ml = mla + j;
							mluv.uv[0] = tf.uv[j-mp->loopstart][0];
							mluv.uv[1] = tf.uv[j-mp->loopstart][1];
							mluvs[ml_index] = mluv;
							ml_index++;
						}

						mtface[f_index] = tf;
						f_index++;
					}
				}

				if (mtface)
				{
					CustomData *fdata, *pdata, *ldata;
					fdata = &result->faceData;
					ldata = &result->loopData;
					pdata = &result->polyData;

					CustomData_add_layer(fdata, CD_MTFACE , CD_DUPLICATE, mtface, f_index);
					CustomData_add_layer(pdata, CD_MTEXPOLY, CD_DUPLICATE, mtps, f_index);
					CustomData_add_layer(ldata, CD_MLOOPUV, CD_DUPLICATE, mluvs, ml_index);

					MEM_freeN(mtface);
					MEM_freeN(mtps);
					MEM_freeN(mluvs);
				}
			}
			emd->use_cache = MOD_VORONOI_USECACHE;
			return result;
#else
			emd->mode = eFractureMode_Faces;
			return derivedData;
#endif
		}


		else if (emd->mode == eFractureMode_Faces)
		{
			ParticleSystem *psys;
			if (psmd == NULL) return derivedData;

			psys = psmd->psys;
			DM_ensure_tessface(dm); /* BMESH - UNTIL MODIFIER IS UPDATED FOR MPoly */

			if (psys == NULL || psys->totpart == 0) return derivedData;
			if (psys->part == NULL || psys->particles == NULL) return derivedData;
			if (psmd->dm == NULL) return derivedData;

			/* 1. find faces to be exploded if needed */
			if (emd->facepa == NULL ||
					psmd->flag & eParticleSystemFlag_Pars ||
					emd->flag & eExplodeFlag_CalcFaces ||
					MEM_allocN_len(emd->facepa) / sizeof(int) != dm->getNumTessFaces(dm))
			{
				if (psmd->flag & eParticleSystemFlag_Pars)
					psmd->flag &= ~eParticleSystemFlag_Pars;

				if (emd->flag & eExplodeFlag_CalcFaces)
					emd->flag &= ~eExplodeFlag_CalcFaces;

				createFacepa(emd, psmd, derivedData);
			}
			/* 2. create new mesh */
			if (emd->flag & eExplodeFlag_EdgeCut) {
				int *facepa = emd->facepa;
				DerivedMesh *splitdm = cutEdges(emd, dm);
				DerivedMesh *explode = explodeMesh(emd, psmd, md->scene, ob, splitdm);

				MEM_freeN(emd->facepa);
				emd->facepa = facepa;
				splitdm->release(splitdm);
				return explode;
			}
			else
				return explodeMesh(emd, psmd, md->scene, ob, derivedData);
		}
	}
	return derivedData;
}

static void foreachIDLink(ModifierData *md, Object *ob,
						  IDWalkFunc walk, void *userData)
{
	ExplodeModifierData *emd = (ExplodeModifierData *) md;
	
	walk(userData, ob, (ID **)&emd->inner_material);
	walk(userData, ob, (ID **)&emd->extra_group);
}


ModifierTypeInfo modifierType_Explode = {
	/* name */              "Explode",
	/* structName */        "ExplodeModifierData",
	/* structSize */        sizeof(ExplodeModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
							eModifierTypeFlag_Single,	//more modifiers dont really make sense
	/* copyData */          copyData,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  requiredDataMask,
	/* freeData */          freeData,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* dependsOnTime */     dependsOnTime,
	/* dependsOnNormals */  NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     foreachIDLink,
	/* foreachTexLink */    NULL,
};
