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
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * Contributor(s): Blender Foundation
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/intern/submesh.c
 *  \ingroup bke
 */

#include "MEM_guardedalloc.h"
#include "BKE_submesh.h"
#include "BKE_customdata.h"
#include "BLI_math_vector.h"

/*BMesh* BKE_submesh_to_bmesh(SMesh* sm)
{
	int i, j;

	BMesh* bm = BM_mesh_create(&bm_mesh_allocsize_default);
	BM_mesh_elem_toolflags_ensure(bm);

	CustomData_copy(&sm->vdata, &bm->vdata, CD_MASK_BMESH, CD_CALLOC, sm->totvert);
	CustomData_copy(&sm->edata, &bm->edata, CD_MASK_BMESH, CD_CALLOC, sm->totedge);
	CustomData_copy(&sm->ldata, &bm->ldata, CD_MASK_BMESH, CD_CALLOC, sm->totloop);
	CustomData_copy(&sm->pdata, &bm->pdata, CD_MASK_BMESH, CD_CALLOC, sm->totface);

	CustomData_bmesh_init_pool(&bm->vdata, sm->totvert, BM_VERT);
	CustomData_bmesh_init_pool(&bm->edata, sm->totedge, BM_EDGE);
	CustomData_bmesh_init_pool(&bm->ldata, sm->totloop, BM_LOOP);
	CustomData_bmesh_init_pool(&bm->pdata, sm->totface, BM_FACE);


	for (i = 0; i < sm->totvert; i++)
	{
		SMVert sv = sm->vpool[i];
		BMVert* v = BM_vert_create(bm, sv.co, NULL, BM_CREATE_SKIP_CD);
		CustomData_to_bmesh_block(&sm->vdata, &bm->vdata, i, &v->head.data, false);
	}

	for (i = 0; i < sm->totedge; i++)
	{
		SMEdge se = sm->epool[i];
		BMVert* v1 = BM_vert_at_index(bm, se.v1);
		BMVert* v2 = BM_vert_at_index(bm, se.v2);
		BMEdge* e = BM_edge_create(bm, v1, v2, NULL, BM_CREATE_SKIP_CD);
		CustomData_to_bmesh_block(&sm->edata, &bm->edata, i, &e->head.data, true);
	}

	for (i = 0; i < sm->totface; i++)
	{
		BMFace* f = NULL;
		BMVert** ve = MEM_mallocN(sizeof(BMVert*), "read_submesh->ve");
		BMEdge** ed = MEM_mallocN(sizeof(BMEdge*), "read_submesh->ed");
		SMFace sf = sm->fpool[i];
		BMIter iter;
		int k = 0;
		BMLoop* l;

		for (j = sf.l_first; j < sf.l_first + sf.len; j++)
		{
			SMLoop sl = sm->lpool[j];
			BMVert* v = BM_vert_at_index(bm, sl.v);
			BMEdge* e = BM_edge_at_index(bm, sl.e);
			ed = MEM_reallocN(ed, sizeof(BMEdge*) * (j-sf.l_first+1));
			ed[j-sf.l_first] = e;

			ve = MEM_reallocN(ve, sizeof(BMVert*) * (j-sf.l_first+1));
			ve[j-sf.l_first] = v;
		}

		f = BM_face_create(bm, ve, ed, sf.len, BM_CREATE_SKIP_CD);
		CustomData_to_bmesh_block(&sm->pdata, &bm->pdata, i, &f->head.data, true);

		BM_ITER_ELEM(l, &iter, f, BM_LOOPS_OF_FACE) {
			CustomData_to_bmesh_block(&sm->ldata, &bm->ldata, k, &l->head.data, true);
			k++;
		}

		MEM_freeN(ve);
		MEM_freeN(ed);
	}

	return bm;
}

static SMesh* submesh_create() {
	SMesh* sm = MEM_mallocN(sizeof(SMesh), "sm");
	sm->vpool = MEM_mallocN(sizeof(SMVert), "sm->vpool");
	sm->epool = MEM_mallocN(sizeof(SMEdge), "sm->epool");
	sm->lpool = MEM_mallocN(sizeof(SMLoop), "sm->lpool");
	sm->fpool = MEM_mallocN(sizeof(SMFace), "sm->fpool");

	sm->totvert = 0;
	sm->totedge = 0;
	sm->totloop = 0;
	sm->totface = 0;

	return sm;
}

static SMFace smface_from_bmface(BMFace* f)
{
	SMFace sf;
	sf.head.index = f->head.index;
	sf.head.data = NULL;
	sf.l_first = f->l_first->head.index;
	sf.len = f->len;
	sf.mat_nr = f->mat_nr;
	copy_v3_v3(sf.no, f->no);

	return sf;
}

static SMLoop smloop_from_bmloop(BMLoop* l)
{
	SMLoop sl;
	sl.head.index = l->head.index;
	sl.head.data = NULL;
	sl.e = l->e->head.index;
	sl.v = l->v->head.index;
	sl.f = l->f->head.index;

	return sl;
}

static SMEdge smedge_from_bmedge(BMEdge* e)
{
	SMEdge se;
	se.v1 = e->v1->head.index;
	se.v2 = e->v2->head.index;
	if (e->l)
		se.l = e->l->head.index;
	else
		se.l = -1;
	se.head.index = e->head.index;
	se.head.data = NULL;
	return se;
}

static SMVert smvert_from_bmvert(BMVert* v) {
	SMVert sv;
	sv.head.index = v->head.index;
	sv.head.data = NULL;
	copy_v3_v3(sv.co, v->co);
	copy_v3_v3(sv.no, v->no);
	if (v->e)
		sv.e = v->e->head.index;
	else
		sv.e = -1;
	return sv;
}

void BKE_submesh_free(SMesh* sm)
{
	if (sm == NULL) {
		return;
	}

	if (sm->vpool != NULL) {
		MEM_freeN(sm->vpool);
		sm->vpool = NULL;
	}

	if (sm->epool != NULL) {
		MEM_freeN(sm->epool);
		sm->epool = NULL;
	}

	if (sm->lpool != NULL) {
		MEM_freeN(sm->lpool);
		sm->lpool = NULL;
	}

	if (sm->fpool != NULL) {
		MEM_freeN(sm->fpool);
		sm->fpool = NULL;
	}

	CustomData_free(&sm->vdata, sm->totvert);
	CustomData_free(&sm->edata, sm->totedge);
	CustomData_free(&sm->ldata, sm->totloop);
	CustomData_free(&sm->pdata, sm->totface);


	MEM_freeN(sm);
	sm = NULL;
}

SMesh* BKE_bmesh_to_submesh(BMesh* bm)
{
	BMIter iter, iter2;
	BMVert* v;
	BMEdge* e;
	BMLoop* l;
	BMFace* f;
	SMesh* sm = submesh_create();
	int i = 0, j = 0;

	CustomData_copy(&bm->vdata, &sm->vdata, CD_MASK_BMESH, CD_CALLOC, bm->totvert);
	CustomData_copy(&bm->edata, &sm->edata, CD_MASK_BMESH, CD_CALLOC, bm->totedge);
	CustomData_copy(&bm->ldata, &sm->ldata, CD_MASK_BMESH, CD_CALLOC, bm->totloop);
	CustomData_copy(&bm->pdata, &sm->pdata, CD_MASK_BMESH, CD_CALLOC, bm->totface);

	//first set indexes everywhere,
	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH)
	{
		BM_elem_index_set(v, i);
		i++;
	}

	i = 0;
	BM_ITER_MESH(e, &iter, bm, BM_EDGES_OF_MESH)
	{
		BM_elem_index_set(e, i);
		i++;
	}

	i = 0;
	BM_ITER_MESH(f, &iter, bm, BM_FACES_OF_MESH)
	{
		BM_elem_index_set(f, i);
		i++;


		BM_ITER_ELEM(l, &iter2, f, BM_LOOPS_OF_FACE)
		{
			BM_elem_index_set(l, j);
			j++;
		}
	}

	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH)
	{
		SMVert sv = smvert_from_bmvert(v);
		CustomData_from_bmesh_block(&bm->vdata, &sm->vdata, v->head.data, v->head.index);
		sm->vpool = MEM_reallocN(sm->vpool, sizeof(SMVert) * (sm->totvert+1));
		sm->vpool[sm->totvert] = sv;
		sm->totvert++;
	}


	BM_ITER_MESH(e, &iter, bm, BM_EDGES_OF_MESH)
	{
		SMEdge se = smedge_from_bmedge(e);
		CustomData_from_bmesh_block(&bm->edata, &sm->edata, e->head.data, e->head.index);
		sm->epool = MEM_reallocN(sm->epool, sizeof(SMEdge) * (sm->totedge+1));
		sm->epool[sm->totedge] = se;
		sm->totedge++;
	}

	BM_ITER_MESH(f, &iter, bm, BM_FACES_OF_MESH)
	{
		SMFace sf = smface_from_bmface(f);
		CustomData_from_bmesh_block(&bm->pdata, &sm->pdata, f->head.data, f->head.index);
		sm->fpool = MEM_reallocN(sm->fpool, sizeof(SMFace) * (sm->totface+1));
		sm->fpool[sm->totface] = sf;
		sm->totface++;

		BM_ITER_ELEM(l, &iter2, f, BM_LOOPS_OF_FACE)
		{
			SMLoop sl = smloop_from_bmloop(l);
			CustomData_from_bmesh_block(&bm->ldata, &sm->ldata, l->head.data, l->head.index);
			sm->lpool = MEM_reallocN(sm->lpool, sizeof(SMLoop) * (sm->totloop+1));
			sm->lpool[sm->totloop] = sl;
			sm->totloop++;
		}
	}

	return sm;
}*/
