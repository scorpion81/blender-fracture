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
 * Copyright (C) 2014 by Martin Felke.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/intern/fracture_util.c
 *  \ingroup blenkernel
 *  \brief CSG operations
 */

#include "BKE_cdderivedmesh.h"
#include "BKE_deform.h"
#include "BKE_editmesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_material.h"
#include "BKE_modifier.h"
#include "BKE_object.h"

#include "BLI_alloca.h"
#include "BLI_boxpack2d.h"
#include "BLI_convexhull2d.h"
#include "BLI_ghash.h"
#include "BLI_math.h"
#include "BLI_rand.h"
#include "BLI_sys_types.h"
#include "BLI_kdtree.h"

#include "DNA_fracture_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_material_types.h"
#include "DNA_modifier_types.h"

#include "MEM_guardedalloc.h"

#include "bmesh.h"
#include "bmesh_tools.h"
#include "../../modifiers/intern/MOD_boolean_util.h"

/*prototypes*/
void uv_bbox(float uv[][2], int num_uv, float minv[2], float maxv[2]);
void uv_translate(float uv[][2], int num_uv, float trans[2]);
void uv_scale(float uv[][2], int num_uv, float scale);
void uv_transform(float uv[][2], int num_uv, float mat[2][2]);
void unwrap_shard_dm(DerivedMesh *dm, char uv_layer[], bool do_boxpack);

/* UV Helpers */
void uv_bbox(float uv[][2], int num_uv, float minv[2], float maxv[2])
{
	int v;
	INIT_MINMAX2(minv, maxv);

	for (v = 0; v < num_uv; v++) {
		minmax_v2v2_v2(minv, maxv, uv[v]);
	}
}

void uv_translate(float uv[][2], int num_uv, float trans[2])
{
	int v;
	for (v = 0; v < num_uv; v++) {
		uv[v][0] += trans[0];
		uv[v][1] += trans[1];
	}
}

void uv_scale(float uv[][2], int num_uv, float scale)
{
	int v;
	for (v = 0; v < num_uv; v++) {
		uv[v][0] *= scale;
		uv[v][1] *= scale;
	}
}

void uv_transform(float uv[][2], int num_uv, float mat[2][2])
{
	int v;
	for (v = 0; v < num_uv; v++) {
		mul_m2v2(mat, uv[v]);
	}
}

static void do_clean_uv(DerivedMesh *dm, char uv_layer[64])
{
	MLoopUV* mluv = CustomData_get_layer_named(&dm->loopData, CD_MLOOPUV, uv_layer);
	int i, totpoly = dm->getNumPolys(dm);
	MPoly *mp, *mpoly = dm->getPolyArray(dm);

	if (mluv)
	{
		for (i = 0, mp = mpoly; i < totpoly; i++, mp++)
		{
			if (mp->mat_nr != 1)
			{	//clean up (set uv coords to zero) all except inner faces (material based)
				int j;
				for (j = mp->loopstart; j < mp->loopstart + mp->totloop; j++)
				{
					mluv[j].uv[0] = 0.0f;
					mluv[j].uv[1] = 0.0f;
				}
			}
		}
	}
}

static void do_unwrap(MPoly *mp, MVert *mvert, MLoop* mloop, int i, MLoopUV **mluv, BoxPack **boxpack)
{
	MLoop *ml;
	int j = 0;
	float (*verts)[3] = MEM_mallocN(sizeof(float[3]) * mp->totloop, "unwrap_shard_dm verts");
	float nor[3];
	float mat[3][3];
	float (*uv)[2] = MEM_mallocN(sizeof(float[2]) * mp->totloop, "unwrap_shard_dm_uv");
	BoxPack *box;
	float uvbbox[2][2];
	float angle;

	/* uv unwrap cells, so inner faces get a uv map */
	for (j = 0; j < mp->totloop; j++) {
		ml = mloop + mp->loopstart + j;
		copy_v3_v3(verts[j], (mvert + ml->v)->co);
	}

	normal_poly_v3(nor, (const float (*)[3])verts, mp->totloop);
	normalize_v3(nor);
	axis_dominant_v3_to_m3(mat, nor);

	for (j = 0; j < mp->totloop; j++) {
		mul_v2_m3v3(uv[j], mat, verts[j]);
	}

	/* rotate uvs for better packing */
	angle = BLI_convexhull_aabb_fit_points_2d((const float (*)[2])uv, mp->totloop);

	if (angle != 0.0f) {
		float matt[2][2];
		angle_to_mat2(matt, angle);
		uv_transform((float (*)[2])uv, mp->totloop, matt);
	}

	/* prepare box packing... one poly is a box */
	box = (*boxpack) + i;
	uv_bbox((float (*)[2])uv, mp->totloop, uvbbox[0], uvbbox[1]);

	uvbbox[0][0] = -uvbbox[0][0];
	uvbbox[0][1] = -uvbbox[0][1];

	uv_translate((float (*)[2])uv, mp->totloop, uvbbox[0]);

	box->w = uvbbox[1][0] + uvbbox[0][0];
	box->h = uvbbox[1][1] + uvbbox[0][1];
	box->index = i;

	/* copy coords back */
	for (j = 0; j < mp->totloop; j++) {
		copy_v2_v2((*mluv)[j + mp->loopstart].uv, uv[j]);
		(*mluv)[j + mp->loopstart].flag = 0;
	}

	MEM_freeN(uv);
	MEM_freeN(verts);
}

void unwrap_shard_dm(DerivedMesh *dm, char uv_layer[64], bool do_boxpack)
{
	MVert *mvert;
	MLoop *mloop;
	MPoly *mpoly, *mp;
	int totpoly, i = 0;
	MLoopUV *mluv = MEM_callocN(sizeof(MLoopUV) * dm->numLoopData, "mluv");
	BoxPack *boxpack = MEM_mallocN(sizeof(BoxPack) * dm->numPolyData, "boxpack");
	float scale, tot_width, tot_height;

	/* set inner material on child shard */
	mvert = dm->getVertArray(dm);
	mpoly = dm->getPolyArray(dm);
	mloop = dm->getLoopArray(dm);
	totpoly = dm->getNumPolys(dm);
	for (i = 0, mp = mpoly; i < totpoly; i++, mp++) {
		do_unwrap(mp, mvert, mloop, i, &mluv, &boxpack);
	}

	if (do_boxpack)
	{
		/* do box packing and match uvs according to it */
		BLI_box_pack_2d(boxpack, totpoly, &tot_width, &tot_height);

		if (tot_height > tot_width)
			scale = 1.0f / tot_height;
		else
			scale = 1.0f / tot_width;

		for (i = 0, mp = mpoly; i < totpoly; i++, mp++) {
			float trans[2];
			BoxPack *box;
			int j;

			box = boxpack + i;
			trans[0] = box->x;
			trans[1] = box->y;

			for (j = 0; j < mp->totloop; j++)
			{
				uv_translate((float (*)[2])mluv[j + mp->loopstart].uv, 1, trans);
				uv_scale((float (*)[2])mluv[j + mp->loopstart].uv, 1, scale);
			}
		}
	}

	MEM_freeN(boxpack);

	CustomData_add_layer_named(&dm->loopData, CD_MLOOPUV, CD_ASSIGN, mluv, dm->numLoopData, uv_layer);
	CustomData_add_layer_named(&dm->polyData, CD_MTEXPOLY, CD_CALLOC, NULL, totpoly, uv_layer);
}

static bool check_non_manifold(DerivedMesh* dm)
{
	BMesh *bm;
	BMVert* v;
	BMIter iter;
	BMEdge *e;

	/*check for watertightness*/
	bm = BM_mesh_create(&bm_mesh_allocsize_default, &((struct BMeshCreateParams){.use_toolflags = true,}));
	DM_to_bmesh_ex(dm, bm, true);

	if (bm->totface < 4) {
		BM_mesh_free(bm);
		printf("Empty mesh...\n");
		return true;
	}

	BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
		if (!BM_vert_is_manifold(v)) {
			BM_mesh_free(bm);
			printf("Mesh not watertight...\n");
			return true;
		}
	}

	BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
		if (BM_edge_is_wire(e) ||
			BM_edge_is_boundary(e) ||
			(BM_edge_is_manifold(e) && !BM_edge_is_contiguous(e)) ||
			BM_edge_face_count(e) > 2)
		{
			/* check we never select perfect edge (in test above) */
			BLI_assert(!(BM_edge_is_manifold(e) && BM_edge_is_contiguous(e)));
			BM_mesh_free(bm);
			printf("Mesh not watertight...\n");
			return true;
		}
	}

	BM_mesh_free(bm);
	return false;
}

static int DM_mesh_minmax(DerivedMesh *dm, float r_min[3], float r_max[3])
{
	MVert *v;
	int i = 0;
	for (i = 0; i < dm->numVertData; i++) {
		v = CDDM_get_vert(dm, i);
		minmax_v3v3_v3(r_min, r_max, v->co);
	}

	return (dm->numVertData != 0);
}

static bool compare_dm_size(DerivedMesh *dmOld, DerivedMesh *dmNew)
{
	float min[3], max[3];
	float size[3];
	float v1, v2;

	INIT_MINMAX(min, max);
	DM_mesh_minmax(dmOld, min, max);
	sub_v3_v3v3(size, max, min);

	v1 = size[0] * size[1] * size[2];

	INIT_MINMAX(min, max);
	DM_mesh_minmax(dmNew, min, max);
	sub_v3_v3v3(size, max, min);

	v2 = size[0] * size[1] * size[2];

	if (v2 > (v1 + 0.000001))
	{
		printf("Size mismatch !\n");
	}

	return v2 <= (v1 + 0.000001);
}

static bool do_other_output(DerivedMesh** other_dm, Shard** other, DerivedMesh** output_dm, DerivedMesh** left_dm, float mat[4][4])
{
	if (*other_dm)
	{
		*other = BKE_create_fracture_shard((*other_dm)->getVertArray(*other_dm),
											(*other_dm)->getPolyArray(*other_dm),
											(*other_dm)->getLoopArray(*other_dm),
											(*other_dm)->getEdgeArray(*other_dm),
											(*other_dm)->getNumVerts(*other_dm),
											(*other_dm)->getNumPolys(*other_dm),
											(*other_dm)->getNumLoops(*other_dm),
											(*other_dm)->getNumEdges(*other_dm),
											 true);

		*other = BKE_custom_data_to_shard(*other, *other_dm);

	#if 0
		/* XXX TODO this might be wrong by now ... */
		output_s->neighbor_count = child->neighbor_count;
		output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
		memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
	#endif
		BKE_fracture_shard_center_centroid_area(*other, (*other)->centroid);

		/* free the temp derivedmesh */
		(*other_dm)->needsFree = 1;
		(*other_dm)->release(*other_dm);
		*other_dm = NULL;
	}
	else
	{
		if (other != NULL)
			*other = NULL;
		if (*left_dm != NULL) {
			(*left_dm)->needsFree = 1;
			(*left_dm)->release(*left_dm);
			(*left_dm) = NULL;
		}
		if (*other_dm != NULL)
		{
			(*other_dm)->needsFree = 1;
			(*other_dm)->release(*other_dm);
			(*other_dm) = NULL;
		}

		/*discard only at fractal boolean */
		if (mat != NULL)
		{
			if (*output_dm != NULL) {
				(*output_dm)->needsFree = 1;
				(*output_dm)->release(*output_dm);
				(*output_dm) = NULL;
			}
			return true;
		}
	}

	return false;
}

static Shard *do_output_shard_dm(DerivedMesh** output_dm, Shard *child, int num_cuts, float fractal, Shard **other)
{
	Shard* output_s = BKE_create_fracture_shard((*output_dm)->getVertArray(*output_dm),
	                                     (*output_dm)->getPolyArray(*output_dm),
	                                     (*output_dm)->getLoopArray(*output_dm),
	                                     (*output_dm)->getEdgeArray(*output_dm),
	                                     (*output_dm)->getNumVerts(*output_dm),
	                                     (*output_dm)->getNumPolys(*output_dm),
	                                     (*output_dm)->getNumLoops(*output_dm),
	                                     (*output_dm)->getNumEdges(*output_dm),
	                                     true);

	output_s = BKE_custom_data_to_shard(output_s, *output_dm);

	/* useless, because its a bisect fast-like approach here */
	if (num_cuts == 0 || fractal == 0.0f || other == NULL) {
		/* XXX TODO this might be wrong by now ... */
		output_s->neighbor_count = child->neighbor_count;
		output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
		memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
		copy_v3_v3(output_s->raw_centroid, child->raw_centroid);
		output_s->raw_volume = child->raw_volume;
	}

	BKE_fracture_shard_center_centroid_area(output_s, output_s->centroid);

	/* free the temp derivedmesh */
	(*output_dm)->needsFree = 1;
	(*output_dm)->release(*output_dm);
	*output_dm = NULL;

	return output_s;
}

static DerivedMesh* do_fractal(float radius, float mat[4][4], bool use_smooth_inner, short inner_material_index,
                         int num_levels, int num_cuts, float fractal)
{
	BMFace* f;
	BMIter iter;
	BMesh *bm;
	int i;
	DerivedMesh *ret = NULL;

	/*create a grid plane */
	bm = BM_mesh_create(&bm_mesh_allocsize_default,  &((struct BMeshCreateParams){.use_toolflags = true,}));
	BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	        "create_grid x_segments=%i y_segments=%i size=%f matrix=%m4",
	        1, 1, radius, mat);

	/*subdivide the plane fractally*/
	for (i = 0; i < num_levels; i++)
	{
		BMO_op_callf(bm,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
					 "subdivide_edges edges=ae "
					 "smooth=%f smooth_falloff=%i use_smooth_even=%b "
					 "fractal=%f along_normal=%f "
					 "cuts=%i "
					 "quad_corner_type=%i "
					 "use_single_edge=%b use_grid_fill=%b "
					 "use_only_quads=%b "
					 "seed=%i",
					 0.0f, SUBD_FALLOFF_ROOT, false,
					 fractal, 1.0f,
					 num_cuts,
					 SUBD_CORNER_INNERVERT,
					 false, true,
					 true,
					 0);
	}

	BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	        "triangulate faces=af quad_method=%i ngon_method=%i", MOD_TRIANGULATE_QUAD_BEAUTY, MOD_TRIANGULATE_NGON_BEAUTY);

	BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	        "recalc_face_normals faces=af");

	BM_ITER_MESH(f, &iter, bm, BM_FACES_OF_MESH)
	{
		if (use_smooth_inner)
		{
			BM_elem_flag_enable(f, BM_ELEM_SMOOTH);
		}

		if (inner_material_index > 0)
		{
			f->mat_nr = inner_material_index;
		}
	}

	/*convert back*/
	ret = CDDM_from_bmesh(bm, false);
	BM_mesh_free(bm);

	return ret;
}

static bool do_check_watertight_other(DerivedMesh **other_dm, DerivedMesh **output_dm, Shard **other, DerivedMesh *right_dm,
                                      DerivedMesh **left_dm, float mat[4][4])
{
	bool do_return = false;

	if (!other_dm || check_non_manifold(*other_dm) || !compare_dm_size(right_dm, *other_dm)) {
		if (other != NULL)
			*other = NULL;
		if (*left_dm != NULL) {
			(*left_dm)->needsFree = 1;
			(*left_dm)->release(*left_dm);
			(*left_dm) = NULL;
		}
		if (*other_dm != NULL)
		{
			(*other_dm)->needsFree = 1;
			(*other_dm)->release(*other_dm);
			(*other_dm) = NULL;
		}

		/*discard only at fractal boolean */
		if (mat != NULL)
		{
			if (*output_dm != NULL) {
				(*output_dm)->needsFree = 1;
				(*output_dm)->release(*output_dm);
				(*output_dm) = NULL;
			}
			do_return = true;
		}
	}

	return do_return;
}

static bool do_check_watertight(DerivedMesh **output_dm, DerivedMesh** left_dm, DerivedMesh *right_dm, Shard **other, float mat[4][4])
{
	bool do_return = false;

	if (!(*output_dm) || check_non_manifold(*output_dm) || !compare_dm_size(right_dm, (*output_dm))) {
		if (mat != NULL)
		{
			if (other != NULL)
				*other = NULL;

			if (*left_dm != NULL) {
				(*left_dm)->needsFree = 1;
				(*left_dm)->release(*left_dm);
				*left_dm = NULL;
			}
		}

		if (*output_dm != NULL) {
			(*output_dm)->needsFree = 1;
			(*output_dm)->release(*output_dm);
			*output_dm = NULL;
		}

		if (mat != NULL)
		{
			do_return = true;
		}
	}

	return do_return;
}

static void do_set_inner_material(Shard **other, float mat[4][4], DerivedMesh* left_dm, short inner_material_index, Shard* s, Object *ob)
{
	MPoly *mpoly, *mp;
	int totpoly, i = 0;
	MDeformVert *dvert;
	int totvert = left_dm->getNumVerts(left_dm);
	FractureModifierData *fmd = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);

	/* set inner material on child shard */
	if (other == NULL || mat == NULL)
	{
		mpoly = left_dm->getPolyArray(left_dm);
		totpoly = left_dm->getNumPolys(left_dm);
		for (i = 0, mp = mpoly; i < totpoly; i++, mp++) {
			if (inner_material_index > 0) {
				mp->mat_nr = inner_material_index;
			}
			mp->flag |= ME_FACE_SEL;
			//set flag on shard too to have it available on load
			s->mpoly[i].flag |= ME_FACE_SEL;
		}
	}

	if (fmd && fmd->inner_defgrp_name[0]) {
		int defgrp = defgroup_name_index(ob, fmd->inner_defgrp_name);
		dvert = CustomData_add_layer(&left_dm->vertData, CD_MDEFORMVERT, CD_CALLOC, NULL, totvert);
		for (i = 0; i < totvert; i++) {
			defvert_add_index_notest(dvert + i, defgrp, 1.0f);
		}
	}
}

Shard *BKE_fracture_shard_boolean(Object *obj, DerivedMesh *dm_parent, Shard *child, short inner_material_index,
                                  int num_cuts, float fractal, Shard** other, float mat[4][4], float radius,
                                  bool use_smooth_inner, int num_levels, char uv_layer[64], int solver, float thresh)
{
	DerivedMesh *left_dm = NULL, *right_dm, *output_dm, *other_dm;
	if (other != NULL && mat != NULL)
	{
		left_dm = do_fractal(radius, mat, use_smooth_inner, inner_material_index, num_levels, num_cuts, fractal);
		unwrap_shard_dm(left_dm, uv_layer, false);
	}
	else
	{
		left_dm = BKE_shard_create_dm(child, false);
		unwrap_shard_dm(left_dm, uv_layer, true);
	}

	do_set_inner_material(other, mat, left_dm, inner_material_index, child, obj);

	right_dm = dm_parent;

	if (solver == eBooleanModifierSolver_Carve)
	{
		output_dm = NewBooleanDerivedMesh(right_dm, obj, left_dm, obj, 1); /*1 == intersection, 3 == difference*/
	}
	else {
		output_dm = NewBooleanDerivedMeshBMesh(right_dm, obj, left_dm, obj, 0, thresh, NULL); /*0 == intersection, 2 == difference*/
	}

	/*check for watertightness, but for fractal only*/
	if (other != NULL && do_check_watertight(&output_dm, &left_dm, right_dm, other, mat))
	{
		return NULL;
	}

	if (other != NULL)
	{
		if (solver == eBooleanModifierSolver_Carve)
		{
			other_dm = NewBooleanDerivedMesh(left_dm, obj, right_dm, obj, 3);
		}
		else {
			other_dm = NewBooleanDerivedMeshBMesh(left_dm, obj, right_dm, obj, 2, thresh, NULL);
		}

		/*check for watertightness again, true means do return NULL here*/
		if (!other_dm || do_check_watertight_other(&other_dm, &output_dm, other, right_dm, &left_dm, mat))
		{
			if (!other_dm) {
				/* in case of failed boolean op, clean up other dms too before returning NULL */
				if (left_dm) {
					left_dm->needsFree = 1;
					left_dm->release(left_dm);
					left_dm = NULL;
				}

				if (output_dm) {
					output_dm->needsFree = 1;
					output_dm->release(output_dm);
					output_dm = NULL;
				}
			}

			return NULL;
		}

		/*return here if this function returns true */
		if (do_other_output(&other_dm, other, &output_dm, &left_dm, mat))
		{
			return NULL;
		}
	}

	if (left_dm)
	{
		left_dm->needsFree = 1;
		left_dm->release(left_dm);
		left_dm = NULL;
	}

	if (output_dm)
	{
		do_clean_uv(output_dm, uv_layer);
		return do_output_shard_dm(&output_dm, child, num_cuts, fractal, other);
	}

	return NULL;
}

static Shard *do_output_shard(BMesh* bm_parent, Shard *child, char uv_layer[64])
{
	Shard *output_s = NULL;
	DerivedMesh *dm_out;

	if (bm_parent->totvert >= 3)
	{	/* atleast 3 verts form a face, so strip out invalid stuff */
		dm_out = CDDM_from_bmesh(bm_parent, true);

		//"cleanup" dm here, set UVs to 0,0 whose poly->mat_nr = 1 (i cant find where its originally created... grrr)
		do_clean_uv(dm_out, uv_layer);

		output_s = BKE_create_fracture_shard(dm_out->getVertArray(dm_out),
											 dm_out->getPolyArray(dm_out),
											 dm_out->getLoopArray(dm_out),
		                                     dm_out->getEdgeArray(dm_out),
											 dm_out->getNumVerts(dm_out),
											 dm_out->getNumPolys(dm_out),
											 dm_out->getNumLoops(dm_out),
											 dm_out->getNumEdges(dm_out), true);

		output_s = BKE_custom_data_to_shard(output_s, dm_out);

		/*XXX TODO this might be wrong by now ... */
		output_s->neighbor_count = child->neighbor_count;
		output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
		memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
		BKE_fracture_shard_center_centroid_area(output_s, output_s->centroid);
		copy_v3_v3(output_s->raw_centroid, child->raw_centroid);
		output_s->raw_volume = child->raw_volume;

		dm_out->needsFree = 1;
		dm_out->release(dm_out);
		dm_out = NULL;
	}

	return output_s;
}

static void do_fill(float plane_no[3], bool clear_outer, bool clear_inner, BMOperator bmop, short inner_mat_index, BMesh* bm_parent)
{
	float normal_fill[3];
	BMOperator bmop_fill;
	//BMOperator bmop_attr;

	normalize_v3_v3(normal_fill, plane_no);
	if (clear_outer == true && clear_inner == false) {
		negate_v3(normal_fill);
	}
/*
	if (inner_mat_index == 0) { // dont use inner material here
		BMO_op_initf(
		    bm_parent, &bmop_fill, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		    "triangle_fill edges=%S normal=%v use_dissolve=%b use_beauty=%b",
		    &bmop, "geom_cut.out", normal_fill, true, true);
		BMO_op_exec(bm_parent, &bmop_fill);

		BMO_op_initf(bm_parent, &bmop_attr, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		             "face_attribute_fill faces=%S use_normals=%b use_data=%b",
		             &bmop_fill, "geom.out", false, true);
		BMO_op_exec(bm_parent, &bmop_attr);
		BMO_op_finish(bm_parent, &bmop_attr);

		BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "geom.out", BM_FACE, BM_ELEM_TAG | BM_ELEM_SELECT, true);
	}
	else { */
		/* use edgenet fill with inner material */
		BMO_op_initf(
		    bm_parent, &bmop_fill, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		    "edgenet_fill edges=%S mat_nr=%i use_smooth=%b sides=%i",
		    &bmop, "geom_cut.out", inner_mat_index, false, 2);
		BMO_op_exec(bm_parent, &bmop_fill);
	//}

	/*gah, edgenetfill now overwrites the set materials internally, so attempt to re-set them */
	BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "faces.out", BM_FACE, BM_ELEM_TAG, true);
	{
		BMFace* f;
		BMIter iter;
		BM_ITER_MESH(f, &iter, bm_parent, BM_FACES_OF_MESH) {
			if (BM_elem_flag_test(f, BM_ELEM_TAG)) {
				f->mat_nr = inner_mat_index;
			}
		}
	}

	BMO_op_finish(bm_parent, &bmop_fill);
}

static void do_bisect(BMesh* bm_parent, BMesh* bm_child, float obmat[4][4], bool use_fill, bool clear_inner,
               bool clear_outer, int cutlimit, float centroid[3], short inner_mat_index, float normal[3])
{
	BMIter iter;
	BMFace *f;

	BMOperator bmop;
	float plane_co[3];
	float plane_no[3];
	float imat[4][4];

	float thresh = 0.00001f;
	bool do_break = false;

	int cut_index = 0;
	invert_m4_m4(imat, obmat);

	BM_ITER_MESH_INDEX (f, &iter, bm_child, BM_FACES_OF_MESH, cut_index)
	{
		if (do_break) {
			break;
		}

		if (cutlimit > -1) {
			//f = BM_face_at_index_find(bm_child, cutlimit);
			copy_v3_v3(plane_co, centroid);
			copy_v3_v3(plane_no, normal); //f->no /*normal*/);
			do_break = true;

			//mul_qt_v3(quat, plane_no);
		}
		else {
			copy_v3_v3(plane_co, f->l_first->v->co);
			copy_v3_v3(plane_no, f->no);
		}

		mul_m4_v3(imat, plane_co);
		mul_mat3_m4_v3(imat, plane_no);

		BM_mesh_elem_hflag_enable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);

		BMO_op_initf(bm_parent, &bmop, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		             "bisect_plane geom=%hvef dist=%f plane_co=%v plane_no=%v use_snap_center=%b clear_inner=%b clear_outer=%b",
		             BM_ELEM_TAG, thresh, plane_co, plane_no, false, clear_inner, clear_outer);
		BMO_op_exec(bm_parent, &bmop);

		BM_mesh_elem_hflag_disable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);

		if (use_fill) {
			do_fill(plane_no, clear_outer, clear_inner, bmop, inner_mat_index, bm_parent);
		}

		BMO_slot_buffer_hflag_enable(bm_parent, bmop.slots_out, "geom_cut.out", BM_VERT | BM_EDGE, BM_ELEM_TAG, true);

		BMO_op_finish(bm_parent, &bmop);
	}
}

static BMesh *do_preselection(BMesh* bm_orig, Shard *child, KDTree *preselect_tree)
{
	int i = 0, r = 0;
	float max_dist = 0;
	KDTreeNearest* n = NULL;
	BMesh *bm_new = BM_mesh_create(&bm_mesh_allocsize_default, &((struct BMeshCreateParams){.use_toolflags = true,}));
	BMIter iter;
	BMFace *f;
#define MY_TAG (1 << 6)

	BM_mesh_elem_toolflags_ensure(bm_new);  /* needed for 'duplicate' bmo */

	CustomData_copy(&bm_orig->vdata, &bm_new->vdata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_orig->edata, &bm_new->edata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_orig->ldata, &bm_new->ldata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_orig->pdata, &bm_new->pdata, CD_MASK_BMESH, CD_CALLOC, 0);

	CustomData_bmesh_init_pool(&bm_new->vdata, bm_mesh_allocsize_default.totvert, BM_VERT);
	CustomData_bmesh_init_pool(&bm_new->edata, bm_mesh_allocsize_default.totedge, BM_EDGE);
	CustomData_bmesh_init_pool(&bm_new->ldata, bm_mesh_allocsize_default.totloop, BM_LOOP);
	CustomData_bmesh_init_pool(&bm_new->pdata, bm_mesh_allocsize_default.totface, BM_FACE);

	for (i = 0; i < child->totvert; i++)
	{
		float dist = len_squared_v3v3(child->raw_centroid, child->mvert[i].co);
		if (dist > max_dist) {
			max_dist = dist;
		}
	}

	BM_mesh_elem_hflag_disable_all(bm_orig, BM_VERT | BM_EDGE | BM_FACE, MY_TAG, false);

	//do a range search first in case we have many verts as in dense geometry
	r = BLI_kdtree_range_search(preselect_tree, child->raw_centroid, &n, sqrt(max_dist)*1.5f);

	//skip empty cells
	/*if (r == 0) {
		BM_mesh_free(bm_new);
		return NULL;
	}*/

	//if we have sparse geometry, just return all
	if (r < child->totvert) {

		//BM_mesh_free(bm_new);
		//return BM_mesh_copy(bm_orig);
		int j = 0, s = 0;
		KDTreeNearest *n2 = MEM_callocN(sizeof(KDTreeNearest) * child->totvert, "n2 kdtreenearest");
		s = BLI_kdtree_find_nearest_n(preselect_tree, child->raw_centroid, n2, child->totvert);
		for (j = 0; j < s; j++)
		{
			BMVert* v;
			int index = n2[j].index;
			v = BM_vert_at_index(bm_orig, index);
			BM_elem_flag_enable(v, MY_TAG);
		}
		if (n2) {
			MEM_freeN(n2);
			n2 = NULL;
		}
	}
	else {
		BMVert *v = NULL;
		for (i = 0; i < r; i++) {
			int index = n[i].index;
			v = BM_vert_at_index(bm_orig, index);
			BM_elem_flag_enable(v, MY_TAG);
		}	
	}

	BM_ITER_MESH(f, &iter, bm_orig, BM_FACES_OF_MESH) {
		//select bigger squarish faces and long skinny ones
		float area = BM_face_calc_area(f);
		BMIter eiter;
		BMEdge *e;
		BM_ITER_ELEM(e, &eiter, f, BM_EDGES_OF_FACE) {
			if ((area > 0.75f) || (BM_edge_calc_length_squared(e) > max_dist * 3.0f) ||
				BM_elem_flag_test(e->v1, MY_TAG) || BM_elem_flag_test(e->v2, MY_TAG))
			{
				BM_elem_flag_enable(e->v1, MY_TAG);
				BM_elem_flag_enable(e->v2, MY_TAG);
			}
		}
	}

	/* Flush the selection to get edge/face selections matching
	 * the vertex selection */
	BKE_bm_mesh_hflag_flush_vert(bm_orig, MY_TAG);

	BMO_op_callf(bm_orig, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	             "duplicate geom=%hvef dest=%p", MY_TAG, bm_new);
#undef MY_TAG
	BM_mesh_elem_index_ensure(bm_new, BM_VERT | BM_EDGE | BM_FACE);

	return bm_new;
}


Shard *BKE_fracture_shard_bisect(BMesh *bm_orig, Shard *child, float obmat[4][4], bool use_fill, bool clear_inner,
                                 bool clear_outer, int cutlimit, float centroid[3], short inner_mat_index, char uv_layer[64],
                                 KDTree *preselect_tree, float normal[3])
{

	Shard *output_s;
	DerivedMesh *dm_child = BKE_shard_create_dm(child, false);

	BMesh *bm_parent;
	BMesh *bm_child;

	unwrap_shard_dm(dm_child, uv_layer, true);

	bm_child = BM_mesh_create(&bm_mesh_allocsize_default,  &((struct BMeshCreateParams){.use_toolflags = true,}));
	DM_to_bmesh_ex(dm_child, bm_child, true);

	//hmmm need to copy only preselection !!! or rebuild bm_parent from selected data only....
	if (preselect_tree != NULL) {
		bm_parent = do_preselection(bm_orig, child, preselect_tree);
	}
	else {
		bm_parent = BM_mesh_copy(bm_orig);
	}

	if (bm_parent != NULL) {
		BM_mesh_elem_hflag_enable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
		do_bisect(bm_parent, bm_child, obmat, use_fill, clear_inner, clear_outer, cutlimit, centroid, inner_mat_index, normal);
		output_s = do_output_shard(bm_parent, child, uv_layer);
		BM_mesh_free(bm_parent);
	}
	else {
		output_s = NULL;
	}

	BM_mesh_free(bm_child);

	dm_child->needsFree = 1;
	dm_child->release(dm_child);
	dm_child = NULL;

	return output_s;
}
