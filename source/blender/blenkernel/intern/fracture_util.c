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
 * The Original Code is: all of this file.
 *
 * Contributor(s): Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/intern/fracture_util.c
 *  \ingroup blenkernel
 *  \brief CSG operations
 */

#include "BKE_cdderivedmesh.h"
#include "BKE_editmesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_material.h"

#include "BLI_alloca.h"
#include "BLI_boxpack2d.h"
#include "BLI_convexhull2d.h"
#include "BLI_ghash.h"
#include "BLI_math.h"
#include "BLI_rand.h"
#include "BLI_sys_types.h"

#include "DNA_fracture_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_material_types.h"

#include "MEM_guardedalloc.h"

#include "bmesh.h"
#include "../../modifiers/intern/MOD_boolean_util.h"

/*prototypes*/
void uv_bbox(float uv[][2], int num_uv, float minv[2], float maxv[2]);
void uv_translate(float uv[][2], int num_uv, float trans[2]);
void uv_scale(float uv[][2], int num_uv, float scale);
void uv_transform(float uv[][2], int num_uv, float mat[2][2]);
void unwrap_shard_dm(DerivedMesh *dm);

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

void unwrap_shard_dm(DerivedMesh *dm)
{
	MPoly *mpoly, *mp;
	MLoop *mloop;
	MVert *mvert;
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
			float mat[2][2];
			angle_to_mat2(mat, angle);
			uv_transform((float (*)[2])uv, mp->totloop, mat);
		}

		/* prepare box packing... one poly is a box */
		box = boxpack + i;
		uv_bbox((float (*)[2])uv, mp->totloop, uvbbox[0], uvbbox[1]);

		uvbbox[0][0] = -uvbbox[0][0];
		uvbbox[0][1] = -uvbbox[0][1];

		uv_translate((float (*)[2])uv, mp->totloop, uvbbox[0]);

		box->w = uvbbox[1][0] + uvbbox[0][0];
		box->h = uvbbox[1][1] + uvbbox[0][1];
		box->index = i;

		/* copy coords back */
		for (j = 0; j < mp->totloop; j++) {
			copy_v2_v2(mluv[j + mp->loopstart].uv, uv[j]);
			mluv[j + mp->loopstart].flag = 0;
		}

		MEM_freeN(uv);
		MEM_freeN(verts);
	}

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

	MEM_freeN(boxpack);

	CustomData_add_layer_named(&dm->loopData, CD_MLOOPUV, CD_ASSIGN, mluv, dm->numLoopData, "InnerUV");
	CustomData_add_layer_named(&dm->polyData, CD_MTEXPOLY, CD_CALLOC, NULL, totpoly, "InnerUV");
}

Shard *BKE_fracture_shard_boolean(Object *obj, DerivedMesh *dm_parent, Shard *child, short inner_material_index)
{
	Shard *output_s;
	DerivedMesh *left_dm, *right_dm, *output_dm;
	MPoly *mpoly, *mp;
	int totpoly, i = 0;

	left_dm = BKE_shard_create_dm(child, false);
	unwrap_shard_dm(left_dm);

	/* set inner material on child shard */
	mpoly = left_dm->getPolyArray(left_dm);
	totpoly = left_dm->getNumPolys(left_dm);
	for (i = 0, mp = mpoly; i < totpoly; i++, mp++) {
		if (inner_material_index > 0) {
			mp->mat_nr = inner_material_index;
		}
		mp->flag |= ME_FACE_SEL;
	}

	right_dm = dm_parent;
	output_dm = NewBooleanDerivedMesh(right_dm, obj, left_dm, obj, 1);

	left_dm->needsFree = 1;
	left_dm->release(left_dm);
	left_dm = NULL;

	if (output_dm)
	{
		output_s = BKE_create_fracture_shard(output_dm->getVertArray(output_dm),
		                                     output_dm->getPolyArray(output_dm),
		                                     output_dm->getLoopArray(output_dm),
		                                     output_dm->getNumVerts(output_dm),
		                                     output_dm->getNumPolys(output_dm),
		                                     output_dm->getNumLoops(output_dm),
		                                     true);

		output_s = BKE_custom_data_to_shard(output_s, output_dm);

		/* XXX TODO this might be wrong by now ... */
		output_s->neighbor_count = child->neighbor_count;
		output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
		memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
		BKE_fracture_shard_center_centroid(output_s, output_s->centroid);


		/* free the temp derivedmesh */
		output_dm->needsFree = 1;
		output_dm->release(output_dm);
		output_dm = NULL;

		return output_s;
	}

	return NULL;
}


Shard *BKE_fracture_shard_bisect(BMesh *bm_orig, Shard *child, float obmat[4][4], bool use_fill, bool clear_inner,
                                 bool clear_outer, int cutlimit, float centroid[3], short inner_mat_index)
{
	#define MYTAG (1 << 6)

	Shard *output_s;
	DerivedMesh *dm_child = BKE_shard_create_dm(child, false);
	DerivedMesh *dm_out;
	BMesh *bm_parent = BM_mesh_copy(bm_orig);
	BMesh *bm_child;
	BMIter iter;
	BMFace *f;

	BMOperator bmop;
	float plane_co[3];
	float plane_no[3];
	float imat[4][4];

	float thresh = 0.00001f;
	bool do_break = false;

	int cut_index = 0;

	unwrap_shard_dm(dm_child);
	bm_child = DM_to_bmesh(dm_child, true);

	invert_m4_m4(imat, obmat);

	BM_mesh_elem_hflag_enable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);

	BM_ITER_MESH_INDEX (f, &iter, bm_child, BM_FACES_OF_MESH, cut_index)
	{
		if (do_break) {
			break;
		}

		if (cutlimit > 0) {
			f = BM_face_at_index_find(bm_child, cutlimit);
			copy_v3_v3(plane_co, centroid);
			copy_v3_v3(plane_no, f->no /*normal*/);
			do_break = true;
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
			float normal_fill[3];
			BMOperator bmop_fill;
			BMOperator bmop_attr;

			normalize_v3_v3(normal_fill, plane_no);
			if (clear_outer == true && clear_inner == false) {
				negate_v3(normal_fill);
			}

			/* Fill, XXX attempted different fill algorithms here, needs further thoughts because none really suited */
#if 0
			BMO_op_initf(bm_parent, &bmop_fill, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			             "contextual_create geom=%S mat_nr=%i use_smooth=%b",
			             &bmop, "geom_cut.out", 0, false);
			BMO_op_exec(bm_parent, &bmop_fill);

			BMO_op_initf(bm_parent, &bmop_attr, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			             "face_attribute_fill faces=%S use_normals=%b use_data=%b",
			             &bmop_fill, "faces.out", false, true);
			BMO_op_exec(bm_parent, &bmop_attr);

			BMO_op_initf(bm_parent, &bmop_del, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			             "delete geom=%S context=%i", &bmop_fill, "edges.out", DEL_EDGESFACES);
			BMO_op_exec(bm_parent, &bmop_del);

			BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "faces.out", BM_FACE, BM_ELEM_TAG, true);
#endif

			if (inner_mat_index == 0) { /* dont use inner material here*/
				BMO_op_initf(
				    bm_parent, &bmop_fill, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				    "triangle_fill edges=%S normal=%v use_dissolve=%b use_beauty=%b",
				    &bmop, "geom_cut.out", normal_fill, true, true);
				BMO_op_exec(bm_parent, &bmop_fill);

				BMO_op_initf(bm_parent, &bmop_attr, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				             "face_attribute_fill faces=%S use_normals=%b use_data=%b",
				             &bmop_fill, "geom.out", false, true);
				BMO_op_exec(bm_parent, &bmop_attr);

				BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "geom.out", BM_FACE, BM_ELEM_TAG | BM_ELEM_SELECT, true);
			}
			else {
				/* use edgenet fill with inner material */
				BMO_op_initf(
				    bm_parent, &bmop_fill, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				    "edgenet_fill edges=%S mat_nr=%i use_smooth=%b sides=%i",
				    &bmop, "geom_cut.out", inner_mat_index, false, 2);
				BMO_op_exec(bm_parent, &bmop_fill);

				/* Copy Attributes */
				BMO_op_initf(bm_parent, &bmop_attr, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				             "face_attribute_fill faces=%S use_normals=%b use_data=%b",
				             &bmop_fill, "faces.out", true, false);
				BMO_op_exec(bm_parent, &bmop_attr);

				BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "faces.out", BM_FACE, BM_ELEM_TAG | BM_ELEM_SELECT, true);
			}

			BMO_op_finish(bm_parent, &bmop_attr);
			BMO_op_finish(bm_parent, &bmop_fill);
		}

		BMO_slot_buffer_hflag_enable(bm_parent, bmop.slots_out, "geom_cut.out", BM_VERT | BM_EDGE, BM_ELEM_TAG, true);

		BMO_op_finish(bm_parent, &bmop);
	}

	dm_out = CDDM_from_bmesh(bm_parent, true);
	output_s = BKE_create_fracture_shard(dm_out->getVertArray(dm_out),
	                                     dm_out->getPolyArray(dm_out),
	                                     dm_out->getLoopArray(dm_out),
	                                     dm_out->getNumVerts(dm_out),
	                                     dm_out->getNumPolys(dm_out),
	                                     dm_out->getNumLoops(dm_out), true);

	output_s = BKE_custom_data_to_shard(output_s, dm_out);

	/*XXX TODO this might be wrong by now ... */
	output_s->neighbor_count = child->neighbor_count;
	output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
	memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
	BKE_fracture_shard_center_centroid(output_s, output_s->centroid);

	BM_mesh_free(bm_child);
	BM_mesh_free(bm_parent);

	dm_child->needsFree = 1;
	dm_child->release(dm_child);
	dm_child = NULL;

	dm_out->needsFree = 1;
	dm_out->release(dm_out);
	dm_out = NULL;

	return output_s;
}
