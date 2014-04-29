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
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 * CSG operations.
 */

/** \file blender/blenkernel/intern/fracture_util.c
 *  \ingroup blenkernel
 */

//#include "stdbool.h"
#include "BLI_sys_types.h"
#include "carve-capi.h"

#include "DNA_meshdata_types.h"
#include "DNA_material_types.h"
#include "DNA_fracture_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_alloca.h"
#include "BLI_ghash.h"
#include "BLI_math.h"
#include "BLI_rand.h"
#include "BKE_editmesh.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_material.h"
#include "bmesh.h"
#include "../../modifiers/intern/MOD_boolean_util.h"

Shard *BKE_fracture_shard_boolean(Object* obj, DerivedMesh *dm_parent, Shard* child)
{
	Shard *output_s;
	DerivedMesh *left_dm, *right_dm, *output_dm;

	left_dm = BKE_shard_create_dm(child, false);
	right_dm = dm_parent;
	output_dm = NewBooleanDerivedMesh(right_dm, obj, left_dm, obj, 1);

	left_dm->needsFree = 1;
	left_dm->release(left_dm);
	left_dm = NULL;

	/*right_dm->needsFree = 1;
	right_dm->release(right_dm);
	right_dm = NULL;*/

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

		/*XXX TODO this might be wrong by now ... */
		output_s->neighbor_count = child->neighbor_count;
		output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
		memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
		BKE_fracture_shard_center_centroid(output_s, output_s->centroid);

		/*free the bbox shard*/
		//BKE_shard_free(child, false);

		/*free the temp derivedmesh*/
		output_dm->needsFree = 1;
		output_dm->release(output_dm);
		output_dm = NULL;

		return output_s;
	}

	return NULL;
}


Shard *BKE_fracture_shard_bisect(BMesh* bm_orig, Shard* child, float obmat[4][4], bool use_fill, bool clear_inner,
								 bool clear_outer, int cutlimit, float normal[3], float centroid[3])
{
	#define MYTAG (1 << 6)

	Shard *output_s;
	//DerivedMesh *dm_parent = BKE_shard_create_dm(parent, true);
	DerivedMesh *dm_child = BKE_shard_create_dm(child, false);
	DerivedMesh *dm_out;
	//BMesh *bm_parent = DM_to_bmesh(dm_parent, true);
	//BMesh* bm_parent = BM_mesh_create(&bm_mesh_allocsize_default);
	BMesh *bm_parent = BM_mesh_copy(bm_orig);
	BMesh *bm_child = DM_to_bmesh(dm_child, true);
	BMIter iter, fiter, fiter2;
	BMFace *f, **faces;

	BMOperator bmop;
	float plane_co[3];
	float plane_no[3];
	float imat[4][4];

	float thresh = 0.00001f;
	//bool clear_inner = false;
	//bool clear_outer = true;
	bool do_break = false;

	int fcount = 0, findex = 0, i = 0, cut_index = 0;

	invert_m4_m4(imat, obmat);

	//copy ORIGINDEX...
	//CustomData_copy(&bm_orig->pdata, &bm_parent->pdata, CD_MASK_ORIGINDEX, CD_CALLOC, bm_orig->totface);
	//CustomData_bmesh_init_pool(&bm_parent->pdata, bm_mesh_allocsize_default.totface, BM_FACE);

	//CustomData_add_layer(&bm_parent->pdata, CD_ORIGINDEX, CD_CALLOC, NULL, bm_parent->totface);
	//CustomData_bmesh_init_pool(&bm_parent->pdata, bm_parent->totface, BM_FACE);

	/*for (i = 0; i < bm_parent->totface; i++)
	{
		BMFace *f1, *f2;
		int *oindex = NULL;
		f1 = BM_face_at_index_find(bm_orig, i);
		f2 = BM_face_at_index(bm_parent, i);

		if ((f1 != NULL) && (f2 != NULL))
		{
			CustomData_bmesh_set_default(&bm_parent->pdata, &f2->head.data);
			oindex = CustomData_bmesh_get(&bm_orig->pdata, f1->head.data, CD_ORIGINDEX);

			if (oindex != NULL)
			{
				CustomData_bmesh_set(&bm_parent->pdata, f2->head.data, CD_ORIGINDEX, &i);
			}
			else
			{
				int *none = MEM_callocN(sizeof(int), "NONE");
				*none = ORIGINDEX_NONE;
				CustomData_bmesh_set(&bm_parent->pdata, f2->head.data, CD_ORIGINDEX, none);
			}
		}
		else if (f2 != NULL)
		{
			int *none = MEM_callocN(sizeof(int), "NONE");
			*none = ORIGINDEX_NONE;

			CustomData_bmesh_set_default(&bm_parent->pdata, &f2->head.data);
			CustomData_bmesh_set(&bm_parent->pdata, f2->head.data, CD_ORIGINDEX, none);
		}
	}*/


	CustomData_add_layer(&bm_parent->pdata, CD_ORIGINDEX, CD_CALLOC, NULL, bm_parent->totface);
	CustomData_bmesh_init_pool(&bm_parent->pdata, bm_parent->totface, BM_FACE);

	BM_ITER_MESH_INDEX(f, &fiter, bm_parent, BM_FACES_OF_MESH, findex)
	{
		CustomData_bmesh_set_default(&bm_parent->pdata, &f->head.data);
		CustomData_bmesh_set(&bm_parent->pdata, f->head.data, CD_ORIGINDEX, &findex);
	}

	//then enable tags...
	BM_mesh_elem_hflag_enable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
	BM_mesh_elem_hflag_enable_all(bm_parent, BM_FACE, MYTAG, false);

	BM_ITER_MESH_INDEX(f, &iter, bm_child, BM_FACES_OF_MESH, cut_index)
	{
		if (do_break)
		{
			break;
		}

		if (cutlimit > 0)
		{
			//int index = (int)(BLI_frand() * (bm_child->totface-1));
			f = BM_face_at_index_find(bm_child, cutlimit);
			copy_v3_v3(plane_co, centroid);
			copy_v3_v3(plane_no, f->no /*normal*/);
			do_break = true;
		}
		else
		{
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

		//untag inner geometry of this cell and skip it ?
		//BMO_slot_buffer_hflag_disable(bm_parent, bmop.slots_out, "geom.out", BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT, false);

		BM_mesh_elem_hflag_disable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);

		if (use_fill) {
			float normal_fill[3];
			BMOperator bmop_fill;
			BMOperator bmop_attr;

			normalize_v3_v3(normal_fill, plane_no);
			{//if (clear_outer == true && clear_inner == false) {
				negate_v3(normal_fill);
			}

			/* Fill */
			/*BMO_op_initf(
			        bm_parent, &bmop_fill,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			        "triangle_fill edges=%S use_beauty=%b",
			        &bmop, "geom_cut.out", true);
			BMO_op_exec(bm_parent, &bmop_fill);*/

			/*BMO_op_initf(bm_parent, &bmop_fill, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			            "contextual_create geom=%S mat_nr=%i use_smooth=%b",
			            &bmop, "geom_cut.out", 0, false);
			BMO_op_exec(bm_parent, &bmop_fill);*/

			BMO_op_initf(
			        bm_parent, &bmop_fill,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			        "triangle_fill edges=%S normal=%v use_dissolve=%b",
			        &bmop, "geom_cut.out", normal_fill, true);
			BMO_op_exec(bm_parent, &bmop_fill);

			/*BMO_op_initf(
			        bm_parent, &bmop_fill,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			       "edgenet_fill edges=%S mat_nr=%i use_smooth=%b sides=%i",
			        &bmop, "geom_cut.out", 0, false, 1);
			BMO_op_exec(bm_parent, &bmop_fill);*/

			/* Copy Attributes */
			/*BMO_op_initf(bm_parent, &bmop_attr, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			             "face_attribute_fill faces=%S use_normals=%b use_data=%b",
			             &bmop_fill, "faces.out", false, true);
			BMO_op_exec(bm_parent, &bmop_attr);

			BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "faces.out", BM_FACE, BM_ELEM_SELECT, true);*/

			BMO_op_initf(bm_parent, &bmop_attr, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
			             "face_attribute_fill faces=%S use_normals=%b use_data=%b",
			             &bmop_fill, "geom.out", false, true);
			BMO_op_exec(bm_parent, &bmop_attr);

			BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "geom.out", BM_FACE, BM_ELEM_TAG, true);

			BMO_op_finish(bm_parent, &bmop_attr);
			BMO_op_finish(bm_parent, &bmop_fill);
		}

		BMO_slot_buffer_hflag_enable(bm_parent, bmop.slots_out, "geom_cut.out", BM_VERT | BM_EDGE, BM_ELEM_TAG, true);
		//BMO_slot_buffer_hflag_enable(bm_parent, bmop.slots_out, "geom.out", BM_FACE, MYTAG, false);
		BMO_slot_buffer_hflag_disable(bm_parent, bmop.slots_out, "geom_cut.out", BM_FACE, MYTAG, false);

		BMO_op_finish(bm_parent, &bmop);
	}

	i = 0;
	//faces = MEM_callocN(sizeof(BMFace*) * bm_parent->totface, "ftokill");

	if (cutlimit == 0)
	{
		BM_ITER_MESH(f, &fiter2, bm_parent, BM_FACES_OF_MESH)
		{
			if (BM_elem_flag_test(f, MYTAG))
			{
				int *orig_index = CustomData_bmesh_get(&bm_parent->pdata, f->head.data, CD_ORIGINDEX);
				if ((orig_index != NULL) && (*orig_index > -1) && (*orig_index < bm_orig->totface))
				{
					BMFace *face = BM_face_at_index(bm_orig, *orig_index);
					if ((face != NULL) && (face->head.index == *orig_index))
					{
						//faces[i] = face;
						BM_elem_flag_enable(face, MYTAG);
						fcount++;
					}
					else
					{
						//faces[i] = NULL;
					}
				}
				else
				{
					//faces[i] = NULL;
				}
			}

			i++;
		}

		BMO_op_callf(bm_orig, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
					 "delete context=%i geom=%hf", DEL_FACES, MYTAG);
	}

	/*for (i = 0; i < bm_parent->totface; i++)
	{
		BMFace *face = faces[i];
		if (face != NULL)
		{
			BM_face_kill(bm_orig, face);
			fcount++;
		}
	}*/

	BM_mesh_elem_table_ensure(bm_orig, BM_FACE);
	//MEM_freeN(faces);

	printf("Removed faces: %d \n", fcount);

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

	/*free the bbox shard*/
	//BKE_shard_free(child, false);

	BM_mesh_free(bm_child);
	BM_mesh_free(bm_parent);

	//*bm_work = bm_rest;

	/*dm_parent->needsFree = 1;
	dm_parent->release(dm_parent);
	dm_parent = NULL;*/

	dm_child->needsFree = 1;
	dm_child->release(dm_child);
	dm_child = NULL;

	dm_out->needsFree = 1;
	dm_out->release(dm_out);
	dm_out = NULL;

	return output_s;
}
