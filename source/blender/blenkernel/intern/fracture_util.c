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

//#include "carve-util.h"
#include "stdbool.h"
#include "carve-capi.h"

#include "DNA_fracture_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BKE_editmesh.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "bmesh.h"


/* **** Importer from shard to Carve ****  */
typedef struct ImportMeshData {
	Shard *s;
	float obmat[4][4];
	MVert *mvert;
	MLoop *mloop;
	MPoly *mpoly;
} ImportMeshData;

/* Get number of vertices. */
static int importer_GetNumVerts(ImportMeshData *import_data)
{
	Shard *s = import_data->s;
	return s->totvert;
}

/* Get number of polys. */
static int importer_GetNumPolys(ImportMeshData *import_data)
{
	Shard *s = import_data->s;
	return s->totpoly;
}

/* Get 3D coordinate of vertex with given index. */
static void importer_GetVertexCoord(ImportMeshData *import_data, int index, float coord[3])
{
	MVert *mvert = import_data->mvert;

	BLI_assert(index >= 0 && index < import_data->s->totvert);

	mul_v3_m4v3(coord, import_data->obmat, mvert[index].co);
}

/* Get number of adjucent vertices to the poly specified by it's index. */
static int importer_GetPolyNumVerts(ImportMeshData *import_data, int index)
{
	MPoly *mpoly = import_data->mpoly;
	return mpoly[index].totloop;
}

/* Get list of adjucent vertices to the poly specified by it's index. */
static void importer_GetPolyVerts(ImportMeshData *import_data, int index, int *verts)
{
	MPoly *mpoly = &import_data->mpoly[index];
	MLoop *mloop = import_data->mloop + mpoly->loopstart;// + mpoly->totloop-1;
	int i;
	BLI_assert(index >= 0 && index < import_data->s->totpoly);
	for (i = 0; i < mpoly->totloop; i++, mloop++) {
		verts[i] = mloop->v;
	}
}

static CarveMeshImporter MeshImporter = {
	importer_GetNumVerts,
	importer_GetNumPolys,
	importer_GetVertexCoord,
	importer_GetPolyNumVerts,
	importer_GetPolyVerts
};


/* **** Exporter from Carve to derivedmesh ****  */

typedef struct ExportMeshData {
	DerivedMesh *dm;
	float obimat[4][4];
	MVert *mvert;
	MEdge *medge;
	MLoop *mloop;
	MPoly *mpoly;
} ExportMeshData;

/* Create new external mesh */
static void exporter_InitGeomArrays(ExportMeshData *export_data,
                                    int num_verts, int num_edges,
                                    int num_loops, int num_polys)
{
	DerivedMesh *dm = CDDM_new(num_verts, num_edges, 0,
	                           num_loops, num_polys);
	export_data->dm = dm;
	export_data->mvert = dm->getVertArray(dm);
	export_data->medge = dm->getEdgeArray(dm);
	export_data->mloop = dm->getLoopArray(dm);
	export_data->mpoly = dm->getPolyArray(dm);
}

/* Set coordinate of vertex with given index. */
static void exporter_SetVertexCoord(ExportMeshData *export_data, int index, float coord[3])
{
	MVert *mvert = export_data->mvert;
	mul_v3_m4v3(mvert[index].co, export_data->obimat, coord);
}

/* Set vertices which are adjucent to the edge specified by it's index. */
static void exporter_SetEdgeVerts(ExportMeshData *export_data, int index, int v1, int v2)
{
	MEdge *medge = &export_data->medge[index];

	BLI_assert(index >= 0 && index < export_data->dm->getNumEdges(export_data->dm));
	BLI_assert(v1 >= 0 && v1 < export_data->dm->getNumVerts(export_data->dm));
	BLI_assert(v2 >= 0 && v2 < export_data->dm->getNumVerts(export_data->dm));

	medge->v1 = v1;
	medge->v2 = v2;

	medge->flag |= ME_EDGEDRAW | ME_EDGERENDER;
}

// Set list of adjucent loops to the poly specified by it's index.
static void exporter_SetPolyLoops(ExportMeshData *export_data, int index, int start_loop, int num_loops)
{
	MPoly *mpoly = &export_data->mpoly[index];

	BLI_assert(index >= 0 && index < export_data->dm->getNumPolys(export_data->dm));
	BLI_assert(start_loop >= 0 && start_loop <= export_data->dm->getNumLoops(export_data->dm) - num_loops);
	BLI_assert(num_loops >= 3);

	mpoly->loopstart = start_loop;
	mpoly->totloop = num_loops;
}

/* Set list vertex and edge which are adjucent to loop with given index. */
static void exporter_SetLoopVertEdge(ExportMeshData *export_data, int index, int vertex, int edge)
{
	MLoop *mloop = &export_data->mloop[index];

	BLI_assert(index >= 0 && index < export_data->dm->getNumLoops(export_data->dm));
	BLI_assert(vertex >= 0 && vertex < export_data->dm->getNumVerts(export_data->dm));
	BLI_assert(edge >= 0 && vertex < export_data->dm->getNumEdges(export_data->dm));

	mloop->v = vertex;
	mloop->e = edge;
}

static CarveMeshExporter MeshExporter = {
	exporter_InitGeomArrays,
	exporter_SetVertexCoord,
	exporter_SetEdgeVerts,
	exporter_SetPolyLoops,
	exporter_SetLoopVertEdge
};

static int operation_from_optype(int int_op_type)
{
	int operation;

	switch (int_op_type) {
		case 1:
			operation = CARVE_OP_INTERSECTION;
			break;
		case 2:
			operation = CARVE_OP_UNION;
			break;
		case 3:
			operation = CARVE_OP_A_MINUS_B;
			break;
		default:
			BLI_assert(!"Should not happen");
			operation = -1;
			break;
	}

	return operation;
}

static void prepare_import_data(float obmat[4][4], Shard *s, ImportMeshData *import_data, bool flip)
{
	import_data->s = s;
	copy_m4_m4(import_data->obmat, obmat);
	import_data->mvert = s->mvert;
	import_data->mloop = s->mloop;

	if (flip)
	{
		//swap loops, per poly
		int i, j;
		MPoly *p = s->mpoly;
		import_data->mloop = MEM_mallocN(sizeof(MLoop) * s->totloop, __func__);
		for (i = 0; i < s->totpoly; i++, p++)
		{
			MLoop* l = &s->mloop[p->loopstart + p->totloop - 1];
			for (j = 0; j < p->totloop; j++, l--)
			{
				import_data->mloop[j + p->loopstart].v = l->v;
				import_data->mloop[j + p->loopstart].e = l->e;
			}
		}
	}
	else
	{
		import_data->mloop = s->mloop;
	}

	import_data->mpoly = s->mpoly;
}

static struct MeshSet3D *carve_mesh_from_shard(float obmat[4][4], Shard *s, bool flip)
{
	ImportMeshData import_data;
	prepare_import_data(obmat, s, &import_data, flip);
	return carve_addMesh(&import_data, &MeshImporter);
}

static void prepare_export_data(float obmat[4][4], ExportMeshData *export_data)
{
	/* Only initialize inverse object matrix here, all the rest will be
	 * initialized in initGeomArrays
	 */
	invert_m4_m4(export_data->obimat, obmat);
}

Shard *BKE_fracture_shard_boolean(Shard *parent, Shard* child, float obmat[4][4])
{
	//float inv_mat[4][4];
	//float map_mat[4][4];
	float offs_mat[4][4];
	bool result;

	Shard *output_s;
	DerivedMesh *output_dm;
	struct MeshSet3D *left, *right, *output = NULL;
	int operation = -1;
	int int_op_type = 1;

	if (parent == NULL || child == NULL) {
		return NULL;
	}

	if ((parent->totpoly == 0) || (child->totpoly == 0)) return NULL;

	copy_m4_m4(offs_mat, obmat);
	//unit_m4(offs_mat);
	translate_m4(offs_mat, child->centroid[0], child->centroid[1], child->centroid[2]);

	/* we map the final object back into ob's local coordinate space. For this
	 * we need to compute the inverse transform from global to ob (inv_mat),
	 * and the transform from ob to ob_select for use in interpolation (map_mat) */
	//invert_m4_m4(inv_mat, obmat);
	//mul_m4_m4m4(map_mat, inv_mat, offs_mat);
	//invert_m4_m4(inv_mat, offs_mat);

	operation = operation_from_optype(int_op_type);
	if (operation == -1) {
		return NULL;
	}

	left = carve_mesh_from_shard(obmat, child, false);
	right = carve_mesh_from_shard(obmat, parent, false);

	result = carve_performBooleanOperation(left, right, operation, &output);

	carve_deleteMesh(left);
	carve_deleteMesh(right);

	if (result) {
		ExportMeshData export_data;
		prepare_export_data(obmat, &export_data);

		carve_exportMesh(output, &MeshExporter, &export_data);
		output_dm = export_data.dm;
		output_dm->dirty |= DM_DIRTY_NORMALS;
		carve_deleteMesh(output);

		if (output_dm)
		{
			output_s = BKE_create_fracture_shard(output_dm->getVertArray(output_dm),
			                                     output_dm->getPolyArray(output_dm),
			                                     output_dm->getLoopArray(output_dm),
			                                     output_dm->getNumVerts(output_dm),
			                                     output_dm->getNumPolys(output_dm),
			                                     output_dm->getNumLoops(output_dm),
			                                     true);

			/*XXX TODO this might be wrong by now ... */
			output_s->neighbor_count = child->neighbor_count;
			output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
			memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
			BKE_fracture_shard_center_centroid(output_s, output_s->centroid);

			/*free the bbox shard*/
			BKE_shard_free(child);

			/*free the temp derivedmesh*/
			output_dm->needsFree = 1;
			output_dm->release(output_dm);
			output_dm = NULL;

			return output_s;
		}

		return NULL;
	}
	return NULL;
}


Shard *BKE_fracture_shard_bisect(Shard* parent, Shard* child, float obmat[4][4])
{

	Shard *output_s;
	DerivedMesh *dm_parent = BKE_shard_create_dm(parent);
	DerivedMesh *dm_child = BKE_shard_create_dm(child);
	DerivedMesh *dm_out;
	BMesh *bm_parent = DM_to_bmesh(dm_parent, true);
	BMesh *bm_child = DM_to_bmesh(dm_child, true);
	BMIter iter;
	BMFace *f;

	BMOperator bmop;
	float plane_co[3];
	float plane_no[3];
	float imat[4][4];

	float thresh = 0.00001f;
	bool clear_inner = true;
	bool clear_outer = false;
	bool use_fill = true;

	invert_m4_m4(imat, obmat);

	BM_ITER_MESH(f, &iter, bm_child, BM_FACES_OF_MESH)
	{
		copy_v3_v3(plane_co, f->l_first->v->co);
		copy_v3_v3(plane_no, f->no);

		mul_m4_v3(imat, plane_co);
		mul_mat3_m4_v3(imat, plane_no);

		BM_mesh_elem_hflag_enable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT, false);

		BMO_op_initf(bm_parent, &bmop, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		             "bisect_plane geom=%hvef plane_co=%v plane_no=%v dist=%f clear_inner=%b clear_outer=%b",
		             BM_ELEM_SELECT, plane_co, plane_no, thresh, clear_inner, clear_outer);
		BMO_op_exec(bm_parent, &bmop);

		BM_mesh_elem_hflag_disable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT, false);

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

			BMO_slot_buffer_hflag_enable(bm_parent, bmop_fill.slots_out, "geom.out", BM_FACE, BM_ELEM_SELECT, true);

			BMO_op_finish(bm_parent, &bmop_attr);
			BMO_op_finish(bm_parent, &bmop_fill);
		}

		BMO_slot_buffer_hflag_enable(bm_parent, bmop.slots_out, "geom_cut.out", BM_VERT | BM_EDGE, BM_ELEM_SELECT, true);

		BMO_op_finish(bm_parent, &bmop);
		BM_mesh_select_flush(bm_parent);
	}

	dm_out = CDDM_from_bmesh(bm_parent, true);
	output_s = BKE_create_fracture_shard(dm_out->getVertArray(dm_out),
	                                   dm_out->getPolyArray(dm_out),
	                                   dm_out->getLoopArray(dm_out),
	                                   dm_out->getNumVerts(dm_out),
	                                   dm_out->getNumPolys(dm_out),
	                                   dm_out->getNumLoops(dm_out), true);

	/*XXX TODO this might be wrong by now ... */
	output_s->neighbor_count = child->neighbor_count;
	output_s->neighbor_ids = MEM_mallocN(sizeof(int) * child->neighbor_count, __func__);
	memcpy(output_s->neighbor_ids, child->neighbor_ids, sizeof(int) * child->neighbor_count);
	BKE_fracture_shard_center_centroid(output_s, output_s->centroid);

	/*free the bbox shard*/
	BKE_shard_free(child);

	BM_mesh_free(bm_child);
	BM_mesh_free(bm_parent);

	dm_parent->needsFree = 1;
	dm_parent->release(dm_parent);
	dm_parent = NULL;

	dm_child->needsFree = 1;
	dm_child->release(dm_child);
	dm_child = NULL;

	dm_out->needsFree = 1;
	dm_out->release(dm_out);
	dm_out = NULL;

	return output_s;
}
