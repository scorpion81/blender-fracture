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

#include "stdbool.h"
#include "carve-capi.h"

#include "DNA_meshdata_types.h"
#include "DNA_material_types.h"
#include "DNA_fracture_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_alloca.h"
#include "BLI_ghash.h"
#include "BLI_math.h"
#include "BKE_editmesh.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_material.h"
#include "bmesh.h"
#include "../../modifiers/intern/MOD_boolean_util.h"

#if 0
static void DM_loop_interp_from_poly(DerivedMesh *source_dm, MPoly *source_poly,
                                     DerivedMesh *target_dm, int target_loop_index)
{
	/* TODO(sergey): Arrays we could get once per poly, or once per mesh even. */
	MVert *source_mvert = source_dm->getVertArray(source_dm);
	MVert *target_mvert = target_dm->getVertArray(target_dm);
	MLoop *source_mloop = source_dm->getLoopArray(source_dm);
	MLoop *target_mloop = target_dm->getLoopArray(target_dm);
	float (*cos_3d)[3] = BLI_array_alloca(cos_3d, source_poly->totloop);
	int *source_indices = BLI_array_alloca(source_indices, source_poly->totloop);
	float *weights = BLI_array_alloca(weights, source_poly->totloop);
	int i;
	int target_vert_index = target_mloop[target_loop_index].v;

	for (i = 0; i < source_poly->totloop; ++i) {
		MLoop *mloop = &source_mloop[source_poly->loopstart + i];
		source_indices[i] = source_poly->loopstart + i;
		copy_v3_v3(cos_3d[i], source_mvert[mloop->v].co);
	}

	interp_weights_poly_v3(weights, cos_3d, source_poly->totloop,
	                       target_mvert[target_vert_index].co);

	DM_interp_loop_data(source_dm, target_dm, source_indices, weights,
	                    source_poly->totloop, target_loop_index);
}

/* **** Importer from shard to Carve ****  */
typedef struct ImportMeshData {
	DerivedMesh *dm;
	float obmat[4][4];
	MVert *mvert;
	MEdge *medge;
	MLoop *mloop;
	MPoly *mpoly;
} ImportMeshData;

/* Get number of vertices. */
static int importer_GetNumVerts(ImportMeshData *import_data)
{
	DerivedMesh* dm = import_data->dm;
	return dm->getNumVerts(dm);
}

/* Get number of polys. */
static int importer_GetNumPolys(ImportMeshData *import_data)
{
	DerivedMesh* dm = import_data->dm;
	return dm->getNumPolys(dm);
}

/* Get 3D coordinate of vertex with given index. */
static void importer_GetVertCoord(ImportMeshData *import_data, int index, float coord[3])
{
	MVert *mvert = import_data->mvert;

	BLI_assert(index >= 0 && index < import_data->dm->getNumVerts(import_data->dm));

	mul_v3_m4v3(coord, import_data->obmat, mvert[index].co);
}

/* Get index of vertices which are adjucent to edge specified by it's index. */
static void importer_GetEdgeVerts(ImportMeshData *import_data, int edge_index, int *v1, int *v2)
{
	MEdge *medge = &import_data->medge[edge_index];

	BLI_assert(edge_index >= 0 && edge_index < import_data->dm->getNumEdges(import_data->dm));

	*v1 = medge->v1;
	*v2 = medge->v2;
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
	MLoop *mloop = import_data->mloop + mpoly->loopstart;
	int i;
	BLI_assert(index >= 0 && index < import_data->dm->getNumPolys(import_data->dm));
	for (i = 0; i < mpoly->totloop; i++, mloop++) {
		verts[i] = mloop->v;
	}
}

static CarveMeshImporter MeshImporter = {
	importer_GetNumVerts,
	importer_GetNumPolys,
	importer_GetVertexCoord,
	importer_GetEdgeVerts,
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
	int *edge_origindex;
	int *poly_origindex;
	int *loop_origindex;

	/* Objects (wtf) and derived meshes of left and right operands.
	 * Used for custom data merge and interpolation.
	 */
	Object *ob_left;
	Object *ob_right;
	DerivedMesh *dm_left;
	DerivedMesh *dm_right;

	/* Hash to map materials from right object to result. */
	GHash *material_hash;

} ExportMeshData;

static Object *which_object(ExportMeshData *export_data, int which_mesh)
{
	Object *object = NULL;
	switch (which_mesh) {
		case CARVE_MESH_LEFT:
			object = export_data->ob_left;
			break;
		case CARVE_MESH_RIGHT:
			object = export_data->ob_right;
			break;
	}
	return object;
}

static DerivedMesh *which_dm(ExportMeshData *export_data, int which_mesh)
{
	DerivedMesh *dm = NULL;
	switch (which_mesh) {
		case CARVE_MESH_LEFT:
			dm = export_data->dm_left;
			break;
		case CARVE_MESH_RIGHT:
			dm = export_data->dm_right;
			break;
	}
	return dm;
}

static void allocate_custom_layers(CustomData *data, int type, int num_elements, int num_layers)
{
	int i;
	/* TODO(sergey): Names of those layers will be default,
	 * ideally we need to copy names as well/
	 */
	for (i = 0; i < num_layers; i++) {
		CustomData_add_layer(data, type, CD_CALLOC, NULL, num_elements);
	}
}

/* Create new external mesh */
static void exporter_InitGeomArrays(ExportMeshData *export_data,
                                    int num_verts, int num_edges,
                                    int num_loops, int num_polys)
{
	DerivedMesh *dm = CDDM_new(num_verts, num_edges, 0,
	                           num_loops, num_polys);
	DerivedMesh *dm_left = export_data->dm_left,
	            *dm_right = export_data->dm_right;

	/* Mask for custom data layers to be merhed from operands. */
	CustomDataMask merge_mask = CD_MASK_DERIVEDMESH & ~CD_MASK_ORIGINDEX;

	export_data->dm = dm;
	export_data->mvert = dm->getVertArray(dm);
	export_data->medge = dm->getEdgeArray(dm);
	export_data->mloop = dm->getLoopArray(dm);
	export_data->mpoly = dm->getPolyArray(dm);

	/* Allocate layers for UV layers and vertex colors.
	 * Without this interpolation of those data will not happen.
	 */
	allocate_custom_layers(&dm->loopData, CD_MLOOPCOL, num_loops,
	                       CustomData_number_of_layers(&dm_left->loopData, CD_MLOOPCOL));
	allocate_custom_layers(&dm->loopData, CD_MLOOPUV, num_loops,
	                       CustomData_number_of_layers(&dm_left->loopData, CD_MLOOPUV));

	/* Merge custom data layers from operands.
	 *
	 * Will only create custom data layers for all the layers which appears in
	 * the operand. Data for those layers will not be allocated or initialized.
	 */
	CustomData_merge(&dm_left->polyData, &dm->polyData, merge_mask, CD_DEFAULT, num_polys);
	CustomData_merge(&dm_right->polyData, &dm->polyData, merge_mask, CD_DEFAULT, num_polys);

	export_data->edge_origindex = dm->getEdgeDataArray(dm, CD_ORIGINDEX);
	export_data->poly_origindex = dm->getPolyDataArray(dm, CD_ORIGINDEX);
	export_data->loop_origindex = dm->getLoopDataArray(dm, CD_ORIGINDEX);
}

/* Set coordinate of vertex with given index. */
static void exporter_SetVert(ExportMeshData *export_data, int vert_index, float coord[3])
{
	DerivedMesh *dm = export_data->dm;
	MVert *mvert = export_data->mvert;

	BLI_assert(vert_index >= 0 && vert_index <= dm->getNumVerts(dm));

	mul_v3_m4v3(mvert[vert_index].co, export_data->obimat, coord);

	CustomData_copy_data(&export_data->dm_left->vertData, &dm->vertData, 0, 0, 1);
}

/* Set vertices which are adjucent to the edge specified by it's index. */
static void exporter_SetEdge(ExportMeshData *export_data,
                             int edge_index, int v1, int v2,
                             int which_orig_mesh, int orig_edge_index)
{
	DerivedMesh *dm = export_data->dm;
	MEdge *medge = &export_data->medge[edge_index];
	DerivedMesh *dm_orig;

	BLI_assert(edge_index >= 0 && edge_index < dm->getNumEdges(dm));
	BLI_assert(v1 >= 0 && v1 < dm->getNumVerts(dm));
	BLI_assert(v2 >= 0 && v2 < dm->getNumVerts(dm));

	dm_orig = which_dm(export_data, which_orig_mesh);
	if (dm_orig) {
		BLI_assert(orig_edge_index >= 0 && orig_edge_index < dm_orig->getNumEdges(dm_orig));

		/* Copy all edge layers, including mpoly. */
		CustomData_copy_data(&dm_orig->edgeData, &dm->edgeData, orig_edge_index, edge_index, 1);
	}

	/* Set original index of the edge. */
	if (export_data->edge_origindex) {
		if (which_orig_mesh == CARVE_MESH_LEFT) {
			export_data->edge_origindex[edge_index] = orig_edge_index;
		}
		else {
			export_data->edge_origindex[edge_index] = ORIGINDEX_NONE;
		}
	}

	medge->v1 = v1;
	medge->v2 = v2;

	medge->flag |= ME_EDGEDRAW | ME_EDGERENDER;
}

static void setMPolyMaterial(ExportMeshData *export_data,
                             MPoly *mpoly,
                             int which_orig_mesh)
{
	Object *orig_object;
	GHash *material_hash;
	Material *orig_mat;

	if (which_orig_mesh == CARVE_MESH_LEFT) {
		/* No need to change materian index for faces from left operand */
		return;
	}

	material_hash = export_data->material_hash;
	//orig_object = which_object(export_data, which_orig_mesh);

	/* Set material, based on lookup in hash table. */
	orig_mat = give_current_material(orig_object, mpoly->mat_nr + 1);

	if (orig_mat) {
		/* For faces from right operand check if there's requested material
		 * in the left operand. And if it is, use index of that material,
		 * otherwise fallback to first material (material with index=0).
		 */
		if (!BLI_ghash_haskey(material_hash, orig_mat)) {
			int a, mat_nr;;

			mat_nr = 0;
			for (a = 0; a < export_data->ob_left->totcol; a++) {
				if (give_current_material(export_data->ob_left, a + 1) == orig_mat) {
					mat_nr = a;
					break;
				}
			}

			BLI_ghash_insert(material_hash, orig_mat, SET_INT_IN_POINTER(mat_nr));

			mpoly->mat_nr = mat_nr;
		}
		else
			mpoly->mat_nr = GET_INT_FROM_POINTER(BLI_ghash_lookup(material_hash, orig_mat));
	}
	else {
		mpoly->mat_nr = 0;
	}
}

/* Set list of adjucent loops to the poly specified by it's index. */
static void exporter_SetPoly(ExportMeshData *export_data,
                             int poly_index, int start_loop, int num_loops,
                             int which_orig_mesh, int orig_poly_index)
{
	DerivedMesh *dm = export_data->dm;
	MPoly *mpoly = &export_data->mpoly[poly_index];
	DerivedMesh *dm_orig;

	/* Poly is always to be either from left or right operand. */
	dm_orig = which_dm(export_data, which_orig_mesh);

	BLI_assert(poly_index >= 0 && poly_index < dm->getNumPolys(dm));
	BLI_assert(start_loop >= 0 && start_loop <= dm->getNumLoops(dm) - num_loops);
	BLI_assert(num_loops >= 3);
	BLI_assert(dm_orig != NULL);
	BLI_assert(orig_poly_index >= 0 && orig_poly_index < dm_orig->getNumPolys(dm_orig));

	/* Copy all poly layers, including mpoly. */
	/* TODO(sergey): This we can avoid actually, we'll interpolate later anyway. */
	CustomData_copy_data(&dm_orig->polyData, &dm->polyData, orig_poly_index, poly_index, 1);

	setMPolyMaterial(export_data, mpoly, which_orig_mesh);

	/* Set original index of the poly. */
	if (export_data->poly_origindex) {
		if (which_orig_mesh == CARVE_MESH_LEFT) {
			export_data->poly_origindex[poly_index] = orig_poly_index;
		}
		else {
			export_data->poly_origindex[poly_index] = ORIGINDEX_NONE;
		}
	}

	mpoly->loopstart = start_loop;
	mpoly->totloop = num_loops;
}

/* Set list vertex and edge which are adjucent to loop with given index. */
static void exporter_SetLoop(ExportMeshData *export_data,
                             int loop_index, int vertex, int edge,
                             int which_orig_mesh, int orig_loop_index)
{
	DerivedMesh *dm = export_data->dm;
	MLoop *mloop = &export_data->mloop[loop_index];
	DerivedMesh *dm_orig;

	BLI_assert(loop_index >= 0 && loop_index < dm->getNumLoops(dm));
	BLI_assert(vertex >= 0 && vertex < dm->getNumVerts(dm));
	BLI_assert(edge >= 0 && vertex < dm->getNumEdges(dm));

	dm_orig = which_dm(export_data, which_orig_mesh);
	if (dm_orig) {
		BLI_assert(orig_loop_index >= 0 && orig_loop_index < dm_orig->getNumLoops(dm_orig));

		/* Copy all loop layers, including mpoly. */
		CustomData_copy_data(&dm_orig->loopData, &dm->loopData, orig_loop_index, loop_index, 1);
	}

	/* Set original index of the loop. */
	if (export_data->loop_origindex) {
		if (which_orig_mesh == CARVE_MESH_LEFT) {
			export_data->loop_origindex[loop_index] = orig_loop_index;
		}
		else {
			export_data->loop_origindex[loop_index] = ORIGINDEX_NONE;
		}
	}

	mloop->v = vertex;
	mloop->e = edge;
}

/* TOOO(sergey): We could move this code to the bottom of SetPoly and
 * reshuffle calls in carve-capi.cc. This would save one API call.
 */
static void exporter_InterpPoly(ExportMeshData *export_data,
                                int poly_index, int which_orig_mesh, int orig_poly_index)
{
	MPoly *mpoly, *mpoly_orig;
	DerivedMesh *dm = export_data->dm;
	DerivedMesh *dm_orig;
	int i;

	/* Poly is always to be either from left or right operand. */
	dm_orig = which_dm(export_data, which_orig_mesh);

	BLI_assert(poly_index >= 0 && poly_index < dm->getNumPolys(dm));
	BLI_assert(orig_poly_index >= 0 && orig_poly_index < dm_orig->getNumPolys(dm_orig));

	mpoly = &export_data->mpoly[poly_index];
	mpoly_orig = &(dm_orig->getPolyArray(dm_orig) [orig_poly_index]);

	for (i = 0; i < mpoly->totloop; i++) {
		DM_loop_interp_from_poly(dm_orig, mpoly_orig, dm, i + mpoly->loopstart);
	}
}

static CarveMeshExporter MeshExporter = {
	exporter_InitGeomArrays,
	exporter_SetVert,
	exporter_SetEdge,
	exporter_SetPoly,
	exporter_SetLoop,
	exporter_InterpPoly
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

static void prepare_import_data(Object *obj, DerivedMesh* dm, ImportMeshData *import_data)
{
	import_data->dm = dm;
	copy_m4_m4(import_data->obmat, obj->obmat);
	import_data->mvert = dm->getVertArray(dm);
	import_data->mloop = dm->getLoopArray(dm);
	import_data->mpoly = dm->getPolyArray(dm);
	import_data->medge = dm->getEdgeArray(dm);

#if 0
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
		import_data->mloop = dm->mloop;
	}
#endif
}

static struct CarveMeshDescr *carve_mesh_from_dm(Object* obj, DerivedMesh *dm)
{
	ImportMeshData import_data;
	prepare_import_data(obj, dm, &import_data);
	return carve_addMesh(&import_data, &MeshImporter);
}

static void prepare_export_data(Object* obj, DerivedMesh *dm_left, DerivedMesh *dm_right,
                                ExportMeshData *export_data)
{
	/* Only initialize inverse object matrix here, all the rest will be
	 * initialized in initGeomArrays
	 */
	invert_m4_m4(export_data->obimat, obj->obmat);

	export_data->ob_left = obj;
	export_data->ob_right = obj;
	export_data->dm_left = dm_left;
	export_data->dm_right = dm_right;

	export_data->material_hash = BLI_ghash_ptr_new("CSG_mat gh");
}

Shard *BKE_fracture_shard_boolean(Object* obj, Shard *parent, Shard* child)
{
	bool result;

	Shard *output_s;
	DerivedMesh *output_dm, *left_dm, *right_dm;
	struct CarveMeshDescr *left, *right, *output = NULL;
	int operation = -1;
	int int_op_type = 1;

	if (parent == NULL || child == NULL) {
		return NULL;
	}

	if ((parent->totpoly == 0) || (child->totpoly == 0)) return NULL;

	left_dm = BKE_shard_create_dm(child);
	right_dm = BKE_shard_create_dm(parent);

	operation = operation_from_optype(int_op_type);
	if (operation == -1) {
		return NULL;
	}

	left = carve_mesh_from_dm(obj, left_dm);
	right = carve_mesh_from_dm(obj, right_dm);

	result = carve_performBooleanOperation(left, right, operation, &output);

	carve_deleteMesh(left);
	carve_deleteMesh(right);

	if (result) {
		ExportMeshData export_data;
		prepare_export_data(obj, left_dm, right_dm, &export_data);

		carve_exportMesh(output, &MeshExporter, &export_data);
		output_dm = export_data.dm;
		output_dm->dirty |= DM_DIRTY_NORMALS;

		/* Free memory used by export mesh. */
		BLI_ghash_free(export_data.material_hash, NULL, NULL);

		carve_deleteMesh(output);

		left_dm->needsFree = 1;
		left_dm->release(left_dm);
		left_dm = NULL;

		right_dm->needsFree = 1;
		right_dm->release(right_dm);
		right_dm = NULL;

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
#endif

Shard *BKE_fracture_shard_boolean(Object* obj, Shard *parent, Shard* child)
{
	Shard *output_s;
	DerivedMesh *left_dm, *right_dm, *output_dm;

	left_dm = BKE_shard_create_dm(child, false);
	right_dm = BKE_shard_create_dm(parent, true);

	output_dm = NewBooleanDerivedMesh(right_dm, obj, left_dm, obj, 1);

	left_dm->needsFree = 1;
	left_dm->release(left_dm);
	left_dm = NULL;

	right_dm->needsFree = 1;
	right_dm->release(right_dm);
	right_dm = NULL;

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
		BKE_shard_free(child);

		/*free the temp derivedmesh*/
		output_dm->needsFree = 1;
		output_dm->release(output_dm);
		output_dm = NULL;

		return output_s;
	}

	return NULL;
}


Shard *BKE_fracture_shard_bisect(Shard* parent, Shard* child, float obmat[4][4], bool use_fill)
{

	Shard *output_s;
	DerivedMesh *dm_parent = BKE_shard_create_dm(parent, true);
	DerivedMesh *dm_child = BKE_shard_create_dm(child, false);
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
	bool clear_inner = false;
	bool clear_outer = true;

	invert_m4_m4(imat, obmat);

	BM_ITER_MESH(f, &iter, bm_child, BM_FACES_OF_MESH)
	{
		copy_v3_v3(plane_co, f->l_first->v->co);
		copy_v3_v3(plane_no, f->no);

		mul_m4_v3(imat, plane_co);
		mul_mat3_m4_v3(imat, plane_no);

		BM_mesh_elem_hflag_enable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT, false);

		BMO_op_initf(bm_parent, &bmop, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		             "bisect_plane geom=%hvef dist=%f plane_co=%v plane_no=%v use_snap_center=%b clear_inner=%b clear_outer=%b",
		             BM_ELEM_SELECT, thresh, plane_co, plane_no, false, clear_inner, clear_outer);
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

	output_s = BKE_custom_data_to_shard(output_s, dm_out);

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
