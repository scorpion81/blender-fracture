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

/** \file blender/modifiers/intern/MOD_boolean_util.c
 *  \ingroup modifiers
 */


#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"
#include "BLI_listbase.h"
#include "BLI_ghash.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_depsgraph.h"
#include "BKE_global.h"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_object.h"

#include "MOD_boolean_util.h"

#include "carve-capi.h"

/* **** Importer from derived mesh to Carve ****  */

typedef struct ImportMeshData {
	DerivedMesh *dm;
	float obmat[4][4];
	MVert *mvert;
	MLoop *mloop;
	MPoly *mpoly;
} ImportMeshData;

/* Get number of vertices. */
static int importer_GetNumVerts(ImportMeshData *import_data)
{
	DerivedMesh *dm = import_data->dm;
	return dm->getNumVerts(dm);
}

/* Get number of polys. */
static int importer_GetNumPolys(ImportMeshData *import_data)
{
	DerivedMesh *dm = import_data->dm;
	return dm->getNumPolys(dm);
}

/* Get 3D coordinate of vertex with given index. */
static void importer_GetVertexCoord(ImportMeshData *import_data, int index, float coord[3])
{
	MVert *mvert = import_data->mvert;

	BLI_assert(index >= 0 && index < import_data->dm->getNumVerts(import_data->dm));

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
	importer_GetPolyNumVerts,
	importer_GetPolyVerts
};

/* **** Exporter from Carve to derived mesh ****  */

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

static void prepare_import_data(Object *object, DerivedMesh *dm, ImportMeshData *import_data)
{
	import_data->dm = dm;
	copy_m4_m4(import_data->obmat, object->obmat);
	import_data->mvert = dm->getVertArray(dm);
	import_data->mloop = dm->getLoopArray(dm);
	import_data->mpoly = dm->getPolyArray(dm);
}

static struct MeshSet3D *carve_mesh_from_dm(Object *object, DerivedMesh *dm)
{
	ImportMeshData import_data;
	prepare_import_data(object, dm, &import_data);
	return carve_addMesh(&import_data, &MeshImporter);
}

static void prepare_export_data(Object *object, ExportMeshData *export_data)
{
	/* Only initialize inverse object matrix here, all the rest will be
	 * initialized in initGeomArrays
	 */
	invert_m4_m4(export_data->obimat, object->obmat);
}

DerivedMesh *NewBooleanDerivedMesh(DerivedMesh *dm, struct Object *ob,
                                   DerivedMesh *dm_select, struct Object *ob_select,
                                   int int_op_type)
{

	struct MeshSet3D *left, *right, *output = NULL;
	DerivedMesh *output_dm = NULL;
	int operation;
	bool result;

	if (dm == NULL || dm_select == NULL) {
		return NULL;
	}

	operation = operation_from_optype(int_op_type);
	if (operation == -1) {
		return NULL;
	}

	left = carve_mesh_from_dm(ob_select, dm_select);
	right = carve_mesh_from_dm(ob, dm);

	result = carve_performBooleanOperation(left, right, operation, &output);

	carve_deleteMesh(left);
	carve_deleteMesh(right);

	if (result) {
		ExportMeshData export_data;
		prepare_export_data(ob_select, &export_data);

		carve_exportMesh(output, &MeshExporter, &export_data);
		output_dm = export_data.dm;
		output_dm->dirty |= DM_DIRTY_NORMALS;
		carve_deleteMesh(output);
	}

	return output_dm;
}
