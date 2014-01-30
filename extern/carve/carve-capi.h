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
 * The Original Code is Copyright (C) 2014 Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Blender Foundation,
 *                 Sergey Sharybin
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#ifndef __CARVE_CAPI_H__
#define __CARVE_CAPI_H__

#ifdef __cplusplus
extern "C" {
#endif

struct MeshSet3D;

//
// Importer from external storage to Carve module
//

struct ImportMeshData;

// Get number of vertices.
typedef int (*CarveImporter_GetNumVerts) (struct ImportMeshData *import_data);

// Get number of polys.
typedef int (*CarveImporter_GetNumPolys) (struct ImportMeshData *import_data);

// Get 3D coordinate of vertex with given index.
typedef void (*CarveImporter_GetVertexCoord) (struct ImportMeshData *import_data, int index, float coord[3]);

// Get number of adjucent vertices to the poly specified by it's index.
typedef int (*CarveImporter_GetPolyNumVerts) (struct ImportMeshData *import_data, int index);

// Get list of adjucent vertices to the poly specified by it's index.
typedef void (*CarveImporter_GetPolyVerts) (struct ImportMeshData *import_data, int index, int *verts);

typedef struct CarveMeshImporter {
	CarveImporter_GetNumVerts getNumVerts;
	CarveImporter_GetNumPolys getNumPolys;
	CarveImporter_GetVertexCoord getVertexCoord;
	CarveImporter_GetPolyNumVerts getNumPolyVerts;
	CarveImporter_GetPolyVerts getPolyVerts;
} CarveMeshImporter;

//
// Exporter from Carve module to external storage
//

struct ExportMeshData;

// Initialize arrays for geometry.
typedef void (*CarveExporter_InitGeomArrays) (struct ExportMeshData *export_data,
                                              int num_verts, int num_edges,
                                              int num_polys, int num_loops);

// Set coordinate of vertex with given index.
typedef void (*CarveExporter_SetVertexCoord) (struct ExportMeshData *export_data, int index, float coord[3]);

// Set vertices which are adjucent to the edge specified by it's index.
typedef void (*CarveExporter_SetEdgeVerts) (struct ExportMeshData *export_data, int index, int v1, int v2);

// Set adjucent loops to the poly specified by it's index.
typedef void (*CarveExporter_SetPolyLoops) (struct ExportMeshData *export_data, int index, int start_loop, int num_loops);

// Set list vertex and edge which are adjucent to loop with given index.
typedef void (*CarveExporter_SetLoopVertEdge) (struct ExportMeshData *export_data, int index, int vertex, int edge);

typedef struct CarveMeshExporter {
	CarveExporter_InitGeomArrays initGeomArrays;
	CarveExporter_SetVertexCoord setVertexCoord;
	CarveExporter_SetEdgeVerts setEdgeVerts;
	CarveExporter_SetPolyLoops setPolyLoops;
	CarveExporter_SetLoopVertEdge setLoopVertEdge;
} CarveMeshExporter;

enum {
	CARVE_OP_UNION,
	CARVE_OP_INTERSECTION,
	CARVE_OP_A_MINUS_B,
};

struct MeshSet3D *carve_addMesh(struct ImportMeshData *import_data,
                                CarveMeshImporter *mesh_importer);

void carve_deleteMesh(struct MeshSet3D *mesh);

bool carve_performBooleanOperation(struct MeshSet3D *left_mesh,
                                   struct MeshSet3D *right_mesh,
                                   int operation,
                                   struct MeshSet3D **output_mesh);

void carve_exportMesh(struct MeshSet3D *mesh,
                      CarveMeshExporter *mesh_exporter,
                      struct ExportMeshData *export_data);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // __CARVE_CAPI_H__
