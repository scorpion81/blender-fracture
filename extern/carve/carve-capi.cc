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

#include "carve-capi.h"

#include <carve/interpolator.hpp>
#include <carve/rescale.hpp>

#include "carve-util.h"

using carve::mesh::MeshSet;

struct MeshSet3D *carve_addMesh(struct ImportMeshData *import_data,
                                CarveMeshImporter *mesh_importer)
{
#define MAX_STATIC_VERTS 64

	// Import verices from external mesh to Carve
	int num_verts = mesh_importer->getNumVerts(import_data);
	std::vector<carve::geom3d::Vector> vertices;
	for (int i = 0; i < num_verts; i++) {
		float position[3];
		mesh_importer->getVertexCoord(import_data, i, position);
		vertices.push_back(carve::geom::VECTOR(position[0],
		                                       position[1],
		                                       position[2]));
	}

	// Import polys from external mesh to Carve
	int verts_of_poly_static[MAX_STATIC_VERTS];
	int *verts_of_poly_dynamic = NULL;
	int verts_of_poly_dynamic_size = 0;

	int num_polys = mesh_importer->getNumPolys(import_data);
	std::vector<int> face_indices;
	for (int i = 0; i < num_polys; i++) {
		int verts_per_poly =
			mesh_importer->getNumPolyVerts(import_data, i);
		int *verts_of_poly;

		if (verts_per_poly <= MAX_STATIC_VERTS) {
			verts_of_poly = verts_of_poly_static;
		}
		else {
			if (verts_of_poly_dynamic_size < verts_per_poly) {
				if (verts_of_poly_dynamic != NULL) {
					delete [] verts_of_poly_dynamic;
				}
				verts_of_poly_dynamic = new int[verts_per_poly];
				verts_of_poly_dynamic_size = verts_per_poly;
			}
			verts_of_poly = verts_of_poly_dynamic;
		}

		mesh_importer->getPolyVerts(import_data, i, verts_of_poly);

		face_indices.push_back(verts_per_poly);
		for (int j = 0; j < verts_per_poly; j++) {
			face_indices.push_back(verts_of_poly[j]);
		}
	}

	if (verts_of_poly_dynamic != NULL) {
		delete [] verts_of_poly_dynamic;
	}

	MeshSet<3> *poly = new MeshSet<3> (vertices,
	                                   num_polys,
	                                   face_indices);

	return (MeshSet3D*) poly;

#undef MAX_STATIC_VERTS
}

void carve_deleteMesh(struct MeshSet3D *mesh)
{
	MeshSet<3> *poly = (MeshSet<3> *) mesh;
	delete poly;
}

bool carve_performBooleanOperation(struct MeshSet3D *left_mesh,
                                   struct MeshSet3D *right_mesh,
                                   int operation,
                                   struct MeshSet3D **output_mesh)
{
	*output_mesh = NULL;

	carve::csg::CSG::OP op;
	switch (operation) {
#define OP_CONVERT(the_op) \
		case CARVE_OP_ ## the_op: \
			op = carve::csg::CSG::the_op; \
			break;
		OP_CONVERT(UNION)
		OP_CONVERT(INTERSECTION)
		OP_CONVERT(A_MINUS_B)
		default:
			return false;
#undef OP_CONVERT
	}

	MeshSet<3> *output = NULL;
	try {
		MeshSet<3> *left = (MeshSet<3> *) left_mesh,
		           *right = (MeshSet<3> *) right_mesh;
		carve::geom3d::Vector min, max;

		carve_getRescaleMinMax(left, right, &min, &max);

		carve::rescale::rescale scaler(min.x, min.y, min.z, max.x, max.y, max.z);
		carve::rescale::fwd fwd_r(scaler);
		carve::rescale::rev rev_r(scaler);

		left->transform(fwd_r);
		right->transform(fwd_r);

		carve::csg::CSG csg;
		output = csg.compute(left, right, op, NULL, carve::csg::CSG::CLASSIFY_EDGE);
		output->transform(rev_r);
	}
	catch (carve::exception e) {
		std::cerr << "CSG failed, exception " << e.str() << std::endl;
	}
	catch (...) {
		throw "Unknown error in Carve library";
	}

	*output_mesh = (MeshSet3D *) output;

	if (output == NULL) {
		return false;
	}

	return true;
}

template <typename T>
static inline int indexOf(T *element, std::vector<T> &vector_from)
{
	return element - &vector_from.at(0);
}

void carve_exportMesh(struct MeshSet3D *mesh,
                      CarveMeshExporter *mesh_exporter,
                      struct ExportMeshData *export_data)
{
	MeshSet<3> *poly = (MeshSet<3> *) mesh;

	int num_vertices = poly->vertex_storage.size();
	int num_edges = 0, num_loops = 0, num_polys = 0;

	// Count edges from all manifolds.
	for (int i = 0; i < poly->meshes.size(); ++i) {
		carve::mesh::Mesh<3> *mesh = poly->meshes[i];
		num_edges += mesh->closed_edges.size()/* + mesh->open_edges.size()*/;
	}

	// Count polys and loops from all manifolds.
	for (MeshSet<3>::face_iter face_iter = poly->faceBegin();
	     face_iter != poly->faceEnd();
	     ++face_iter, ++num_polys)
	{
		MeshSet<3>::face_t *face = *face_iter;
		num_loops += face->n_edges;
	}

	// Initialize arrays for geometry in exported mesh.
	mesh_exporter->initGeomArrays(export_data, num_vertices, num_edges, num_loops, num_polys);

	// Export all the vertices.
	std::vector<MeshSet<3>::vertex_t>::iterator vertex_iter = poly->vertex_storage.begin();
	for (int i = 0; vertex_iter != poly->vertex_storage.end(); ++i, ++vertex_iter) {
		MeshSet<3>::vertex_t *vertex = &(*vertex_iter);
		float coord[3];
		coord[0] = vertex->v[0];
		coord[1] = vertex->v[1];
		coord[2] = vertex->v[2];
		mesh_exporter->setVertexCoord(export_data, i, coord);
	}

	// Export all the edges.
	std::map<std::pair<MeshSet<3>::vertex_t *, MeshSet<3>::vertex_t *>, int > edge_map;
	for (int i = 0, edge_index = 0; i < poly->meshes.size(); ++i) {
		carve::mesh::Mesh<3> *mesh = poly->meshes[i];
		for (int j = 0; j < mesh->closed_edges.size(); ++j, ++edge_index) {
			MeshSet<3>::edge_t *edge = mesh->closed_edges.at(j);
			MeshSet<3>::vertex_t *v1 =edge->vert;
			MeshSet<3>::vertex_t *v2 =edge->next->vert;

			mesh_exporter->setEdgeVerts(export_data,
			                            edge_index,
			                            indexOf(v1, poly->vertex_storage),
			                            indexOf(v2, poly->vertex_storage));

			edge_map[std::make_pair(v1, v2)] = edge_index;
			edge_map[std::make_pair(v2, v1)] = edge_index;
		}
	}

	// Export all the loops and polys.
	MeshSet<3>::face_iter face_iter = poly->faceBegin();
	for (int loop_index = 0, poly_index = 0; face_iter != poly->faceEnd(); ++face_iter, ++poly_index) {
		MeshSet<3>::face_t *face = *face_iter;

		mesh_exporter->setPolyLoops(export_data, poly_index, loop_index, face->n_edges);

		for (MeshSet<3>::face_t::edge_iter_t edge_iter = face->begin();
		     edge_iter != face->end();
		     ++edge_iter, ++loop_index)
		{
			MeshSet<3>::edge_t &edge = *edge_iter;
			mesh_exporter->setLoopVertEdge(export_data,
		                                   loop_index,
		                                   indexOf(edge.vert, poly->vertex_storage),
		                                   edge_map[std::make_pair(edge.vert, edge.next->vert)]);
		}
	}
}
