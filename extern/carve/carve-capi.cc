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

typedef std::pair<int, int> OrigIndex;
typedef std::pair<MeshSet<3>::vertex_t *, MeshSet<3>::vertex_t *> VertexPair;
typedef carve::interpolate::FaceVertexAttr<OrigIndex> OrigFaceVertMapping;
typedef carve::interpolate::FaceAttr<OrigIndex> OrigFaceMapping;

// Optimization trick, we store both original edge and loop indices as a pair
// of the same attribute. This redces number of attribute interpolation in Carve.
typedef carve::interpolate::FaceEdgeAttr< std::pair<OrigIndex, OrigIndex> > OrigFaceEdgeMapping;

typedef struct CarveMeshDescr {
	// Stores mesh data itself.
	MeshSet<3> *poly;

	std::map<std::pair<int, int>, int > edge_index_map;

	// The folloving mapping is only filled in for output mesh.

	// Mapping from the face verts back to original vert index.
	OrigFaceVertMapping orig_face_vert_mapping;

	// Mapping from the face edges back to (original edge index, original loop index).
	OrigFaceEdgeMapping orig_face_edge_mapping;

	// Mapping from the faces back to original poly index.
	OrigFaceMapping orig_face_mapping;
} CarveMeshDescr;

namespace {

// Original index could not be interpolated, so we hack this around a bit.
//
// Weighting is only allowed to happen with weight of 1.0. In this case
// result is the same exact as input weight. For weight != 1.0 we output
// ORIGINDEX_NONE.
//
// Sum is a bit more tricky, but basically it is supposed to only output
// original index of single non-ORIGINDEX_NONE item and if there're more
// than one such items we output ORIGINDEX_NONE.
OrigIndex operator*(double w, const OrigIndex &index)
{
	if (w == 1.0) {
		return index;
	}
	return std::make_pair((int)CARVE_MESH_NONE, -1);
}

OrigIndex operator+=(OrigIndex &a, const OrigIndex &b)
{
	if (a.first != CARVE_MESH_NONE && b.first != CARVE_MESH_NONE) {
		a.first = CARVE_MESH_NONE;
		a.second = -1;
	}
	else if (b.first != CARVE_MESH_NONE) {
		a.first = b.first;
		a.second = b.second;
	}
	return a;
}

template <typename T1, typename T2>
void edgeIndexMap_put(std::map<std::pair<T1, T1>, T2 > *edge_map,
                      T1 v1,
                      T1 v2,
                      const T2 &index)
{
	if (v2 > v1) {
		std::swap(v1, v2);
	}
	(*edge_map)[std::make_pair(v1, v2)] = index;
}

template <typename T1, typename T2>
const T2 &edgeIndexMap_get(const std::map<std::pair<T1, T1>, T2 > &edge_map,
                           T1 v1,
                           T1 v2)
{
	typedef std::map<std::pair<T1, T1>, T2 > Map;

	if (v2 > v1) {
		std::swap(v1, v2);
	}
	const typename Map::const_iterator found =
		edge_map.find(std::make_pair(v1, v2));
	assert(found != edge_map.end());
	return found->second;
}

template <typename T>
inline int indexOf(const T *element, const std::vector<T> &vector_from)
{
	return element - &vector_from.at(0);
}

void initOrigIndexMeshFaceMapping(CarveMeshDescr *mesh,
                                  int which_mesh,
                                  OrigFaceVertMapping *orig_face_vert_mapping,
                                  OrigFaceEdgeMapping *orig_face_edge_mapping,
                                  OrigFaceMapping *orig_face_attr)
{
	MeshSet<3> *poly = mesh->poly;

	MeshSet<3>::face_iter face_iter = poly->faceBegin();
	for (int i = 0, loop_index = 0;
	     face_iter != poly->faceEnd();
	     ++face_iter, ++i)
	{
		const MeshSet<3>::face_t *face = *face_iter;

		// Mapping from carve face back to original poly index.
		orig_face_attr->setAttribute(face, std::make_pair(which_mesh, i));

		for (MeshSet<3>::face_t::const_edge_iter_t edge_iter = face->begin();
		     edge_iter != face->end();
		     ++edge_iter, ++loop_index)
		{
			const MeshSet<3>::edge_t &edge = *edge_iter;
			int v1 = indexOf(edge.vert, poly->vertex_storage),
			    v2 = indexOf(edge.next->vert, poly->vertex_storage);

			// Mapping from carve face vertex back to original vertex index.
			orig_face_vert_mapping->setAttribute(face,
			                                     edge_iter.idx(),
			                                     std::make_pair(which_mesh, v1));

			int edge_index = edgeIndexMap_get(mesh->edge_index_map, v1, v2);

			// Mapping from carve face edge back to original edge index.
			OrigIndex orig_edge_index = std::make_pair(which_mesh, edge_index);

			// Mapping from carve face edge back to original loop index.
			OrigIndex orig_loop_index = std::make_pair(which_mesh, loop_index);

			orig_face_edge_mapping->setAttribute(face,
			                                     edge_iter.idx(),
			                                     std::make_pair(orig_edge_index, orig_loop_index));
		}
	}
}

void initOrigIndexMapping(CarveMeshDescr *left_mesh,
                          CarveMeshDescr *right_mesh,
                          OrigFaceVertMapping *orig_face_vert_mapping,
                          OrigFaceEdgeMapping *orig_face_edge_mapping,
                          OrigFaceMapping *orig_face_mapping)
{
	initOrigIndexMeshFaceMapping(left_mesh,
	                             CARVE_MESH_LEFT,
	                             orig_face_vert_mapping,
	                             orig_face_edge_mapping,
	                             orig_face_mapping);

	initOrigIndexMeshFaceMapping(right_mesh,
	                             CARVE_MESH_RIGHT,
	                             orig_face_vert_mapping,
	                             orig_face_edge_mapping,
	                             orig_face_mapping);
}

}  // namespace

CarveMeshDescr *carve_addMesh(struct ImportMeshData *import_data,
                              CarveMeshImporter *mesh_importer)
{
#define MAX_STATIC_VERTS 64

	CarveMeshDescr *mesh_descr = new CarveMeshDescr;

	// Import verices from external mesh to Carve.
	int num_verts = mesh_importer->getNumVerts(import_data);
	std::vector<carve::geom3d::Vector> vertices;
	vertices.reserve(num_verts);
	for (int i = 0; i < num_verts; i++) {
		float position[3];
		mesh_importer->getVertCoord(import_data, i, position);
		vertices.push_back(carve::geom::VECTOR(position[0],
		                                       position[1],
		                                       position[2]));
	}

	// Fill in edge mapping so later we can distinguish original edge index.
	int num_edges = mesh_importer->getNumEdges(import_data);
	for (int i = 0; i < num_edges; i++) {
		int v1, v2;
		mesh_importer->getEdgeVerts(import_data, i, &v1, &v2);
		edgeIndexMap_put(&mesh_descr->edge_index_map, v1, v2, i);
	}

	// Import polys from external mesh to Carve.
	int verts_of_poly_static[MAX_STATIC_VERTS];
	int *verts_of_poly_dynamic = NULL;
	int verts_of_poly_dynamic_size = 0;

	int num_polys = mesh_importer->getNumPolys(import_data);
	std::vector<int> face_indices;
	face_indices.reserve(3 * num_polys);
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

	mesh_descr->poly = new MeshSet<3> (vertices,
	                                   num_polys,
	                                   face_indices);

	return mesh_descr;

#undef MAX_STATIC_VERTS
}

void carve_deleteMesh(CarveMeshDescr *mesh_descr)
{
	delete mesh_descr->poly;
	delete mesh_descr;
}

bool carve_performBooleanOperation(CarveMeshDescr *left_mesh,
                                   CarveMeshDescr *right_mesh,
                                   int operation,
                                   CarveMeshDescr **output_mesh)
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

	CarveMeshDescr *output_descr = new CarveMeshDescr;
	output_descr->poly = NULL;
	try {
		MeshSet<3> *left = left_mesh->poly, *right = right_mesh->poly;
		carve::geom3d::Vector min, max;

		// TODO(sergey): Make importer/exporter to care about re-scale to save extra
		// mesh iteration here.
		carve_getRescaleMinMax(left, right, &min, &max);

		carve::rescale::rescale scaler(min.x, min.y, min.z, max.x, max.y, max.z);
		carve::rescale::fwd fwd_r(scaler);
		carve::rescale::rev rev_r(scaler);

		left->transform(fwd_r);
		right->transform(fwd_r);

		// Initialize attributes for maping from boolean result mesh back to
		// original geometry indices.
		initOrigIndexMapping(left_mesh, right_mesh,
		                     &output_descr->orig_face_vert_mapping,
		                     &output_descr->orig_face_edge_mapping,
		                     &output_descr->orig_face_mapping);

		carve::csg::CSG csg;

		output_descr->orig_face_vert_mapping.installHooks(csg);
		output_descr->orig_face_edge_mapping.installHooks(csg);
		output_descr->orig_face_mapping.installHooks(csg);

		// Prepare operands for actual boolean operation.
		//
		// It's needed because operands might consist of several intersecting meshes and in case
		// of another operands intersect an edge loop of intersecting that meshes tessellation of
		// operation result can't be done properly. The only way to make such situations working is
		// to union intersecting meshes of the same operand.
		carve_unionIntersections(&csg, &left, &right);
		left_mesh->poly = left;
		right_mesh->poly = right;

		output_descr->poly = csg.compute(left, right, op, NULL, carve::csg::CSG::CLASSIFY_EDGE);
		if (output_descr->poly) {
			output_descr->poly->transform(rev_r);
		}
	}
	catch (carve::exception e) {
		std::cerr << "CSG failed, exception " << e.str() << std::endl;
	}
	catch (...) {
		delete output_descr;
		throw "Unknown error in Carve library";
	}

	*output_mesh = output_descr;

	return output_descr->poly != NULL;
}

void carve_exportMesh(CarveMeshDescr *mesh_descr,
                      CarveMeshExporter *mesh_exporter,
                      struct ExportMeshData *export_data)
{
	MeshSet<3> *poly = mesh_descr->poly;

	int num_vertices = poly->vertex_storage.size();
	int num_edges = 0, num_loops = 0, num_polys = 0;

	OrigIndex origindex_none = std::make_pair((int)CARVE_MESH_NONE, -1);
	std::pair<OrigIndex, OrigIndex> origindex_pair_none =
		std::make_pair(origindex_none, origindex_none);

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

	// Get mapping from edge denoted by vertex pair to original edge index,
	// also get mapping from face-vertex back to original vertex index.
	//
	// This is needed because internally Carve interpolates data for per-face
	// edges rather then having some global edge storage.
	std::map<MeshSet<3>::vertex_t*, OrigIndex> vert_origindex_map;
	std::map<VertexPair, OrigIndex> edge_origindex_map;
	for (MeshSet<3>::face_iter face_iter = poly->faceBegin();
	     face_iter != poly->faceEnd();
	     ++face_iter)
	{
		MeshSet<3>::face_t *face = *face_iter;
		for (MeshSet<3>::face_t::edge_iter_t edge_iter = face->begin();
		     edge_iter != face->end();
		     ++edge_iter)
		{
			MeshSet<3>::edge_t &edge = *edge_iter;

			// TODO(sergey): Use pointers instead.
			vert_origindex_map[edge.vert] =
				mesh_descr->orig_face_vert_mapping.getAttribute(face,
				                                                edge_iter.idx(),
				                                                origindex_none);

			const OrigIndex &orig_edge_index =
				mesh_descr->orig_face_edge_mapping.getAttribute(face,
				                                                edge_iter.idx(),
				                                                origindex_pair_none).first;
			MeshSet<3>::vertex_t *v1 = edge.vert;
			MeshSet<3>::vertex_t *v2 = edge.next->vert;

			edgeIndexMap_put(&edge_origindex_map, v1, v2, orig_edge_index);
		}
	}

	// Export all the vertices.
	std::vector<MeshSet<3>::vertex_t>::iterator vertex_iter = poly->vertex_storage.begin();
	for (int i = 0; vertex_iter != poly->vertex_storage.end(); ++i, ++vertex_iter) {
		MeshSet<3>::vertex_t *vertex = &(*vertex_iter);

		OrigIndex orig_vert_index;
		std::map<MeshSet<3>::vertex_t*, OrigIndex>::iterator found =
			vert_origindex_map.find(vertex);
		if (found != vert_origindex_map.end()) {
			orig_vert_index = found->second;
		} else {
			orig_vert_index = origindex_none;
		}

		float coord[3];
		coord[0] = vertex->v[0];
		coord[1] = vertex->v[1];
		coord[2] = vertex->v[2];
		mesh_exporter->setVert(export_data, i, coord,
			                   orig_vert_index.first,
			                   orig_vert_index.second);
	}

	// Export all the edges.
	std::map<VertexPair, int > edge_map;
	for (int i = 0, edge_index = 0; i < poly->meshes.size(); ++i) {
		carve::mesh::Mesh<3> *mesh = poly->meshes[i];
		for (int j = 0; j < mesh->closed_edges.size(); ++j, ++edge_index) {
			MeshSet<3>::edge_t *edge = mesh->closed_edges.at(j);
			MeshSet<3>::vertex_t *v1 = edge->vert;
			MeshSet<3>::vertex_t *v2 = edge->next->vert;

			const OrigIndex &orig_edge_index =
				edgeIndexMap_get(edge_origindex_map, v1, v2);

			mesh_exporter->setEdge(export_data,
			                       edge_index,
			                       indexOf(v1, poly->vertex_storage),
			                       indexOf(v2, poly->vertex_storage),
			                       orig_edge_index.first,
			                       orig_edge_index.second);

			edgeIndexMap_put(&edge_map, v1, v2, edge_index);
		}
	}

	// Export all the loops and polys.
	MeshSet<3>::face_iter face_iter = poly->faceBegin();
	for (int loop_index = 0, poly_index = 0;
	     face_iter != poly->faceEnd();
	     ++face_iter, ++poly_index)
	{
		MeshSet<3>::face_t *face = *face_iter;
		const OrigIndex &orig_face_index =
			mesh_descr->orig_face_mapping.getAttribute(face, origindex_none);

		mesh_exporter->setPoly(export_data,
		                       poly_index, loop_index, face->n_edges,
		                       orig_face_index.first, orig_face_index.second);

		for (MeshSet<3>::face_t::edge_iter_t edge_iter = face->begin();
		     edge_iter != face->end();
		     ++edge_iter, ++loop_index)
		{
			MeshSet<3>::edge_t &edge = *edge_iter;
			const OrigIndex &orig_loop_index =
				mesh_descr->orig_face_edge_mapping.getAttribute(face,
				                                                edge_iter.idx(),
				                                                origindex_pair_none).second;

			mesh_exporter->setLoop(export_data,
		                           loop_index,
		                           indexOf(edge.vert, poly->vertex_storage),
		                           edgeIndexMap_get(edge_map, edge.vert, edge.next->vert),
		                           orig_loop_index.first,
		                           orig_loop_index.second);
		}

		mesh_exporter->interpPoly(export_data, poly_index,
		                          orig_face_index.first, orig_face_index.second);
	}
}
