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

#include "carve-util.h"

#include <carve/csg.hpp>
#include <carve/rtree.hpp>

using carve::csg::Intersections;
using carve::geom::aabb;
using carve::geom::RTreeNode;
using carve::geom3d::Vector;
using carve::mesh::Face;
using carve::mesh::MeshSet;

typedef std::map< MeshSet<3>::mesh_t*, RTreeNode<3, Face<3> *> * > RTreeCache;
typedef std::map< MeshSet<3>::mesh_t*, bool > IntersectCache;

static void meshSetMinMax(const MeshSet<3> *mesh,
                          Vector *min,
                          Vector *max)
{
	for (uint i = 0; i < mesh->vertex_storage.size(); ++i) {
		min->x = std::min(min->x, mesh->vertex_storage[i].v.x);
		min->y = std::min(min->y, mesh->vertex_storage[i].v.y);
		min->z = std::min(min->z, mesh->vertex_storage[i].v.z);
		max->x = std::max(max->x, mesh->vertex_storage[i].v.x);
		max->y = std::max(max->y, mesh->vertex_storage[i].v.y);
		max->z = std::max(max->z, mesh->vertex_storage[i].v.z);
	}
}

void carve_getRescaleMinMax(const MeshSet<3> *left,
                            const MeshSet<3> *right,
                            Vector *min,
                            Vector *max)
{
	min->x = max->x = left->vertex_storage[0].v.x;
	min->y = max->y = left->vertex_storage[0].v.y;
	min->z = max->z = left->vertex_storage[0].v.z;

	meshSetMinMax(left, min, max);
	meshSetMinMax(right, min, max);

	// Make sure we don't scale object with zero scale.
	if ((min->x - max->x) < carve::EPSILON) {
		min->x = -1.0;
		max->x = 1.0;
	}
	if ((min->y - max->y) < carve::EPSILON) {
		min->y = -1.0;
		max->y = 1.0;
	}
	if ((min->z - max->z) < carve::EPSILON) {
		min->z = -1.0;
		max->z = 1.0;
	}
}

namespace {

void copyMeshes(const std::vector<MeshSet<3>::mesh_t*> &meshes,
                std::vector<MeshSet<3>::mesh_t*> *new_meshes)
{
	std::vector<MeshSet<3>::mesh_t*>::const_iterator it = meshes.begin();
	new_meshes->reserve(meshes.size());
	for (; it != meshes.end(); it++) {
		MeshSet<3>::mesh_t *mesh = *it;
		MeshSet<3>::mesh_t *new_mesh = new MeshSet<3>::mesh_t(mesh->faces);

		new_meshes->push_back(new_mesh);
	}
}

MeshSet<3> *meshSetFromMeshes(const std::vector<MeshSet<3>::mesh_t*> &meshes)
{
	std::vector<MeshSet<3>::mesh_t*> new_meshes;

	copyMeshes(meshes, &new_meshes);

	return new MeshSet<3>(new_meshes);
}

MeshSet<3> *meshSetFromTwoMeshes(const std::vector<MeshSet<3>::mesh_t*> &left_meshes,
                                 const std::vector<MeshSet<3>::mesh_t*> &right_meshes)
{
	std::vector<MeshSet<3>::mesh_t*> new_meshes;

	copyMeshes(left_meshes, &new_meshes);
	copyMeshes(right_meshes, &new_meshes);

	return new MeshSet<3>(new_meshes);
}

bool checkEdgeFaceIntersections_do(Intersections &intersections,
                                   MeshSet<3>::face_t *face_a,
                                   MeshSet<3>::edge_t *edge_b)
{
	if (intersections.intersects(edge_b, face_a))
		return true;

	carve::mesh::MeshSet<3>::vertex_t::vector_t _p;
	if (face_a->simpleLineSegmentIntersection(carve::geom3d::LineSegment(edge_b->v1()->v, edge_b->v2()->v), _p))
		return true;

	return false;
}

bool checkEdgeFaceIntersections(Intersections &intersections,
                                MeshSet<3>::face_t *face_a,
                                MeshSet<3>::face_t *face_b)
{
	MeshSet<3>::edge_t *edge_b;

	edge_b = face_b->edge;
	do {
		if (checkEdgeFaceIntersections_do(intersections, face_a, edge_b))
			return true;
		edge_b = edge_b->next;
	} while (edge_b != face_b->edge);

	return false;
}

inline bool facesAreCoplanar(const MeshSet<3>::face_t *a, const MeshSet<3>::face_t *b)
{
	carve::geom3d::Ray temp;
	// XXX: Find a better definition. This may be a source of problems
	// if floating point inaccuracies cause an incorrect answer.
	return !carve::geom3d::planeIntersection(a->plane, b->plane, temp);
}

bool checkMeshSetInterseciton_do(Intersections &intersections,
                                 const RTreeNode<3, Face<3> *> *a_node,
                                 const RTreeNode<3, Face<3> *> *b_node,
                                 bool descend_a = true)
{
	if (!a_node->bbox.intersects(b_node->bbox)) {
		return false;
	}

	if (a_node->child && (descend_a || !b_node->child)) {
		for (RTreeNode<3, Face<3> *> *node = a_node->child; node; node = node->sibling) {
			if (checkMeshSetInterseciton_do(intersections, node, b_node, false)) {
				return true;
			}
		}
	}
	else if (b_node->child) {
		for (RTreeNode<3, Face<3> *> *node = b_node->child; node; node = node->sibling) {
			if (checkMeshSetInterseciton_do(intersections, a_node, node, true)) {
				return true;
			}
		}
	}
	else {
		for (size_t i = 0; i < a_node->data.size(); ++i) {
			MeshSet<3>::face_t *fa = a_node->data[i];
			aabb<3> aabb_a = fa->getAABB();
			if (aabb_a.maxAxisSeparation(b_node->bbox) > carve::EPSILON) {
				continue;
			}

			for (size_t j = 0; j < b_node->data.size(); ++j) {
				MeshSet<3>::face_t *fb = b_node->data[j];
				aabb<3> aabb_b = fb->getAABB();
				if (aabb_b.maxAxisSeparation(aabb_a) > carve::EPSILON) {
					continue;
				}

				std::pair<double, double> a_ra = fa->rangeInDirection(fa->plane.N, fa->edge->vert->v);
				std::pair<double, double> b_ra = fb->rangeInDirection(fa->plane.N, fa->edge->vert->v);
				if (carve::rangeSeparation(a_ra, b_ra) > carve::EPSILON) {
					continue;
				}

				std::pair<double, double> a_rb = fa->rangeInDirection(fb->plane.N, fb->edge->vert->v);
				std::pair<double, double> b_rb = fb->rangeInDirection(fb->plane.N, fb->edge->vert->v);
				if (carve::rangeSeparation(a_rb, b_rb) > carve::EPSILON) {
					continue;
				}

				if (!facesAreCoplanar(fa, fb)) {
					if (checkEdgeFaceIntersections(intersections, fa, fb)) {
						return true;
					}
				}
			}
		}
	}

	return false;
}

bool checkMeshSetInterseciton(RTreeNode<3, Face<3> *> *rtree_a, RTreeNode<3, Face<3> *> *rtree_b)
{
	Intersections intersections;
	return checkMeshSetInterseciton_do(intersections, rtree_a, rtree_b);
}

void getIntersectedOperandMeshes(std::vector<MeshSet<3>::mesh_t*> *meshes,
                                 const MeshSet<3>::aabb_t &otherAABB,
                                 std::vector<MeshSet<3>::mesh_t*> *operandMeshes,
                                 RTreeCache *rtree_cache,
                                 IntersectCache *intersect_cache)
{
	std::vector<MeshSet<3>::mesh_t*>::iterator it = meshes->begin();
	std::vector< RTreeNode<3, Face<3> *> *> meshRTree;

	while (it != meshes->end()) {
		MeshSet<3>::mesh_t *mesh = *it;
		bool isAdded = false;

		RTreeNode<3, Face<3> *> *rtree;
		bool intersects;

		RTreeCache::iterator rtree_found = rtree_cache->find(mesh);
		if (rtree_found != rtree_cache->end()) {
			rtree = rtree_found->second;
		}
		else {
			rtree = RTreeNode<3, Face<3> *>::construct_STR(mesh->faces.begin(), mesh->faces.end(), 4, 4);
			(*rtree_cache)[mesh] = rtree;
		}

		IntersectCache::iterator intersect_found = intersect_cache->find(mesh);
		if (intersect_found != intersect_cache->end()) {
			intersects = intersect_found->second;
		} else {
			intersects = rtree->bbox.intersects(otherAABB);
			(*intersect_cache)[mesh] = intersects;
		}

		if (intersects) {
			bool isIntersect = false;

			std::vector<MeshSet<3>::mesh_t*>::iterator operand_it = operandMeshes->begin();
			std::vector<RTreeNode<3, Face<3> *> *>::iterator tree_it = meshRTree.begin();
			for (; operand_it!=operandMeshes->end(); operand_it++, tree_it++) {
				RTreeNode<3, Face<3> *> *operandRTree = *tree_it;

				if (checkMeshSetInterseciton(rtree, operandRTree)) {
					isIntersect = true;
					break;
				}
			}

			if (!isIntersect) {
				operandMeshes->push_back(mesh);
				meshRTree.push_back(rtree);

				it = meshes->erase(it);
				isAdded = true;
			}
		}

		if (!isAdded) {
			//delete rtree;
			it++;
		}
	}

	std::vector<RTreeNode<3, Face<3> *> *>::iterator tree_it = meshRTree.begin();
	for (; tree_it != meshRTree.end(); tree_it++) {
		//delete *tree_it;
	}
}

MeshSet<3> *getIntersectedOperand(std::vector<MeshSet<3>::mesh_t*> *meshes,
                                  const MeshSet<3>::aabb_t &otherAABB,
                                  RTreeCache *rtree_cache,
                                  IntersectCache *intersect_cache)
{
	std::vector<MeshSet<3>::mesh_t*> operandMeshes;
	getIntersectedOperandMeshes(meshes, otherAABB, &operandMeshes, rtree_cache, intersect_cache);

	if (operandMeshes.size() == 0)
		return NULL;

	return meshSetFromMeshes(operandMeshes);
}

MeshSet<3> *unionIntersectingMeshes(carve::csg::CSG *csg,
                                    MeshSet<3> *poly,
                                    const MeshSet<3>::aabb_t &otherAABB)
{
	if (poly->meshes.size() <= 1) {
		return poly;
	}

	std::vector<MeshSet<3>::mesh_t*> orig_meshes =
			std::vector<MeshSet<3>::mesh_t*>(poly->meshes.begin(), poly->meshes.end());

	RTreeCache rtree_cache;
	IntersectCache intersect_cache;

	MeshSet<3> *left = getIntersectedOperand(&orig_meshes,
	                                         otherAABB,
	                                         &rtree_cache,
	                                         &intersect_cache);

	if (!left) {
		// No maniforlds which intersects another object at all.
		return poly;
	}

	while (orig_meshes.size()) {
		MeshSet<3> *right = getIntersectedOperand(&orig_meshes,
		                                          otherAABB,
		                                          &rtree_cache,
		                                          &intersect_cache);

		if (!right) {
			// No more intersecting manifolds which intersects other object
			break;
		}

		try {
			if (left->meshes.size()==0) {
				delete left;

				left = right;
			}
			else {
				MeshSet<3> *result = csg->compute(left, right,
				                                  carve::csg::CSG::UNION,
				                                  NULL, carve::csg::CSG::CLASSIFY_EDGE);

				delete left;
				delete right;

				left = result;
			}
		}
		catch (carve::exception e) {
			std::cerr << "CSG failed, exception " << e.str() << std::endl;

			MeshSet<3> *result = meshSetFromTwoMeshes(left->meshes, right->meshes);

			delete left;
			delete right;

			left = result;
		}
		catch (...) {
			delete left;
			delete right;

			throw "Unknown error in Carve library";
		}
	}

	for (RTreeCache::iterator it = rtree_cache.begin();
	     it != rtree_cache.end();
	     it++)
	{
		delete it->second;
	}

	// Append all meshes which doesn't have intersection with another operand as-is.
	if (orig_meshes.size()) {
		MeshSet<3> *result = meshSetFromTwoMeshes(left->meshes, orig_meshes);

		delete left;
		left = result;
	}

	return left;
}

}  // namespace

// TODO(sergey): This function is to be totally re-implemented to make it
// more clear what's going on and hopefully optimize it as well.
void carve_unionIntersections(carve::csg::CSG *csg,
                              MeshSet<3> **left_r,
                             MeshSet<3> **right_r)
{
	MeshSet<3> *left = *left_r, *right = *right_r;

	MeshSet<3>::aabb_t leftAABB = left->getAABB();
	MeshSet<3>::aabb_t rightAABB = right->getAABB();;

	left = unionIntersectingMeshes(csg, left, rightAABB);
	right = unionIntersectingMeshes(csg, right, leftAABB);

	if (left != *left_r) {
		delete *left_r;
	}

	if (right != *right_r)
		delete *right_r;

	*left_r = left;
	*right_r = right;
}
