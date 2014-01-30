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

using carve::mesh::MeshSet;
using carve::geom3d::Vector;

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
