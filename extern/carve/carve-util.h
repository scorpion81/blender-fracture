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

#ifndef __CARVE_UTIL_H__
#define __CARVE_UTIL_H__
#include <carve/csg.hpp>
#include <carve/geom3d.hpp>
#include <carve/mesh.hpp>

void carve_getRescaleMinMax(const carve::mesh::MeshSet<3> *left,
                            const carve::mesh::MeshSet<3> *right,
                            carve::geom3d::Vector *min,
                            carve::geom3d::Vector *max);

void carve_unionIntersections(carve::csg::CSG *csg,
                              carve::mesh::MeshSet<3> **left_r, 
                              carve::mesh::MeshSet<3> **right_r);
 
#endif  // __CARVE_UTIL_H__
