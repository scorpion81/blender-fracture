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
 * Copyright (C) 2014 by Martin Felke.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/BKE_fracture_util.h
 *  \ingroup blenkernel
 *  \brief CSG operations
 */

#ifndef BKE_FRACTURE_UTIL_H
#define BKE_FRACTURE_UTIL_H

#include "DNA_fracture_types.h"

Shard *BKE_fracture_shard_boolean(Object *obj, DerivedMesh *dm_parent, Shard* child, short inner_material_index, int num_cuts, float fractal,
                                  Shard **other, float mat[4][4], float radius, bool use_smooth_inner, int num_levels, char uv_layer[], int solver, float thresh);
Shard *BKE_fracture_shard_bisect(struct BMesh *bm_orig, Shard* child, float obmat[4][4], bool use_fill,
                                 bool clear_inner, bool clear_outer, int cutlimit, float centroid[],
                                 short inner_mat_index, char uv_layer[], struct KDTree *preselect_tree, float normal[]);

#endif /* BKE_FRACTURE_UTIL_H*/
