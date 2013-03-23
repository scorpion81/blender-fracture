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
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s):
 *
 * ***** END GPL LICENSE BLOCK *****
 */
#ifndef __BKE_SUBMESH_H__
#define __BKE_SUBMESH_H__

/** \file BKE_submesh.h
 *  \ingroup bke
 */

#include "DNA_submesh_types.h"
#include "BLI_utildefines.h"
#include "bmesh.h"

struct BMesh;
struct SMesh;

SMesh* BKE_bmesh_to_submesh(BMesh* bm);
BMesh* BKE_submesh_to_bmesh(SMesh* sm);
void BKE_submesh_free(SMesh* sm);

#endif /* __BKE_SUBMESH_H__ */
