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
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file DNA_submesh_types.h
 *  \ingroup DNA
 */

#ifndef __DNA_SUBMESH_TYPES_H__
#define __DNA_SUBMESH_TYPES_H__

#include "DNA_defs.h"
#include "DNA_listBase.h"
#include "DNA_ID.h"
#include "DNA_customdata_types.h"

typedef struct SMHeader {
	void *data; /* customdata layers */
	int index; //index in SubMesh
	char pad[4];
} SMHeader;

typedef struct SMVert {
	SMHeader head;
	float co[3];
	float no[3];
	int e;
} SMVert;

typedef struct SMEdge {
	SMHeader head;
	int v1, v2;
	int l;
} SMEdge;

typedef struct SMLoop {
	SMHeader head;
	int v;
	int e;
	int f;

} SMLoop;

typedef struct SMFace {
	SMHeader head;
	int   l_first;
	int   len;
	float no[3];
	short mat_nr;
	char pad[2];
} SMFace;

typedef struct SMesh {
	int totvert, totedge, totloop, totface;
	struct SMVert **vpool;
	struct SMEdge **epool;
	struct SMLoop **lpool;
	struct SMFace **fpool;

	CustomData vdata, edata, ldata, pdata;

} SMesh;

#endif /*__DNA_SUBMESH_TYPES_H__*/
