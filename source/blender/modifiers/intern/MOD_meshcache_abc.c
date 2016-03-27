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
 * Contributor(s): Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/modifiers/intern/MOD_meshcache_abc.c
 *  \ingroup modifiers
 */

#include <stdio.h>  /* needed for forward declaration of FILE */

#include "DNA_modifier_types.h"

#include "BLI_utildefines.h"

#include "MOD_meshcache_util.h"

#include "ABC_alembic.h"

bool MOD_meshcache_read_abc_index(const char *filepath, const char *sub_object,
                                  float (*vertexCos)[3], const int verts_tot,
                                  const int index, const float factor,
                                  const char **err_str, ModifierData *md)
{
    abcGetVertexCache(filepath, factor, (void *)md, vertexCos, verts_tot, sub_object, 0);

	UNUSED_VARS(index, err_str);
	return true;
}

bool MOD_meshcache_read_abc_frame(const char *filepath, const char *sub_object,
                                  float (*vertexCos)[3], const int verts_tot, const char interp,
                                  const float frame,
                                  const char **err_str, ModifierData *md)
{
	return MOD_meshcache_read_abc_index(filepath, sub_object, vertexCos, verts_tot, interp, frame, err_str, md);
}

bool MOD_meshcache_read_abc_times(const char *filepath, const char *sub_object,
                                  float (*vertexCos)[3], const int verts_tot, const char interp,
                                  const float time, const float fps, const char time_mode,
                                  const char **err_str, ModifierData *md)
{
	if (!checkSubobjectValid(filepath, sub_object)) {
		return false;
	}

	float frame;

	switch (time_mode) {
		/* TODO */
		case MOD_MESHCACHE_TIME_FRAME:
		case MOD_MESHCACHE_TIME_SECONDS:
		case MOD_MESHCACHE_TIME_FACTOR:
		default:
		{
			frame = time;
			break;
		}
	}

	UNUSED_VARS(fps);

	return MOD_meshcache_read_abc_frame(filepath, sub_object, vertexCos, verts_tot, interp, frame, err_str, md);
}
