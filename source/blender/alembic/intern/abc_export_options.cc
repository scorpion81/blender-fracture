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
 * Contributor(s): Esteban Tovagliari, Cedric Paille, Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "abc_export_options.h"

#include <string>

#include "abc_util.h"

extern "C" {
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_idprop.h"
}

AbcExportOptions::AbcExportOptions(Scene *scene)
    : m_scene(scene)
{
	selected_only = false;
	visible_layers_only = false;
	renderable_only = false;

	startframe = 1;
	endframe = 10;
	xform_frame_step = 1;
	shape_frame_step = 1;
	shutter_open = 0.0;
	shutter_close = 1.0;
	global_scale = 1.0f;

	flatten_hierarchy = false;

	export_normals = true;
	export_uvs = true;
	export_vcols = true;
	export_face_sets = false;
	export_mat_indices = false;
	export_vweigths = false;

	export_subsurfs_as_meshes = false;
	export_props_as_geo_params = true;
}

bool AbcExportOptions::exportTransform(Object *obj) const
{
	return !isAbcRoot(obj);
}

bool AbcExportOptions::checkIsAbcRoot(Object *ob)
{
	Object *parent = ob;

	while (parent) {
		if (isAbcRoot(parent)) {
			return true;
		}

		parent = parent->parent;
	}

	return false;
}

bool AbcExportOptions::isAbcRoot(Object *obj) const
{
	ID *id = reinterpret_cast<ID *>(obj);
	IDProperty *xport_props = IDP_GetProperties(id, false);

	if (!xport_props) {
		return false;
	}

	IDProperty *enable_xport = IDP_GetPropertyFromGroup(xport_props, "isAbcRoot");

	if (enable_xport) {
		return true;
	}

	return false;
}

bool AbcExportOptions::exportObject(Object *obj) const
{
	if (!exportTransform(obj)) {
		return false;
	}

	if (selected_only && !parent_selected(obj)) {
		return false;
	}

	if (visible_layers_only && !(m_scene->lay & obj->lay)) {
		return false;
	}

	if (renderable_only && (obj->restrictflag & 4)) {
		return false;
	}

	return true;
}
