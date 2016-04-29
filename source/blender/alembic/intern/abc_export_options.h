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

#ifndef __ABC_EXPORT_OPTIONS_H__
#define __ABC_EXPORT_OPTIONS_H__

struct Object;
struct Scene;

struct AbcExportOptions {
	explicit AbcExportOptions(Scene *scene);

	bool exportTransform(Object *obj) const;
	bool isAbcRoot(Object *obj) const;
	bool exportObject(Object *obj) const;
	bool checkIsAbcRoot(Object *ob);

	bool selected_only;
	bool visible_layers_only;
	bool renderable_only;

	double startframe, endframe;
	double xform_frame_step;
	double shape_frame_step;
	double shutter_open;
	double shutter_close;
	float global_scale;

	bool flatten_hierarchy;

	bool export_normals;
	bool export_uvs;
	bool export_vcols;
	bool export_face_sets;
	bool export_mat_indices;
	bool export_vweigths;

	bool export_subsurfs_as_meshes;
	bool export_props_as_geo_params;
	bool use_subdiv_schema;
	bool export_child_hairs;
	bool export_ogawa;
	bool pack_uv;

	bool do_convert_axis;
	float convert_matrix[3][3];

private:
	Scene *m_scene;
};

#endif  /* __ABC_EXPORT_OPTIONS_H__ */
