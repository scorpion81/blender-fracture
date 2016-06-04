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

#include "abc_transform.h"

#include <OpenEXR/ImathBoxAlgo.h>

#include "abc_util.h"

extern "C" {
#include "DNA_object_types.h"

#include "BLI_math.h"

#include "BKE_object.h"
}

using Alembic::AbcGeom::OObject;
using Alembic::AbcGeom::OXform;

/* ************************************************************************** */

AbcTransformWriter::AbcTransformWriter(Object *ob,
                                       const OObject &abc_parent,
                                       AbcTransformWriter *parent,
                                       unsigned int sampling_time,
                                       ExportSettings &settings)
    : AbcObjectWriter(NULL, ob, sampling_time, settings, parent)
{
	m_is_animated = hasAnimation(m_object);
	m_parent = NULL;

	if (!m_is_animated) {
		sampling_time = 0;
	}

    m_xform = OXform(abc_parent, get_id_name(m_object), sampling_time);
	m_schema = m_xform.getSchema();
}

void AbcTransformWriter::do_write()
{
	if (m_first_frame) {
		if (hasProperties(reinterpret_cast<ID *>(m_object))) {
			if (m_settings.export_props_as_geo_params) {
				writeProperties(reinterpret_cast<ID *>(m_object), m_schema.getArbGeomParams());
			}
			else {
				writeProperties(reinterpret_cast<ID *>(m_object), m_schema.getUserProperties());
			}
		}

		m_visibility = Alembic::AbcGeom::CreateVisibilityProperty(m_xform, m_xform.getSchema().getTimeSampling());
	}

	m_visibility.set(!(m_object->restrictflag & OB_RESTRICT_VIEW));

	if (!m_first_frame && !m_is_animated) {
		return;
	}

	float mat[4][4];
	create_transform_matrix(m_object, mat);

	if (m_object->type == OB_CAMERA) {
		float rot_mat[4][4];
		unit_m4(rot_mat);
		rotate_m4(rot_mat, 'X', -M_PI_2);
		mul_m4_m4m4(mat, mat, rot_mat);
	}

    m_matrix = convert_matrix(mat);

	m_sample.setMatrix(m_matrix);
	m_schema.set(m_sample);
}

Imath::Box3d AbcTransformWriter::bounds()
{
	Imath::Box3d bounds;

	for (int i = 0; i < m_children.size(); ++i) {
		Imath::Box3d box(m_children[i]->bounds());
		bounds.extendBy(box);
	}

	return Imath::transform(bounds, m_matrix);
}

bool AbcTransformWriter::hasAnimation(Object */*ob*/) const
{
	/* TODO: implement this */
	return true;
}

/* ************************************************************************** */

AbcEmptyReader::AbcEmptyReader(const Alembic::Abc::IObject &object, ImportSettings &settings)
    : AbcObjectReader(object, settings)
{}

bool AbcEmptyReader::valid() const
{
	return true; // TODO? m_schema.valid();
}

void AbcEmptyReader::readObjectData(Main *bmain, Scene *scene, float /*time*/)
{
	m_object = BKE_object_add(bmain, scene, OB_EMPTY, m_object_name.c_str());
}
