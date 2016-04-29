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
}

using Alembic::AbcGeom::OObject;
using Alembic::AbcGeom::OXform;

AbcTransformWriter::AbcTransformWriter(Object *obj,
                                       OObject abcParent,
                                       AbcTransformWriter *writerParent,
                                       unsigned int timeSampling,
                                       AbcExportOptions &opts)
    : AbcObjectWriter(obj, opts)
{
	m_is_animated = hasAnimation(m_object);
	m_parent = NULL;
	m_no_parent_invert = false;

	if (!m_is_animated) {
		timeSampling = 0;
	}

    m_xform = OXform(abcParent, get_id_name(m_object), timeSampling);
	m_schema = m_xform.getSchema();

	if (writerParent) {
		writerParent->addChild(this);
	}
}

void AbcTransformWriter::do_write()
{
	if (m_first_frame) {
		if (hasProperties(reinterpret_cast<ID *>(m_object))) {
			if (m_options.export_props_as_geo_params) {
				writeProperties(reinterpret_cast<ID *>(m_object), m_schema.getArbGeomParams());
			}
			else {
				writeProperties(reinterpret_cast<ID *>(m_object), m_schema.getUserProperties());
			}
		}

		m_visibility = Alembic::AbcGeom::CreateVisibilityProperty(m_xform, m_xform.getSchema().getTimeSampling());
	}

	bool visibility = ! (m_object->restrictflag & OB_RESTRICT_VIEW);
	m_visibility.set(visibility);

	if (!m_first_frame && !m_is_animated) {
		return;
	}

	/* get local matrix */
    float mat[4][4];
	if (m_object->parent) {
		float invmat[4][4];
        invert_m4_m4(invmat, m_object->parent->obmat);
        mul_m4_m4m4(mat, invmat, m_object->obmat);
    }
    else {
        copy_m4_m4(mat, m_object->obmat);
    }

	float smat[4][4];
	scale_m4_fl(smat, m_options.global_scale);
	mul_m4_m4m4(mat, smat, mat);

	if (m_options.do_convert_axis) {
		mul_m4_m3m4(mat, m_options.convert_matrix, mat);
	}

    m_matrix = convert_matrix(mat);

	m_sample.setMatrix(m_matrix);
	m_schema.set(m_sample);
}

Imath::Box3d AbcTransformWriter::bounds() const
{
	Imath::Box3d bounds;

	for (int i = 0; i < m_children.size(); ++i) {
		Imath::Box3d box(m_children[i]->bounds());
		bounds.extendBy(box);
	}

	return Imath::transform(bounds, m_matrix);
}

bool AbcTransformWriter::hasAnimation(Object *obj) const
{
	/* TODO: implement this */
	return true;
}
