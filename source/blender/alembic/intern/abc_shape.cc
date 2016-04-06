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

#include "abc_shape.h"

#include "abc_transform.h"

AbcShapeWriter::AbcShapeWriter(Scene *sce,
                               Object *obj,
                               AbcTransformWriter *parent,
                               Alembic::Util::uint32_t timeSampling,
                               AbcExportOptions &opts)
    : AbcObjectWriter(obj, opts)
{
	m_first_frame = true;
	m_scene = sce;
	m_time_sampling = timeSampling;
	parent->addChild(this);
}

void AbcShapeWriter::calcBounds(const std::vector<float> &points)
{
	m_bounds.makeEmpty();

	for (int i = 0, e = points.size() / 3; i < e; i += 3) {
		m_bounds.min.x = std::min(m_bounds.min.x, (double) points[i]);
		m_bounds.min.y = std::min(m_bounds.min.y, (double) points[i+1]);
		m_bounds.min.z = std::min(m_bounds.min.z, (double) points[i+2]);

		m_bounds.max.x = std::max(m_bounds.max.x, (double) points[i]);
		m_bounds.max.y = std::max(m_bounds.max.y, (double) points[i+1]);
		m_bounds.max.z = std::max(m_bounds.max.z, (double) points[i+2]);
	}
}
