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

#ifndef __ABC_NURBS_WRITER_H__
#define __ABC_NURBS_WRITER_H__

#include "abc_shape.h"

class AbcNurbsWriter : public AbcShapeWriter {
	std::vector<Alembic::AbcGeom::ONuPatchSchema> m_nurbs_schema;
	bool m_is_animated;

public:
	AbcNurbsWriter(Scene *sce, Object *obj, AbcTransformWriter *parent, Alembic::Util::uint32_t timeSampling, AbcExportOptions &opts);
	~AbcNurbsWriter();

private:
	virtual void do_write();

    bool isAnimated() const;

	void writeNurbs();
};

#endif  /* __ABC_NURBS_WRITER_H__ */
