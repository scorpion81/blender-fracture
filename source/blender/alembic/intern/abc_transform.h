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

#ifndef __ABC_TRANSFORM_WRITER_H__
#define __ABC_TRANSFORM_WRITER_H__

#include "abc_object.h"

#include <Alembic/AbcGeom/All.h>

class AbcTransformWriter : public AbcObjectWriter {
	Alembic::AbcGeom::OXform m_xform;
    Alembic::AbcGeom::OXformSchema m_schema;
    Alembic::AbcGeom::XformSample m_sample;
	Alembic::AbcGeom::OVisibilityProperty m_visibility;
	Alembic::Abc::M44d m_matrix;

	bool m_is_animated;
	bool m_no_parent_invert;
	Object *m_parent;
	bool m_visible;

public:
	AbcTransformWriter(Object *obj,
	                   Alembic::Abc::OObject abcParent,
	                   AbcTransformWriter *writerParent,
	                   unsigned int timeSampling,
	                   AbcExportOptions &opts);

	Alembic::AbcGeom::OXform &alembicXform() { return m_xform;}
    virtual Imath::Box3d bounds() const;
    void setParent(Object *p) { m_parent = p; }

private:
	virtual void do_write();

	bool hasAnimation(Object *obj) const;
};

#endif  /* __ABC_TRANSFORM_WRITER_H__ */
