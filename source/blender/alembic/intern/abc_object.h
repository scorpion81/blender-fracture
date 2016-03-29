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

#ifndef __ABC_OBJECT_WRITER_H__
#define __ABC_OBJECT_WRITER_H__

#include <Alembic/Abc/All.h>

#include "abc_export_options.h"

extern "C" {
#include "DNA_ID.h"
}

struct Main;

class AbcObjectWriter {
protected:
	Object *m_object;
    AbcExportOptions &m_options;

    Alembic::Abc::Box3d m_bounds;
    std::vector<AbcObjectWriter *> m_children;

    std::vector< std::pair<std::string, IDProperty *> > m_props;

    bool m_first_frame;
    bool m_rotate_matrix;

public:
    AbcObjectWriter(Object *obj, AbcExportOptions &opts);
    virtual ~AbcObjectWriter();

	void addChild(AbcObjectWriter *child);

    virtual Alembic::Abc::Box3d bounds() const;

    void write();

    void setRotateMatrix(bool s) { m_rotate_matrix = s; }
    bool getRotateMatrix() { return m_rotate_matrix; }

private:
    virtual void do_write() = 0;

	void getAllProperties(IDProperty *group, std::vector<std::pair<std::string, IDProperty*> > &allProps, const std::string &parent);

	void writeArrayProperty(IDProperty *p, Alembic::Abc::OCompoundProperty abcProps);
	void writeProperty(IDProperty *p, const std::string &name, Alembic::Abc::OCompoundProperty abcProps);
	void writeGeomProperty(IDProperty *p, const std::string &name, Alembic::Abc::OCompoundProperty abcProps);

protected:
	bool hasProperties(ID *id);
	void writeProperties(ID *id, Alembic::Abc::OCompoundProperty props, bool writeAsUserData = true);

	bool getPropertyValue(ID *id, const std::string &name, double &val);
};

class AbcObjectReader {
protected:
	std::string m_name;
	std::string m_object_name;
	std::string m_data_name;
	Object *m_object;

	/* TODO(kevin): move this out of here, becomes redundant when importing
	 * multiple object */
	bool m_do_convert_mat;
	float m_conversion_mat[3][3];

public:
	explicit AbcObjectReader(const std::string &name, int from_forward, int from_up);

	virtual ~AbcObjectReader();

	virtual void init(const Alembic::Abc::IObject &object) = 0;
	virtual bool valid() const = 0;
	virtual void readObject(Main *bmain, Scene *scene, float time) = 0;
};

#endif
