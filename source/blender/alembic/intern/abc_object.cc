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

#include "abc_object.h"

#include <Alembic/AbcGeom/All.h>

#include "abc_util.h"

extern "C" {
#include "BKE_idprop.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"
}

AbcObjectWriter::AbcObjectWriter(Object *obj, AbcExportOptions &opts)
    : m_object(obj)
    , m_options(opts)
    , m_first_frame(true)
    , m_rotate_matrix(false)
{}

AbcObjectWriter::~AbcObjectWriter()
{}

void AbcObjectWriter::addChild(AbcObjectWriter *child)
{
	m_children.push_back(child);
}

Alembic::Abc::Box3d AbcObjectWriter::bounds() const
{
	return m_bounds;
}

void AbcObjectWriter::write()
{
	do_write();
	m_first_frame = false;
}

bool AbcObjectWriter::hasProperties(ID *id)
{
	IDProperty *idgroup = IDP_GetProperties(id, false);
	getAllProperties(idgroup, m_props, "");
	return !m_props.empty();
}

void AbcObjectWriter::writeProperties(ID *id, Alembic::Abc::OCompoundProperty props, bool writeAsUserData)
{
	for (int i = 0, e = m_props.size(); i < e; ++i) {
		if (props.getPropertyHeader(m_props[i].first))
			continue;
		if (writeAsUserData)
			writeProperty(m_props[i].second, m_props[i].first, props);
		else
			writeGeomProperty(m_props[i].second, m_props[i].first, props);
	}
}

void AbcObjectWriter::getAllProperties(IDProperty *group, std::vector<std::pair<std::string, IDProperty*> > &allProps, const std::string &parent)
{
	if (!group)
		return;

	const char *separator = ".";

	IDProperty *prop = static_cast<IDProperty *>(group->data.group.first);

	while (prop) {
		switch (prop->type) {
			case IDP_STRING:
			case IDP_INT:
			case IDP_FLOAT:
			case IDP_DOUBLE:
			{
				std::string name(parent);

				if (!name.empty()) {
					name.append(separator);
				}

				name.append(prop->name);
				allProps.push_back(std::make_pair(name, prop));
			}
			break;

			case IDP_GROUP:
			{
				/* ignore ui properties */
				if (STREQ("_RNA_UI", prop->name)) {
					std::string name(parent);

					if (!name.empty()) {
						name.append(separator);
					}

					name.append(prop->name);
					getAllProperties(prop, allProps, name);
				}
			}
			break;

			case IDP_ARRAY:
			case IDP_ID:
			case IDP_IDPARRAY:
			break;
		}

		prop = prop->next;
	}
}

void AbcObjectWriter::writeArrayProperty(IDProperty *p, Alembic::Abc::OCompoundProperty abcProps)
{
	std::string name(p->name);

	switch(p->subtype) {
		case IDP_INT:
		{
			Alembic::Abc::OInt32ArrayProperty op(abcProps, name);
			op.set(Alembic::Abc::OInt32ArrayProperty::sample_type(static_cast<int *>(IDP_Array(p)), p->len));
		}
		break;

		case IDP_FLOAT:
		{
			Alembic::Abc::OFloatArrayProperty op(abcProps, name);
			op.set(Alembic::Abc::OFloatArrayProperty::sample_type(static_cast<float*>(IDP_Array(p)), p->len));
		}
		break;

		case IDP_DOUBLE:
		{
			Alembic::Abc::ODoubleArrayProperty op(abcProps, name);
			op.set(Alembic::Abc::ODoubleArrayProperty::sample_type(static_cast<double *>(IDP_Array(p)), p->len));
		}
		break;
	}
}

void AbcObjectWriter::writeProperty(IDProperty *p, const std::string &name, Alembic::Abc::OCompoundProperty abcProps)
{
	// TODO: check this...
	switch(p->type) {
		case IDP_STRING:
		{
			Alembic::Abc::OStringProperty op(abcProps, name);
			op.set(IDP_String(p));
		}
		break;

		case IDP_INT:
		{
			Alembic::Abc::OInt32Property op(abcProps, name);
			op.set(IDP_Int(p));
		}
		break;

		case IDP_FLOAT:
		{
			Alembic::Abc::OFloatProperty op(abcProps, name);
			op.set(IDP_Float(p));
		}
		break;

		case IDP_DOUBLE:
		{
			Alembic::Abc::ODoubleProperty op(abcProps, name);
			op.set(IDP_Double(p));
		}
		break;

		case IDP_ARRAY:
			writeArrayProperty(p, abcProps);
		break;

		case IDP_ID:
		case IDP_IDPARRAY:
		case IDP_GROUP:
		break;
	}
}

void AbcObjectWriter::writeGeomProperty(IDProperty *p, const std::string &name, Alembic::Abc::OCompoundProperty abcProps)
{
	switch(p->type)
	{
		case IDP_STRING:
		{
			std::string val = IDP_String(p);
			Alembic::AbcGeom::OStringGeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::OStringGeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::StringArraySample(&val, 1));
			param.set(samp);
		}
		break;

		case IDP_INT:
		{
			int val = IDP_Int(p);
			Alembic::AbcGeom::OInt32GeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::OInt32GeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::Int32ArraySample(&val, 1));
			param.set(samp);
		}
		break;

		case IDP_FLOAT:
		{
			float val = IDP_Float(p);
			Alembic::AbcGeom::OFloatGeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::OFloatGeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::FloatArraySample(&val, 1));
			param.set(samp);
		}
		break;

		case IDP_DOUBLE:
		{
			double val = IDP_Double(p);
			Alembic::AbcGeom::ODoubleGeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::ODoubleGeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::DoubleArraySample(&val, 1));
			param.set(samp);
		}
		break;

		case IDP_ARRAY:
		case IDP_IDPARRAY:
		case IDP_GROUP:
		case IDP_ID:
		break;
	}
}

bool AbcObjectWriter::getPropertyValue(ID *id, const std::string &name, double &val)
{
	IDProperty *idgroup = IDP_GetProperties(id, false);
	IDProperty *prop = IDP_GetPropertyFromGroup(idgroup, name.c_str());

	if (prop) {
		switch(prop->type) {
			case IDP_FLOAT:
				val = IDP_Float(prop);
				return true;

			case IDP_DOUBLE:
				val = IDP_Double(prop);
				return true;

			case IDP_INT:
				val = IDP_Int(prop);
				return true;
		}
	}

	return false;
}

/* ****************************** object reader ***************************** */

AbcObjectReader::AbcObjectReader(const Alembic::Abc::IObject &object, int from_forward, int from_up)
    : m_name("")
    , m_object_name("")
    , m_data_name("")
    , m_object(NULL)
    , m_iobject(object)
    , m_do_convert_mat(false)
{
	m_name = object.getFullName();
	std::vector<std::string> parts;
	split(m_name, "/", parts);

	assert(parts.size() >= 2);

	m_object_name = parts[parts.size() - 2];
	m_data_name = parts[parts.size() - 1];

	if (mat3_from_axis_conversion(from_forward, from_up, 1, 2, m_conversion_mat)) {
		m_do_convert_mat = true;
	}
}

AbcObjectReader::~AbcObjectReader()
{}

std::vector<AbcObjectReader *> AbcObjectReader::children() const
{
	return m_children;
}

void AbcObjectReader::addChild(AbcObjectReader *child)
{
	m_children.push_back(child);
}

Object *AbcObjectReader::object() const
{
	return m_object;
}
