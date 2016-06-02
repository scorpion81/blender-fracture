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
#include "DNA_constraint_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_constraint.h"
#include "BKE_idprop.h"
#include "BKE_modifier.h"
#include "BKE_object.h"

#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"
#include "BLI_string.h"

#include "BKE_depsgraph.h"
}

using Alembic::AbcGeom::IObject;
using Alembic::AbcGeom::IXform;
using Alembic::AbcGeom::IXformSchema;

using Alembic::AbcGeom::OCompoundProperty;
using Alembic::AbcGeom::ODoubleArrayProperty;
using Alembic::AbcGeom::ODoubleProperty;
using Alembic::AbcGeom::OFloatArrayProperty;
using Alembic::AbcGeom::OFloatProperty;
using Alembic::AbcGeom::OInt32ArrayProperty;
using Alembic::AbcGeom::OInt32Property;
using Alembic::AbcGeom::OStringArrayProperty;
using Alembic::AbcGeom::OStringProperty;

AbcObjectWriter::AbcObjectWriter(Object *obj, ExportSettings &settings)
    : m_object(obj)
    , m_settings(settings)
    , m_first_frame(true)
{
	m_name = get_id_name(m_object) + "Shape";
}

AbcObjectWriter::~AbcObjectWriter()
{}

void AbcObjectWriter::addChild(AbcObjectWriter *child)
{
	m_children.push_back(child);
}

Imath::Box3d AbcObjectWriter::bounds() const
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

void AbcObjectWriter::writeProperties(ID *id, const OCompoundProperty &props, bool writeAsUserData)
{
	for (int i = 0, e = m_props.size(); i < e; ++i) {
		if (props.getPropertyHeader(m_props[i].first)){
			continue;
		}

		if (writeAsUserData) {
			writeProperty(m_props[i].second, m_props[i].first, props);
		}
		else {
			writeGeomProperty(m_props[i].second, m_props[i].first, props);
		}
	}
}

void AbcObjectWriter::getAllProperties(IDProperty *group,
                                       std::vector<std::pair<std::string, IDProperty *> > &allProps,
                                       const std::string &parent)
{
	if (!group) {
		return;
	}

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

				break;
			}
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

				break;
			}
			case IDP_ARRAY:
			case IDP_ID:
			case IDP_IDPARRAY:
			break;
		}

		prop = prop->next;
	}
}

void AbcObjectWriter::writeArrayProperty(IDProperty *p, const OCompoundProperty &abcProps)
{
	std::string name(p->name);

	switch(p->subtype) {
		case IDP_INT:
		{
			OInt32ArrayProperty op(abcProps, name);
			op.set(OInt32ArrayProperty::sample_type(static_cast<int *>(IDP_Array(p)), p->len));
			break;
		}
		case IDP_FLOAT:
		{
			OFloatArrayProperty op(abcProps, name);
			op.set(OFloatArrayProperty::sample_type(static_cast<float *>(IDP_Array(p)), p->len));
			break;
		}
		case IDP_DOUBLE:
		{
			ODoubleArrayProperty op(abcProps, name);
			op.set(ODoubleArrayProperty::sample_type(static_cast<double *>(IDP_Array(p)), p->len));
			break;
		}
	}
}

void AbcObjectWriter::writeProperty(IDProperty *p, const std::string &name, const OCompoundProperty &abcProps)
{
	/* TODO: check this... */
	switch(p->type) {
		case IDP_STRING:
		{
			OStringProperty op(abcProps, name);
			op.set(IDP_String(p));
			break;
		}
		case IDP_INT:
		{
			OInt32Property op(abcProps, name);
			op.set(IDP_Int(p));
			break;
		}
		case IDP_FLOAT:
		{
			OFloatProperty op(abcProps, name);
			op.set(IDP_Float(p));
			break;
		}
		case IDP_DOUBLE:
		{
			ODoubleProperty op(abcProps, name);
			op.set(IDP_Double(p));
			break;
		}
		case IDP_ARRAY:
		{
			writeArrayProperty(p, abcProps);
			break;
		}
		case IDP_ID:
		case IDP_IDPARRAY:
		case IDP_GROUP:
			break;
	}
}

void AbcObjectWriter::writeGeomProperty(IDProperty *p, const std::string &name, const OCompoundProperty &abcProps)
{
	switch(p->type) {
		case IDP_STRING:
		{
			std::string val = IDP_String(p);
			Alembic::AbcGeom::OStringGeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::OStringGeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::StringArraySample(&val, 1));
			param.set(samp);

			break;
		}
		case IDP_INT:
		{
			int val = IDP_Int(p);
			Alembic::AbcGeom::OInt32GeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::OInt32GeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::Int32ArraySample(&val, 1));
			param.set(samp);

			break;
		}
		case IDP_FLOAT:
		{
			float val = IDP_Float(p);
			Alembic::AbcGeom::OFloatGeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::OFloatGeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::FloatArraySample(&val, 1));
			param.set(samp);

			break;
		}
		case IDP_DOUBLE:
		{
			double val = IDP_Double(p);
			Alembic::AbcGeom::ODoubleGeomParam param(abcProps, name, false, Alembic::AbcGeom::kConstantScope, 1, 0);
			Alembic::AbcGeom::ODoubleGeomParam::Sample samp;
			samp.setScope(Alembic::AbcGeom::kConstantScope);
			samp.setVals(Alembic::AbcGeom::DoubleArraySample(&val, 1));
			param.set(samp);

			break;
		}
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

AbcObjectReader::AbcObjectReader(const IObject &object, ImportSettings &settings)
    : m_name("")
    , m_object_name("")
    , m_data_name("")
    , m_object(NULL)
    , m_iobject(object)
    , m_settings(&settings)
{
	m_name = object.getFullName();
	std::vector<std::string> parts;
	split(m_name, '/', parts);

	m_object_name = m_data_name = parts[parts.size() - 1];
}

AbcObjectReader::~AbcObjectReader()
{}

const IObject &AbcObjectReader::iobject() const
{
	return m_iobject;
}

Object *AbcObjectReader::object() const
{
	return m_object;
}

void AbcObjectReader::readObjectMatrix(const float time)
{
	IXform ixform;

	if (IXform::matches(m_iobject.getMetaData())) {
		ixform = IXform(m_iobject, Alembic::AbcGeom::kWrapExisting);
	}
	else if (IXform::matches(m_iobject.getParent().getMetaData())) {
		ixform = IXform(m_iobject.getParent(), Alembic::AbcGeom::kWrapExisting);
	}
	else {
		return;
	}

	const IXformSchema &schema(ixform.getSchema());

	if (!schema.valid()) {
		return;
	}

	Alembic::AbcGeom::ISampleSelector sample_sel(time);
	Alembic::AbcGeom::XformSample xs;
	schema.get(xs, sample_sel);

	create_input_transform(sample_sel, ixform, m_object, m_object->obmat);

	invert_m4_m4(m_object->imat, m_object->obmat);

	BKE_object_apply_mat4(m_object, m_object->obmat, false,  false);

	if (!schema.isConstant()) {
		bConstraint *con = BKE_constraint_add_for_object(m_object, NULL, CONSTRAINT_TYPE_TRANSFORMCACHE);
		bTransformCacheConstraint *data = static_cast<bTransformCacheConstraint *>(con->data);
		BLI_strncpy(data->filepath, m_iobject.getArchive().getName().c_str(), 1024);
		BLI_strncpy(data->abc_object_path, m_iobject.getFullName().c_str(), 1024);
	}
}

void AbcObjectReader::addDefaultModifier() const
{
	ModifierData *md = modifier_new(eModifierType_MeshSequenceCache);
	BLI_addtail(&m_object->modifiers, md);

	MeshSeqCacheModifierData *mcmd = reinterpret_cast<MeshSeqCacheModifierData *>(md);

	BLI_strncpy(mcmd->filepath, m_iobject.getArchive().getName().c_str(), 1024);
	BLI_strncpy(mcmd->abc_object_path, m_iobject.getFullName().c_str(), 1024);
	mcmd->is_sequence = m_settings->is_sequence;
}
