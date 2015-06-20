/*
 * Copyright 2015, Blender Foundation.
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
 */

#ifndef PTC_ABC_CUSTOMDATA_H
#define PTC_ABC_CUSTOMDATA_H

#include <map>

#include <Alembic/AbcGeom/IGeomParam.h>
#include <Alembic/AbcGeom/OGeomParam.h>
#include <Alembic/Abc/IBaseProperty.h>
#include <Alembic/Abc/TypedPropertyTraits.h>

#include "abc_reader.h"
#include "abc_writer.h"

extern "C" {
#include "BKE_customdata.h"

#include "DNA_customdata_types.h"
}

namespace PTC {

using namespace Alembic;

std::string abc_customdata_layer_name(CustomData *cdata, CustomDataType type, int n);

struct CustomDataWriter {
	typedef std::map<std::string, Abc::BasePropertyWriterPtr> LayerPropsMap;
	typedef std::pair<std::string, Abc::BasePropertyWriterPtr> LayerPropsPair;
	
	CustomDataWriter(const std::string &name, CustomDataMask cdmask);
	~CustomDataWriter();
	
	void init(Abc::TimeSamplingPtr time_sampling);
	
	void write_sample(CustomData *cdata, int num_data, Abc::OCompoundProperty &parent);
	
	Abc::OCompoundProperty &props() { return m_props; }
	
	template <typename PropertyT, typename ParentT>
	PropertyT add_scalar_property(const std::string &name, ParentT &parent)
	{
		LayerPropsMap::iterator it = m_layer_props.find(name);
		if (it == m_layer_props.end()) {
			PropertyT prop = PropertyT(parent, name, m_time_sampling);
			m_layer_props.insert(LayerPropsPair(name, prop.getPtr()));
			return prop;
		}
		else {
			return PropertyT(it->second->asScalarPtr(), Abc::kWrapExisting);
		}
	}
	
	template <typename PropertyT, typename ParentT>
	PropertyT add_array_property(const std::string &name, ParentT &parent)
	{
		LayerPropsMap::iterator it = m_layer_props.find(name);
		if (it == m_layer_props.end()) {
			PropertyT prop = PropertyT(parent, name, m_time_sampling);
			m_layer_props.insert(LayerPropsPair(name, prop.getPtr()));
			return prop;
		}
		else {
			return PropertyT(it->second->asArrayPtr(), Abc::kWrapExisting);
		}
	}
	
	template <typename PropertyT, typename ParentT>
	PropertyT add_compound_property(const std::string &name, ParentT &parent)
	{
		LayerPropsMap::iterator it = m_layer_props.find(name);
		if (it == m_layer_props.end()) {
			PropertyT prop = PropertyT(parent, name, m_time_sampling);
			m_layer_props.insert(LayerPropsPair(name, prop.getPtr()));
			return prop;
		}
		else {
			return PropertyT(it->second->asCompoundPtr(), Abc::kWrapExisting);
		}
	}
	
	std::string cdtype_to_name(CustomData *cdata, CustomDataType type, int n);
	
private:
	std::string m_name;
	CustomDataMask m_cdmask;
	
	Abc::TimeSamplingPtr m_time_sampling;
	Abc::OCompoundProperty m_props;
	LayerPropsMap m_layer_props;
};

struct CustomDataReader {
	typedef std::map<std::string, Abc::BasePropertyReaderPtr> LayerPropsMap;
	typedef std::pair<std::string, Abc::BasePropertyReaderPtr> LayerPropsPair;
	
	CustomDataReader(const std::string &name, CustomDataMask cdmask);
	~CustomDataReader();
	
	PTCReadSampleResult read_sample(const Abc::ISampleSelector &ss, CustomData *cdata, int num_data, Abc::ICompoundProperty &parent);
	
	Abc::ICompoundProperty &props() { return m_props; }
	
	template <typename PropertyT, typename ParentT>
	PropertyT add_scalar_property(const std::string &name, ParentT &parent)
	{
		LayerPropsMap::iterator it = m_layer_props.find(name);
		if (it == m_layer_props.end()) {
			PropertyT prop = PropertyT(parent, name, 0);
			m_layer_props.insert(LayerPropsPair(name, prop.getPtr()));
			return prop;
		}
		else {
			return PropertyT(it->second->asScalarPtr(), Abc::kWrapExisting);
		}
	}
	
	template <typename PropertyT, typename ParentT>
	PropertyT add_array_property(const std::string &name, ParentT &parent)
	{
		LayerPropsMap::iterator it = m_layer_props.find(name);
		if (it == m_layer_props.end()) {
			PropertyT prop = PropertyT(parent, name, 0);
			m_layer_props.insert(LayerPropsPair(name, prop.getPtr()));
			return prop;
		}
		else {
			return PropertyT(it->second->asArrayPtr(), Abc::kWrapExisting);
		}
	}
	
	template <typename PropertyT, typename ParentT>
	PropertyT add_compound_property(const std::string &name, ParentT &parent)
	{
		LayerPropsMap::iterator it = m_layer_props.find(name);
		if (it == m_layer_props.end()) {
			PropertyT prop = PropertyT(parent, name, 0);
			m_layer_props.insert(LayerPropsPair(name, prop.getPtr()));
			return prop;
		}
		else {
			return PropertyT(it->second->asCompoundPtr(), Abc::kWrapExisting);
		}
	}
	
	void cdtype_from_name(CustomData *cdata, const std::string &name, int type, int *n, char *layer_name, int max_layer_name);
	
private:
	std::string m_name;
	CustomDataMask m_cdmask;
	
	Abc::ICompoundProperty m_props;
	LayerPropsMap m_layer_props;
};

} /* namespace PTC */

#endif  /* PTC_CLOTH_H */
