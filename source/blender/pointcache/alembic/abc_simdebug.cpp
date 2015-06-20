/*
 * Copyright 2014, Blender Foundation.
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

#include "abc_simdebug.h"

extern "C" {
#include "BLI_ghash.h"
#include "BLI_math.h"
}

#include "PTC_api.h"

namespace PTC {

using namespace Abc;

struct SimDebugSample {
	std::vector<uint32_t> category_hash;
	std::vector<uint32_t> hash;
	
	std::vector<int32_t> type;
	std::vector<C3f> color;
	std::vector<V3f> v1;
	std::vector<V3f> v2;
};

AbcSimDebugWriter::AbcSimDebugWriter(const std::string &name, SimDebugData *data) :
    m_name(name),
    m_data(data)
{
}

AbcSimDebugWriter::~AbcSimDebugWriter()
{
}

void AbcSimDebugWriter::init_abc(OObject parent)
{
	if (m_object)
		return;
	
	m_object = OObject(parent, m_name, abc_archive()->frame_sampling_index());
	OCompoundProperty props = m_object.getProperties();
	
	m_prop_category_hash = OUInt32ArrayProperty(props, "category_hash", abc_archive()->frame_sampling_index());
	m_prop_hash = OUInt32ArrayProperty(props, "hash", abc_archive()->frame_sampling_index());
	m_prop_type = OInt32ArrayProperty(props, "type", abc_archive()->frame_sampling_index());
	m_prop_color = OC3fArrayProperty(props, "color", abc_archive()->frame_sampling_index());
	m_prop_v1 = OV3fArrayProperty(props, "v1", abc_archive()->frame_sampling_index());
	m_prop_v2 = OV3fArrayProperty(props, "v2", abc_archive()->frame_sampling_index());
}

static void create_sample(SimDebugData *data, SimDebugSample &sample)
{
	int numelem = BLI_ghash_size(data->gh);
	GHashIterator iter;
	
	sample.category_hash.reserve(numelem);
	sample.hash.reserve(numelem);
	sample.type.reserve(numelem);
	sample.color.reserve(numelem);
	sample.v1.reserve(numelem);
	sample.v2.reserve(numelem);
	
	for (BLI_ghashIterator_init(&iter, data->gh); !BLI_ghashIterator_done(&iter); BLI_ghashIterator_step(&iter)) {
		SimDebugElement *elem = (SimDebugElement *)BLI_ghashIterator_getValue(&iter);
		
		sample.category_hash.push_back(elem->category_hash);
		sample.hash.push_back(elem->hash);
		sample.type.push_back(elem->type);
		sample.color.push_back(C3f(elem->color[0], elem->color[1], elem->color[2]));
		sample.v1.push_back(V3f(elem->v1[0], elem->v1[1], elem->v1[2]));
		sample.v2.push_back(V3f(elem->v2[0], elem->v2[1], elem->v2[2]));
	}
}

void AbcSimDebugWriter::write_sample()
{
	if (!m_object)
		return;
	
	SimDebugSample sample;
	
	create_sample(m_data, sample);
	
	m_prop_category_hash.set(UInt32ArraySample(sample.category_hash));
	m_prop_hash.set(UInt32ArraySample(sample.hash));
	m_prop_type.set(Int32ArraySample(sample.type));
	m_prop_color.set(C3fArraySample(sample.color));
	m_prop_v1.set(V3fArraySample(sample.v1));
	m_prop_v2.set(V3fArraySample(sample.v2));
}

/* ========================================================================= */

AbcSimDebugReader::AbcSimDebugReader(SimDebugData *data) :
    m_data(data)
{
}

AbcSimDebugReader::~AbcSimDebugReader()
{
}

void AbcSimDebugReader::init_abc(IObject object)
{
	if (m_object)
		return;
	m_object = IObject(object, kWrapExisting);
	ICompoundProperty props = m_object.getProperties();
	
	m_prop_category_hash = IUInt32ArrayProperty(props, "category_hash");
	m_prop_hash = IUInt32ArrayProperty(props, "hash");
	m_prop_type = IInt32ArrayProperty(props, "type");
	m_prop_color = IC3fArrayProperty(props, "color");
	m_prop_v1 = IV3fArrayProperty(props, "v1");
	m_prop_v2 = IV3fArrayProperty(props, "v2");
}

static PTCReadSampleResult apply_sample(SimDebugData *data,
                                        UInt32ArraySamplePtr sample_category_hash, UInt32ArraySamplePtr sample_hash,
                                        Int32ArraySamplePtr sample_type, C3fArraySamplePtr sample_color,
                                        V3fArraySamplePtr sample_v1, V3fArraySamplePtr sample_v2)
{
	int numelem = sample_hash->size();
	
	if (sample_category_hash->size() != numelem ||
	    sample_type->size() != numelem ||
	    sample_color->size() != numelem ||
	    sample_v1->size() != numelem ||
	    sample_v2->size() != numelem)
	{
		return PTC_READ_SAMPLE_INVALID;
	}
	
	const uint32_t *data_category_hash = sample_category_hash->get();
	const uint32_t *data_hash = sample_hash->get();
	const int32_t *data_type = sample_type->get();
	const C3f *data_color = sample_color->get();
	const V3f *data_v1 = sample_v1->get();
	const V3f *data_v2 = sample_v2->get();
	
	for (int i = 0; i < numelem; ++i) {
		BKE_sim_debug_data_add_element_ex(data, *data_type, data_v1->getValue(), data_v2->getValue(), data_color->x, data_color->y, data_color->z, *data_category_hash, *data_hash);
		
		++data_category_hash;
		++data_hash;
		++data_type;
		++data_color;
		++data_v1;
		++data_v2;
	}
	
	return PTC_READ_SAMPLE_EXACT;
}

PTCReadSampleResult AbcSimDebugReader::read_sample_abc(chrono_t time)
{
	if (!m_object)
		return PTC_READ_SAMPLE_INVALID;
	
	ISampleSelector ss = get_frame_sample_selector(time);
	
	apply_sample(m_data, m_prop_category_hash.getValue(ss), m_prop_hash.getValue(ss), m_prop_type.getValue(ss),
	             m_prop_color.getValue(ss), m_prop_v1.getValue(ss), m_prop_v2.getValue(ss));
	
	return PTC_READ_SAMPLE_EXACT;
}

} /* namespace PTC */
