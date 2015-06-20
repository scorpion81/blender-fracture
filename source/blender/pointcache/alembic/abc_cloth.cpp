/*
 * Copyright 2013, Blender Foundation.
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

#include "abc_cloth.h"

extern "C" {
#include "BLI_math.h"

#include "DNA_cloth_types.h"
#include "DNA_object_types.h"
#include "DNA_modifier_types.h"

#include "BKE_cloth.h"
}

#include "PTC_api.h"

namespace PTC {

using namespace Abc;
using namespace AbcGeom;

AbcClothWriter::AbcClothWriter(const std::string &name, Object *ob, ClothModifierData *clmd) :
    ClothWriter(ob, clmd, name)
{
	set_error_handler(new ModifierErrorHandler(&clmd->modifier));
}

AbcClothWriter::~AbcClothWriter()
{
}

void AbcClothWriter::init_abc(OObject parent)
{
	if (m_points)
		return;
	
	m_points = OPoints(parent, m_name, abc_archive()->frame_sampling_index());
	
	OPointsSchema &schema = m_points.getSchema();
	OCompoundProperty geom_params = schema.getArbGeomParams();
	
	m_param_velocities = OV3fGeomParam(geom_params, "velocities", false, kVaryingScope, 1, 0);
	m_param_goal_positions = OP3fGeomParam(geom_params, "goal_positions", false, kVaryingScope, 1, 0);
}

static V3fArraySample create_sample_velocities(Cloth *cloth, std::vector<V3f> &data)
{
	ClothVertex *vert;
	int i, totvert = cloth->numverts;
	
	data.reserve(totvert);
	for (i = 0, vert = cloth->verts; i < totvert; ++i, ++vert) {
		float *co = vert->v;
		data.push_back(V3f(co[0], co[1], co[2]));
	}
	
	return V3fArraySample(data);
}

static P3fArraySample create_sample_goal_positions(Cloth *cloth, std::vector<V3f> &data)
{
	ClothVertex *vert;
	int i, totvert = cloth->numverts;
	
	data.reserve(totvert);
	for (i = 0, vert = cloth->verts; i < totvert; ++i, ++vert) {
		float *co = vert->xconst;
		data.push_back(V3f(co[0], co[1], co[2]));
	}
	
	return P3fArraySample(data);
}

void AbcClothWriter::write_sample()
{
	if (!m_points)
		return;
	
	Cloth *cloth = m_clmd->clothObject;
	if (!cloth)
		return;
	
	OPointsSchema &schema = m_points.getSchema();
	
	int totpoint = cloth->numverts;
	ClothVertex *vert;
	int i;
	
	/* XXX TODO only needed for the first frame/sample */
	std::vector<Util::uint64_t> ids;
	ids.reserve(totpoint);
	for (i = 0, vert = cloth->verts; i < totpoint; ++i, ++vert)
		ids.push_back(i);
	
	std::vector<V3f> positions;
	positions.reserve(totpoint);
	for (i = 0, vert = cloth->verts; i < totpoint; ++i, ++vert) {
		float *co = vert->x;
		positions.push_back(V3f(co[0], co[1], co[2]));
	}
	
	std::vector<V3f> velocities_buffer;
	std::vector<V3f> goal_positions_buffer;
	V3fArraySample velocities = create_sample_velocities(cloth, velocities_buffer);
	P3fArraySample goal_positions = create_sample_goal_positions(cloth, goal_positions_buffer);
	
	OPointsSchema::Sample sample = OPointsSchema::Sample(V3fArraySample(positions), UInt64ArraySample(ids));
	schema.set(sample);
	
	m_param_velocities.set(OV3fGeomParam::Sample(velocities, kVaryingScope));
	m_param_goal_positions.set(OP3fGeomParam::Sample(goal_positions, kVaryingScope));
}


AbcClothReader::AbcClothReader(const std::string &name, Object *ob, ClothModifierData *clmd) :
    ClothReader(ob, clmd, name)
{
	set_error_handler(new ModifierErrorHandler(&clmd->modifier));
}

AbcClothReader::~AbcClothReader()
{
}

void AbcClothReader::init_abc(IObject object)
{
	if (m_points)
		return;
	m_points = IPoints(object, kWrapExisting);
	
	IPointsSchema &schema = m_points.getSchema();
	ICompoundProperty geom_params = schema.getArbGeomParams();
	
	m_param_velocities = IV3fGeomParam(geom_params, "velocities", 0);
	m_param_goal_positions = IP3fGeomParam(geom_params, "goal_positions", 0);
}

static void apply_sample_positions(Cloth *cloth, P3fArraySamplePtr sample)
{
	ClothVertex *vert;
	int i, totvert = cloth->numverts;
	
	BLI_assert(sample->size() == totvert);
	
	const V3f *data = sample->get();
	for (i = 0, vert = cloth->verts; i < totvert; ++i, ++vert) {
		const V3f &co = data[i];
		copy_v3_v3(vert->x, co.getValue());
	}
}

static void apply_sample_velocities(Cloth *cloth, V3fArraySamplePtr sample)
{
	ClothVertex *vert;
	int i, totvert = cloth->numverts;
	
	BLI_assert(sample->size() == totvert);
	
	const V3f *data = sample->get();
	for (i = 0, vert = cloth->verts; i < totvert; ++i, ++vert) {
		const V3f &vel = data[i];
		copy_v3_v3(vert->v, vel.getValue());
	}
}

static void apply_sample_goal_positions(Cloth *cloth, P3fArraySamplePtr sample)
{
	ClothVertex *vert;
	int i, totvert = cloth->numverts;
	
	BLI_assert(sample->size() == totvert);
	
	const V3f *data = sample->get();
	for (i = 0, vert = cloth->verts; i < totvert; ++i, ++vert) {
		const V3f &co = data[i];
		copy_v3_v3(vert->xconst, co.getValue());
	}
}

PTCReadSampleResult AbcClothReader::read_sample_abc(chrono_t time)
{
	Cloth *cloth = m_clmd->clothObject;
	
	if (!m_points)
		return PTC_READ_SAMPLE_INVALID;
	
	IPointsSchema &schema = m_points.getSchema();
	if (schema.getNumSamples() == 0)
		return PTC_READ_SAMPLE_INVALID;
	
	ISampleSelector ss = get_frame_sample_selector(time);
	
	IPointsSchema::Sample sample;
	schema.get(sample, ss);
	
	P3fArraySamplePtr positions = sample.getPositions();
	
	V3fArraySamplePtr velocities;
	if (m_param_velocities && m_param_velocities.getNumSamples() > 0) {
		IV3fGeomParam::Sample sample_velocities;
		m_param_velocities.getExpanded(sample_velocities, ss);
		velocities = sample_velocities.getVals();
	}
	
	P3fArraySamplePtr goal_positions;
	if (m_param_goal_positions && m_param_goal_positions.getNumSamples() > 0) {
		IP3fGeomParam::Sample sample_goal_positions;
		m_param_goal_positions.getExpanded(sample_goal_positions, ss);
		goal_positions = sample_goal_positions.getVals();
	}
	
	apply_sample_positions(cloth, positions);
	if (velocities)
		apply_sample_velocities(cloth, velocities);
	if (goal_positions)
		apply_sample_goal_positions(cloth, goal_positions);
	
	return PTC_READ_SAMPLE_EXACT;
}

} /* namespace PTC */
