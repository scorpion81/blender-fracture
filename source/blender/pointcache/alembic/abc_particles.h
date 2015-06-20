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

#ifndef PTC_ABC_PARTICLES_H
#define PTC_ABC_PARTICLES_H

#include <Alembic/AbcGeom/IPoints.h>
#include <Alembic/AbcGeom/OPoints.h>
#include <Alembic/AbcGeom/ICurves.h>
#include <Alembic/AbcGeom/OCurves.h>

#include "ptc_types.h"

#include "PTC_api.h"

#include "abc_reader.h"
#include "abc_schema.h"
#include "abc_writer.h"
#include "abc_cloth.h"

struct ListBase;
struct Object;
struct ParticleSystem;
struct ParticleCacheKey;
struct Strands;
struct StrandsChildren;

namespace PTC {

class AbcDerivedMeshWriter;
class AbcDerivedMeshReader;


class AbcHairChildrenWriter : public ParticlesWriter, public AbcWriter {
public:
	AbcHairChildrenWriter(const std::string &name, Object *ob, ParticleSystem *psys);
	~AbcHairChildrenWriter();
	
	void init_abc(Abc::OObject parent);
	
	void write_sample();
	
private:
	ParticleSystemModifierData *m_psmd;
	
	AbcGeom::OCurves m_curves;
	AbcGeom::OQuatfArrayProperty m_prop_root_rot;
	AbcGeom::OV3fArrayProperty m_prop_root_positions;
	AbcGeom::OFloatGeomParam m_param_cutoff;
	AbcGeom::OFloatGeomParam m_param_times;
	AbcGeom::OInt32ArrayProperty m_prop_parents;
	AbcGeom::OFloatArrayProperty m_prop_parent_weights;
	AbcGeom::OV2fArrayProperty m_prop_curve_uvs;
	AbcGeom::OC3fArrayProperty m_prop_curve_vcols;
};


class AbcHairWriter : public ParticlesWriter, public AbcWriter {
public:
	AbcHairWriter(const std::string &name, Object *ob, ParticleSystem *psys);
	~AbcHairWriter();
	
	void init(WriterArchive *archive);
	void init_abc(Abc::OObject parent);
	
	void write_sample();
	
private:
	ParticleSystemModifierData *m_psmd;
	
	AbcGeom::OCurves m_curves;
	AbcGeom::OQuatfGeomParam m_param_root_rot;
	AbcGeom::OUInt32GeomParam m_param_root_orig_verts;
	AbcGeom::OFloatGeomParam m_param_root_orig_weights;
	AbcGeom::OInt32GeomParam m_param_root_orig_poly;
	AbcGeom::OUInt32GeomParam m_param_root_orig_loops;
	AbcGeom::OFloatGeomParam m_param_times;
	AbcGeom::OFloatGeomParam m_param_weights;
	
	AbcHairChildrenWriter m_child_writer;
};


class AbcStrandsChildrenWriter : public AbcWriter {
public:
	AbcStrandsChildrenWriter(const std::string &name, const std::string &abc_name, DupliObjectData *dobdata);
	
	StrandsChildren *get_strands() const;
	
	void init_abc(Abc::OObject parent);
	
	void write_sample();
	
private:
	std::string m_name;
	std::string m_abc_name;
	DupliObjectData *m_dobdata;
	
	AbcGeom::OCurves m_curves;
	AbcGeom::OQuatfArrayProperty m_prop_root_rot;
	AbcGeom::OV3fArrayProperty m_prop_root_positions;
	AbcGeom::OFloatGeomParam m_param_cutoff;
	AbcGeom::OFloatGeomParam m_param_times;
	AbcGeom::OInt32ArrayProperty m_prop_parents;
	AbcGeom::OFloatArrayProperty m_prop_parent_weights;
	AbcGeom::OV2fArrayProperty m_prop_curve_uvs;
	AbcGeom::OC3fArrayProperty m_prop_curve_vcols;
};


class AbcStrandsWriter : public AbcWriter {
public:
	AbcStrandsWriter(const std::string &name, DupliObjectData *dobdata);
	
	Strands *get_strands() const;
	
	void init(WriterArchive *archive);
	void init_abc(Abc::OObject parent);
	
	void write_sample();
	
private:
	std::string m_name;
	DupliObjectData *m_dobdata;
	
	AbcGeom::OCurves m_curves;
	AbcGeom::OQuatfGeomParam m_param_root_rot;
	AbcGeom::OUInt32GeomParam m_param_root_orig_verts;
	AbcGeom::OFloatGeomParam m_param_root_orig_weights;
	AbcGeom::OInt32GeomParam m_param_root_orig_poly;
	AbcGeom::OUInt32GeomParam m_param_root_orig_loops;
	AbcGeom::OFloatGeomParam m_param_times;
	AbcGeom::OFloatGeomParam m_param_weights;
	AbcGeom::OCompoundProperty m_param_motion_state;
	AbcGeom::OP3fGeomParam m_param_motion_co;
	AbcGeom::OV3fGeomParam m_param_motion_vel;
	
	AbcStrandsChildrenWriter m_child_writer;
};


class AbcStrandsChildrenReader : public AbcReader {
public:
	AbcStrandsChildrenReader(StrandsChildren *strands);
	~AbcStrandsChildrenReader();
	
	void init_abc(Abc::IObject object);
	
	PTCReadSampleResult read_sample_abc(chrono_t time);
	
	StrandsChildren *get_result() { return m_strands; }
	StrandsChildren *acquire_result();
	void discard_result();
	
private:
	StrandsChildren *m_strands;
	
	AbcGeom::ICurves m_curves;
	AbcGeom::IQuatfArrayProperty m_prop_root_rot;
	AbcGeom::IV3fArrayProperty m_prop_root_positions;
	AbcGeom::IFloatGeomParam m_param_cutoff;
	AbcGeom::IFloatGeomParam m_param_times;
	AbcGeom::IInt32ArrayProperty m_prop_parents;
	AbcGeom::IFloatArrayProperty m_prop_parent_weights;
	AbcGeom::IV2fArrayProperty m_prop_curve_uvs;
	AbcGeom::IC3fArrayProperty m_prop_curve_vcols;
};


class AbcStrandsReader : public AbcReader {
public:
	AbcStrandsReader(Strands *strands, StrandsChildren *children, bool read_motion, bool read_children);
	~AbcStrandsReader();
	
	void init(ReaderArchive *archive);
	void init_abc(Abc::IObject object);
	
	PTCReadSampleResult read_sample_abc(chrono_t time);
	
	Strands *acquire_result();
	void discard_result();
	
	AbcStrandsChildrenReader &child_reader() { return m_child_reader; }
	
private:
	bool m_read_motion, m_read_children;
	Strands *m_strands;
	
	AbcGeom::ICurves m_curves;
	AbcGeom::IQuatfGeomParam m_param_root_rot;
	AbcGeom::IUInt32GeomParam m_param_root_orig_verts;
	AbcGeom::IFloatGeomParam m_param_root_orig_weights;
	AbcGeom::IInt32GeomParam m_param_root_orig_poly;
	AbcGeom::IUInt32GeomParam m_param_root_orig_loops;
	AbcGeom::IFloatGeomParam m_param_times;
	AbcGeom::IFloatGeomParam m_param_weights;
	AbcGeom::ICompoundProperty m_param_motion_state;
	AbcGeom::IP3fGeomParam m_param_motion_co;
	AbcGeom::IV3fGeomParam m_param_motion_vel;
	
	AbcStrandsChildrenReader m_child_reader;
};


} /* namespace PTC */

#endif  /* PTC_PARTICLES_H */
