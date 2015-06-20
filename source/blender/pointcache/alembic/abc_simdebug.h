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

#ifndef PTC_ABC_SIMDEBUG_H
#define PTC_ABC_SIMDEBUG_H

#include <Alembic/Abc/IObject.h>
#include <Alembic/Abc/OObject.h>
#include <Alembic/AbcGeom/Foundation.h>

#include "ptc_types.h"

#include "abc_reader.h"
#include "abc_writer.h"

extern "C" {
#include "BKE_effect.h"
}

namespace PTC {

class AbcSimDebugWriter : public AbcWriter {
public:
	AbcSimDebugWriter(const std::string &name, SimDebugData *data);
	~AbcSimDebugWriter();
	
	void init_abc(Abc::OObject parent);
	
	void write_sample();
	
private:
	std::string m_name;
	SimDebugData *m_data;
	
	Abc::OObject m_object;
	AbcGeom::OUInt32ArrayProperty m_prop_category_hash;
	AbcGeom::OUInt32ArrayProperty m_prop_hash;
	AbcGeom::OInt32ArrayProperty m_prop_type;
	AbcGeom::OC3fArrayProperty m_prop_color;
	AbcGeom::OV3fArrayProperty m_prop_v1;
	AbcGeom::OV3fArrayProperty m_prop_v2;
};

class AbcSimDebugReader : public AbcReader {
public:
	AbcSimDebugReader(SimDebugData *data);
	~AbcSimDebugReader();
	
	void init_abc(Abc::IObject object);
	
	PTCReadSampleResult read_sample_abc(chrono_t time);
	
private:
	SimDebugData *m_data;
	
	Abc::IObject m_object;
	AbcGeom::IUInt32ArrayProperty m_prop_category_hash;
	AbcGeom::IUInt32ArrayProperty m_prop_hash;
	AbcGeom::IInt32ArrayProperty m_prop_type;
	AbcGeom::IC3fArrayProperty m_prop_color;
	AbcGeom::IV3fArrayProperty m_prop_v1;
	AbcGeom::IV3fArrayProperty m_prop_v2;
};

} /* namespace PTC */

#endif  /* PTC_SIMDEBUG_H */
