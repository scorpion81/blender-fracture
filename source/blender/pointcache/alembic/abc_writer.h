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

#ifndef PTC_ABC_WRITER_H
#define PTC_ABC_WRITER_H

#include <string>

#include <Alembic/Abc/OArchive.h>
#include <Alembic/Abc/OObject.h>

#include "writer.h"

#include "abc_frame_mapper.h"

#include "util_error_handler.h"
#include "util_types.h"

extern "C" {
#include "BLI_utildefines.h"

#include "DNA_ID.h"
}

struct tm;

namespace PTC {

using namespace Alembic;

class AbcWriterArchive : public WriterArchive, public FrameMapper {
public:
	virtual ~AbcWriterArchive();
	
	static AbcWriterArchive *open(double fps, float start_frame, const std::string &filename, PTCArchiveResolution resolutions,
	                              const char *app_name, const char *description, const struct tm *time, IDProperty *metadata, ErrorHandler *error_handler);
	
	bool use_render() const { return m_use_render; }
	void use_render(bool enable) { m_use_render = enable; }
	
	Abc::OArchive abc_archive() const { return m_abc_archive; }
	Abc::OObject root();
	
	Abc::OObject get_id_object(ID *id);
	bool has_id_object(ID *id);
	
	template <class OObjectT>
	OObjectT add_id_object(ID *id);
	
	uint32_t frame_sampling_index() const { return m_frame_sampling; }
	Abc::TimeSamplingPtr frame_sampling();
	
protected:
	AbcWriterArchive(double fps, float start_frame, PTCArchiveResolution resolutions, ErrorHandler *error_handler, Abc::OArchive abc_archive);
	
protected:
	ErrorHandler *m_error_handler;
	uint32_t m_frame_sampling;
	bool m_use_render;
	
	Abc::OArchive m_abc_archive;
	Abc::OObject m_abc_root;
	Abc::OObject m_abc_root_render;
};

class AbcWriter : public Writer {
public:
	Abc::TimeSamplingPtr frame_sampling() { return m_abc_archive->frame_sampling(); }
	
	void init(WriterArchive *archive)
	{
		BLI_assert(dynamic_cast<AbcWriterArchive*>(archive));
		m_abc_archive = static_cast<AbcWriterArchive*>(archive);
		
		init_abc();
	}
	
	/* one of these should be implemented by subclasses */
	virtual void init_abc() {}
	virtual void init_abc(Abc::OObject /*parent*/) {}
	
	AbcWriterArchive *abc_archive() const { return m_abc_archive; }
	
private:
	AbcWriterArchive *m_abc_archive;
};

/* ------------------------------------------------------------------------- */

template <class OObjectT>
OObjectT AbcWriterArchive::add_id_object(ID *id)
{
	using namespace Abc;
	
	if (!m_abc_archive)
		return OObjectT();
	
	ObjectWriterPtr root_ptr = this->root().getPtr();
	
	ObjectWriterPtr child = root_ptr->getChild(id->name);
	if (child)
		return OObjectT(child, kWrapExisting);
	else {
		const ObjectHeader *child_header = root_ptr->getChildHeader(id->name);
		if (child_header)
			return OObjectT(root_ptr->createChild(*child_header), kWrapExisting);
		else {
			return OObjectT(root_ptr, id->name, frame_sampling_index());
		}
	}
}

} /* namespace PTC */

#endif  /* PTC_WRITER_H */
