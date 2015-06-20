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

#ifndef PTC_ABC_READER_H
#define PTC_ABC_READER_H

#include <string>

#include <Alembic/Abc/IArchive.h>
#include <Alembic/Abc/IObject.h>
#include <Alembic/Abc/ISampleSelector.h>

#include "reader.h"

#include "abc_frame_mapper.h"

#include "util_error_handler.h"
#include "util_types.h"

namespace PTC {

using namespace Alembic;

using Abc::chrono_t;

class AbcReaderArchive : public ReaderArchive, public FrameMapper {
public:
	virtual ~AbcReaderArchive();
	
	static AbcReaderArchive *open(double fps, float start_frame, const std::string &filename, ErrorHandler *error_handler);
	
	PTCArchiveResolution get_resolutions();
	bool use_render() const { return m_use_render; }
	void use_render(bool enable) { m_use_render = enable; }
	
	Abc::IArchive abc_archive() const { return m_abc_archive; }
	Abc::IObject root();
	
	Abc::IObject get_id_object(ID *id);
	bool has_id_object(ID *id);
	
	bool get_frame_range(int &start_frame, int &end_frame);
	Abc::ISampleSelector get_frame_sample_selector(float frame);
	Abc::ISampleSelector get_frame_sample_selector(chrono_t time);
	
	void get_info_stream(void (*stream)(void *, const char *), void *userdata);
	void get_info(CacheArchiveInfo *info, IDProperty *metadata);
	void get_info_nodes(CacheArchiveInfo *info, bool calc_bytes_size);
	
protected:
	AbcReaderArchive(double fps, float start_frame, ErrorHandler *error_handler, Abc::IArchive abc_archive);
	
protected:
	ErrorHandler *m_error_handler;
	bool m_use_render;
	
	Abc::IArchive m_abc_archive;
	Abc::IObject m_abc_root;
	Abc::IObject m_abc_root_render;
};

class AbcReader : public Reader {
public:
	AbcReader() :
	    m_abc_archive(0)
	{}
	
	void init(ReaderArchive *archive)
	{
		BLI_assert(dynamic_cast<AbcReaderArchive*>(archive));
		m_abc_archive = static_cast<AbcReaderArchive*>(archive);
	}
	
	virtual void init_abc(Abc::IObject /*object*/) {}
	
	AbcReaderArchive *abc_archive() const { return m_abc_archive; }
	
	bool get_frame_range(int &start_frame, int &end_frame);
	
	Abc::ISampleSelector get_frame_sample_selector(float frame) { return m_abc_archive->get_frame_sample_selector(frame); }
	Abc::ISampleSelector get_frame_sample_selector(chrono_t time) { return m_abc_archive->get_frame_sample_selector(time); }
	
	PTCReadSampleResult test_sample(float frame);
	PTCReadSampleResult read_sample(float frame);
	virtual PTCReadSampleResult read_sample_abc(chrono_t time) = 0;
	
private:
	AbcReaderArchive *m_abc_archive;
};

} /* namespace PTC */

#endif  /* PTC_READER_H */
