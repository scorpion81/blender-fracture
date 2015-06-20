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

//#include <Alembic/AbcCoreHDF5/ReadWrite.h>
#include <Alembic/AbcCoreOgawa/ReadWrite.h>
#include <Alembic/Abc/OObject.h>
#include <Alembic/Abc/ArchiveInfo.h>

#include "alembic.h"
#include "abc_writer.h"

#include "util_error_handler.h"

extern "C" {
#include <ctime>

#include "BLI_fileops.h"
#include "BLI_path_util.h"
}

namespace PTC {

using namespace Abc;

/* make sure the file's directory exists */
static void ensure_directory(const char *filename)
{
	char dir[FILE_MAXDIR];
	BLI_split_dir_part(filename, dir, sizeof(dir));
	BLI_dir_create_recursive(dir);
}

AbcWriterArchive *AbcWriterArchive::open(double fps, float start_frame, const std::string &filename, PTCArchiveResolution resolutions,
                                         const char *app_name, const char *description, const struct tm *time, IDProperty *metadata, ErrorHandler *error_handler)
{
	ensure_directory(filename.c_str());
	
	MetaData md = abc_create_archive_info(app_name, description, time, metadata);
	
	OArchive abc_archive;
	PTC_SAFE_CALL_BEGIN
//	abc_archive = OArchive(AbcCoreHDF5::WriteArchive(), filename, md, Abc::ErrorHandler::kThrowPolicy);
	abc_archive = OArchive(AbcCoreOgawa::WriteArchive(), filename, md, Abc::ErrorHandler::kThrowPolicy);
	PTC_SAFE_CALL_END_HANDLER(error_handler)
	
	if (abc_archive)
		return new AbcWriterArchive(fps, start_frame, resolutions, error_handler, abc_archive);
	else
		return NULL;
}

AbcWriterArchive::AbcWriterArchive(double fps, float start_frame, PTCArchiveResolution resolutions, ErrorHandler *error_handler, OArchive abc_archive) :
    FrameMapper(fps, start_frame),
    m_error_handler(error_handler),
    m_use_render(false),
    m_abc_archive(abc_archive)
{
	if (m_abc_archive) {
		chrono_t cycle_time = this->seconds_per_frame();
		chrono_t start_time = this->start_time();
		m_frame_sampling = m_abc_archive.addTimeSampling(TimeSampling(cycle_time, start_time));
		
		if (resolutions & PTC_RESOLUTION_PREVIEW)
			m_abc_root = OObject(m_abc_archive.getTop(), "root");
		if (resolutions & PTC_RESOLUTION_RENDER)
			m_abc_root_render = OObject(m_abc_archive.getTop(), "root_render");
	}
}

AbcWriterArchive::~AbcWriterArchive()
{
}

OObject AbcWriterArchive::get_id_object(ID *id)
{
	if (!m_abc_archive)
		return OObject();
	
	ObjectWriterPtr root_ptr = root().getPtr();
	
	ObjectWriterPtr child = root_ptr->getChild(id->name);
	if (child)
		return OObject(child, kWrapExisting);
	else {
		const ObjectHeader *child_header = root_ptr->getChildHeader(id->name);
		if (child_header)
			return OObject(root_ptr->createChild(*child_header), kWrapExisting);
		else {
			return OObject();
		}
	}
}

OObject AbcWriterArchive::root()
{
	if (m_use_render)
		return m_abc_root_render ? m_abc_root_render : OObject();
	else
		return m_abc_root ? m_abc_root : OObject();
}

bool AbcWriterArchive::has_id_object(ID *id)
{
	if (!m_abc_archive)
		return false;
	
	ObjectWriterPtr root_ptr = root().getPtr();
	
	return root_ptr->getChildHeader(id->name) != NULL;
}

TimeSamplingPtr AbcWriterArchive::frame_sampling()
{
	return m_abc_archive.getTimeSampling(m_frame_sampling);
}

} /* namespace PTC */
