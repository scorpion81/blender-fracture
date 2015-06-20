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

#ifndef PTC_ABC_FRAME_MAPPER_H
#define PTC_ABC_FRAME_MAPPER_H

#ifdef WITH_ALEMBIC
#include <Alembic/AbcCoreAbstract/Foundation.h>
#include <Alembic/Abc/ISampleSelector.h>
#endif

struct Scene;

namespace PTC {

#ifdef WITH_ALEMBIC

using namespace Alembic;
using Alembic::AbcCoreAbstract::chrono_t;

class FrameMapper {
public:
	FrameMapper(double fps, float start_frame);
	
	double frames_per_second() const { return m_frames_per_sec; }
	double seconds_per_frame() const { return m_sec_per_frame; }
	double start_frame() const { return m_start_frame; }
	double start_time() const { return m_start_time; }
	
	chrono_t frame_to_time(float frame) const;
	float time_to_frame(chrono_t time) const;
	
private:
	double m_frames_per_sec, m_sec_per_frame;
	double m_start_frame, m_start_time;
};

#endif /* WITH_ALEMBIC */

} /* namespace PTC */

#endif  /* PTC_UTIL_FRAME_MAPPER_H */
