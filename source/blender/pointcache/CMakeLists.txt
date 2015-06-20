# ***** BEGIN GPL LICENSE BLOCK *****
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# The Original Code is Copyright (C) 2013, Blender Foundation
# All rights reserved.
#
# The Original Code is: all of this file.
#
# ***** END GPL LICENSE BLOCK *****

set(INC
	.
	intern
	util
	../blenkernel
	../blenlib
	../makesdna
	../makesrna
	../../../intern/guardedalloc
)

set(INC_SYS
)

set(SRC
	intern/ptc_types.h
	intern/ptc_types.cpp
	intern/reader.h
	intern/reader.cpp
	intern/writer.h
	intern/writer.cpp

	util/util_error_handler.h
	util/util_error_handler.cpp
	util/util_task.cpp
	util/util_task.h
	util/util_thread.h
	util/util_types.h

	PTC_api.h
	PTC_api.cpp
)

if(NOT WITH_CPP11)
	list(APPEND INC_SYS ${BOOST_INCLUDE_DIR})
endif()

if(WITH_ALEMBIC)
	add_definitions(-DWITH_PTC_ALEMBIC)
	add_subdirectory(alembic)
endif()

blender_add_lib(bf_pointcache "${SRC}" "${INC}" "${INC_SYS}")
