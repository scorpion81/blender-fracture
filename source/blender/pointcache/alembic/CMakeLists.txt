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
	../
	../intern
	../util
	../../blenkernel
	../../blenlib
	../../makesdna
	../../makesrna
	../../../../intern/guardedalloc
)

set(INC_SYS
	${ALEMBIC_INCLUDE_DIRS}
	${OPENEXR_INCLUDE_DIR}/OpenEXR
)

set(SRC
	alembic.cpp
	alembic.h

	abc_frame_mapper.cpp
	abc_frame_mapper.h
	abc_info.cpp
	abc_interpolate.cpp
	abc_interpolate.h
	abc_reader.cpp
	abc_reader.h
	abc_schema.h
	abc_split.cpp
	abc_writer.cpp
	abc_writer.h

	abc_cloth.cpp
	abc_cloth.h
	abc_customdata.cpp
	abc_customdata.h
	abc_group.cpp
	abc_group.h
	abc_mesh.cpp
	abc_mesh.h
	abc_object.cpp
	abc_object.h
	abc_particles.cpp
	abc_particles.h
	abc_simdebug.cpp
	abc_simdebug.h
)

add_definitions(-DWITH_ALEMBIC)

blender_add_lib(bf_pointcache_alembic "${SRC}" "${INC}" "${INC_SYS}")
