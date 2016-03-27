# $Id: CMakeLists.txt 21789 2009-07-22 05:35:12Z kazanbas $
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
# The Original Code is Copyright (C) 2006, Blender Foundation
# All rights reserved.
#
# The Original Code is: all of this file.
#
# Contributor(s): Esteban Tovagliari.
#
# ***** END GPL LICENSE BLOCK *****

remove_strict_flags()

set(INC
	.
	../blenkernel
	../blenlib
	../blenloader
	../editors/include
	../makesdna
	../makesrna
	../windowmanager
	../../../intern/guardedalloc
)

set(INC_SYS
	${ALEMBIC_INCLUDE_DIRS}
	${HDF5_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OPENEXR_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
)

set(SRC
	intern/abc_camera.cc
	intern/abc_exporter.cc
	intern/abc_export_options.cc
	intern/abc_hair.cc
	intern/abc_mesh.cc
	intern/abc_nurbs.cc
	intern/abc_object.cc
	intern/abc_shape.cc
	intern/abc_transform.cc
	intern/abc_util.cc
	intern/alembic_capi.cc

	ABC_alembic.h
	intern/abc_camera.h
	intern/abc_exporter.h
	intern/abc_export_options.h
	intern/abc_hair.h
	intern/abc_mesh.h
	intern/abc_nurbs.h
	intern/abc_object.h
	intern/abc_shape.h
	intern/abc_transform.h
	intern/abc_util.h
)

blender_add_lib(bf_abc "${SRC}" "${INC}" "${INC_SYS}")
