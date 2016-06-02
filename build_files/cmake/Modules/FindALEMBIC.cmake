# - Find Alembic library
# Find the native Alembic includes and libraries
# This module defines
#  ALEMBIC_INCLUDE_DIRS, where to find samplerate.h, Set when
#                        ALEMBIC_INCLUDE_DIR is found.
#  ALEMBIC_LIBRARIES, libraries to link against to use Samplerate.
#  ALEMBIC_ROOT_DIR, The base directory to search for Samplerate.
#                    This can also be an environment variable.
#  ALEMBIC_FOUND, If false, do not try to use Samplerate.
#

#=============================================================================
# Copyright 2011 Blender Foundation.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

# If ALEMBIC_ROOT_DIR was defined in the environment, use it.
IF(NOT ALEMBIC_ROOT_DIR AND NOT $ENV{ALEMBIC_ROOT_DIR} STREQUAL "")
  SET(ALEMBIC_ROOT_DIR $ENV{ALEMBIC_ROOT_DIR})
ENDIF()

SET(_alembic_SEARCH_DIRS
  ${ALEMBIC_ROOT_DIR}
  ${ALEMBIC_ROOT_DIR}/lib/static
  ${HDF5_ROOT_DIR}/lib
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
)

FIND_PATH(ALEMBIC_INCLUDE_DIR NAMES Alembic/Abc/All.h HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES include)

FIND_LIBRARY(ALEMBIC_ABC_LIBRARY 			NAMES AlembicAbc HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_ABCGEOM_LIBRARY 		NAMES AlembicAbcGeom HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_ABCCORE_ABC_LIBRARY 	NAMES AlembicAbcCoreAbstract HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_ABCUTIL_LIBRARY 		NAMES AlembicUtil HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_ABCCORE_HDF5_LIBRARY 	NAMES AlembicAbcCoreHDF5 HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_OGAWA_LIBRARY 	        NAMES AlembicOgawa HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_ABCOGAWA_LIBRARY 	    NAMES AlembicAbcCoreOgawa HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_ABCMATERIAL     	    NAMES AlembicAbcMaterial HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(ALEMBIC_ABCCOREFACTORY    	    NAMES AlembicAbcCoreFactory HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)

FIND_LIBRARY(HDF5_LIBRARY 					NAMES hdf5 HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)
FIND_LIBRARY(HDF5_HL_LIBRARY 				NAMES hdf5_hl HINTS ${_alembic_SEARCH_DIRS} PATH_SUFFIXES lib64 lib)

# handle the QUIETLY and REQUIRED arguments and set ALEMBIC_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ALEMBIC DEFAULT_MSG ALEMBIC_ABC_LIBRARY ALEMBIC_INCLUDE_DIR)

IF(ALEMBIC_FOUND)
  SET(ALEMBIC_LIBRARIES ${ALEMBIC_ABC_LIBRARY}
  						${ALEMBIC_OGAWA_LIBRARY}
						${ALEMBIC_ABCGEOM_LIBRARY}
						${ALEMBIC_ABCCORE_ABC_LIBRARY}
						${ALEMBIC_ABCCOREFACTORY}
						${ALEMBIC_ABCUTIL_LIBRARY}
						${ALEMBIC_ABCCORE_HDF5_LIBRARY}
						${ALEMBIC_ABCOGAWA_LIBRARY}
						${ALEMBIC_ABCMATERIAL}
						${HDF5_HL_LIBRARY}
						${HDF5_LIBRARY}
						)

  SET(ALEMBIC_INCLUDE_DIRS ${ALEMBIC_INCLUDE_DIR})
ENDIF(ALEMBIC_FOUND)

MARK_AS_ADVANCED(
  ALEMBIC_INCLUDE_DIR
  ALEMBIC_LIBRARY
)
