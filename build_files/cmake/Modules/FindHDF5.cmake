# - Find HDF5 library
# Find the native hdf5 includes and library
# This module defines
#  HDF5_INCLUDE_DIRS, where to find hdf5 headers.
#  HDF5_LIBRARIES, libraries to link against to use hdf5.
#  HDF5_ROOT_DIR, The base directory to search for hdf5.
#                 This can also be an environment variable.
#  HDF5_FOUND, If false, do not try to use hdf5.

#=============================================================================
# Copyright 2013 Blender Foundation.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

# If HDF5_ROOT_DIR was defined in the environment, use it.
IF(NOT HDF5_ROOT_DIR AND NOT $ENV{HDF5_ROOT_DIR} STREQUAL "")
  SET(HDF5_ROOT_DIR $ENV{HDF5_ROOT_DIR})
ENDIF()

SET(_hdf5_SEARCH_DIRS
  ${HDF5_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/hdf5
)

SET(_hdf5_FIND_COMPONENTS
  hdf5
  hdf5_hl
)

FIND_PATH(_hdf5_INCLUDE_DIRS
  NAMES
    hdf5.h
  HINTS
    ${_hdf5_SEARCH_DIRS}
)

SET(_hdf5_LIBRARIES)
FOREACH(COMPONENT ${_hdf5_FIND_COMPONENTS})
  STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)

  FIND_LIBRARY(HDF5_${UPPERCOMPONENT}_LIBRARY
    NAMES
      ${COMPONENT}
    HINTS
      ${_hdf5_SEARCH_DIRS}
    )
  MARK_AS_ADVANCED(HDF5_${UPPERCOMPONENT}_LIBRARY)
  LIST(APPEND _hdf5_LIBRARIES "${HDF5_${UPPERCOMPONENT}_LIBRARY}")
ENDFOREACH()

# handle the QUIETLY and REQUIRED arguments and set HDF5_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(hdf5 DEFAULT_MSG
    _hdf5_LIBRARIES _hdf5_INCLUDE_DIRS)

IF(HDF5_FOUND)
  SET(HDF5_LIBRARIES ${_hdf5_LIBRARIES})
  SET(HDF5_INCLUDE_DIRS ${_hdf5_INCLUDE_DIRS})
ENDIF(HDF5_FOUND)

MARK_AS_ADVANCED(
  HDF5_INCLUDE_DIRS
  HDF5_LIBRARIES
)
