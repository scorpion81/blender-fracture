# - Find JPEG library
# Find the native JPEG includes and library
# This module defines
#  JPEG_INCLUDE_DIRS, where to find jpeg.h, Set when
#                    JPEG is found.
#  JPEG_LIBRARIES, libraries to link against to use JPEG.
#  JPEG_ROOT_DIR, The base directory to search for JPEG.
#                This can also be an environment variable.
#  JPEG_FOUND, If false, do not try to use JPEG.
#
# also defined, but not for general use are
#  JPEG_LIBRARY, where to find the JPEG library.

#=============================================================================
# Copyright 2018 Martin Felke
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

# If JPEG_ROOT_DIR was defined in the environment, use it.
IF(NOT JPEG_ROOT_DIR AND NOT $ENV{JPEG_ROOT_DIR} STREQUAL "")
  SET(JPEG_ROOT_DIR $ENV{JPEG_ROOT_DIR})
ENDIF()

SET(_jpeg_SEARCH_DIRS
  ${JPEG_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/lib/jpeg
)

FIND_PATH(JPEG_INCLUDE_DIR
  NAMES
    jpeglib.h
  HINTS
    ${_jpeg_SEARCH_DIRS}
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(JPEG_LIBRARY
  NAMES
    jpeg
  HINTS
    ${_jpeg_SEARCH_DIRS}
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set JPEG_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(JPEG DEFAULT_MSG
    JPEG_LIBRARY JPEG_INCLUDE_DIR)

IF(JPEG_FOUND)
  SET(JPEG_LIBRARIES ${JPEG_LIBRARY})
  SET(JPEG_INCLUDE_DIRS ${JPEG_INCLUDE_DIR})
ELSE()
  SET(JPEG_JPEG_FOUND FALSE)
ENDIF()

MARK_AS_ADVANCED(
  JPEG_INCLUDE_DIR
  JPEG_LIBRARY
)
