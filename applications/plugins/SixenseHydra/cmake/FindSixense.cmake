# This script locates the Sixense SDK
# ------------------------------------
#
# usage:
# find_package(Sixense ...)
#
# searches in SIXENSE_ROOT and usual locations
#
#  SIXENSE_INCLUDE_DIR -  where to find SIXENSE.h, etc.
#  SIXENSE_LIBRARIES    - List of libraries when using SIXENSE.
#  SIXENSE_FOUND        - True if SIXENSE found.
#
# Sets SIXENSE_INCLUDE_DIR, SIXENSE_LIBRARY and SIXENSE_UTILS_LIBRARY

set(SIXENSE_POSSIBLE_PATHS
	${SIXENSE_ROOT}
	$ENV{SIXENSE_ROOT}
	$ENV{SIXENSE_SDK_PATH}
	"$ENV{ProgramFiles}/Steam/SteamApps/common/Sixense SDK/SixenseSDK"
	"$ENV{ProgramFiles(x86)}/Steam/SteamApps/common/Sixense SDK/SixenseSDK"
	"$ENV{ProgramFiles}/SixenseSDK"
	"$ENV{ProgramFiles(x86)}/SixenseSDK"
	"C:/SixenseSDK"
	~/Library/Frameworks
	/Library/Frameworks
	/usr/local/
	/usr/
	/sw # Fink
	/opt/local/ # DarwinPorts
	/opt/csw/ # Blastwave
	/opt/
    ${CMAKE_CURRENT_SOURCE_DIR}
    $ENV{HOME} # user installation
	)


IF( WIN32 )
  IF( CMAKE_CL_64 )
    SET( SDK_LIB "x64/release_dll" )
    SET( LIB_SUFFIX "_x64" )
  ELSE( CMAKE_CL_64 )
    SET( SDK_LIB "win32/release_dll" )
    SET( LIB_SUFFIX "" )
  ENDIF( CMAKE_CL_64 )
ELSE( WIN32 )
  IF( APPLE )
    IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
      SET( SDK_LIB "osx_x64/release_dll" )
      SET( LIB_SUFFIX "_x64" )
    ELSE("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
      SET( SDK_LIB "osx/release_dll" )
      SET( LIB_SUFFIX "" )
    ENDIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
  ELSE( APPLE )
    IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
      SET( SDK_LIB "linux_x64/release" )
      SET( LIB_SUFFIX "_x64" )
    ELSE("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
      SET( SDK_LIB "linux/release" )
      SET( LIB_SUFFIX "" )
    ENDIF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
  ENDIF( APPLE )
ENDIF( WIN32 )


find_path(SIXENSE_INCLUDE_DIR sixense.h
	PATH_SUFFIXES
		"include"
	PATHS
		${SIXENSE_POSSIBLE_PATHS}
	)

find_library(SIXENSE_LIBRARY NAMES sixense${LIB_SUFFIX} sixense
	PATH_SUFFIXES
		"lib"
		"lib/${SDK_LIB}"
	PATHS
		${SIXENSE_POSSIBLE_PATHS}
	)

find_library(SIXENSE_UTILS_LIBRARY NAMES sixense_utils${LIB_SUFFIX} sixense_utils
	PATH_SUFFIXES
		"lib"
		"lib/${SDK_LIB}"
	PATHS
		${SIXENSE_POSSIBLE_PATHS}
	)

# Copy the results to the output variables.
IF(SIXENSE_INCLUDE_DIR AND SIXENSE_LIBRARY)
  SET(SIXENSE_FOUND 1)
  SET(SIXENSE_LIBRARIES ${SIXENSE_LIBRARY})
  SET(SIXENSE_INCLUDE_DIR ${SIXENSE_INCLUDE_DIR})
ELSE(SIXENSE_INCLUDE_DIR AND SIXENSE_LIBRARY)
  SET(SIXENSE_FOUND 0)
  SET(SIXENSE_LIBRARIES)
  SET(SIXENSE_INCLUDE_DIR)
ENDIF(SIXENSE_INCLUDE_DIR AND SIXENSE_LIBRARY)

# Report the results.
IF(NOT SIXENSE_FOUND)
  SET(SIXENSE_DIR_MESSAGE
    "SIXENSE was not found. Make sure SIXENSE_LIBRARY and SIXENSE_INCLUDE_DIR are set to where you have your sixense sdk header and lib files. If you do not have the library you will not be able to use the HydraSensor.")
  IF(SIXENSE_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${SIXENSE_DIR_MESSAGE}")
  ELSEIF(NOT SIXENSE_FIND_QUIETLY)
    MESSAGE(STATUS "${SIXENSE_DIR_MESSAGE}")
  ENDIF(SIXENSE_FIND_REQUIRED)
ELSE(NOT SIXENSE_FOUND)
  IF (NOT Sixense_FIND_QUIETLY)
	MESSAGE(STATUS "Found Sixense SDK: Headers ${SIXENSE_INCLUDE_DIR} Library ${SIXENSE_LIBRARY} Utils ${SIXENSE_UTILS_LIBRARY}")
  ENDIF (NOT Sixense_FIND_QUIETLY)
ENDIF(NOT SIXENSE_FOUND)
