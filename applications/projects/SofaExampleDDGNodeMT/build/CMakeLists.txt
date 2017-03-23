cmake_minimum_required(VERSION 2.8.12)

set(SOLUTION_NAME "SofaExampleDDGNodeMT")
project(${SOLUTION_NAME})

set(ISSOFA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." CACHE PATH "Path to the ISSofa root directory")

# MSVC: Use solution folders.
if(MSVC)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

if( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
  set( CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
endif (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )

message("Install prefix: ${CMAKE_INSTALL_PREFIX}")

# Set the output directories globally
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# CMake Environment
set(SOFA_CMAKE_DIR "${ISSOFA_DIR}/cmake" )
list(APPEND CMAKE_MODULE_PATH "${SOFA_CMAKE_DIR}/Modules")
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

# Compilation flags
set ( CMAKE_CXX_STANDARD 11 ) # Enable c++11
if(${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
    # Enable multi-processor compilation by default. Disable some warnings
    set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /wd4250 /wd4251 /wd4275 /wd4675 /wd4996" )
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
    set ( CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined -lc ${CMAKE_SHARED_LINKER_FLAGS}")
    set ( CMAKE_MODULE_LINKER_FLAGS "-Wl,--no-undefined -lc ${CMAKE_MODULE_LINKER_FLAGS}")
endif()

#include(CMakeSofaBuildFlags)

# Sofa libraries
set(ISSOFA_EXTLIBS_DIR "${ISSOFA_DIR}/extlibs")

# cmake functions specific to sofa that needs to be redistributed.
add_subdirectory( ${SOFA_CMAKE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/SofaCMake)

# win32 externals
if(WIN32)
  set(ISSOFA_DEPENDENCIES_DIR ${ISSOFA_DIR}/../SofaWinDependencies CACHE PATH "ISSofa dependencies directory")
  # I don't think I can set CMAKE_INCLUDE_PATH and CMAKE_LIBRARY_PATH in a subdirectory of the main CMakeLists.txt
  # without doing something which can causes hard to find hard to debug side effects.
  list(APPEND CMAKE_INCLUDE_PATH ${ISSOFA_DEPENDENCIES_DIR}/include)
  if(CMAKE_CL_64)
    list(APPEND CMAKE_LIBRARY_PATH ${ISSOFA_DEPENDENCIES_DIR}/lib/win64)
  else()
    list(APPEND CMAKE_LIBRARY_PATH ${ISSOFA_DEPENDENCIES_DIR}/lib/win32)
  endif()
  add_subdirectory(${ISSOFA_DEPENDENCIES_DIR} ${CMAKE_CURRENT_BINARY_DIR}/SofaWinDependencies )
  # disable png and zlib, they are broken in the dependency package
  set(CMAKE_DISABLE_FIND_PACKAGE_ZLIB TRUE CACHE BOOL "")
  set(CMAKE_DISABLE_FIND_PACKAGE_PNG  TRUE CACHE BOOL "")
  
  set(ISEXTERNALS_DIR ${ISSOFA_DIR}/../ISExternals CACHE PATH "ISExternals directory")
  add_subdirectory(${ISEXTERNALS_DIR}/boost-thread ${CMAKE_CURRENT_BINARY_DIR}/boost-thread)
endif()


option( SOFA_INSTALL_HEADERS_GENERATED_ONLY "Disable installation of non-generated headers" OFF )
option( ISSOFA_ENABLE_UNITTESTS "Activate compilation of unit tests" ON)

if( ISSOFA_ENABLE_UNITTESTS )
  #  gtest
  set(gtest_force_shared_crt ON CACHE BOOL "Use shared (DLL) run-time lib even when Google Test is built as static lib.")
  add_subdirectory(${ISSOFA_EXTLIBS_DIR}/gtest ${CMAKE_CURRENT_BINARY_DIR}/gtest)
endif()

#  eigen
set(EIGEN_ROOT ${ISSOFA_EXTLIBS_DIR}/eigen-3.2.7 )
# tinyxml
add_subdirectory(${ISSOFA_EXTLIBS_DIR}/tinyxml ${CMAKE_CURRENT_BINARY_DIR}/tinyxml)
# SofaFramework
add_subdirectory( ${ISSOFA_DIR}/SofaFramework ${CMAKE_CURRENT_BINARY_DIR}/SofaFramework)
# SofaExampleDDGNodeMT
add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/SofaExampleDDGNodeMT )

