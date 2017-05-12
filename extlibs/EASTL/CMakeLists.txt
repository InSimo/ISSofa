#-------------------------------------------------------------------------------------------
# Copyright (C) Electronic Arts Inc.  All rights reserved.
#-------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(EASTL)

set(EASTL_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(EASTL_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(EASTL_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(EASTL_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/EASTL)

# Offer the user the choice of overriding the installation directories
set(EASTL_INSTALL_LIB_DIR     lib                       CACHE PATH "Installation directory for libraries relative to CMAKE_INSTALL_PREFIX variable")
set(EASTL_INSTALL_BIN_DIR     bin                       CACHE PATH "Installation directory for executables relative to CMAKE_INSTALL_PREFIX variable")
set(EASTL_INSTALL_INCLUDE_DIR include                   CACHE PATH "Installation directory for header files relative to CMAKE_INSTALL_PREFIX variable")
set(EASTL_INSTALL_CMAKE_DIR   ${EASTL_INSTALL_LIB_DIR}/cmake/EASTL )

#-------------------------------------------------------------------------------------------
# Options
#-------------------------------------------------------------------------------------------
option(EASTL_BUILD_BENCHMARK "Enable generation of build files for benchmark" OFF)
option(EASTL_BUILD_TESTS "Enable generation of build files for tests" OFF)

#-------------------------------------------------------------------------------------------
# Sub-projects
#-------------------------------------------------------------------------------------------
add_subdirectory(test/packages/EABase)

if(EASTL_BUILD_TESTS OR EASTL_BUILD_BENCHMARK)
    add_subdirectory(test/packages/EAAssert)
    add_subdirectory(test/packages/EAStdC)
    add_subdirectory(test/packages/EAMain)
    add_subdirectory(test/packages/EATest)
    add_subdirectory(test/packages/EAThread)
endif()

if(EASTL_BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()

if(EASTL_BUILD_TESTS)
    add_subdirectory(test)
endif()

#-------------------------------------------------------------------------------------------
# Library definition
#-------------------------------------------------------------------------------------------
file(GLOB HEADER_FILES "include/EASTL/*.h")
file(GLOB SOURCE_FILES "source/*.cpp")
add_library(EASTL SHARED ${HEADER_FILES} ${SOURCE_FILES})

#-------------------------------------------------------------------------------------------
# Compiler Flags
#-------------------------------------------------------------------------------------------
set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/scripts/CMake")
include(CommonCppFlags)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_SIZEOF_VOID_P EQUAL 4 AND
        (CMAKE_BUILD_TYPE MATCHES "Release" OR CMAKE_BUILD_TYPE MATCHES "MinSizeRel"
            OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo") AND MINGW)
    message(FATAL_ERROR "FIXME: 32bits mingw-w64 clang crashes when compiling with any optimizations.")
endif()

set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DEASTL_BUILD"  )

#-------------------------------------------------------------------------------------------
# Include dirs
#-------------------------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:${EASTL_INSTALL_INCLUDE_DIR}>")

#-------------------------------------------------------------------------------------------
# Libraries
#-------------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME} PUBLIC EABase)
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

find_package(Threads REQUIRED)
target_link_libraries("${PROJECT_NAME}" PUBLIC "${CMAKE_THREAD_LIBS_INIT}")

#-------------------------------------------------------------------------------------------
# Installation
#-------------------------------------------------------------------------------------------
install(TARGETS "${PROJECT_NAME}"
        EXPORT  SofaFrameworkTargets
        RUNTIME DESTINATION "${EASTL_INSTALL_BIN_DIR}"
        LIBRARY DESTINATION "${EASTL_INSTALL_LIB_DIR}"
        ARCHIVE DESTINATION "${EASTL_INSTALL_LIB_DIR}"
    )

if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/EASTL
        COMPONENT EASTL_headers
        DESTINATION "${EASTL_INSTALL_INCLUDE_DIR}"
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.inl"
            PATTERN "*.hxx"
            PATTERN "*.hpp"
    )

    install(DIRECTORY test/packages/EABase/include/Common/EABase
            COMPONENT EASTL_headers
            DESTINATION "${EASTL_INSTALL_INCLUDE_DIR}")
endif()
