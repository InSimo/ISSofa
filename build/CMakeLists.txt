cmake_minimum_required(VERSION 2.8.12)

set(SOLUTION_NAME "Sofa")
project(${SOLUTION_NAME})

get_filename_component(SOFA_ACTUAL_SOURCE_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

# Environment
set(SOFA_CMAKE_DIR "${SOFA_ACTUAL_SOURCE_DIR}/cmake" )
list(APPEND CMAKE_MODULE_PATH "${SOFA_CMAKE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${SOFA_CMAKE_DIR}/Modules")

# Options 
option(SOFA_INSTALL_HEADERS_GENERATED_ONLY "" ON)
option(SOFA_ENABLE_UNIT_TESTS  "Set to true to enable unit tests for sofa" ON)
option(SOFA_ALLOW_UNITY_BUILDS "Set to true to build sofa using unity builds and get the option to toggle unity build on/off for each sofa project" OFF)

# Forced options
set(SOFAFRAMEWORK_DETECTIONOUTPUT_BARYCENTRICINFO ON CACHE BOOL "Activate DETECTIONOUTPUT_BARYCENTRICINFO option" FORCE)
set(SOFAFRAMEWORK_CHECK_CONTAINER_ACCESS ON CACHE BOOL "Compile SofaHelper library with with SOFA_CONTAINER_ACCESS_FAILURE preprocessor definition" FORCE)

# Sofa extlibs
set(SOFA_EXTLIBS_DIR ${SOFA_ACTUAL_SOURCE_DIR}/extlibs)
set(SOFA_EXTLIBS_TARGETS_DEFAULT eigen-3.2.7;miniFlowVR;newmat;tinyxml;metis-5.1.0;csparse;qwt-6.0.1;gtest)
set(SOFA_EXTLIBS_TARGETS ${SOFA_EXTLIBS_TARGETS_DEFAULT} CACHE STRING "An ordered list (semicolon separated) list of sofa extlibs to add to the build.")

if(${SOFA_ENABLE_UNIT_TESTS})
    list(FIND SOFA_EXTLIBS_TARGETS gtest SOFA_EXTLIBS_TARGET_HAS_GTEST)
    if(NOT ${SOFA_EXTLIBS_TARGET_HAS_GTEST} )
        message(WARNING "SOFA_ENABLE_UNIT_TESTS is ON. Adding gtest to SOFA_EXTLIBS_TARGETS")
        list(APPEND ${SOFA_EXTLIBS_TARGETS} "gtest")
    endif()
    set(gtest_force_shared_crt ON CACHE BOOL "Use shared (DLL) run-time lib even when Google Test is built as static lib." FORCE)
    enable_testing()
endif()

# Sofa libraries
set(SOFA_LIBRARIES_DIR ${SOFA_ACTUAL_SOURCE_DIR})
set(SOFA_LIBRARIES_TARGETS_DEFAULT SofaFramework;SofaSimulation;SofaBase;SofaCommon;SofaGeneral;SofaAdvanced;SofaMisc)
set(SOFA_LIBRARIES_TARGETS ${SOFA_LIBRARIES_TARGETS_DEFAULT} CACHE STRING "An ordered list (semicolon separated) of sofa libraries to add to the build.")

# Sofa plugins
set(SOFA_PLUGINS_DIR "${SOFA_ACTUAL_SOURCE_DIR}/applications/plugins")
set(SOFA_PLUGINS_TARGETS_DEFAULT SofaPython;MultiThreading)
set(SOFA_PLUGINS_TARGETS ${SOFA_PLUGINS_TARGETS_DEFAULT} CACHE STRING "An ordered list (semicolon separated) of sofa plugins to add to the build.")

# Sofa projects
set(SOFA_PROJECTS_DIR "${SOFA_ACTUAL_SOURCE_DIR}/applications/projects" )
set(SOFA_PROJECTS_TARGETS_DEFAULT runSofa)
set(SOFA_PROJECTS_TARGETS ${SOFA_PROJECTS_TARGETS_DEFAULT} CACHE STRING "An ordered list (semicolon separated) of sofa projects to add to the build.")


foreach(TARGET ${SOFA_EXTLIBS_TARGETS} )
    if( EXISTS ${SOFA_EXTLIBS_DIR}/${TARGET}/CMakeLists.txt )
        add_subdirectory( ${SOFA_EXTLIBS_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
    elseif( EXISTS ${SOFA_EXTLIBS_DIR}/${TARGET}/src/CMakeLists.txt )
        add_subdirectory( ${SOFA_EXTLIBS_DIR}/${TARGET}/src ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
    endif()
endforeach()

foreach(TARGET ${SOFA_LIBRARIES_TARGETS} )
    add_subdirectory( ${SOFA_LIBRARIES_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
endforeach()

foreach(TARGET ${SOFA_PLUGINS_TARGETS} )
    add_subdirectory( ${SOFA_PLUGINS_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
endforeach()

foreach(TARGET ${SOFA_PROJECTS_TARGETS} )
    add_subdirectory( ${SOFA_PROJECTS_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
endforeach()