cmake_minimum_required(VERSION 2.8.12)

set(SOLUTION_NAME "Sofa")
project(${SOLUTION_NAME})

get_filename_component(SOFA_ACTUAL_SOURCE_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

# Environment
include(GNUInstallDirs)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
set(SOFA_CMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )
list(APPEND CMAKE_MODULE_PATH "${SOFA_CMAKE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${SOFA_CMAKE_DIR}/Modules")

# Options
option(SOFA_INSTALL_HEADERS_GENERATED_ONLY "" ON)
option(SOFA_ENABLE_UNIT_TESTS  "Set to true to enable unit tests for sofa" ON)
option(SOFA_ENABLE_BENCHMARKS  "Set to true to enable compilation and installation of ISSofa benchmarks" OFF)
option(SOFA_ALLOW_UNITY_BUILDS "Set to true to build sofa using unity builds and get the option to toggle unity build on/off for each sofa project" OFF)

# sofa.env options
option(SOFA_GENERATE_ENV_FILE "Generate a sofa.env file in the build directory. Disable if this file is not needed or created some other way" ON)
set(SOFA_ENV_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "If SOFA_GENERATE_ENV_FILE is enabled: directory where sofa.env will be written.")
file(RELATIVE_PATH SOFA_SOURCE_PATH_RELATIVE_TO_ENV ${SOFA_ENV_DIR} ${SOFA_ACTUAL_SOURCE_DIR})
set(SOFA_DATA_PATH_ENV "${SOFA_SOURCE_PATH_RELATIVE_TO_ENV}/examples" "${SOFA_SOURCE_PATH_RELATIVE_TO_ENV}/share" CACHE LIST "A list (semicolon separated) of sofa data paths to write in sofa.env file. They should be relative to the SOFA_ENV_DIR directory (CMAKE_BINARY_DIR by default).")
set(SOFA_PLUGIN_PATH_ENV CACHE LIST "A list (semicolon separated) of sofa data paths to write in sofa.env file. They should be relative to the SOFA_ENV_DIR directory (CMAKE_BINARY_DIR by default).")
set(SOFA_PRELOAD_ENV CACHE LIST "A list (semicolon separated) of preloaded sofa plugins to write in sofa.env file.")

# Offer the user the choice of overriding the installation directories
set(SOFA_INSTALL_BIN_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for executables relative to CMAKE_INSTALL_PREFIX variable")

# Forced options
set(SOFAFRAMEWORK_DETECTIONOUTPUT_BARYCENTRICINFO ON CACHE BOOL "Activate DETECTIONOUTPUT_BARYCENTRICINFO option" FORCE)
set(SOFAFRAMEWORK_CHECK_CONTAINER_ACCESS ON CACHE BOOL "Compile SofaHelper library with with SOFA_CONTAINER_ACCESS_FAILURE preprocessor definition" FORCE)

# Sofa extlibs
set(SOFA_EXTLIBS_DIR ${SOFA_ACTUAL_SOURCE_DIR}/extlibs)
set(SOFA_EXTLIBS_TARGETS_DEFAULT glew-2.1.0;eigen-3.2.7;miniFlowVR;newmat;tinyxml;metis-5.1.0;csparse;qwt-6.0.1;gtest;cpu_features)
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
set(SOFA_PLUGINS_TARGETS_DEFAULT MultiThreading)
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
    elseif( EXISTS ${SOFA_EXTLIBS_DIR}/${TARGET}/build/cmake/CMakeLists.txt )
        add_subdirectory( ${SOFA_EXTLIBS_DIR}/${TARGET}/build/cmake ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
    endif()
endforeach()

foreach(TARGET ${SOFA_LIBRARIES_TARGETS} )
    add_subdirectory( ${SOFA_LIBRARIES_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
endforeach()

# write sofa.env if SOFA_GENERATE_ENV_FILE is enabled
if(${SOFA_GENERATE_ENV_FILE})
    # using SofaHelper target properties allow for later cmake scripts to append values
    set_property(TARGET SofaHelper APPEND PROPERTY SOFA_DATA_PATH ${SOFA_DATA_PATH_ENV})
    set_property(TARGET SofaHelper APPEND PROPERTY SOFA_PLUGIN_PATH ${SOFA_PLUGIN_PATH_ENV})
    set_property(TARGET SofaHelper APPEND PROPERTY SOFA_PRELOAD ${SOFA_PRELOAD_ENV})
    file(GENERATE OUTPUT ${SOFA_ENV_DIR}/sofa.env
         CONTENT "\
# sofa.env generated by CMake.\n\
# Do not edit manually, instead change SOFA_<var>_ENV options in cmake\n\
# or use set_property(TARGET SofaHelper APPEND PROPERTY SOFA_<var> <value>)\n\
SOFA_DATA_PATH=$<JOIN:$<TARGET_PROPERTY:SofaHelper,SOFA_DATA_PATH>,\nSOFA_DATA_PATH=>\n\
SOFA_PLUGIN_PATH=$<JOIN:$<TARGET_PROPERTY:SofaHelper,SOFA_PLUGIN_PATH>,\nSOFA_PLUGIN_PATH=>\n\
SOFA_PRELOAD=$<JOIN:$<TARGET_PROPERTY:SofaHelper,SOFA_PRELOAD>,\nSOFA_PRELOAD=>\n"
        )
    # the code below tried to use a combination of configure_file and file(GENERATE) to combine
    # generator expressions and cached variables substitution, but this did not appear to take
    # into account later changes in cached variables, unless cmake is run multiple times...
    #configure_file("sofa.env.in" ${SOFA_ENV_DIR}/sofa.env.tmp @ONLY)
    #file(GENERATE OUTPUT ${SOFA_ENV_DIR}/sofa.env
    #     INPUT ${SOFA_ENV_DIR}/sofa.env.tmp)

    # Also install the file
    # We only put SOFA_PRELOAD values in the installed version
    # (if at some point we need SOFA_DATA_PATH / SOFA_PLUGIN_PATH in this version
    # we will have to modify them to suit the install tree... but the default values
    # are theoretically sufficient)
    file(GENERATE OUTPUT ${SOFA_ENV_DIR}/sofa-install.env
         CONTENT "\
# sofa.env generated by CMake.\n\
# Do not edit manually, instead change SOFA_<var>_ENV options in cmake\n\
# or use set_property(TARGET SofaHelper APPEND PROPERTY SOFA_<var> <value>)\n\
SOFA_PRELOAD=$<JOIN:$<TARGET_PROPERTY:SofaHelper,SOFA_PRELOAD>,\nSOFA_PRELOAD=>\n"
        )
    install(FILES ${SOFA_ENV_DIR}/sofa-install.env
            DESTINATION ${SOFA_INSTALL_BIN_DIR}
            RENAME sofa.env)
endif()

foreach(TARGET ${SOFA_PLUGINS_TARGETS} )
    add_subdirectory( ${SOFA_PLUGINS_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
endforeach()

foreach(TARGET ${SOFA_PROJECTS_TARGETS} )
    add_subdirectory( ${SOFA_PROJECTS_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET} )
endforeach()
