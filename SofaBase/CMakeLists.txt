cmake_minimum_required(VERSION 2.8.12)
set(SOLUTION_NAME "SofaBase")
project(${SOLUTION_NAME})

#temporary until we move the sub projects
get_filename_component(SofaBase_PARENT_SOURCE_DIR ${SofaBase_SOURCE_DIR} DIRECTORY)
set(SofaBase_ACTUAL_SOURCE_DIR ${SofaBase_PARENT_SOURCE_DIR}/modules/)
set(SOFABASE_SOURCE_INCLUDE_DIR ${SofaBase_ACTUAL_SOURCE_DIR})
##
## SofaBase version
##
set( SOFABASE_MAJOR_VERSION 0 )
set( SOFABASE_MINOR_VERSION 0 )
set( SOFABASE_BUILD_VERSION 1 )

set( SOFABASE_VERSION ${SOFABASE_MAJOR_VERSION}.${SOFABASE_MINOR_VERSION}.${SOFABASE_BUILD_VERSION} CACHE INTERNAL "Set to internal so it can be changed by sub directories.")

set(SOFABASE_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(SOFABASE_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(SOFABASE_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(SOFABASE_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/SofaBase)

# Offer the user the choice of overriding the installation directories
set(SOFABASE_INSTALL_LIB_DIR     ${CMAKE_INSTALL_LIBDIR}     CACHE PATH "Installation directory for libraries")
set(SOFABASE_INSTALL_BIN_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for executables")
set(SOFABASE_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
set(SOFABASE_INSTALL_CMAKE_DIR   ${SOFABASE_INSTALL_LIB_DIR}/cmake/SofaBase)

set(SOFABASE_DEPENDENCY_INCLUDE_DIR)

## SofaBase dependencies
##
if( NOT TARGET SofaSimulationCommon )
    find_package(SofaSimulation REQUIRED)
endif()

list(APPEND SOFABASE_DEPENDENCY_INCLUDE_DIR ${SofaSimulation_INCLUDE_DIRS})

configure_file( ${SofaBase_SOURCE_DIR}/SofaBase.h.in
                ${CMAKE_CURRENT_BINARY_DIR}/sofa/SofaBase.h )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/sofa/SofaBase.h
         DESTINATION "${SOFABASE_INSTALL_INCLUDE_DIR}/sofa")

set(SOFABASE_TARGETS SofaBaseTopology SofaBaseLinearSolver SofaBaseMechanics SofaBaseCollision
                       SofaBaseAnimationLoop SofaBaseVisual SofaComponentBase)

foreach(TARGET ${SOFABASE_TARGETS})
  add_subdirectory(${SofaBase_ACTUAL_SOURCE_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET})
  if(MSVC)
    set_target_properties(${TARGET} PROPERTIES FOLDER "SofaBase")
  endif()
endforeach()

include(CMakePackageConfigHelpers)

# SofaBaseConfigVersion.cmake file (same for build tree and intall)
write_basic_package_version_file(${SOFABASE_BUILD_CMAKE_DIR}/SofaBaseConfigVersion.cmake
                                 VERSION ${SOFABASE_VERSION}
                                 COMPATIBILITY AnyNewerVersion)
install(FILES ${SOFABASE_BUILD_CMAKE_DIR}/SofaBaseConfigVersion.cmake
        DESTINATION ${SOFABASE_INSTALL_CMAKE_DIR})

# SofaBaseConfig.cmake (build tree)
set(SOFABASE_INCLUDE_DIR "${SOFABASE_BUILD_INCLUDE_DIR}")
configure_package_config_file(SofaBaseConfig.cmake.in
                              ${SOFABASE_BUILD_CMAKE_DIR}/SofaBaseConfig.cmake
                              INSTALL_DESTINATION ${SOFABASE_BUILD_CMAKE_DIR}
                              INSTALL_PREFIX ${CMAKE_BINARY_DIR}
                              PATH_VARS SOFABASE_INCLUDE_DIR)

# SofaBaseConfig.cmake (installed)
set(SOFABASE_INCLUDE_DIR "${SOFABASE_INSTALL_INCLUDE_DIR}")
configure_package_config_file(SofaBaseConfig.cmake.in
                              ${SOFABASE_BUILD_CMAKE_DIR}/SofaBaseConfigInstall.cmake
                              INSTALL_DESTINATION ${SOFABASE_INSTALL_CMAKE_DIR}
                              PATH_VARS SOFABASE_INCLUDE_DIR )

install(FILES ${SOFABASE_BUILD_CMAKE_DIR}/SofaBaseConfigInstall.cmake
        DESTINATION ${SOFABASE_INSTALL_CMAKE_DIR}
        RENAME SofaBaseConfig.cmake)

# SofaBaseTargets.cmake (installed)
install(EXPORT SofaBaseTargets
        DESTINATION ${SOFABASE_INSTALL_CMAKE_DIR} )

# compatibility with find_package
set(SofaBase_LIBRARIES SofaComponentBase CACHE INTERNAL "")
set(SOFABASE_INCLUDE_DIR "$<BUILD_INTERFACE:${SOFABASE_BUILD_INCLUDE_DIR}>$<INSTALL_INTERFACE:${SOFABASE_INSTALL_INCLUDE_DIR}>" CACHE INTERNAL "")
set(SofaBase_INCLUDE_DIRS ${SOFABASE_INCLUDE_DIR} ${SofaSimulation_INCLUDE_DIRS} CACHE INTERNAL "")
