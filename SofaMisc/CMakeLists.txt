cmake_minimum_required(VERSION 2.8.11)
set(SOLUTION_NAME "SofaMisc")
project(${SOLUTION_NAME})

#temporary until we move the sub projects
get_filename_component(SofaMisc_PARENT_SOURCE_DIR ${SofaMisc_SOURCE_DIR} DIRECTORY)
set(SofaMisc_ACTUAL_SOURCE_DIR ${SofaMisc_PARENT_SOURCE_DIR}/modules)
set(SOFAMISC_SOURCE_INCLUDE_DIR ${SofaMisc_PARENT_SOURCE_DIR}/modules)

##
## SofaMisc version
##
set( SOFAMISC_MAJOR_VERSION 0 )
set( SOFAMISC_MINOR_VERSION 0 )
set( SOFAMISC_BUILD_VERSION 1 )

set( SOFAMISC_VERSION ${SOFAMISC_MAJOR_VERSION}.${SOFAMISC_MINOR_VERSION}.${SOFAMISC_BUILD_VERSION} CACHE INTERNAL "Set to internal so it can be changed by sub directories.")

set(SOFAMISC_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(SOFAMISC_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(SOFAMISC_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(SOFAMISC_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/SofaMisc)

# Offer the user the choice of overriding the installation directories
set(SOFAMISC_INSTALL_LIB_DIR     ${CMAKE_INSTALL_LIBDIR}     CACHE PATH "Installation directory for libraries")
set(SOFAMISC_INSTALL_BIN_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for executables")
set(SOFAMISC_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
set(SOFAMISC_INSTALL_CMAKE_DIR   ${SOFAMISC_INSTALL_LIB_DIR}/cmake/SofaMisc)

## SofaMisc dependencies
##
if( NOT TARGET SofaComponentGeneral )
    find_package(SofaGeneral REQUIRED)
endif()
if( NOT TARGET SofaComponentAdvanced )
    find_package(SofaAdvanced REQUIRED)
endif()
set(SOFAMISC_DEPENDENCY_DIR ${SOFAMISC_BUILD_INCLUDE_DIR} ${SofaGeneral_INCLUDE_DIRS} ${SofaAdvanced_INCLUDE_DIRS})

configure_file( ${SofaMisc_SOURCE_DIR}/SofaMisc.h.in
                ${SOFAMISC_BUILD_INCLUDE_DIR}/sofa/SofaMisc.h )
install( FILES ${SOFAMISC_BUILD_INCLUDE_DIR}/sofa/SofaMisc.h
         DESTINATION "${SOFAMISC_INSTALL_INCLUDE_DIR}/sofa")

set(SOFAMISC_TARGETS SofaMiscTopology SofaMiscMapping SofaMiscForceField SofaMiscFem SofaMiscEngine 
    SofaMiscCollision SofaMiscSolver SofaMisc SofaComponentMisc)

foreach(TARGET ${SOFAMISC_TARGETS})
  add_subdirectory(${SofaMisc_ACTUAL_SOURCE_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET})
  # set IDE project filter
  if(MSVC)
    set_target_properties(${TARGET} PROPERTIES FOLDER "SofaMisc")
  endif()
endforeach()

include(CMakePackageConfigHelpers)

# SofaMiscConfigVersion.cmake file (same for build tree and intall)
write_basic_package_version_file(${SOFAMISC_BUILD_CMAKE_DIR}/SofaMiscConfigVersion.cmake
                                 VERSION ${SOFAMISC_VERSION}
                                 COMPATIBILITY AnyNewerVersion)
install(FILES ${SOFAMISC_BUILD_CMAKE_DIR}/SofaMiscConfigVersion.cmake
        DESTINATION ${SOFAMISC_INSTALL_CMAKE_DIR})

# SofaMiscConfig.cmake (build tree)
set(SOFAMISC_INCLUDE_DIR "${SOFAMISC_BUILD_INCLUDE_DIR}")
configure_package_config_file(SofaMiscConfig.cmake.in
                              ${SOFAMISC_BUILD_CMAKE_DIR}/SofaMiscConfig.cmake
                              INSTALL_DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
                              PATH_VARS SOFAMISC_INCLUDE_DIR)

# SofaMiscConfig.cmake (installed)
set(SOFAMISC_INCLUDE_DIR "${SOFAMISC_INSTALL_INCLUDE_DIR}")
configure_package_config_file(SofaMiscConfig.cmake.in
                              ${SOFAMISC_BUILD_CMAKE_DIR}/SofaMiscConfigInstall.cmake
                              INSTALL_DESTINATION ${SOFAMISC_INSTALL_CMAKE_DIR}
                              PATH_VARS SOFAMISC_INCLUDE_DIR )

install(FILES ${SOFAMISC_BUILD_CMAKE_DIR}/SofaMiscConfigInstall.cmake
        DESTINATION ${SOFAMISC_INSTALL_CMAKE_DIR}
        RENAME SofaMiscConfig.cmake)

# SofaMiscTargets.cmake (installed)
install(EXPORT SofaMiscTargets
        DESTINATION ${SOFAMISC_INSTALL_CMAKE_DIR} )

# compatibility with find_package
set(SofaMisc_LIBRARIES SofaComponentMisc CACHE INTERNAL "")
set(SOFAMISC_INCLUDE_DIR "$<BUILD_INTERFACE:${SOFAMISC_BUILD_INCLUDE_DIR}>$<INSTALL_INTERFACE:${SOFAMISC_INSTALL_INCLUDE_DIR}>" CACHE INTERNAL "")
set(SofaMisc_INCLUDE_DIRS ${SOFAMISC_INCLUDE_DIR} ${SofaAdvanced_INCLUDE_DIRS} CACHE INTERNAL "")
