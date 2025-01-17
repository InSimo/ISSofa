cmake_minimum_required(VERSION 2.8.11)
set(SOLUTION_NAME "SofaAdvanced")
project(${SOLUTION_NAME})

#temporary until we move the sub projects
get_filename_component(SofaAdvanced_PARENT_SOURCE_DIR ${SofaAdvanced_SOURCE_DIR} DIRECTORY)
set(SofaAdvanced_ACTUAL_SOURCE_DIR ${SofaAdvanced_PARENT_SOURCE_DIR}/modules)
set(SOFAADVANCED_SOURCE_INCLUDE_DIR ${SofaAdvanced_PARENT_SOURCE_DIR}/modules)

##
## SofaAdvanced version
##
set( SOFAADVANCED_MAJOR_VERSION 0 )
set( SOFAADVANCED_MINOR_VERSION 0 )
set( SOFAADVANCED_BUILD_VERSION 1 )

set( SOFAADVANCED_VERSION ${SOFAADVANCED_MAJOR_VERSION}.${SOFAADVANCED_MINOR_VERSION}.${SOFAADVANCED_BUILD_VERSION} CACHE INTERNAL "Set to internal so it can be changed by sub directories.")

set(SOFAADVANCED_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(SOFAADVANCED_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(SOFAADVANCED_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(SOFAADVANCED_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/SofaAdvanced)

# Offer the user the choice of overriding the installation directories
set(SOFAADVANCED_INSTALL_LIB_DIR     ${CMAKE_INSTALL_LIBDIR}     CACHE PATH "Installation directory for libraries")
set(SOFAADVANCED_INSTALL_BIN_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for executables")
set(SOFAADVANCED_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
set(SOFAADVANCED_INSTALL_CMAKE_DIR   ${SOFAADVANCED_INSTALL_LIB_DIR}/cmake/SofaAdvanced )

## SofaAdvanced dependencies
##
if( NOT TARGET SofaComponentGeneral )
    find_package(SofaGeneral REQUIRED)
endif()
if( TARGET miniflowvr )
    set(MiniFlowVR_FOUND TRUE)
else()
    find_package(MiniFlowVR)
endif()

set(SOFA_HAVE_MINIFLOWVR ${MiniFlowVR_FOUND} CACHE INTERNAL "")

set(SOFAADVANCED_DEPENDENCY_DIR ${SofaGeneral_INCLUDE_DIRS} ${MiniFlowVR_INCLUDE_DIRS} )
set(SOFAADVANCED_DEPENDENCY_LINK ${MiniFlowVR_LIBRARIES})


configure_file( ${SofaAdvanced_SOURCE_DIR}/SofaAdvanced.h.in
                ${CMAKE_CURRENT_BINARY_DIR}/sofa/SofaAdvanced.h )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/sofa/SofaAdvanced.h
         DESTINATION "${SOFAADVANCED_INSTALL_INCLUDE_DIR}/sofa")

set(SOFAADVANCED_TARGETS SofaEulerianFluid SofaSphFluid SofaVolumetricData SofaNonUniformFem SofaComponentAdvanced)

foreach(TARGET ${SOFAADVANCED_TARGETS})
  add_subdirectory(${SofaAdvanced_ACTUAL_SOURCE_DIR}/${TARGET} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET})
  if(MSVC)
    set_target_properties(${TARGET} PROPERTIES FOLDER "SofaAdvanced")
  endif()
endforeach()

include(CMakePackageConfigHelpers)

# SofaAdvancedConfigVersion.cmake file (same for build tree and intall)
write_basic_package_version_file(${SOFAADVANCED_BUILD_CMAKE_DIR}/SofaAdvancedConfigVersion.cmake
                                 VERSION ${SOFAADVANCED_VERSION}
                                 COMPATIBILITY AnyNewerVersion)
install(FILES ${SOFAADVANCED_BUILD_CMAKE_DIR}/SofaAdvancedConfigVersion.cmake
        DESTINATION ${SOFAADVANCED_INSTALL_CMAKE_DIR})

# SofaAdvancedConfig.cmake (build tree)
set(SOFAADVANCED_INCLUDE_DIR "${SOFAADVANCED_BUILD_INCLUDE_DIR}")
configure_package_config_file(SofaAdvancedConfig.cmake.in
                              ${SOFAADVANCED_BUILD_CMAKE_DIR}/SofaAdvancedConfig.cmake
                              INSTALL_DESTINATION ${SOFAADVANCED_BUILD_CMAKE_DIR}
                              INSTALL_PREFIX ${CMAKE_BINARY_DIR}
                              PATH_VARS SOFAADVANCED_INCLUDE_DIR)

# SofaAdvancedConfig.cmake (installed)
set(SOFAADVANCED_INCLUDE_DIR "${SOFAADVANCED_INSTALL_INCLUDE_DIR}")
configure_package_config_file(SofaAdvancedConfig.cmake.in
                              ${SOFAADVANCED_BUILD_CMAKE_DIR}/SofaAdvancedConfigInstall.cmake
                              INSTALL_DESTINATION ${SOFAADVANCED_INSTALL_CMAKE_DIR}
                              PATH_VARS SOFAADVANCED_INCLUDE_DIR )

install(FILES ${SOFAADVANCED_BUILD_CMAKE_DIR}/SofaAdvancedConfigInstall.cmake
        DESTINATION ${SOFAADVANCED_INSTALL_CMAKE_DIR}
        RENAME SofaAdvancedConfig.cmake)

# SofaAdvancedTargets.cmake (installed)
install(EXPORT SofaAdvancedTargets
        DESTINATION ${SOFAADVANCED_INSTALL_CMAKE_DIR} )

# compatibility with find_package
set(SofaAdvanced_LIBRARIES SofaComponentAdvanced CACHE INTERNAL "")
set(SOFAADVANCED_INCLUDE_DIR "$<BUILD_INTERFACE:${SOFAADVANCED_BUILD_INCLUDE_DIR}>$<INSTALL_INTERFACE:${SOFAADVANCED_INSTALL_INCLUDE_DIR}>" CACHE INTERNAL "")
set(SofaAdvanced_INCLUDE_DIRS ${SOFAADVANCED_INCLUDE_DIR} ${SofaGeneral_INCLUDE_DIRS} CACHE INTERNAL "")
