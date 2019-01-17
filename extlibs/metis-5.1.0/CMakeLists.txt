cmake_minimum_required(VERSION 2.8.12)

project(metis)

set(METIS_INSTALL FALSE)

unset(GKLIB_PATH CACHE)
get_filename_component(GKLIB_PATH "GKlib" ABSOLUTE)
include(GKlib/GKlibSystem.cmake)

file(GLOB metis_sources libmetis/*.c)

# The code must be relocatable if we want to link a shared library against it.
if("x${CMAKE_CXX_COMPILER_ID}" STREQUAL "xGNU" OR "x${CMAKE_CXX_COMPILER_ID}" STREQUAL "xClang")
    add_compile_options(-fPIC)
endif()

add_library(${PROJECT_NAME} STATIC ${metis_sources} ${GKlib_sources})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/GKlib>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libmetis>")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/metis.h)

if(UNIX)
    target_link_libraries(${PROJECT_NAME} m)
endif()


include(CMakePackageConfigHelpers)

# MetisConfig.cmake (build tree)
set(METIS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_package_config_file(MetisConfig.cmake.in
                              ${CMAKE_CURRENT_BINARY_DIR}/MetisConfig.cmake
                              INSTALL_DESTINATION metis
                              PATH_VARS METIS_INCLUDE_DIR)

# MetisConfig.cmake (installed)
set(METIS_INCLUDE_DIR include)
configure_package_config_file(MetisConfig.cmake.in
                              ${CMAKE_CURRENT_BINARY_DIR}/MetisConfigInstall.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Metis
                              PATH_VARS METIS_INCLUDE_DIR)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/MetisConfigInstall.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Metis
        RENAME MetisConfig.cmake)

install(TARGETS ${PROJECT_NAME}
    EXPORT MetisTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Metis_libraries
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Metis_libraries
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Metis_libraries
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT Metis_headers)

# MetisTargets.cmake (installed)
install(EXPORT MetisTargets
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Metis)

# compatibility with FindMETIS.cmake

set(METIS_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Path to search for METIS include files" FORCE )
set(METIS_LIBRARIES "metis" CACHE PATH "METIS library files" FORCE )
