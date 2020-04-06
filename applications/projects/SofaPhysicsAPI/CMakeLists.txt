cmake_minimum_required(VERSION 2.8.12)
project(SofaPhysicsAPI)

##
## SofaPhysicsAPI version
##
set( SOFAPHYSICSAPI_MAJOR_VERSION 0 )
set( SOFAPHYSICSAPI_MINOR_VERSION 0 )
set( SOFAPHYSICSAPI_BUILD_VERSION 1 )

set( SOFAPHYSICSAPI_VERSION ${SOFAPHYSICSAPI_MAJOR_VERSION}.${SOFAPHYSICSAPI_MINOR_VERSION}.${SOFAPHYSICSAPI_BUILD_VERSION} CACHE INTERNAL "Set to internal so it can be changed by sub directories.")

get_filename_component(SOFAPHYSICSAPI_SOURCE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

set(SOFAPHYSICSAPI_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(SOFAPHYSICSAPI_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(SOFAPHYSICSAPI_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(SOFAPHYSICSAPI_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/SofaPhysicsAPI)

# Offer the user the choice of overriding the installation directories
set(SOFAPHYSICSAPI_INSTALL_LIB_DIR     ${CMAKE_INSTALL_LIBDIR}     CACHE PATH "Installation directory for libraries relative to CMAKE_INSTALL_PREFIX variable")
set(SOFAPHYSICSAPI_INSTALL_BIN_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for executables relative to CMAKE_INSTALL_PREFIX variable")
set(SOFAPHYSICSAPI_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files relative to CMAKE_INSTALL_PREFIX variable")
set(SOFAPHYSICSAPI_INSTALL_CMAKE_DIR   ${SOFAPHYSICSAPI_INSTALL_LIB_DIR}/cmake/SofaPhysicsAPI)

set(HEADER_FILES
    
    SofaPhysicsAPI.h 
    SofaPhysicsSimulation_impl.h 
    SofaPhysicsOutputMesh_impl.h 
    SofaPhysicsDataMonitor_impl.h 
    SofaPhysicsDataController_impl.h 
    SofaPhysicsOutputMesh_Tetrahedron_impl.h
    
    )

set(SOURCE_FILES

    #main.cpp
    SofaPhysicsSimulation.cpp 
    SofaPhysicsOutputMesh.cpp 
    SofaPhysicsDataMonitor.cpp 
    SofaPhysicsDataController.cpp 
    SofaPhysicsOutputMesh_Tetrahedron.cpp
    )

if( NOT TARGET SofaComponentGeneral )
    find_package(SofaGeneral  REQUIRED)
endif()
if( NOT TARGET SofaComponentAdvanced )
    find_package(SofaAdvanced REQUIRED)
endif()
if( NOT TARGET SofaComponentMisc )
    find_package(SofaMisc     REQUIRED)
endif()
set(SOFAPHYSICSAPI_DEPENDENCY_INCLUDE_DIR ${SofaGeneral_INCLUDE_DIRS}) 
    


include( CMakeDependentOption )
cmake_dependent_option(UNITY_BUILD_${PROJECT_NAME} "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." ON  SOFA_ALLOW_UNITY_BUILDS OFF)

if( UNITY_BUILD_${PROJECT_NAME} )
  # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
  # source files.
  set( UNITYBUILD_INCLUDES "" )
  foreach(filename ${SOURCE_FILES})
    SET( UNITYBUILD_INCLUDES "${UNITYBUILD_INCLUDES}#include \"${filename}\"\n" )
  endforeach(filename)

  # Using a cached variable with our string in it. Because cmake doesn't support multi-line strings we have to replace the newlines with a delimiter, so we arbitrarily use +=+.
  string(REPLACE "
" "+=+" UnitybuildIncludesConverted ${UNITYBUILD_INCLUDES}) # Convert the file we're going to write to use our delimiter instead of newlines
  if(NOT (UNITY_BUILD_CACHE_${PROJECT_NAME}) OR NOT (UnitybuildIncludesConverted STREQUAL UNITY_BUILD_CACHE_${PROJECT_NAME})) # If we don't have the cache variable or if its contents don't match our new string then we write the unmodified new UnityBuild file and store the one with the swapped out delimiters in the cache variable
    MESSAGE(STATUS "Updating UnityBuild.cpp for " ${PROJECT_NAME})
    string(REPLACE "
" "+=+" unityBuildCacheNew ${UNITYBUILD_INCLUDES})
    SET(UNITY_BUILD_CACHE_${PROJECT_NAME} ${unityBuildCacheNew} CACHE INTERNAL "Used for determining if UnityBuild.cpp should be updated or not.")
    file(WRITE UnityBuild.cpp ${UNITYBUILD_INCLUDES})
  else()
    MESSAGE(STATUS "Unitybuild.cpp for ${PROJECT_NAME} already up to date")
  endif()
  # overwrite the source files to only include the generated unity build file.
  set( SOURCE_FILES UnityBuild.cpp )
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} )
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaComponentGeneral SofaComponentAdvanced SofaComponentMisc SofaComponentBase SofaComponentCommon) 
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAPHYSICSAPI_SOURCE_INCLUDE_DIR}>") 
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAPHYSICSAPI_DEPENDENCY_INCLUDE_DIR}>")

set( SOFAPHYSICSAPI_COMPILER_FLAGS "-DSOFA_BUILD_SOFAPHYSICSAPI")
# make the name of debug libraries end in _d.
set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFAPHYSICSAPI_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFAPHYSICSAPI_COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

include(CMakePackageConfigHelpers)
# SofaPhysicsAPIConfigVersion.cmake file (same for build tree and intall)
write_basic_package_version_file(${SOFAPHYSICSAPI_BUILD_CMAKE_DIR}/SofaPhysicsAPIConfigVersion.cmake
                                 VERSION ${SOFAPHYSICSAPI_VERSION}
                                 COMPATIBILITY AnyNewerVersion)

install(FILES ${SOFAPHYSICSAPI_BUILD_CMAKE_DIR}/SofaPhysicsAPIConfigVersion.cmake
        DESTINATION ${SOFAPHYSICSAPI_INSTALL_CMAKE_DIR})


# SofaPhysicsAPIConfig.cmake (build tree)
set(SOFAPHYSICSAPI_INCLUDE_DIR "${SOFAPHYSICSAPI_BUILD_INCLUDE_DIR}")
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/SofaPhysicsAPIConfig.cmake.in
                              ${SOFAPHYSICSAPI_BUILD_CMAKE_DIR}/SofaPhysicsAPIConfig.cmake
                              INSTALL_DESTINATION ${SOFAPHYSICSAPI_BUILD_CMAKE_DIR}
                              INSTALL_PREFIX ${CMAKE_BINARY_DIR}
                              PATH_VARS SOFAPHYSICSAPI_INCLUDE_DIR)

# SofaPhysicsAPIConfig.cmake (installed)
set(SOFAPHYSICSAPI_INCLUDE_DIR "${SOFAPHYSICSAPI_INSTALL_INCLUDE_DIR}")
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/SofaPhysicsAPIConfig.cmake.in
                              ${SOFAPHYSICSAPI_BUILD_CMAKE_DIR}/SofaPhysicsAPIConfigInstall.cmake
                              INSTALL_DESTINATION ${SOFAPHYSICSAPI_INSTALL_CMAKE_DIR}
                              PATH_VARS SOFAPHYSICSAPI_INCLUDE_DIR)


install(TARGETS ${PROJECT_NAME}
    COMPONENT SofaPhysicsAPI_libraries
    EXPORT  SofaPhysicsAPITargets
    RUNTIME DESTINATION "${SOFAPHYSICSAPI_INSTALL_BIN_DIR}"
    LIBRARY DESTINATION "${SOFAPHYSICSAPI_INSTALL_LIB_DIR}"
    ARCHIVE DESTINATION "${SOFAPHYSICSAPI_INSTALL_LIB_DIR}"
    PUBLIC_HEADER DESTINATION "${SOFAPHYSICSAPI_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}"
    )

install(EXPORT SofaPhysicsAPITargets DESTINATION ${SOFAPHYSICSAPI_INSTALL_CMAKE_DIR} )

# compatibility with find_package
set(SofaPhysicsAPI_LIBRARIES SofaPhysicsAPI CACHE INTERNAL "")
set(SOFAPHYSICSAPI_INCLUDE_DIR "$<BUILD_INTERFACE:${SOFAPHYSICSAPI_BUILD_INCLUDE_DIR}>$<INSTALL_INTERFACE:${SOFAPHYSICSAPI_INSTALL_INCLUDE_DIR}>" CACHE INTERNAL "")
set(SofaPhysicsAPI_INCLUDE_DIRS ${SOFAPYTHON_INCLUDE_DIR} ${SofaGeneral_INCLUDE_DIRS} CACHE INTERNAL "")
