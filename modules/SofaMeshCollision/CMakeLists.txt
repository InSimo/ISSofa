cmake_minimum_required(VERSION 2.8.12)
project(SofaMeshCollision)

set(HEADER_FILES

    initMeshCollision.h
    MeshNewProximityIntersection.h
    MeshNewProximityIntersection.inl
    BarycentricPenalityContact.h
    BarycentricPenalityContact.inl
    BarycentricContactMapper.h
    BarycentricContactMapper.inl
    IdentityContactMapper.h
    IdentityContactMapper.inl
    RigidContactMapper.h
    RigidContactMapper.inl
    SubsetContactMapper.h
    SubsetContactMapper.inl
    MeshDiscreteIntersection.h
    MeshDiscreteIntersection.inl
    MeshMinProximityIntersection.h
    GenericPointIterator.h
    Point.h
    PointModel.h
    PointModel.inl
    GenericLineIterator.h
    Line.h
    LineModel.h
    LineModel.inl
    GenericLineModel.h
    GenericTriangleModel.h
    GenericTriangleIterator.h
    Triangle.h
    TriangleModel.h
    TriangleModel.inl
    TriangleOctree.h
    TriangleOctreeModel.h
    RayTriangleIntersection.h
    PointLocalMinDistanceFilter.h
    LineLocalMinDistanceFilter.h
    TriangleLocalMinDistanceFilter.h
    LocalMinDistanceFilter.h
    MeshIntTool.h
    MeshIntTool.inl
    IntrTriangleOBB.h
    IntrTriangleOBB.inl
    IntrMeshUtility.h
    IntrMeshUtility.inl
    IncrSAP.h
    IncrSAP.inl
    DirectSAP.h
    DirectSAP.inl
    )

set(SOURCE_FILES

    initMeshCollision.cpp
    MeshNewProximityIntersection.cpp
    BarycentricPenalityContact.cpp
    BarycentricContactMapper.cpp
    IdentityContactMapper.cpp
    SubsetContactMapper.cpp
    MeshDiscreteIntersection.cpp
    MeshMinProximityIntersection.cpp
    PointModel.cpp
    LineModel.cpp
    GenericLineModel.cpp
    GenericTriangleModel.cpp
    TriangleModel.cpp
    TriangleOctree.cpp
    TriangleOctreeModel.cpp
    RayTriangleIntersection.cpp
    PointLocalMinDistanceFilter.cpp
    LineLocalMinDistanceFilter.cpp
    TriangleLocalMinDistanceFilter.cpp
    LocalMinDistanceFilter.cpp
    MeshIntTool.cpp
    RigidContactMapper.cpp
    IntrTriangleOBB.cpp
    IntrMeshUtility.cpp
    IncrSAP.cpp
    DirectSAP.cpp
    )

set(GROUP_BASE_DIR "..")



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

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries( ${PROJECT_NAME} PUBLIC SofaObjectInteraction SofaRigid SofaBaseCollision)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFACOMMON_SOURCE_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFACOMMON_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFACOMMON_DEPENDENCY_INCLUDE_DIR}>")

set( SOFAMESHCOLLISION_COMPILER_FLAGS "-DSOFA_BUILD_MESH_COLLISION")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFACOMMON_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFAMESHCOLLISION_COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT  SofaCommonTargets
        RUNTIME DESTINATION "${SOFACOMMON_INSTALL_BIN_DIR}" COMPONENT SofaCommon_libraries
        LIBRARY DESTINATION "${SOFACOMMON_INSTALL_LIB_DIR}" COMPONENT SofaCommon_libraries
        ARCHIVE DESTINATION "${SOFACOMMON_INSTALL_LIB_DIR}" COMPONENT SofaCommon_libraries
        PUBLIC_HEADER DESTINATION "${SOFACOMMON_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT SofaCommon_headers
        )
