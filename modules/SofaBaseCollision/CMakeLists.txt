cmake_minimum_required(VERSION 2.8.11)
project(SofaBaseCollision)

set(HEADER_FILES

    initBaseCollision.h
    BaseContactMapper.h
    DefaultPipeline.h
    DefaultPipelineMT.h
    Sphere.h
    SphereModel.h
    SphereModel.inl
    CollisionDetectionTasks.h
    ComputeIntersectionTasks.h
    Cube.h
    CubeModel.h
    DiscreteIntersection.h
    BruteForceDetection.h
    DefaultContactManager.h
    FilteredBruteForceDetection.h
    FilteredBruteForceDetectionMT.h
    MinProximityIntersection.h
    NewProximityIntersection.h
    NewProximityIntersection.inl
    CapsuleModel.h
    CapsuleModel.inl
    BaseProximityIntersection.h
    CapsuleIntTool.h
    CapsuleIntTool.inl
    OBBModel.inl
    OBBModel.h
	RigidCapsuleModel.h
	RigidCapsuleModel.inl
	CylinderModel.h
	CylinderModel.inl
    OBBIntTool.h
    IntrOBBOBB.h
    IntrOBBOBB.inl
    IntrUtility3.h
    IntrUtility3.inl
    IntrSphereOBB.h
    IntrCapsuleOBB.h
    IntrCapsuleOBB.inl
    IntrSphereOBB.inl
    Intersector.h
    BaseIntTool.h
    ContactListener.h
    )

set(SOURCE_FILES

    initBaseCollision.cpp
    BaseContactMapper.cpp
    DefaultPipeline.cpp
    DefaultPipelineMT.cpp
    SphereModel.cpp
    CollisionDetectionTasks.cpp
    ComputeIntersectionTasks.cpp
    CubeModel.cpp
    CapsuleModel.cpp
    DiscreteIntersection.cpp
    FilteredBruteForceDetection.cpp
    FilteredBruteForceDetectionMT.cpp
    BruteForceDetection.cpp
    DefaultContactManager.cpp
    MinProximityIntersection.cpp
    NewProximityIntersection.cpp
    BaseProximityIntersection.cpp
    CapsuleIntTool.cpp
    OBBModel.cpp
	RigidCapsuleModel.cpp
	CylinderModel.cpp
    OBBIntTool.cpp
    IntrOBBOBB.cpp
    IntrUtility3.cpp
    IntrCapsuleOBB.cpp
    IntrSphereOBB.cpp
    BaseIntTool.cpp
    ContactListener.cpp
    )



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
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaSimulationCommon)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFABASE_SOURCE_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFABASE_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFABASE_DEPENDENCY_INCLUDE_DIR}>")

set( COMPILER_FLAGS "-DSOFA_BUILD_BASE_COLLISION")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFABASE_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT  SofaBaseTargets
        RUNTIME DESTINATION "${SOFABASE_INSTALL_BIN_DIR}" COMPONENT SofaBase_libraries
        LIBRARY DESTINATION "${SOFABASE_INSTALL_LIB_DIR}" COMPONENT SofaBase_libraries
        ARCHIVE DESTINATION "${SOFABASE_INSTALL_LIB_DIR}" COMPONENT SofaBase_libraries
        PUBLIC_HEADER DESTINATION "${SOFABASE_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT SofaBase_headers
        )
