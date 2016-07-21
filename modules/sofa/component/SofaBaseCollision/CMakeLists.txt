include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initBaseCollision.h
    ../collision/BaseContactMapper.h
    ../collision/DefaultPipeline.h
    ../collision/Sphere.h
    ../collision/SphereModel.h
    ../collision/SphereModel.inl
    ../collision/Cube.h
    ../collision/CubeModel.h
    ../collision/DiscreteIntersection.h
    ../collision/BruteForceDetection.h
    ../collision/DefaultContactManager.h
    ../collision/MinProximityIntersection.h
    ../collision/NewProximityIntersection.h
    ../collision/NewProximityIntersection.inl
    ../collision/CapsuleModel.h
    ../collision/CapsuleModel.inl
    ../collision/BaseProximityIntersection.h
    ../collision/CapsuleIntTool.h
    ../collision/CapsuleIntTool.inl
    ../collision/OBBModel.inl
    ../collision/OBBModel.h
	../collision/RigidCapsuleModel.h
	../collision/RigidCapsuleModel.inl
	../collision/CylinderModel.h
	../collision/CylinderModel.inl
    ../collision/OBBIntTool.h
    ../collision/IntrOBBOBB.h
    ../collision/IntrOBBOBB.inl
    ../collision/IntrUtility3.h
    ../collision/IntrUtility3.inl
    ../collision/IntrSphereOBB.h
    ../collision/IntrCapsuleOBB.h
    ../collision/IntrCapsuleOBB.inl
    ../collision/IntrSphereOBB.inl
    ../collision/Intersector.h
    ../collision/BaseIntTool.h
    ../collision/ContactListener.h
    )

set(SOURCE_FILES

    ../initBaseCollision.cpp
    ../collision/BaseContactMapper.cpp
    ../collision/DefaultPipeline.cpp
    ../collision/SphereModel.cpp
    ../collision/CubeModel.cpp
    ../collision/CapsuleModel.cpp
    ../collision/DiscreteIntersection.cpp
    ../collision/BruteForceDetection.cpp
    ../collision/DefaultContactManager.cpp
    ../collision/MinProximityIntersection.cpp
    ../collision/NewProximityIntersection.cpp
    ../collision/BaseProximityIntersection.cpp
    ../collision/CapsuleIntTool.cpp
    ../collision/OBBModel.cpp
	../collision/RigidCapsuleModel.cpp
	../collision/CylinderModel.cpp
    ../collision/OBBIntTool.cpp
    ../collision/IntrOBBOBB.cpp
    ../collision/IntrUtility3.cpp
    ../collision/IntrCapsuleOBB.cpp
    ../collision/IntrSphereOBB.cpp
    ../collision/BaseIntTool.cpp
    ../collision/ContactListener.cpp
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

AddCompilerDefinitions("SOFA_BUILD_BASE_COLLISION")
AddLinkerDependencies(SofaBaseMechanics SofaRigid FlowVR SofaSphFluid)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
