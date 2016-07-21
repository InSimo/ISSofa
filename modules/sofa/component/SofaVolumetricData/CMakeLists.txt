include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initVolumetricData.h 
    ../container/ImplicitSurfaceContainer.h 
    ../container/InterpolatedImplicitSurface.h 
    ../container/InterpolatedImplicitSurface.inl 
    ../forcefield/DistanceGridForceField.h 
    ../forcefield/DistanceGridForceField.inl 
    ../mapping/ImplicitSurfaceMapping.h 
    ../mapping/ImplicitSurfaceMapping.inl 
    ../container/DistanceGrid.h 
    ../collision/DistanceGridCollisionModel.h 
    ../collision/RigidDistanceGridDiscreteIntersection.h 
    ../collision/RigidDistanceGridDiscreteIntersection.inl 
    ../collision/FFDDistanceGridDiscreteIntersection.h 
    ../collision/FFDDistanceGridDiscreteIntersection.inl 

    )
    
set(SOURCE_FILES

    ../initVolumetricData.cpp 
    ../container/ImplicitSurfaceContainer.cpp 
    ../container/InterpolatedImplicitSurface.cpp 
    ../forcefield/DistanceGridForceField.cpp 
    ../mapping/ImplicitSurfaceMapping.cpp 
    ../container/DistanceGrid.cpp 
    ../collision/DistanceGridCollisionModel.cpp 
    ../collision/RayDistanceGridContact.cpp 
    ../collision/RigidDistanceGridDiscreteIntersection.cpp 
    ../collision/FFDDistanceGridDiscreteIntersection.cpp 
    ../collision/BarycentricPenalityContact_DistanceGrid.cpp 
 
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

AddCompilerDefinitions("SOFA_BUILD_VOLUMETRIC_DATA" )
AddLinkerDependencies(SofaBaseCollision SofaMeshCollision SofaUserInteraction FlowVR)
        
include(${SOFA_CMAKE_DIR}/postProject.cmake)
