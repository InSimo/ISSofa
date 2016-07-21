include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initUserInteraction.h 
    ../collision/RayTraceDetection.h 
    ../collision/RayContact.h 
    ../collision/DisabledContact.h 
    ../collision/ComponentMouseInteraction.h 
    ../collision/ComponentMouseInteraction.inl 
    ../collision/MouseInteractor.h 
    ../collision/MouseInteractor.inl 
    ../collision/AttachBodyPerformer.h 
    ../collision/AttachBodyPerformer.inl 
    ../collision/FixParticlePerformer.h 
    ../collision/FixParticlePerformer.inl 
    ../collision/InteractionPerformer.h 
    ../collision/SuturePointPerformer.h 
    ../collision/SuturePointPerformer.inl 
    ../collision/GraspingManager.h 
    ../controller/ArticulatedHierarchyController.h 
    ../controller/ArticulatedHierarchyBVHController.h 
    ../controller/Controller.h 
    ../controller/EdgeSetController.h 
    ../controller/EdgeSetController.inl 
    ../controller/MechanicalStateController.h 
    ../controller/MechanicalStateController.inl 
    ../collision/Ray.h 
    ../collision/RayModel.h 
    ../collision/RayDiscreteIntersection.h 
    ../collision/RayDiscreteIntersection.inl 
    ../collision/RayNewProximityIntersection.h 
    ../collision/RemovePrimitivePerformer.h 
    ../collision/RemovePrimitivePerformer.inl 
    ../collision/InciseAlongPathPerformer.h 
    ../collision/TopologicalChangeManager.h 
    ../collision/CuttingPoint.h 
#    ../collision/AddFramePerformer.h
#    ../collision/AddFramePerformer.inl
    ../collision/AddRecordedCameraPerformer.h 
    ../collision/StartNavigationPerformer.h
    ../controller/MechanicalStateControllerOmni.h 
	../controller/MechanicalStateControllerOmni.inl 
    ../controller/NodeToggleController.h
    ../controller/InterpolationController.h
    )
    
set(SOURCE_FILES

    ../initUserInteraction.cpp 
    ../collision/RayTraceDetection.cpp 
    ../collision/RayContact.cpp 
    ../collision/DisabledContact.cpp 
    ../collision/ComponentMouseInteraction.cpp 
    ../collision/MouseInteractor.cpp 
    ../collision/AttachBodyPerformer.cpp 
    ../collision/FixParticlePerformer.cpp 
    ../collision/InteractionPerformer.cpp 
    ../collision/SuturePointPerformer.cpp 
    ../collision/GraspingManager.cpp 
    ../controller/ArticulatedHierarchyController.cpp 
    ../controller/ArticulatedHierarchyBVHController.cpp 
    ../controller/Controller.cpp 
    ../controller/EdgeSetController.cpp 
    ../controller/MechanicalStateController.cpp 
    ../collision/RayModel.cpp 
    ../collision/RayDiscreteIntersection.cpp 
    ../collision/RayNewProximityIntersection.cpp 
    ../collision/RemovePrimitivePerformer.cpp 
    ../collision/InciseAlongPathPerformer.cpp 
    ../collision/TopologicalChangeManager.cpp 
    ../collision/CuttingPoint.cpp 
#    ../collision/AddFramePerformer.cpp
    ../collision/AddRecordedCameraPerformer.cpp
    ../collision/StartNavigationPerformer.cpp
    ../controller/MechanicalStateControllerOmni.cpp 
    ../controller/NodeToggleController.cpp
    ../controller/InterpolationController.cpp
    ../controller/InterpolationController.inl
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

AddCompilerDefinitions("SOFA_BUILD_USER_INTERACTION" )
AddLinkerDependencies(SofaMeshCollision SofaTopologyMapping SofaDeformable SofaBoundaryCondition SofaGraphComponent SofaBaseVisual )
    
include(${SOFA_CMAKE_DIR}/postProject.cmake)
