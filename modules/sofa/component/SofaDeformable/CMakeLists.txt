include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initDeformable.h 
    ../forcefield/FastTriangularBendingSprings.h 
    ../forcefield/FastTriangularBendingSprings.inl 
    ../forcefield/QuadularBendingSprings.h 
    ../forcefield/QuadularBendingSprings.inl 
    ../forcefield/RestShapeSpringsForceField.h 
    ../forcefield/RestShapeSpringsForceField.inl 
    ../forcefield/TriangularBendingSprings.h 
    ../forcefield/TriangularBendingSprings.inl 
    ../forcefield/TriangularBiquadraticSpringsForceField.h 
    ../forcefield/TriangularBiquadraticSpringsForceField.inl 
    ../forcefield/TriangularQuadraticSpringsForceField.h 
    ../forcefield/TriangularQuadraticSpringsForceField.inl 
    ../forcefield/TriangularTensorMassForceField.h 
    ../forcefield/TriangularTensorMassForceField.inl 
    ../interactionforcefield/FrameSpringForceField.h 
    ../interactionforcefield/FrameSpringForceField.inl 
    ../interactionforcefield/MeshSpringForceField.h 
    ../interactionforcefield/MeshSpringForceField.inl 
    ../interactionforcefield/QuadBendingSprings.h 
    ../interactionforcefield/QuadBendingSprings.inl 
    ../interactionforcefield/RegularGridSpringForceField.h 
    ../interactionforcefield/RegularGridSpringForceField.inl 
    ../interactionforcefield/SpringForceField.h 
    ../interactionforcefield/SpringForceField.inl 
    ../interactionforcefield/StiffSpringForceField.h 
    ../interactionforcefield/StiffSpringForceField.inl 
    ../interactionforcefield/TriangleBendingSprings.h 
    ../interactionforcefield/TriangleBendingSprings.inl 
    ../interactionforcefield/VectorSpringForceField.h 
    ../interactionforcefield/VectorSpringForceField.inl

    )
    
set(SOURCE_FILES

    ../initDeformable.cpp 
    ../forcefield/FastTriangularBendingSprings.cpp 
    ../forcefield/QuadularBendingSprings.cpp 
    ../forcefield/RestShapeSpringsForceField.cpp 
    ../forcefield/TriangularBendingSprings.cpp 
    ../forcefield/TriangularBiquadraticSpringsForceField.cpp 
    ../forcefield/TriangularQuadraticSpringsForceField.cpp 
    ../forcefield/TriangularTensorMassForceField.cpp 
    ../interactionforcefield/FrameSpringForceField.cpp 
    ../interactionforcefield/MeshSpringForceField.cpp 
    ../interactionforcefield/QuadBendingSprings.cpp 
    ../interactionforcefield/RegularGridSpringForceField.cpp 
    ../interactionforcefield/SpringForceField.cpp 
    ../interactionforcefield/StiffSpringForceField.cpp 
    ../interactionforcefield/TriangleBendingSprings.cpp 
    ../interactionforcefield/VectorSpringForceField.cpp

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

AddCompilerDefinitions("SOFA_BUILD_DEFORMABLE")
AddSourceDependencies(eigen)
AddLinkerDependencies(SofaBaseTopology)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
