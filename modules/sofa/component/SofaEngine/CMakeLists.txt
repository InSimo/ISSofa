include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initEngine.h 
    ../engine/AverageCoord.h 
    ../engine/AverageCoord.inl 
    ../engine/BoxROI.h 
    ../engine/BoxROI.inl 
    ../engine/PairBoxRoi.h 
    ../engine/PairBoxRoi.inl
    ../engine/PlaneROI.h 
    ../engine/PlaneROI.inl 
    ../engine/SphereROI.h 
    ../engine/SphereROI.inl 
    ../engine/DilateEngine.h 
    ../engine/DilateEngine.inl
    ../engine/GenerateCylinder.h 
    ../engine/GenerateCylinder.inl  
    ../engine/ExtrudeSurface.h 
    ../engine/ExtrudeSurface.inl 
    ../engine/ExtrudeQuadsAndGenerateHexas.h 
    ../engine/ExtrudeQuadsAndGenerateHexas.inl 
    ../engine/ExtrudeEdgesAndGenerateQuads.h 
    ../engine/ExtrudeEdgesAndGenerateQuads.inl 
    ../engine/GenerateRigidMass.h 
    ../engine/GenerateRigidMass.inl 
    ../engine/GroupFilterYoungModulus.h 
    ../engine/GroupFilterYoungModulus.inl 
    ../engine/MathOp.h 
    ../engine/MathOp.inl 
    ../engine/MergeMeshes.h 
    ../engine/MergeMeshes.inl 
    ../engine/MergePoints.h 
    ../engine/MergePoints.inl 
    ../engine/MergeSets.h 
    ../engine/MergeSets.inl 
    ../engine/MergeVectors.h 
    ../engine/MergeVectors.inl 
    ../engine/MeshBarycentricMapperEngine.h 
    ../engine/MeshBarycentricMapperEngine.inl 
    ../engine/MeshROI.h 
    ../engine/MeshROI.inl 
    ../engine/TestEngine.h
    ../engine/TransformPosition.h 
    ../engine/TransformPosition.inl 
    ../engine/TransformEngine.h 
    ../engine/TransformEngine.inl
    ../engine/TransformMatrixEngine.h
    ../engine/PointsFromIndices.h 
    ../engine/PointsFromIndices.inl 
    ../engine/ValuesFromIndices.h 
    ../engine/ValuesFromIndices.inl 
    ../engine/IndicesFromValues.h 
    ../engine/IndicesFromValues.inl 
    ../engine/IndexValueMapper.h 
    ../engine/IndexValueMapper.inl 
    ../engine/JoinPoints.h 
    ../engine/JoinPoints.inl 
    ../engine/MapIndices.h 
    ../engine/MapIndices.inl 
    ../engine/RandomPointDistributionInSurface.h 
    ../engine/RandomPointDistributionInSurface.inl 
    ../engine/SmoothMeshEngine.h
    ../engine/SmoothMeshEngine.inl
    ../engine/Spiral.h 
    ../engine/Spiral.inl 
    ../engine/Vertex2Frame.h 
    ../engine/Vertex2Frame.inl 
    ../engine/TextureInterpolation.h 
    ../engine/TextureInterpolation.inl 
    ../engine/SubsetTopology.h 
    ../engine/SubsetTopology.inl 
    ../engine/RigidToQuatEngine.h 
    ../engine/RigidToQuatEngine.inl 
    ../engine/QuatToRigidEngine.h 
    ../engine/QuatToRigidEngine.inl 
    ../engine/ValuesFromPositions.h 
    ../engine/ValuesFromPositions.inl 
    ../engine/NormalsFromPoints.h 
    ../engine/NormalsFromPoints.inl 
    ../engine/ClusteringEngine.h 
    ../engine/ClusteringEngine.inl 
    ../engine/ShapeMatching.h 
    ../engine/ShapeMatching.inl  
    ../engine/ProximityROI.h 
    ../engine/ProximityROI.inl
    ../engine/HausdorffDistance.h
    ../engine/HausdorffDistance.inl
    ../engine/MeshSampler.h
    ../engine/MeshSampler.inl
    ../engine/NormEngine.h
    ../engine/NormEngine.inl
    )
    
set(SOURCE_FILES

    ../initEngine.cpp 
    ../engine/AverageCoord.cpp 
    ../engine/BoxROI.cpp 
    ../engine/PairBoxRoi.cpp 
    ../engine/PlaneROI.cpp 
    ../engine/SphereROI.cpp 
    ../engine/DilateEngine.cpp 
    ../engine/GenerateCylinder.cpp 
    ../engine/ExtrudeSurface.cpp 
    ../engine/ExtrudeQuadsAndGenerateHexas.cpp 
    ../engine/ExtrudeEdgesAndGenerateQuads.cpp 
    ../engine/GenerateRigidMass.cpp 
    ../engine/GroupFilterYoungModulus.cpp 
    ../engine/MathOp.cpp 
    ../engine/MergeMeshes.cpp 
    ../engine/MergePoints.cpp 
    ../engine/MergeSets.cpp 
    ../engine/MergeVectors.cpp
    ../engine/MeshBarycentricMapperEngine.cpp 
    ../engine/MeshROI.cpp 
    ../engine/TestEngine.cpp
    ../engine/TransformPosition.cpp 
    ../engine/TransformEngine.cpp 
    ../engine/TransformMatrixEngine.cpp
    ../engine/PointsFromIndices.cpp 
    ../engine/ValuesFromIndices.cpp 
    ../engine/IndicesFromValues.cpp 
    ../engine/IndexValueMapper.cpp 
    ../engine/JoinPoints.cpp 
    ../engine/MapIndices.cpp 
    ../engine/RandomPointDistributionInSurface.cpp 
    ../engine/SmoothMeshEngine.cpp
    ../engine/Spiral.cpp 
    ../engine/Vertex2Frame.cpp 
    ../engine/TextureInterpolation.cpp 
    ../engine/SubsetTopology.cpp 
    ../engine/RigidToQuatEngine.cpp 
    ../engine/QuatToRigidEngine.cpp 
    ../engine/ValuesFromPositions.cpp 
    ../engine/NormalsFromPoints.cpp 
    ../engine/ClusteringEngine.cpp 
    ../engine/ShapeMatching.cpp 
    ../engine/ProximityROI.cpp
    ../engine/HausdorffDistance.cpp
    ../engine/MeshSampler.cpp
    ../engine/NormEngine.cpp
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

AddCompilerDefinitions("SOFA_BUILD_ENGINE")
AddLinkerDependencies(SofaMeshCollision)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
