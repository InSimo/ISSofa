cmake_minimum_required(VERSION 2.8)
project(SofaEngine)

set(HEADER_FILES

    initEngine.h
    AverageCoord.h
    AverageCoord.inl
    BoxROI.h
    BoxROI.inl
    PairBoxRoi.h
    PairBoxRoi.inl
    PlaneROI.h
    PlaneROI.inl
    SphereROI.h
    SphereROI.inl
    DilateEngine.h
    DilateEngine.inl
    GenerateCylinder.h
    GenerateCylinder.inl
    ExtrudeSurface.h
    ExtrudeSurface.inl
    ExtrudeQuadsAndGenerateHexas.h
    ExtrudeQuadsAndGenerateHexas.inl
    ExtrudeEdgesAndGenerateQuads.h
    ExtrudeEdgesAndGenerateQuads.inl
    GenerateRigidMass.h
    GenerateRigidMass.inl
    GroupFilterYoungModulus.h
    GroupFilterYoungModulus.inl
    MathOp.h
    MathOp.inl
    MergeMeshes.h
    MergeMeshes.inl
    MergePoints.h
    MergePoints.inl
    MergeSets.h
    MergeSets.inl
    MergeVectors.h
    MergeVectors.inl
    MeshBarycentricMapperEngine.h
    MeshBarycentricMapperEngine.inl
    MeshROI.h
    MeshROI.inl
    TransformPosition.h
    TransformPosition.inl
    TransformEngine.h
    TransformEngine.inl
    TransformMatrixEngine.h
    PointsFromIndices.h
    PointsFromIndices.inl
    ValuesFromIndices.h
    ValuesFromIndices.inl
    IndicesFromValues.h
    IndicesFromValues.inl
    IndexValueMapper.h
    IndexValueMapper.inl
    JoinPoints.h
    JoinPoints.inl
    MapIndices.h
    MapIndices.inl
    RandomPointDistributionInSurface.h
    RandomPointDistributionInSurface.inl
    SmoothMeshEngine.h
    SmoothMeshEngine.inl
    Spiral.h
    Spiral.inl
    Vertex2Frame.h
    Vertex2Frame.inl
    TextureInterpolation.h
    TextureInterpolation.inl
    SubsetTopology.h
    SubsetTopology.inl
    RigidToQuatEngine.h
    RigidToQuatEngine.inl
    QuatToRigidEngine.h
    QuatToRigidEngine.inl
    ValuesFromPositions.h
    ValuesFromPositions.inl
    NormalsFromPoints.h
    NormalsFromPoints.inl
    ClusteringEngine.h
    ClusteringEngine.inl
    ShapeMatching.h
    ShapeMatching.inl
    ProximityROI.h
    ProximityROI.inl
    HausdorffDistance.h
    HausdorffDistance.inl
    NormEngine.h
    NormEngine.inl
    TestEngine.h
    )

set(SOURCE_FILES

    initEngine.cpp
    AverageCoord.cpp
    BoxROI.cpp
    PairBoxRoi.cpp
    PlaneROI.cpp
    SphereROI.cpp
    DilateEngine.cpp
    GenerateCylinder.cpp
    ExtrudeSurface.cpp
    ExtrudeQuadsAndGenerateHexas.cpp
    ExtrudeEdgesAndGenerateQuads.cpp
    GenerateRigidMass.cpp
    GroupFilterYoungModulus.cpp
    MathOp.cpp
    MergeMeshes.cpp
    MergePoints.cpp
    MergeSets.cpp
    MergeVectors.cpp
    MeshBarycentricMapperEngine.cpp
    MeshROI.cpp
    TransformPosition.cpp
    TransformEngine.cpp
    TransformMatrixEngine.cpp
    PointsFromIndices.cpp
    ValuesFromIndices.cpp
    IndicesFromValues.cpp
    IndexValueMapper.cpp
    JoinPoints.cpp
    MapIndices.cpp
    RandomPointDistributionInSurface.cpp
    SmoothMeshEngine.cpp
    Spiral.cpp
    Vertex2Frame.cpp
    TextureInterpolation.cpp
    SubsetTopology.cpp
    RigidToQuatEngine.cpp
    QuatToRigidEngine.cpp
    ValuesFromPositions.cpp
    NormalsFromPoints.cpp
    ClusteringEngine.cpp
    ShapeMatching.cpp
    ProximityROI.cpp
    HausdorffDistance.cpp
    NormEngine.cpp
    TestEngine.cpp
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
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaMeshCollision )
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_DEPENDENCY_INCLUDE_DIR}>")

set( SOFAENGINE_COMPILER_FLAGS "-DSOFA_BUILD_ENGINE")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFAGENERAL_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFAENGINE_COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT    SofaGeneralTargets
        RUNTIME DESTINATION "${SOFAGENERAL_INSTALL_BIN_DIR}" COMPONENT SofaGeneral_libraries
        LIBRARY DESTINATION "${SOFAGENERAL_INSTALL_LIB_DIR}" COMPONENT SofaGeneral_libraries
        ARCHIVE DESTINATION "${SOFAGENERAL_INSTALL_LIB_DIR}" COMPONENT SofaGeneral_libraries
        PUBLIC_HEADER DESTINATION "${SOFAGENERAL_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT SofaGeneral_headers
)

if(TARGET gtest)
    add_subdirectory("SofaEngine_test")
endif()
