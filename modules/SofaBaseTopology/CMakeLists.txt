cmake_minimum_required(VERSION 2.8.11)
project(SofaBaseTopology)


set(HEADER_FILES

    initBaseTopology.h
    CommonAlgorithms.h
    CubeTopology.h
    CylinderGridTopology.h
    SphereQuadTopology.h
    EdgeSetGeometryAlgorithms.h
    EdgeSetGeometryAlgorithms.inl
    EdgeSetTopologyAlgorithms.h
    EdgeSetTopologyAlgorithms.inl
    EdgeSetTopologyContainer.h
    EdgeSetTopologyModifier.h
    GridTopology.h
    HexahedronSetGeometryAlgorithms.h
    HexahedronSetGeometryAlgorithms.inl
    HexahedronSetTopologyAlgorithms.h
    HexahedronSetTopologyAlgorithms.inl
    HexahedronSetTopologyContainer.h
    HexahedronSetTopologyModifier.h
	NumericalIntegrationDescriptor.h
	NumericalIntegrationDescriptor.inl
    MeshTopology.h
    PointSetGeometryAlgorithms.h
    PointSetGeometryAlgorithms.inl
    PointSetTopologyAlgorithms.h
    PointSetTopologyAlgorithms.inl
    PointSetTopologyContainer.h
    PointSetTopologyModifier.h
    QuadSetGeometryAlgorithms.h
    QuadSetGeometryAlgorithms.inl
    QuadSetTopologyAlgorithms.h
    QuadSetTopologyAlgorithms.inl
    QuadSetTopologyContainer.h
    QuadSetTopologyModifier.h
    RegularGridTopology.h
    SparseGridTopology.h
    TetrahedronSetGeometryAlgorithms.h
    TetrahedronSetGeometryAlgorithms.inl
    BezierTetrahedronSetGeometryAlgorithms.h
    BezierTetrahedronSetGeometryAlgorithms.inl
    TetrahedronSetTopologyAlgorithms.h
    TetrahedronSetTopologyAlgorithms.inl
    TetrahedronSetTopologyContainer.h
    BezierTetrahedronSetTopologyContainer.h
    TetrahedronSetTopologyModifier.h
    TriangleSetGeometryAlgorithms.h
    TriangleSetGeometryAlgorithms.inl
    TriangleSetTopologyAlgorithms.h
    TriangleSetTopologyAlgorithms.inl
    TriangleSetTopologyContainer.h
    TriangleSetTopologyModifier.h
    TopologyData.h
    TopologyData.inl
    TopologyDataHandler.h
    TopologyDataHandler.inl
    TopologySparseDataHandler.h
    TopologySparseDataHandler.inl
    TopologySparseData.h
    TopologySparseData.inl
    TopologySubsetData.h
    TopologySubsetData.inl
    TopologySubsetDataHandler.h
    TopologySubsetDataHandler.inl
    TopologyEngine.h
    TopologyEngine.inl

    )

set( SOURCE_FILES_OUTSIDE_UNITY
    BezierTetrahedronSetTopologyContainer.cpp 
    HexahedronSetTopologyContainer.cpp
    TetrahedronSetGeometryAlgorithms.cpp
    TetrahedronSetTopologyContainer.cpp )
set(SOURCE_FILES_UNITY

    initBaseTopology.cpp
    CubeTopology.cpp
    CylinderGridTopology.cpp
    SphereQuadTopology.cpp
    EdgeSetGeometryAlgorithms.cpp
    EdgeSetTopologyAlgorithms.cpp
    EdgeSetTopologyContainer.cpp
    EdgeSetTopologyModifier.cpp
    GridTopology.cpp
    HexahedronSetGeometryAlgorithms.cpp
    HexahedronSetTopologyAlgorithms.cpp
    HexahedronSetTopologyModifier.cpp
    MeshTopology.cpp
	NumericalIntegrationDescriptor.cpp
    PointSetGeometryAlgorithms.cpp
    PointSetTopologyAlgorithms.cpp
    PointSetTopologyContainer.cpp
    PointSetTopologyModifier.cpp
    QuadSetGeometryAlgorithms.cpp
    QuadSetTopologyAlgorithms.cpp
    QuadSetTopologyContainer.cpp
    QuadSetTopologyModifier.cpp
    RegularGridTopology.cpp
    SparseGridTopology.cpp
    BezierTetrahedronSetGeometryAlgorithms.cpp
    TetrahedronSetTopologyAlgorithms.cpp
    TetrahedronSetTopologyModifier.cpp
    TriangleSetGeometryAlgorithms.cpp
    TriangleSetTopologyAlgorithms.cpp
    TriangleSetTopologyContainer.cpp
    TriangleSetTopologyModifier.cpp

    )



include( CMakeDependentOption )
cmake_dependent_option(UNITY_BUILD_${PROJECT_NAME} "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." ON  SOFA_ALLOW_UNITY_BUILDS OFF)

if( UNITY_BUILD_${PROJECT_NAME} )
  # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
  # source files.
  set( UNITYBUILD_INCLUDES "" )
  foreach(filename ${SOURCE_FILES_UNITY})
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
  set( SOURCE_FILES ${SOURCE_FILES_OUTSIDE_UNITY} UnityBuild.cpp )
else()
  set( SOURCE_FILES ${SOURCE_FILES_OUTSIDE_UNITY} ${SOURCE_FILES_UNITY} )
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaSimulationCommon ) 
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFABASE_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFABASE_DEPENDENCY_INCLUDE_DIR}>")

set( SOFABASETOPOLOGY_COMPILER_FLAGS "-DSOFA_BUILD_BASE_TOPOLOGY")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFABASE_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFABASETOPOLOGY_COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT  SofaBaseTargets
        RUNTIME DESTINATION "${SOFABASE_INSTALL_BIN_DIR}" COMPONENT SofaBase_libraries
        LIBRARY DESTINATION "${SOFABASE_INSTALL_LIB_DIR}" COMPONENT SofaBase_libraries
        ARCHIVE DESTINATION "${SOFABASE_INSTALL_LIB_DIR}" COMPONENT SofaBase_libraries
        PUBLIC_HEADER DESTINATION "${SOFABASE_INSTALL_INCLUDE_DIR}/SofaBaseTopology" COMPONENT SofaBase_headers
        )
        
if(TARGET gtest)
    add_subdirectory("SofaBaseTopology_test")
endif()
