include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initBaseTopology.h 
    ../topology/CommonAlgorithms.h 
    ../topology/CubeTopology.h 
    ../topology/CylinderGridTopology.h 
    ../topology/SphereQuadTopology.h 
    ../topology/EdgeSetGeometryAlgorithms.h 
    ../topology/EdgeSetGeometryAlgorithms.inl 
    ../topology/EdgeSetTopologyAlgorithms.h 
    ../topology/EdgeSetTopologyAlgorithms.inl 
    ../topology/EdgeSetTopologyContainer.h 
    ../topology/EdgeSetTopologyModifier.h 
    ../topology/GridTopology.h 
    ../topology/HexahedronSetGeometryAlgorithms.h 
    ../topology/HexahedronSetGeometryAlgorithms.inl 
    ../topology/HexahedronSetTopologyAlgorithms.h 
    ../topology/HexahedronSetTopologyAlgorithms.inl 
    ../topology/HexahedronSetTopologyContainer.h 
    ../topology/HexahedronSetTopologyModifier.h 
	../topology/NumericalIntegrationDescriptor.h 
	../topology/NumericalIntegrationDescriptor.inl 
    ../topology/MeshTopology.h 
    ../topology/PointSetGeometryAlgorithms.h 
    ../topology/PointSetGeometryAlgorithms.inl 
    ../topology/PointSetTopologyAlgorithms.h 
    ../topology/PointSetTopologyAlgorithms.inl 
    ../topology/PointSetTopologyContainer.h 
    ../topology/PointSetTopologyModifier.h 
    ../topology/QuadSetGeometryAlgorithms.h 
    ../topology/QuadSetGeometryAlgorithms.inl 
    ../topology/QuadSetTopologyAlgorithms.h 
    ../topology/QuadSetTopologyAlgorithms.inl 
    ../topology/QuadSetTopologyContainer.h 
    ../topology/QuadSetTopologyModifier.h 
    ../topology/RegularGridTopology.h 
    ../topology/SparseGridTopology.h 
    ../topology/TetrahedronSetGeometryAlgorithms.h 
    ../topology/TetrahedronSetGeometryAlgorithms.inl 
    ../topology/BezierTetrahedronSetGeometryAlgorithms.h 
    ../topology/BezierTetrahedronSetGeometryAlgorithms.inl 
    ../topology/TetrahedronSetTopologyAlgorithms.h 
    ../topology/TetrahedronSetTopologyAlgorithms.inl 
    ../topology/TetrahedronSetTopologyContainer.h 
    ../topology/BezierTetrahedronSetTopologyContainer.h 
    ../topology/TetrahedronSetTopologyModifier.h 
    ../topology/TriangleSetGeometryAlgorithms.h 
    ../topology/TriangleSetGeometryAlgorithms.inl 
    ../topology/TriangleSetTopologyAlgorithms.h 
    ../topology/TriangleSetTopologyAlgorithms.inl 
    ../topology/TriangleSetTopologyContainer.h 
    ../topology/TriangleSetTopologyModifier.h 
    ../topology/TopologyData.h 
    ../topology/TopologyData.inl 
    ../topology/TopologyDataHandler.h 
    ../topology/TopologyDataHandler.inl 
    ../topology/TopologySparseDataHandler.h 
    ../topology/TopologySparseDataHandler.inl 
    ../topology/TopologySparseData.h 
    ../topology/TopologySparseData.inl 
    ../topology/TopologySubsetData.h 
    ../topology/TopologySubsetData.inl 
    ../topology/TopologySubsetDataHandler.h 
    ../topology/TopologySubsetDataHandler.inl 
    ../topology/TopologyEngine.h 
    ../topology/TopologyEngine.inl

    )
    
set(SOURCE_FILES

    ../initBaseTopology.cpp 
    ../topology/CubeTopology.cpp 
    ../topology/CylinderGridTopology.cpp 
    ../topology/SphereQuadTopology.cpp 
    ../topology/EdgeSetGeometryAlgorithms.cpp 
    ../topology/EdgeSetTopologyAlgorithms.cpp 
    ../topology/EdgeSetTopologyContainer.cpp 
    ../topology/EdgeSetTopologyModifier.cpp 
    ../topology/GridTopology.cpp 
    ../topology/HexahedronSetGeometryAlgorithms.cpp 
    ../topology/HexahedronSetTopologyAlgorithms.cpp 
    ../topology/HexahedronSetTopologyContainer.cpp 
    ../topology/HexahedronSetTopologyModifier.cpp 
    ../topology/MeshTopology.cpp
	../topology/NumericalIntegrationDescriptor.cpp  
    ../topology/PointSetGeometryAlgorithms.cpp 
    ../topology/PointSetTopologyAlgorithms.cpp 
    ../topology/PointSetTopologyContainer.cpp 
    ../topology/PointSetTopologyModifier.cpp 
    ../topology/QuadSetGeometryAlgorithms.cpp 
    ../topology/QuadSetTopologyAlgorithms.cpp 
    ../topology/QuadSetTopologyContainer.cpp 
    ../topology/QuadSetTopologyModifier.cpp 
    ../topology/RegularGridTopology.cpp 
    ../topology/SparseGridTopology.cpp 
    ../topology/TetrahedronSetGeometryAlgorithms.cpp 
    ../topology/BezierTetrahedronSetGeometryAlgorithms.cpp 
    ../topology/TetrahedronSetTopologyAlgorithms.cpp 
    ../topology/TetrahedronSetTopologyContainer.cpp 
    ../topology/BezierTetrahedronSetTopologyContainer.cpp
    ../topology/TetrahedronSetTopologyModifier.cpp 
    ../topology/TriangleSetGeometryAlgorithms.cpp 
    ../topology/TriangleSetTopologyAlgorithms.cpp 
    ../topology/TriangleSetTopologyContainer.cpp 
    ../topology/TriangleSetTopologyModifier.cpp

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

AddCompilerDefinitions("SOFA_BUILD_BASE_TOPOLOGY")
AddLinkerDependencies(SofaSimulationTree)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
