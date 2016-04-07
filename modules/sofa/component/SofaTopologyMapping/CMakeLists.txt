include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initTopologyMapping.h 
    ../mapping/Mesh2PointMechanicalMapping.h 
    ../mapping/Mesh2PointMechanicalMapping.inl 
    ../mapping/SimpleTesselatedTetraMechanicalMapping.h 
    ../mapping/SimpleTesselatedTetraMechanicalMapping.inl 
    ../topology/CenterPointTopologicalMapping.h 
    ../topology/Edge2QuadTopologicalMapping.h 
    ../topology/Hexa2QuadTopologicalMapping.h 
    ../topology/Hexa2TetraTopologicalMapping.h 
    ../topology/Mesh2PointTopologicalMapping.h  
    ../topology/Mesh2BezierTopologicalMapping.h  
    ../topology/Quad2TriangleTopologicalMapping.h 
    ../topology/SimpleTesselatedHexaTopologicalMapping.h 
    ../topology/SimpleTesselatedTetraTopologicalMapping.h 
    ../topology/Tetra2TriangleTopologicalMapping.h 
    ../topology/Triangle2EdgeTopologicalMapping.h 
    ../topology/IdentityTopologicalMapping.h 
    ../topology/SubsetTopologicalMapping.h
           
    )
    
set(SOURCE_FILES

    ../initTopologyMapping.cpp 
    ../mapping/Mesh2PointMechanicalMapping.cpp 
    ../mapping/SimpleTesselatedTetraMechanicalMapping.cpp 
    ../topology/CenterPointTopologicalMapping.cpp 
    ../topology/Edge2QuadTopologicalMapping.cpp 
    ../topology/Hexa2QuadTopologicalMapping.cpp 
    ../topology/Hexa2TetraTopologicalMapping.cpp 
    ../topology/Mesh2PointTopologicalMapping.cpp  
    ../topology/Mesh2BezierTopologicalMapping.cpp  
    ../topology/Quad2TriangleTopologicalMapping.cpp 
    ../topology/SimpleTesselatedHexaTopologicalMapping.cpp 
    ../topology/SimpleTesselatedTetraTopologicalMapping.cpp 
    ../topology/Tetra2TriangleTopologicalMapping.cpp 
    ../topology/Triangle2EdgeTopologicalMapping.cpp 
    ../topology/IdentityTopologicalMapping.cpp 
    ../topology/SubsetTopologicalMapping.cpp

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

  file(WRITE UnityBuild.cpp ${UNITYBUILD_INCLUDES} )

  # overwrite the source files to only include the generated unity build file.
  set( SOURCE_FILES UnityBuild.cpp )
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

AddCompilerDefinitions("SOFA_BUILD_TOPOLOGY_MAPPING")
AddLinkerDependencies(SofaBaseTopology)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
