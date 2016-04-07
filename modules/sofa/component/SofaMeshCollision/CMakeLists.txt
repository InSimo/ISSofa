include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initMeshCollision.h
    ../collision/MeshNewProximityIntersection.h
    ../collision/MeshNewProximityIntersection.inl
    ../collision/BarycentricPenalityContact.h
    ../collision/BarycentricPenalityContact.inl
    ../collision/BarycentricContactMapper.h
    ../collision/BarycentricContactMapper.inl
    ../collision/IdentityContactMapper.h
    ../collision/IdentityContactMapper.inl
    ../collision/RigidContactMapper.h
    ../collision/RigidContactMapper.inl
    ../collision/SubsetContactMapper.h
    ../collision/SubsetContactMapper.inl
    ../collision/MeshDiscreteIntersection.h
    ../collision/MeshDiscreteIntersection.inl
    ../collision/MeshMinProximityIntersection.h
    ../collision/Point.h
    ../collision/PointModel.h
    ../collision/PointModel.inl
    ../collision/Line.h
    ../collision/LineModel.h
    ../collision/LineModel.inl
    ../collision/Triangle.h
    ../collision/TriangleModel.h
    ../collision/TriangleModel.inl
    ../collision/TriangleOctree.h
    ../collision/TriangleOctreeModel.h
    ../collision/RayTriangleIntersection.h
    ../collision/PointLocalMinDistanceFilter.h
    ../collision/LineLocalMinDistanceFilter.h
    ../collision/TriangleLocalMinDistanceFilter.h
    ../collision/LocalMinDistanceFilter.h
    ../collision/MeshIntTool.h
    ../collision/MeshIntTool.inl
    ../collision/IntrTriangleOBB.h
    ../collision/IntrTriangleOBB.inl
    ../collision/IntrMeshUtility.h
    ../collision/IntrMeshUtility.inl
    ../collision/IncrSAP.h
    ../collision/IncrSAP.inl
    ../collision/DirectSAP.h
    ../collision/DirectSAP.inl
    )

set(SOURCE_FILES

    ../initMeshCollision.cpp
    ../collision/MeshNewProximityIntersection.cpp
    ../collision/BarycentricPenalityContact.cpp
    ../collision/BarycentricContactMapper.cpp
    ../collision/IdentityContactMapper.cpp
    ../collision/SubsetContactMapper.cpp
    ../collision/MeshDiscreteIntersection.cpp
    ../collision/MeshMinProximityIntersection.cpp
    ../collision/PointModel.cpp
    ../collision/LineModel.cpp
    ../collision/TriangleModel.cpp
    ../collision/TriangleOctree.cpp
    ../collision/TriangleOctreeModel.cpp
    ../collision/RayTriangleIntersection.cpp
    ../collision/PointLocalMinDistanceFilter.cpp
    ../collision/LineLocalMinDistanceFilter.cpp
    ../collision/TriangleLocalMinDistanceFilter.cpp
    ../collision/LocalMinDistanceFilter.cpp
    ../collision/MeshIntTool.cpp
    ../collision/RigidContactMapper.cpp
    ../collision/IntrTriangleOBB.cpp
    ../collision/IntrMeshUtility.cpp
    ../collision/IncrSAP.cpp
    ../collision/DirectSAP.cpp
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

AddCompilerDefinitions("SOFA_BUILD_MESH_COLLISION")
AddLinkerDependencies(SofaObjectInteraction SofaBaseCollision)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
