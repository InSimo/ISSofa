include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initNonUniformFEM.h 
    ../forcefield/NonUniformHexahedralFEMForceFieldAndMass.h 
    ../forcefield/NonUniformHexahedralFEMForceFieldAndMass.inl 
    ../forcefield/NonUniformHexahedronFEMForceFieldAndMass.h 
    ../forcefield/NonUniformHexahedronFEMForceFieldAndMass.inl 
#    ../forcefield/NonUniformHexahedronFEMForceFieldDensity.h 
#    ../forcefield/NonUniformHexahedronFEMForceFieldDensity.inl 
    ../topology/DynamicSparseGridGeometryAlgorithms.h 
    ../topology/DynamicSparseGridGeometryAlgorithms.inl 
    ../topology/DynamicSparseGridTopologyAlgorithms.h 
    ../topology/DynamicSparseGridTopologyAlgorithms.inl 
    ../topology/DynamicSparseGridTopologyContainer.h 
    ../topology/DynamicSparseGridTopologyModifier.h 
    ../topology/MultilevelHexahedronSetTopologyContainer.h 
    ../topology/SparseGridMultipleTopology.h 
    ../topology/SparseGridRamificationTopology.h 
    ../forcefield/HexahedronCompositeFEMForceFieldAndMass.h 
    ../forcefield/HexahedronCompositeFEMForceFieldAndMass.inl 
    ../mapping/HexahedronCompositeFEMMapping.h 
    ../mapping/HexahedronCompositeFEMMapping.inl
    )
    
set(SOURCE_FILES

    ../initNonUniformFEM.cpp 
    ../forcefield/NonUniformHexahedralFEMForceFieldAndMass.cpp 
    ../forcefield/NonUniformHexahedronFEMForceFieldAndMass.cpp 
#    ../forcefield/NonUniformHexahedronFEMForceFieldDensity.cpp 
    ../topology/MultilevelHexahedronSetTopologyContainer.cpp 
    ../topology/DynamicSparseGridGeometryAlgorithms.cpp 
    ../topology/DynamicSparseGridTopologyAlgorithms.cpp 
    ../topology/DynamicSparseGridTopologyContainer.cpp 
    ../topology/DynamicSparseGridTopologyModifier.cpp 
    ../topology/SparseGridMultipleTopology.cpp 
    ../topology/SparseGridRamificationTopology.cpp 
    ../forcefield/HexahedronCompositeFEMForceFieldAndMass.cpp 
    ../mapping/HexahedronCompositeFEMMapping.cpp
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

AddCompilerDefinitions("SOFA_BUILD_NON_UNIFORM_FEM" )
AddLinkerDependencies(SofaSimpleFem ${SOFA_VISUAL_LIB} SofaDenseSolver SofaVolumetricData )
        
include(${SOFA_CMAKE_DIR}/postProject.cmake)
