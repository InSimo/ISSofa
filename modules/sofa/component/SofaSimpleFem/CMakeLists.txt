include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initSimpleFEM.h 
    ../forcefield/BeamFEMForceField.h 
    ../forcefield/BeamFEMForceField.inl 
    ../forcefield/HexahedralFEMForceField.h 
    ../forcefield/HexahedralFEMForceField.inl 
    ../forcefield/HexahedralFEMForceFieldAndMass.h 
    ../forcefield/HexahedralFEMForceFieldAndMass.inl 
    ../forcefield/HexahedronFEMForceField.h 
    ../forcefield/HexahedronFEMForceField.inl 
    ../forcefield/HexahedronFEMForceFieldAndMass.h 
    ../forcefield/HexahedronFEMForceFieldAndMass.inl 
    ../forcefield/TetrahedralCorotationalFEMForceField.h 
    ../forcefield/TetrahedralCorotationalFEMForceField.inl 
    ../forcefield/TetrahedronFEMForceField.h 
    ../forcefield/TetrahedronFEMForceField.inl 
    ../forcefield/TriangularAnisotropicFEMForceField.h 
    ../forcefield/TriangularAnisotropicFEMForceField.inl 
    ../forcefield/TriangleFEMForceField.h 
    ../forcefield/TriangleFEMForceField.inl 
    ../forcefield/TriangularFEMForceField.h 
    ../forcefield/TriangularFEMForceField.inl 
    ../forcefield/TriangularFEMForceFieldOptim.h 
    ../forcefield/TriangularFEMForceFieldOptim.inl 
    ../container/PoissonContainer.h 
    ../container/StiffnessContainer.h 
    ../container/RadiusContainer.h 
    ../container/LengthContainer.h 
    )
    
set(SOURCE_FILES

    ../initSimpleFEM.cpp 
    ../forcefield/BeamFEMForceField.cpp 
    ../forcefield/HexahedralFEMForceField.cpp 
    ../forcefield/HexahedralFEMForceFieldAndMass.cpp 
    ../forcefield/HexahedronFEMForceField.cpp 
    ../forcefield/HexahedronFEMForceFieldAndMass.cpp 
    ../forcefield/TetrahedralCorotationalFEMForceField.cpp 
    ../forcefield/TetrahedronFEMForceField.cpp 
    ../forcefield/TriangularAnisotropicFEMForceField.cpp 
    ../forcefield/TriangleFEMForceField.cpp 
    ../forcefield/TriangularFEMForceField.cpp 
    ../forcefield/TriangularFEMForceFieldOptim.cpp
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

AddCompilerDefinitions("SOFA_BUILD_SIMPLE_FEM")
AddSourceDependencies(eigen)
AddLinkerDependencies(SofaBaseTopology ${SOFA_VISUAL_LIB} newmat)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
