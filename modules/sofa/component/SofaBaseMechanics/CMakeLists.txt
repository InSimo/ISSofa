include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initBaseMechanics.h 
    ../container/MappedObject.h 
    ../container/MappedObject.inl 
    ../container/MechanicalObject.h 
    ../container/MechanicalObject.inl 
    ../mass/AddMToMatrixFunctor.h 
    ../mass/DiagonalMass.h 
    ../mass/DiagonalMass.inl 
    ../mass/UniformMass.h 
    ../mass/UniformMass.inl 
    ../mapping/BarycentricMapping.h 
    ../mapping/BarycentricMapping.inl 
    ../mapping/IdentityMapping.h 
    ../mapping/IdentityMapping.inl 
    ../mapping/SubsetMapping.h 
    ../mapping/SubsetMapping.inl

    )
    
set(SOURCE_FILES

    ../initBaseMechanics.cpp 
    ../container/MappedObject.cpp 
    ../container/MechanicalObject.cpp 
    ../mass/DiagonalMass.cpp 
    ../mass/UniformMass.cpp 
    ../mapping/BarycentricMapping.cpp 
    ../mapping/IdentityMapping.cpp 
    ../mapping/SubsetMapping.cpp
    
    )
    
if(SOFA-MISC_SMP)
    list(APPEND HEADER_FILES "../container/MechanicalObjectTasks.inl")
endif()

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

AddCompilerDefinitions("SOFA_BUILD_BASE_MECHANICS")
AddSourceDependencies(eigen)
AddLinkerDependencies(SofaBaseTopology SofaBaseLinearSolver)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
