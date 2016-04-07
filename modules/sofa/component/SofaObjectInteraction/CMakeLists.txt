include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initObjectInteraction.h 
    ../projectiveconstraintset/AttachConstraint.h 
    ../projectiveconstraintset/AttachConstraint.inl 
    ../interactionforcefield/BoxStiffSpringForceField.h 
    ../interactionforcefield/BoxStiffSpringForceField.inl 
    ../interactionforcefield/InteractionEllipsoidForceField.h 
    ../interactionforcefield/InteractionEllipsoidForceField.inl 
    ../interactionforcefield/PenalityContactForceField.h 
    ../interactionforcefield/PenalityContactForceField.inl 
    ../interactionforcefield/RepulsiveSpringForceField.h 
    ../interactionforcefield/RepulsiveSpringForceField.inl

    )
    
set(SOURCE_FILES

    ../initObjectInteraction.cpp 
    ../projectiveconstraintset/AttachConstraint.cpp 
    ../interactionforcefield/BoxStiffSpringForceField.cpp 
    ../interactionforcefield/InteractionEllipsoidForceField.cpp 
    ../interactionforcefield/PenalityContactForceField.cpp 
    ../interactionforcefield/RepulsiveSpringForceField.cpp
 
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

AddCompilerDefinitions("SOFA_BUILD_OBJECT_INTERACTION")
AddLinkerDependencies(SofaDeformable SofaRigid SofaMiscMapping)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
