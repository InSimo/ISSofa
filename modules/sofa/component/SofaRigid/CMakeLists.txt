include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initRigid.h 
    ../container/ArticulatedHierarchyContainer.h 
    ../container/ArticulatedHierarchyContainer.inl 
    ../mapping/ArticulatedSystemMapping.h 
    ../mapping/ArticulatedSystemMapping.inl 
    ../mapping/LaparoscopicRigidMapping.h 
    ../mapping/LaparoscopicRigidMapping.inl 
    ../mapping/LineSetSkinningMapping.h 
    ../mapping/LineSetSkinningMapping.inl 
    ../mapping/RigidMapping.h 
    ../mapping/RigidMapping.inl 
    ../mapping/RigidRigidMapping.h 
    ../mapping/RigidRigidMapping.inl 
    ../mapping/SkinningMapping.h 
    ../mapping/SkinningMapping.inl 
    ../interactionforcefield/JointSpringForceField.h 
    ../interactionforcefield/JointSpringForceField.inl

    )
    
set(SOURCE_FILES

    ../initRigid.cpp 
    ../container/ArticulatedHierarchyContainer.cpp 
    ../mapping/ArticulatedSystemMapping.cpp 
    ../mapping/LaparoscopicRigidMapping.cpp 
    ../mapping/LineSetSkinningMapping.cpp 
    ../mapping/RigidMapping.cpp 
    ../mapping/RigidRigidMapping.cpp 
    ../mapping/SkinningMapping.cpp 
    ../interactionforcefield/JointSpringForceField.cpp

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

AddCompilerDefinitions("SOFA_BUILD_RIGID")
AddSourceDependencies(eigen)
AddLinkerDependencies(SofaBaseMechanics)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
