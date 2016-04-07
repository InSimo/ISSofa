include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initMiscMapping.h 
    ../mapping/BeamLinearMapping.h 
    ../mapping/BeamLinearMapping.inl 
    ../mapping/CenterPointMechanicalMapping.h 
    ../mapping/CenterPointMechanicalMapping.inl 
    ../mapping/CenterOfMassMapping.h 
    ../mapping/CenterOfMassMapping.inl 
    ../mapping/CenterOfMassMultiMapping.h 
    ../mapping/CenterOfMassMultiMapping.inl 
    ../mapping/CenterOfMassMulti2Mapping.h 
    ../mapping/CenterOfMassMulti2Mapping.inl 
    ../mapping/CurveMapping.h 
    ../mapping/CurveMapping.inl 
    ../mapping/DistanceMapping.h
    ../mapping/DistanceMapping.inl
    ../mapping/DistanceFromTargetMapping.h
    ../mapping/DistanceFromTargetMapping.inl
    ../mapping/ExternalInterpolationMapping.h
    ../mapping/ExternalInterpolationMapping.inl
    ../mapping/SubsetMultiMapping.h 
    ../mapping/SubsetMultiMapping.inl 
    ../mapping/TubularMapping.h 
    ../mapping/TubularMapping.inl 
    ../mapping/VoidMapping.h 
    ../mapping/BarycentricMappingRigid.h 
    ../mapping/BarycentricMappingRigid.inl 
    ../mapping/DeformableOnRigidFrameMapping.h 
    ../mapping/DeformableOnRigidFrameMapping.inl 
    ../mapping/PCAOnRigidFrameMapping.h 
    ../mapping/PCAOnRigidFrameMapping.inl 
    ../mapping/CatmullRomSplineMapping.h 
    ../mapping/CatmullRomSplineMapping.inl

    )
    
set(SOURCE_FILES

    ../initMiscMapping.cpp 
    ../mapping/BeamLinearMapping.cpp 
    ../mapping/CenterPointMechanicalMapping.cpp 
    ../mapping/CenterOfMassMapping.cpp 
    ../mapping/CenterOfMassMultiMapping.cpp 
    ../mapping/CenterOfMassMulti2Mapping.cpp 
    ../mapping/CurveMapping.cpp 
    ../mapping/DistanceMapping.cpp
    ../mapping/DistanceFromTargetMapping.cpp
    ../mapping/ExternalInterpolationMapping.cpp
    ../mapping/SubsetMultiMapping.cpp 
    ../mapping/TubularMapping.cpp 
    ../mapping/VoidMapping.cpp 
    ../mapping/BarycentricMappingRigid.cpp 
    ../mapping/DeformableOnRigidFrameMapping.cpp 
    ../mapping/PCAOnRigidFrameMapping.cpp 
    ../mapping/CatmullRomSplineMapping.cpp
 
    )

set(GROUP_BASE_DIR "..")

AddSourceDependencies(eigen)



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

AddCompilerDefinitions("SOFA_BUILD_MISC_MAPPING" )
AddSourceDependencies(eigen)
AddLinkerDependencies(SofaSimpleFem SofaRigid)

    
include(${SOFA_CMAKE_DIR}/postProject.cmake)
