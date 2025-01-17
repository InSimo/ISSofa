include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initBoundaryCondition.h 
    ../forcefield/BuoyantForceField.h 
    ../forcefield/BuoyantForceField.inl 
    ../forcefield/ConicalForceField.h 
    ../forcefield/ConicalForceField.inl 
    ../forcefield/ConstantForceField.h 
    ../forcefield/ConstantForceField.inl 
    ../forcefield/EdgePressureForceField.h 
    ../forcefield/EdgePressureForceField.inl 
    ../forcefield/EllipsoidForceField.h 
    ../forcefield/EllipsoidForceField.inl 
    ../forcefield/LinearForceField.h 
    ../forcefield/LinearForceField.inl 
    ../forcefield/OscillatingTorsionPressureForceField.h 
    ../forcefield/OscillatingTorsionPressureForceField.inl 
    ../forcefield/PlaneForceField.h 
    ../forcefield/PlaneForceField.inl 
    ../forcefield/QuadPressureForceField.h 
    ../forcefield/QuadPressureForceField.inl 
    ../forcefield/SphereForceField.h 
    ../forcefield/SphereForceField.inl 
    ../forcefield/SurfacePressureForceField.h 
    ../forcefield/SurfacePressureForceField.inl 
    ../forcefield/TaitSurfacePressureForceField.h 
    ../forcefield/TaitSurfacePressureForceField.inl 
    ../forcefield/TrianglePressureForceField.h 
    ../forcefield/TrianglePressureForceField.inl 
    ../forcefield/VaccumSphereForceField.h 
    ../forcefield/VaccumSphereForceField.inl 
	../forcefield/UniformVelocityDampingForceField.h
	../forcefield/UniformVelocityDampingForceField.inl
	../forcefield/DiagonalVelocityDampingForceField.h
	../forcefield/DiagonalVelocityDampingForceField.inl
    ../projectiveconstraintset/AffineMovementConstraint.inl 
    ../projectiveconstraintset/AffineMovementConstraint.h 
    ../projectiveconstraintset/FixedConstraint.h 
    ../projectiveconstraintset/FixedConstraint.inl 
    ../projectiveconstraintset/FixedPlaneConstraint.h 
    ../projectiveconstraintset/FixedPlaneConstraint.inl 
    ../projectiveconstraintset/FixedRotationConstraint.h 
    ../projectiveconstraintset/FixedRotationConstraint.inl 
    ../projectiveconstraintset/FixedTranslationConstraint.h 
    ../projectiveconstraintset/FixedTranslationConstraint.inl 
    ../projectiveconstraintset/HermiteSplineConstraint.h 
    ../projectiveconstraintset/HermiteSplineConstraint.inl 
     ../projectiveconstraintset/LinearMovementConstraint.h 
    ../projectiveconstraintset/LinearMovementConstraint.inl 
    ../projectiveconstraintset/LinearVelocityConstraint.h 
    ../projectiveconstraintset/LinearVelocityConstraint.inl 
    ../projectiveconstraintset/OscillatorConstraint.h 
    ../projectiveconstraintset/OscillatorConstraint.inl 
    ../projectiveconstraintset/ParabolicConstraint.h 
    ../projectiveconstraintset/ParabolicConstraint.inl 
    ../projectiveconstraintset/PartialFixedConstraint.h 
    ../projectiveconstraintset/PartialFixedConstraint.inl 
    ../projectiveconstraintset/PartialLinearMovementConstraint.h 
    ../projectiveconstraintset/PartialLinearMovementConstraint.inl 
    ../projectiveconstraintset/PatchTestMovementConstraint.inl 
    ../projectiveconstraintset/PatchTestMovementConstraint.h 
    ../projectiveconstraintset/PointConstraint.h 
    ../projectiveconstraintset/PointConstraint.inl 
    ../projectiveconstraintset/PositionBasedDynamicsConstraint.h 
    ../projectiveconstraintset/PositionBasedDynamicsConstraint.inl 
    ../projectiveconstraintset/SkeletalMotionConstraint.h 
    ../projectiveconstraintset/SkeletalMotionConstraint.inl
    )
    
set(SOURCE_FILES

    ../initBoundaryCondition.cpp 
    ../forcefield/BuoyantForceField.cpp 
    ../forcefield/ConicalForceField.cpp 
    ../forcefield/ConstantForceField.cpp 
    ../forcefield/EdgePressureForceField.cpp 
    ../forcefield/EllipsoidForceField.cpp 
    ../forcefield/LinearForceField.cpp 
    ../forcefield/OscillatingTorsionPressureForceField.cpp 
    ../forcefield/PlaneForceField.cpp 
    ../forcefield/QuadPressureForceField.cpp 
    ../forcefield/SphereForceField.cpp 
    ../forcefield/SurfacePressureForceField.cpp 
    ../forcefield/TaitSurfacePressureForceField.cpp 
    ../forcefield/TrianglePressureForceField.cpp 
    ../forcefield/VaccumSphereForceField.cpp 
	../forcefield/UniformVelocityDampingForceField.cpp
	../forcefield/DiagonalVelocityDampingForceField.cpp
    ../projectiveconstraintset/AffineMovementConstraint.cpp 
    ../projectiveconstraintset/FixedConstraint.cpp 
    ../projectiveconstraintset/FixedPlaneConstraint.cpp 
    ../projectiveconstraintset/FixedRotationConstraint.cpp 
    ../projectiveconstraintset/FixedTranslationConstraint.cpp 
    ../projectiveconstraintset/HermiteSplineConstraint.cpp 
    ../projectiveconstraintset/LinearMovementConstraint.cpp 
    ../projectiveconstraintset/LinearVelocityConstraint.cpp 
    ../projectiveconstraintset/OscillatorConstraint.cpp 
    ../projectiveconstraintset/ParabolicConstraint.cpp 
    ../projectiveconstraintset/PartialFixedConstraint.cpp 
    ../projectiveconstraintset/PartialLinearMovementConstraint.cpp 
    ../projectiveconstraintset/PatchTestMovementConstraint.cpp 
    ../projectiveconstraintset/PointConstraint.cpp 
    ../projectiveconstraintset/PositionBasedDynamicsConstraint.cpp 
    ../projectiveconstraintset/SkeletalMotionConstraint.cpp
    )
    
if(SOFA-LIB_COMPONENT_EIGEN2_SOLVER)
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectToLineConstraint.h")
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectToLineConstraint.inl")
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectToPlaneConstraint.h")
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectToPlaneConstraint.inl")
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectToPointConstraint.h")
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectToPointConstraint.inl")
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectDirectionConstraint.h")
    list(APPEND HEADER_FILES "../projectiveconstraintset/ProjectDirectionConstraint.inl")

    list(APPEND SOURCE_FILES "../projectiveconstraintset/ProjectToLineConstraint.cpp")
    list(APPEND SOURCE_FILES "../projectiveconstraintset/ProjectToPlaneConstraint.cpp")
    list(APPEND SOURCE_FILES "../projectiveconstraintset/ProjectToPointConstraint.cpp")
    list(APPEND SOURCE_FILES "../projectiveconstraintset/ProjectDirectionConstraint.cpp")
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

AddCompilerDefinitions("SOFA_BUILD_BOUNDARY_CONDITION")
AddLinkerDependencies(SofaBaseTopology)
if(SOFA-LIB_COMPONENT_EIGEN2_SOLVER)
    AddLinkerDependencies(SofaEigen2Solver)
endif()

include(${SOFA_CMAKE_DIR}/postProject.cmake)
