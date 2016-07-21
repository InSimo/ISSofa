include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initConstraint.h 
    ../collision/ContactIdentifier.h 
    ../collision/LocalMinDistance.h 
    ../collision/LocalMinDistance.inl 
    ../collision/LMDNewProximityIntersection.h 
    ../collision/LMDNewProximityIntersection.inl 
    ../collision/FrictionContact.h 
    ../collision/FrictionContact.inl 
    ../collision/StickContactConstraint.h 
    ../collision/StickContactConstraint.inl 
    ../constraintset/UnilateralInteractionConstraint.h 
    ../constraintset/UnilateralInteractionConstraint.inl 
    ../constraintset/UncoupledConstraintCorrection.h 
    ../constraintset/UncoupledConstraintCorrection.inl 
    ../constraintset/PrecomputedConstraintCorrection.h 
    ../constraintset/PrecomputedConstraintCorrection.inl 
    ../constraintset/LinearSolverConstraintCorrection.h 
    ../constraintset/LinearSolverConstraintCorrection.inl 
    ../constraintset/GenericConstraintCorrection.h
    ../constraintset/GenericConstraintCorrection.inl 
    ../constraintset/LCPConstraintSolver.h 
    ../constraintset/ConstraintSolverImpl.h 
    ../animationloop/FreeMotionAnimationLoop.h 
    ../constraintset/ContactDescription.h 
    ../constraintset/GenericConstraintSolver.h 
    ../constraintset/BilateralInteractionConstraint.h 
    ../constraintset/BilateralInteractionConstraint.inl 
    ../animationloop/ConstraintAnimationLoop.h 
    ../constraintset/StopperConstraint.h 
    ../constraintset/StopperConstraint.inl 
    ../constraintset/SlidingConstraint.h 
    ../constraintset/SlidingConstraint.inl
    ../collision/ConstraintAttachBodyPerformer.h 
    ../collision/ConstraintAttachBodyPerformer.inl

    )
    
set(SOURCE_FILES

    ../initConstraint.cpp 
    ../collision/ContactIdentifier.cpp 
    ../collision/LocalMinDistance.cpp 
    ../collision/LMDNewProximityIntersection.cpp 
    ../collision/FrictionContact.cpp 
    ../collision/StickContactConstraint.cpp 
    ../constraintset/UnilateralInteractionConstraint.cpp 
    ../constraintset/UncoupledConstraintCorrection.cpp 
    ../constraintset/PrecomputedConstraintCorrection.cpp 
    ../constraintset/LinearSolverConstraintCorrection.cpp 
    ../constraintset/LCPConstraintSolver.cpp 
    ../constraintset/ConstraintSolverImpl.cpp 
    ../animationloop/FreeMotionAnimationLoop.cpp 
    ../constraintset/GenericConstraintSolver.cpp 
    ../constraintset/BilateralInteractionConstraint.cpp 
    ../animationloop/ConstraintAnimationLoop.cpp 
    ../constraintset/StopperConstraint.cpp 
    ../constraintset/SlidingConstraint.cpp 
    ../collision/ConstraintAttachBodyPerformer.cpp
    ../constraintset/GenericConstraintCorrection.cpp 

    )
    
if(SOFA-LIB_COMPONENT_EIGEN2_SOLVER)
	list(APPEND HEADER_FILES "../collision/BarycentricDistanceLMConstraintContact.h")
	list(APPEND HEADER_FILES "../collision/BarycentricDistanceLMConstraintContact.inl")
	list(APPEND HEADER_FILES "../constraintset/DOFBlockerLMConstraint.h")
	list(APPEND HEADER_FILES "../constraintset/DOFBlockerLMConstraint.inl")
	list(APPEND HEADER_FILES "../constraintset/FixedLMConstraint.h")
	list(APPEND HEADER_FILES "../constraintset/FixedLMConstraint.inl")
	list(APPEND HEADER_FILES "../constraintset/DistanceLMContactConstraint.h")
	list(APPEND HEADER_FILES "../constraintset/DistanceLMContactConstraint.inl")
	list(APPEND HEADER_FILES "../constraintset/DistanceLMConstraint.h")
	list(APPEND HEADER_FILES "../constraintset/DistanceLMConstraint.inl")
	list(APPEND HEADER_FILES "../constraintset/LMConstraintSolver.h")
	list(APPEND HEADER_FILES "../constraintset/LMConstraintDirectSolver.h")

	list(APPEND SOURCE_FILES "../collision/BarycentricDistanceLMConstraintContact.cpp")
	list(APPEND SOURCE_FILES "../constraintset/DOFBlockerLMConstraint.cpp")
	list(APPEND SOURCE_FILES "../constraintset/FixedLMConstraint.cpp")
	list(APPEND SOURCE_FILES "../constraintset/DistanceLMContactConstraint.cpp")
	list(APPEND SOURCE_FILES "../constraintset/DistanceLMConstraint.cpp")
	list(APPEND SOURCE_FILES "../constraintset/LMConstraintSolver.cpp")
	list(APPEND SOURCE_FILES "../constraintset/LMConstraintDirectSolver.cpp")


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

AddCompilerDefinitions("SOFA_BUILD_CONSTRAINT" )
AddSourceDependencies(eigen)
AddLinkerDependencies(SofaMeshCollision SofaSimpleFem SofaImplicitOdeSolver SofaUserInteraction )
AddSourceDependencies(eigen)
if(SOFA-LIB_COMPONENT_EIGEN2_SOLVER)
    AddLinkerDependencies(SofaEigen2Solver)
endif()

include(${SOFA_CMAKE_DIR}/postProject.cmake)
