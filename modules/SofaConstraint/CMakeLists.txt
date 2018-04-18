cmake_minimum_required(VERSION 2.8)
project(SofaConstraint)

set(HEADER_FILES
    BilateralConstraintResolution.h
    BilateralInteractionConstraint.h
    BilateralInteractionConstraint.inl
    ConstraintAnimationLoop.h
    ConstraintAttachBodyPerformer.h
    ConstraintAttachBodyPerformer.inl
    ConstraintSolverImpl.h
    ConstraintStoreLambdaVisitor.h
    ContactDescription.h
    FreeMotionAnimationLoop.h
    FrictionContact.h
    FrictionContact.inl
    StickContactConstraint.h
    StickContactConstraint.inl
    UnilateralInteractionConstraint.h
    UnilateralInteractionConstraint.inl
    UncoupledConstraintCorrection.h
    UncoupledConstraintCorrection.inl
    PrecomputedConstraintCorrection.h
    PrecomputedConstraintCorrection.inl
    LinearSolverConstraintCorrection.h
    LinearSolverConstraintCorrection.inl
    GenericConstraintCorrection.h
    LCPConstraintSolver.h
    ConstraintSolverImpl.h
    FreeMotionAnimationLoop.h
    ContactDescription.h
    GenericConstraintSolver.h
    BilateralInteractionConstraint.h
    BilateralInteractionConstraint.inl
    ConstraintAnimationLoop.h
    MappingGeometricStiffnessForceField.h
    MappingGeometricStiffnessForceField.inl
    RigidMassComplianceEngine.h
    RigidMassComplianceEngine.inl
    StopperConstraint.h
    StopperConstraint.inl
    SlidingConstraint.h
    SlidingConstraint.inl
    UniformConstraint.h
    UniformConstraint.inl
    initConstraint.h

    )

set(SOURCE_FILES

    initConstraint.cpp
    LocalMinDistance.cpp
    LMDNewProximityIntersection.cpp
    FrictionContact.cpp
    StickContactConstraint.cpp
    UnilateralInteractionConstraint.cpp
    UncoupledConstraintCorrection.cpp
    PrecomputedConstraintCorrection.cpp
    LinearSolverConstraintCorrection.cpp
    LCPConstraintSolver.cpp
    ConstraintSolverImpl.cpp
    ConstraintStoreLambdaVisitor.cpp
    ContactIdentifier.cpp
    FreeMotionAnimationLoop.cpp
    GenericConstraintSolver.cpp
    BilateralInteractionConstraint.cpp
    ConstraintAnimationLoop.cpp
    RigidMassComplianceEngine.cpp
    MappingGeometricStiffnessForceField.cpp
    StopperConstraint.cpp
    SlidingConstraint.cpp
    ConstraintAttachBodyPerformer.cpp
    GenericConstraintCorrection.cpp
    UniformConstraint.cpp

    )

if(SOFA_HAVE_EIGEN2)

list(APPEND HEADER_FILES
    BarycentricDistanceLMConstraintContact.h
    BarycentricDistanceLMConstraintContact.inl
    DOFBlockerLMConstraint.h
    DOFBlockerLMConstraint.inl
    FixedLMConstraint.h
    FixedLMConstraint.inl
    DistanceLMContactConstraint.h
    DistanceLMContactConstraint.inl
    DistanceLMConstraint.h
    DistanceLMConstraint.inl
    LMConstraintSolver.h
    LMConstraintDirectSolver.h
)

list(APPEND SOURCE_FILES
    BarycentricDistanceLMConstraintContact.cpp
    DOFBlockerLMConstraint.cpp
    FixedLMConstraint.cpp
    DistanceLMContactConstraint.cpp
    DistanceLMConstraint.cpp
    LMConstraintSolver.cpp
    LMConstraintDirectSolver.cpp
)

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
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaMeshCollision SofaSimpleFem SofaImplicitOdeSolver SofaUserInteraction SofaBaseLinearSolver)
if(SOFA_HAVE_EIGEN2)
   target_link_libraries( ${PROJECT_NAME} PUBLIC SofaEigen2Solver)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_SOURCE_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_DEPENDENCY_INCLUDE_DIR}>")

set( SOFACONSTRAINT_COMPILER_FLAGS "-DSOFA_BUILD_CONSTRAINT")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFAGENERAL_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFACONSTRAINT_COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT    SofaGeneralTargets
        RUNTIME DESTINATION "${SOFAGENERAL_INSTALL_BIN_DIR}" COMPONENT SofaGeneral_libraries
        LIBRARY DESTINATION "${SOFAGENERAL_INSTALL_LIB_DIR}" COMPONENT SofaGeneral_libraries
        ARCHIVE DESTINATION "${SOFAGENERAL_INSTALL_LIB_DIR}" COMPONENT SofaGeneral_libraries
        PUBLIC_HEADER DESTINATION "${SOFAGENERAL_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT SofaGeneral_headers
)

