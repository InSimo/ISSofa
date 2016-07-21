cmake_minimum_required(VERSION 2.8.12)
project(SofaBoundaryCondition)

set(HEADER_FILES

    initBoundaryCondition.h
    BuoyantForceField.h
    BuoyantForceField.inl
    ConicalForceField.h
    ConicalForceField.inl
    ConstantForceField.h
    ConstantForceField.inl
    EdgePressureForceField.h
    EdgePressureForceField.inl
    EllipsoidForceField.h
    EllipsoidForceField.inl
    LinearForceField.h
    LinearForceField.inl
    OscillatingTorsionPressureForceField.h
    OscillatingTorsionPressureForceField.inl
    PlaneForceField.h
    PlaneForceField.inl
    QuadPressureForceField.h
    QuadPressureForceField.inl
    SphereForceField.h
    SphereForceField.inl
    SurfacePressureForceField.h
    SurfacePressureForceField.inl
    TaitSurfacePressureForceField.h
    TaitSurfacePressureForceField.inl
    TrianglePressureForceField.h
    TrianglePressureForceField.inl
    VaccumSphereForceField.h
    VaccumSphereForceField.inl
	UniformVelocityDampingForceField.h
	UniformVelocityDampingForceField.inl
	DiagonalVelocityDampingForceField.h
	DiagonalVelocityDampingForceField.inl
    AffineMovementConstraint.inl
    AffineMovementConstraint.h
    FixedConstraint.h
    FixedConstraint.inl
    FixedPlaneConstraint.h
    FixedPlaneConstraint.inl
    FixedRotationConstraint.h
    FixedRotationConstraint.inl
    FixedTranslationConstraint.h
    FixedTranslationConstraint.inl
    HermiteSplineConstraint.h
    HermiteSplineConstraint.inl
     LinearMovementConstraint.h
    LinearMovementConstraint.inl
    LinearVelocityConstraint.h
    LinearVelocityConstraint.inl
    OscillatorConstraint.h
    OscillatorConstraint.inl
    ParabolicConstraint.h
    ParabolicConstraint.inl
    PartialFixedConstraint.h
    PartialFixedConstraint.inl
    PartialLinearMovementConstraint.h
    PartialLinearMovementConstraint.inl
    PatchTestMovementConstraint.inl
    PatchTestMovementConstraint.h
    PointConstraint.h
    PointConstraint.inl
    PositionBasedDynamicsConstraint.h
    PositionBasedDynamicsConstraint.inl
    SkeletalMotionConstraint.h
    SkeletalMotionConstraint.inl
    )

set(SOURCE_FILES

    initBoundaryCondition.cpp
    BuoyantForceField.cpp
    ConicalForceField.cpp
    ConstantForceField.cpp
    EdgePressureForceField.cpp
    EllipsoidForceField.cpp
    LinearForceField.cpp
    OscillatingTorsionPressureForceField.cpp
    PlaneForceField.cpp
    QuadPressureForceField.cpp
    SphereForceField.cpp
    SurfacePressureForceField.cpp
    TaitSurfacePressureForceField.cpp
    TrianglePressureForceField.cpp
    VaccumSphereForceField.cpp
	UniformVelocityDampingForceField.cpp
	DiagonalVelocityDampingForceField.cpp
    AffineMovementConstraint.cpp
    FixedConstraint.cpp
    FixedPlaneConstraint.cpp
    FixedRotationConstraint.cpp
    FixedTranslationConstraint.cpp
    HermiteSplineConstraint.cpp
    LinearMovementConstraint.cpp
    LinearVelocityConstraint.cpp
    OscillatorConstraint.cpp
    ParabolicConstraint.cpp
    PartialFixedConstraint.cpp
    PartialLinearMovementConstraint.cpp
    PatchTestMovementConstraint.cpp
    PointConstraint.cpp
    PositionBasedDynamicsConstraint.cpp
    SkeletalMotionConstraint.cpp
    )

if(SOFA_HAVE_EIGEN2)
    list(APPEND HEADER_FILES "ProjectToLineConstraint.h")
    list(APPEND HEADER_FILES "ProjectToLineConstraint.inl")
    list(APPEND HEADER_FILES "ProjectToPlaneConstraint.h")
    list(APPEND HEADER_FILES "ProjectToPlaneConstraint.inl")
    list(APPEND HEADER_FILES "ProjectToPointConstraint.h")
    list(APPEND HEADER_FILES "ProjectToPointConstraint.inl")
    list(APPEND HEADER_FILES "ProjectDirectionConstraint.h")
    list(APPEND HEADER_FILES "ProjectDirectionConstraint.inl")

    list(APPEND SOURCE_FILES "ProjectToLineConstraint.cpp")
    list(APPEND SOURCE_FILES "ProjectToPlaneConstraint.cpp")
    list(APPEND SOURCE_FILES "ProjectToPointConstraint.cpp")
    list(APPEND SOURCE_FILES "ProjectDirectionConstraint.cpp")
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
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaBaseTopology )
#if(SOFA_HAVE_EIGEN2)
#    target_link_libraries( ${PROJECT_NAME} SofaEigen2Solver )
#endif()
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_DEPENDENCY_INCLUDE_DIR}>")

set( SOFABOUNDARY_CONDITION_COMPILER_FLAGS "-DSOFA_BUILD_BOUNDARY_CONDITION")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFAGENERAL_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFABOUNDARY_CONDITION_COMPILER_FLAGS}"  )
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
