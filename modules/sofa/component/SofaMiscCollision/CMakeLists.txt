include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initMiscCollision.h 
    ../collision/TriangleModelInRegularGrid.h
    ../collision/RuleBasedContactManager.h 
    ../collision/DefaultCollisionGroupManager.h 
    ../collision/SolverMerger.h 
    ../collision/TetrahedronDiscreteIntersection.h 
    ../collision/SpatialGridPointModel.h 
    ../collision/TetrahedronModel.h 
    ../collision/BarycentricStickContact.h 
    ../collision/BarycentricStickContact.inl 

    )
    
set(SOURCE_FILES

    ../initMiscCollision.cpp 
	../collision/FrictionContact_DistanceGrid.cpp 
    ../collision/TriangleModelInRegularGrid.cpp
    ../collision/RuleBasedContactManager.cpp 
    ../collision/DefaultCollisionGroupManager.cpp 
    ../collision/SolverMerger.cpp 
	../collision/TetrahedronDiscreteIntersection.cpp 
    ../collision/SpatialGridPointModel.cpp 
    ../collision/TetrahedronModel.cpp 
    ../collision/TetrahedronBarycentricPenalityContact.cpp 
    ../collision/TetrahedronRayContact.cpp 
    ../collision/TetrahedronFrictionContact.cpp 
    ../collision/BarycentricStickContact.cpp 
 
	../collision/TetrahedronBarycentricDistanceLMConstraintContact.cpp # eigen2solver
	../collision/BarycentricDistanceLMConstraintContact_DistanceGrid.cpp # eigen2solver
    )

if(SOFA-MISC_SMP)
	list(APPEND HEADER_FILES "../collision/ParallelCollisionPipeline.h")
	list(APPEND SOURCE_FILES "../collision/ParallelCollisionPipeline.cpp")
endif()
    

#if(SOFA-LIB_COMPONENT_EIGEN2_SOLVER)
#	list(APPEND SOURCE_FILES "../collision/TetrahedronBarycentricDistanceLMConstraintContact.cpp")
#	list(APPEND SOURCE_FILES "../collision/BarycentricDistanceLMConstraintContact_DistanceGrid.cpp")
#endif()

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

AddCompilerDefinitions("SOFA_BUILD_MISC_COLLISION" )
AddSourceDependencies(eigen)
AddLinkerDependencies(SofaMeshCollision SofaConstraint SofaVolumetricData SofaExplicitOdeSolver SofaImplicitOdeSolver SofaEigen2Solver)

#if(SOFA-LIB_COMPONENT_EIGEN2_SOLVER)
#    AddLinkerDependencies(SofaEigen2Solver)
#endif()

include(${SOFA_CMAKE_DIR}/postProject.cmake)
