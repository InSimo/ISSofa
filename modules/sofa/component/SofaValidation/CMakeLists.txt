include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initValidation.h 
    ../misc/CompareState.h 
    ../misc/CompareTopology.h 
    ../misc/DevAngleCollisionMonitor.h 
    ../misc/DevAngleCollisionMonitor.inl 
    ../misc/DevTensionMonitor.h 
    ../misc/DevTensionMonitor.inl 
    ../misc/DevMonitorManager.h 
    ../misc/ExtraMonitor.h 
    ../misc/ExtraMonitor.inl 
    ../misc/Monitor.h 
    ../misc/Monitor.inl 
    ../misc/EvalPointsDistance.h 
    ../misc/EvalPointsDistance.inl 
    ../misc/EvalSurfaceDistance.h 
    ../misc/EvalSurfaceDistance.inl 
    ../misc/DataMonitor.h 
    ../misc/DataController.h 

    )
    
set(SOURCE_FILES

    ../initValidation.cpp 
    ../misc/CompareState.cpp 
    ../misc/CompareTopology.cpp 
    ../misc/DevAngleCollisionMonitor.cpp 
    ../misc/DevTensionMonitor.cpp 
    ../misc/DevMonitorManager.cpp 
    ../misc/ExtraMonitor.cpp 
    ../misc/Monitor.cpp 
    ../misc/EvalPointsDistance.cpp 
    ../misc/EvalSurfaceDistance.cpp 
    ../misc/DataMonitor.cpp 
    ../misc/DataController.cpp 
 
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

AddCompilerDefinitions("SOFA_BUILD_VALIDATION" )
AddLinkerDependencies(${ZLIB_LIBRARIES_OPTIONAL} SofaBaseCollision SofaLoader SofaMeshCollision )

include(${SOFA_CMAKE_DIR}/postProject.cmake)
