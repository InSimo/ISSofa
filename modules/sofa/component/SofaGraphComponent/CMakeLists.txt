include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initGraphComponent.h 
    ../configurationsetting/AddFrameButtonSetting.h 
    ../configurationsetting/AddRecordedCameraButtonSetting.h 
    ../configurationsetting/AttachBodyButtonSetting.h 
    ../configurationsetting/BackgroundSetting.h 
    ../configurationsetting/FixPickedParticleButtonSetting.h 
    ../configurationsetting/MouseButtonSetting.h 
    ../configurationsetting/SofaDefaultPathSetting.h 
    ../configurationsetting/StatsSetting.h 
    ../configurationsetting/ViewerSetting.h 
    ../contextobject/Gravity.h 
    ../misc/PauseAnimation.h 
    ../misc/PauseAnimationOnEvent.h 
    ../misc/RequiredPlugin.h
    ../misc/InteractingBehaviorModel.h

    )
    
set(SOURCE_FILES

    ../initGraphComponent.cpp 
    ../configurationsetting/AddFrameButtonSetting.cpp 
    ../configurationsetting/AddRecordedCameraButtonSetting.cpp 
    ../configurationsetting/AttachBodyButtonSetting.cpp 
    ../configurationsetting/BackgroundSetting.cpp 
    ../configurationsetting/FixPickedParticleButtonSetting.cpp 
    ../configurationsetting/MouseButtonSetting.cpp 
    ../configurationsetting/SofaDefaultPathSetting.cpp 
    ../configurationsetting/StatsSetting.cpp 
    ../configurationsetting/ViewerSetting.cpp 
    ../contextobject/Gravity.cpp 
    ../misc/PauseAnimation.cpp 
    ../misc/PauseAnimationOnEvent.cpp 
    ../misc/RequiredPlugin.cpp 

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

AddCompilerDefinitions("SOFA_BUILD_GRAPH_COMPONENT")
AddLinkerDependencies(SofaSimulationTree)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
