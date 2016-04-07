include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initExporter.h 
    ../misc/WriteState.h 
    ../misc/WriteState.inl 
    ../misc/WriteTopology.h 
    ../misc/WriteTopology.inl 
    ../misc/VTKExporter.h 
    ../misc/OBJExporter.h
	../misc/BlenderExporter.h 
	../misc/BlenderExporter.inl 
    ../misc/STLExporter.h 
    ../misc/MeshExporter.h

    )
    
set(SOURCE_FILES

    ../initExporter.cpp 
    ../misc/WriteState.cpp 
    ../misc/WriteTopology.cpp 
    ../misc/VTKExporter.cpp 
    ../misc/OBJExporter.cpp 
	../misc/BlenderExporter.cpp
    ../misc/STLExporter.cpp 
    ../misc/MeshExporter.cpp
 
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

AddCompilerDefinitions("SOFA_BUILD_EXPORTER" )
AddLinkerDependencies(${ZLIB_LIBRARIES_OPTIONAL} SofaSimulationTree )

include(${SOFA_CMAKE_DIR}/postProject.cmake)
