include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initLoader.h 
    ../loader/GridMeshCreator.h 
    ../loader/StringMeshCreator.h 
    ../loader/MeshGmshLoader.h 
    ../loader/MeshObjLoader.h 
	../loader/GIDMeshLoader.h
    ../loader/MeshOffLoader.h 
    ../loader/MeshTrianLoader.h 
    ../loader/MeshVTKLoader.h 
    ../loader/MeshSTLLoader.h 
    ../loader/MeshXspLoader.h 
    ../loader/OffSequenceLoader.h 
    ../loader/SphereLoader.h 
    ../loader/VoxelGridLoader.h 
    ../misc/InputEventReader.h 
    ../misc/ReadState.h 
    ../misc/ReadState.inl 
    ../misc/ReadTopology.h 
    ../misc/ReadTopology.inl 
    )
    
set(SOURCE_FILES

    ../initLoader.cpp 
    ../loader/GridMeshCreator.cpp 
    ../loader/StringMeshCreator.cpp 
    ../loader/MeshGmshLoader.cpp 
	../loader/MeshObjLoader.cpp
	../loader/GIDMeshLoader.cpp
    ../loader/MeshOffLoader.cpp 
    ../loader/MeshTrianLoader.cpp 
    ../loader/MeshVTKLoader.cpp 
    ../loader/MeshSTLLoader.cpp 
    ../loader/MeshXspLoader.cpp 
    ../loader/OffSequenceLoader.cpp 
    ../loader/SphereLoader.cpp 
    ../loader/VoxelGridLoader.cpp 
    ../misc/InputEventReader.cpp 
    ../misc/ReadState.cpp 
    ../misc/ReadTopology.cpp
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

AddCompilerDefinitions("SOFA_BUILD_LOADER" )
AddLinkerDependencies(${ZLIB_LIBRARIES_OPTIONAL} SofaSimulationTree tinyxml)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
