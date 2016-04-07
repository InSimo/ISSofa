cmake_minimum_required(VERSION 2.8.12)
project(SofaHelper)

set(SOFAHELPER_HEADERS
    ArgumentParser.h
    BackTrace.h
	deque.h
    DualQuat.inl
    DualQuat.h
    fixed_array.h
    Factory.h
    Factory.inl
    FnDispatcher.h
    FnDispatcher.inl
    gl/Trackball.h
    gl/Transformation.h
    integer_id.h
    io/Image.h
    io/ImageBMP.h
    io/ImageRAW.h
    io/MassSpringLoader.h
    io/Mesh.h
    io/MeshOBJ.h
    io/MeshVTK.h
    io/MeshSTL.h
    io/MeshTopologyLoader.h
    io/MeshTrian.h
    io/SphereLoader.h
    io/TriangleLoader.h
    io/bvh/BVHChannels.h
    io/bvh/BVHJoint.h
    io/bvh/BVHLoader.h
    io/bvh/BVHMotion.h
    io/bvh/BVHOffset.h
    LCPcalc.h
    LCPSolver.h
    LCPSolver.inl
    map.h
    map_ptr_stable_compare.h
    MatEigen.h
    list.h
    MarchingCubeUtility.h
    MemoryManager.h
    decompose.h
    decompose.inl
    Quater.h
    Quater.inl
    rmath.h
    RandomGenerator.h
    set.h
    SVector.h
    system/config.h
    system/gl.h
    system/glu.h
    system/SetDirectory.h
    system/FileRepository.h
    system/atomic.h
	system/thread/CircularQueue.h
	system/thread/CircularQueue.inl
    system/thread/CTime.h
    system/thread/debug.h
    system/thread/thread_specific_ptr.h
    system/PipeProcess.h
    system/SofaOStream.h
    system/FileSystem.h
    system/Utils.h
    system/DynamicLibrary.h
    system/PluginManager.h
    TagFactory.h
    accessor.h
    vector.h
    vector_device.h
    vector_algebra.h
    stable_vector.h
    StringUtils.h
    polygon_cube_intersection/vec.h
    polygon_cube_intersection/polygon_cube_intersection.h
    proximity.h
    SimpleTimer.h
    AdvancedTimer.h
    io/ImageDDS.h
	OptionsGroup.h 
    Polynomial_LD.h
    Polynomial_LD.inl
    UnitTest.h
    SortedPermutation.h
    kdTree.inl
    kdTree.h
	)

set (SOFAHELPER_SOURCES_OUTSIDE_UNITY
     AdvancedTimer.cpp
     io/ImageDDS.cpp
     io/SphereLoader.cpp )
 
set(SOFAHELPER_SOURCES_UNITY
	ArgumentParser.cpp
    BackTrace.cpp
    DualQuat.cpp
    Factory.cpp
    gl/Trackball.cpp
    gl/Transformation.cpp
    io/Image.cpp
    io/ImageBMP.cpp
    io/ImageRAW.cpp
    io/MassSpringLoader.cpp
    io/Mesh.cpp
    io/MeshOBJ.cpp
    io/MeshVTK.cpp
    io/MeshSTL.cpp
    io/MeshTopologyLoader.cpp
    io/MeshTrian.cpp
    io/TriangleLoader.cpp
    io/bvh/BVHJoint.cpp
    io/bvh/BVHLoader.cpp
    io/bvh/BVHMotion.cpp
    LCPcalc.cpp
    MarchingCubeUtility.cpp
    decompose.cpp
    Quater.cpp
    RandomGenerator.cpp
	system/gl.cpp
    system/SetDirectory.cpp
    system/FileRepository.cpp
    system/thread/CTime.cpp
    system/thread/debug.cpp
    system/thread/CircularQueue.cpp
    system/PipeProcess.cpp
    system/SofaOStream.cpp
    system/FileSystem.cpp
    system/Utils.cpp
    system/DynamicLibrary.cpp
    system/PluginManager.cpp
    TagFactory.cpp
    polygon_cube_intersection/polygon_cube_intersection.cpp
    polygon_cube_intersection/fast_polygon_cube_intersection.cpp
    vector.cpp
    proximity.cpp
    OptionsGroup.cpp
    Polynomial_LD.cpp
    UnitTest.cpp
	)

if(SOFA_HAVE_PNG)
	list(APPEND SOFAHELPER_HEADERS  io/ImagePNG.h  )
	list(APPEND SOFAHELPER_SOURCES_UNITY  io/ImagePNG.cpp) 
endif()

if(Boost_thread_FOUND )
	list(APPEND SOFAHELPER_HEADERS "system/thread/TimeoutWatchdog.h")
	list(APPEND SOFAHELPER_SOURCES_UNITY "system/thread/TimeoutWatchdog.cpp")
endif()

if(NOT SOFA-MISC_NO_OPENGL)
	if(SOFA_HAVE_GLUT)
		list(APPEND SOFAHELPER_HEADERS
		system/glut.h
		gl/Axis.h
		gl/BasicShapes.h
		gl/Cylinder.h
		gl/glText.inl
		gl/glText.h
		)
		list(APPEND SOFAHELPER_SOURCES_UNITY
		gl/Axis.cpp
		gl/glText.cpp
		)

		list( APPEND SOFAHELPER_SOURCES_OUTSIDE_UNITY
		      gl/Cylinder.cpp
                )
	endif(SOFA_HAVE_GLUT)
	
	list(APPEND SOFAHELPER_HEADERS
	gl/Capture.h
	gl/Color.h
	gl/glfont.h
	gl/RAII.h
	gl/template.h)
			
	list(APPEND SOFAHELPER_SOURCES_UNITY
	gl/Color.cpp
	gl/glfont.cpp
	)

        list(APPEND SOFAHELPER_SOURCES_OUTSIDE_UNITY
	gl/Capture.cpp
        )
	
	if(SOFA_HAVE_GLEW)
		list(APPEND SOFAHELPER_HEADERS
			gl/Texture.h
			gl/FrameBufferObject.h
			gl/GLSLShader.h)
		list(APPEND SOFAHELPER_SOURCES_UNITY
			gl/Texture.cpp
			gl/FrameBufferObject.cpp
			gl/GLSLShader.cpp)
	endif(SOFA_HAVE_GLEW)
endif(NOT SOFA-MISC_NO_OPENGL)

if(SOFA-EXTERNAL_HAVE_FFMPEG)
	list(APPEND SOFAHELPER_HEADERS "gl/VideoRecorder.h")
	list(APPEND SOFAHELPER_SOURCES_UNITY "gl/VideoRecorder.cpp")
endif()

if(PS3)
	list(APPEND SOFAHELPER_HEADERS "gl/ps3gl_compat.h")
	list(APPEND SOFAHELPER_SOURCES_UNITY "gl/ps3gl_compat.cpp")
endif()

if(WIN32)
    # Windows lib, used in system::FileSystem
	list(APPEND SOFA_HELPER_LINK_PRIVATE_DEPENDENCIES Shlwapi)
endif()

if(UNIX)
  list(APPEND SOFA_HELPER_LINK_PRIVATE_DEPENDENCIES dl)
endif()




include( CMakeDependentOption )
cmake_dependent_option(UNITY_BUILD_${PROJECT_NAME} "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." ON  SOFA_ALLOW_UNITY_BUILDS OFF)

if( UNITY_BUILD_${PROJECT_NAME} )
  # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
  # source files.
  set( UNITYBUILD_INCLUDES "" )
  foreach(filename ${SOFAHELPER_SOURCES_UNITY})
    SET( UNITYBUILD_INCLUDES "${UNITYBUILD_INCLUDES}#include \"${filename}\"\n" )
  endforeach(filename)

  file(WRITE UnityBuild.cpp ${UNITYBUILD_INCLUDES} )

  # overwrite the source files to only include the generated unity build file.
  set( SOFAHELPER_SOURCES ${SOFAHELPER_SOURCES_OUTSIDE_UNITY} UnityBuild.cpp )
else()
  set( SOFAHELPER_SOURCES ${SOFAHELPER_SOURCES_OUTSIDE_UNITY} ${SOFAHELPER_SOURCES_UNITY} )
endif()

add_library(${PROJECT_NAME} SHARED ${SOFAHELPER_HEADERS} ${SOFAHELPER_SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${SOFAFRAMEWORK_DEPENDENCY_LINK} )
target_link_libraries(${PROJECT_NAME} PRIVATE ${SOFA_HELPER_LINK_PRIVATE_DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>" 
												  "$<BUILD_INTERFACE:${SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIRECTORIES}>")

# make the name of debug libraries end in _d.
set_target_properties( ${PROJECT_NAME}  PROPERTIES DEBUG_POSTFIX "_d" )
set_property(SOURCE system/PluginManager.cpp PROPERTY COMPILE_DEFINITIONS_DEBUG "SOFA_LIBSUFFIX=_d" )
set_target_properties( ${PROJECT_NAME}  PROPERTIES VERSION ${SOFAFRAMEWORK_VERSION} )

set( SOFAHELPER_COMPILER_FLAGS "-DSOFA_BUILD_HELPER -DUNICODE")

set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFAHELPER_COMPILER_FLAGS}"  )
# set_target_properties( ${PROJECT_NAME} PROPERTIES FOLDER "${GLOBAL_PROJECT_OPTION_FOLDER_${PROJECT_NAME}}")

install(TARGETS ${PROJECT_NAME}
    COMPONENT SofaFramework_libraries
    EXPORT  SofaFrameworkTargets
    RUNTIME DESTINATION "${SOFAFRAMEWORK_INSTALL_BIN_DIR}"
    LIBRARY DESTINATION "${SOFAFRAMEWORK_INSTALL_LIB_DIR}"
    ARCHIVE DESTINATION "${SOFAFRAMEWORK_INSTALL_LIB_DIR}"
    )

if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
install(DIRECTORY ${SofaHelper_SOURCE_DIR}/ DESTINATION "${SOFAFRAMEWORK_INSTALL_INCLUDE_DIR}/sofa/helper"
        COMPONENT SofaFramework_headers
        FILES_MATCHING 
            PATTERN "*.h"
            PATTERN "*.inl"
            PATTERN "*.hxx"
            PATTERN "*.hpp"
       )
endif()
