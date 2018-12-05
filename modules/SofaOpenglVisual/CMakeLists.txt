cmake_minimum_required(VERSION 2.8)
project(SofaOpenglVisual)


set(HEADER_FILES

    initOpenGLVisual.h
    OglModel.h
    OglViewport.h
    Light.h
    LightManager.h
    PointSplatModel.h
    OglRenderingSRGB.h
    ClipPlane.h
    CompositingVisualLoop.h
    ColorMap.h
    DataDisplay.h
    OglClear.h
    OglLabel.h
    OglCylinderModel.h
    OglSceneFrame.h
    OglGrid.h
    OglLineAxis.h

    )

set(SOURCE_FILES_OUTSIDE_UNITY
    OglLabel.cpp
    OglCylinderModel.cpp )

set(SOURCE_FILES_UNITY
    initOpenGLVisual.cpp
    OglClear.cpp
    OglModel.cpp
    OglViewport.cpp
    Light.cpp
    LightManager.cpp
    PointSplatModel.cpp
    OglRenderingSRGB.cpp
    ClipPlane.cpp
    CompositingVisualLoop.cpp
    ColorMap.cpp
    DataDisplay.cpp
    OglSceneFrame.cpp
    OglGrid.cpp
    OglLineAxis.cpp
    )

if(SOFA_HAVE_GLEW)
	list(APPEND HEADER_FILES "OglAttribute.h")
	list(APPEND HEADER_FILES "OglAttribute.inl")
	list(APPEND HEADER_FILES "OglShader.h")
	list(APPEND HEADER_FILES "OglShaderMacro.h")
	list(APPEND HEADER_FILES "OglShaderVisualModel.h")
	list(APPEND HEADER_FILES "OglShadowShader.h")
	list(APPEND HEADER_FILES "OglTetrahedralModel.h")
	list(APPEND HEADER_FILES "OglTetrahedralModel.inl")
	list(APPEND HEADER_FILES "OglTexture.h")
	list(APPEND HEADER_FILES "OglVariable.h")
	list(APPEND HEADER_FILES "OglVariable.inl")
	list(APPEND HEADER_FILES "PostProcessManager.h")
	list(APPEND HEADER_FILES "SlicedVolumetricModel.h")
	list(APPEND HEADER_FILES "VisualManagerPass.h")
	list(APPEND HEADER_FILES "VisualManagerSecondaryPass.h")

	list(APPEND SOURCE_FILES_UNITY "OglAttribute.cpp")
	list(APPEND SOURCE_FILES_UNITY "OglShader.cpp")
	list(APPEND SOURCE_FILES_UNITY "OglShaderMacro.cpp")
	list(APPEND SOURCE_FILES_UNITY "OglShaderVisualModel.cpp")
	list(APPEND SOURCE_FILES_UNITY "OglShadowShader.cpp")
	list(APPEND SOURCE_FILES_UNITY "OglTetrahedralModel.cpp")
	list(APPEND SOURCE_FILES_UNITY "OglTexture.cpp")
	list(APPEND SOURCE_FILES_UNITY "OglVariable.cpp")
	list(APPEND SOURCE_FILES_UNITY "PostProcessManager.cpp")
	list(APPEND SOURCE_FILES_OUTSIDE_UNITY "SlicedVolumetricModel.cpp")
	list(APPEND SOURCE_FILES_UNITY "VisualManagerPass.cpp")
	list(APPEND SOURCE_FILES_UNITY "VisualManagerSecondaryPass.cpp")
endif()

set(GROUP_BASE_DIR "..")



include( CMakeDependentOption )
cmake_dependent_option(UNITY_BUILD_${PROJECT_NAME} "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." ON  SOFA_ALLOW_UNITY_BUILDS OFF)

if( UNITY_BUILD_${PROJECT_NAME} )
  # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
  # source files.
  set( UNITYBUILD_INCLUDES "" )
  foreach(filename ${SOURCE_FILES_UNITY})
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
  set( SOURCE_FILES ${SOURCE_FILES_OUTSIDE_UNITY} UnityBuild.cpp )
else()
  set( SOURCE_FILES ${SOURCE_FILES_OUTSIDE_UNITY} ${SOURCE_FILES_UNITY} )
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaBaseVisual SofaSimulationCommon SofaGraphComponent )
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_SOURCE_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_DEPENDENCY_INCLUDE_DIR}>")

set( SOFAOPENGL_VISUAL_COMPILER_FLAGS "-DSOFA_BUILD_OPENGL_VISUAL")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFAGENERAL_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFAOPENGL_VISUAL_COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

install(TARGETS ${PROJECT_NAME}
        COMPONENT SofaGeneral_libraries
        EXPORT    SofaGeneralTargets
        RUNTIME DESTINATION "${SOFAGENERAL_INSTALL_BIN_DIR}"
        LIBRARY DESTINATION "${SOFAGENERAL_INSTALL_LIB_DIR}"
        ARCHIVE DESTINATION "${SOFAGENERAL_INSTALL_LIB_DIR}"
        PUBLIC_HEADER DESTINATION "${SOFAGENERAL_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}"

)

