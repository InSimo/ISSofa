cmake_minimum_required(VERSION 2.8.8)
project(SofaDefaultType)


set(SOFADEFAULTTYPE_HEADERS
	AbstractTypeInfo.h
	BoundingBox.h	
	DataTypeInfo.h	
	EnumTypeInfo.h	
	DataTypeKind.h	
	Frame.h	 
	LaparoscopicRigidTypes.h	
	MapMapSparseMatrix.h	
	Mat.h	
	Mat_solve_LU.h	
	Mat_solve_SVD.h	
	Mat_solve_Cholesky.h	
	Quat.h	
	Quat.inl	
	#RigidInertia.h	
	#RigidInertia.inl	
	RigidTypes.h	
	RigidVec6Types.h	
	SolidTypes.h	
	SolidTypes.inl	
	StructTypeInfo.h
	#SparseConstraintTypes.h	
	TemplatesAliases.h
	Vec.h	
	VecTypes.h	
	MatSym.h	
	Vec3Types.h
	VirtualTypeInfo.h
	)

set(SOFADEFAULTTYPE_SOURCES
	AbstractTypeInfo.cpp
	BoundingBox.cpp
	Frame.cpp
	#RigidInertia.cpp
	SolidTypes.cpp
	TemplatesAliases.cpp
	)
  


include( CMakeDependentOption )
cmake_dependent_option(UNITY_BUILD_${PROJECT_NAME} "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." ON  SOFA_ALLOW_UNITY_BUILDS OFF)

if( UNITY_BUILD_${PROJECT_NAME} )
  # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
  # source files.
  set( UNITYBUILD_INCLUDES "" )
  foreach(filename ${SOFADEFAULTTYPE_SOURCES})
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
  set( SOFADEFAULTTYPE_SOURCES UnityBuild.cpp )
endif()

add_library(${PROJECT_NAME} SHARED ${SOFADEFAULTTYPE_HEADERS} ${SOFADEFAULTTYPE_SOURCES})
target_link_libraries( ${PROJECT_NAME} PUBLIC SofaHelper )
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAFRAMEWORK_SOURCE_INCLUDE_DIR}>")
set( SOFADEFAULTTYPE_COMPILER_FLAGS "-DSOFA_BUILD_DEFAULTTYPE")

# make the name of debug libraries end in _d.
set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFAFRAMEWORK_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFADEFAULTTYPE_COMPILER_FLAGS}"  )

install(TARGETS ${PROJECT_NAME} 
    COMPONENT SofaFramework_libraries
    EXPORT  SofaFrameworkTargets
    RUNTIME DESTINATION "${SOFAFRAMEWORK_INSTALL_BIN_DIR}"
    LIBRARY DESTINATION "${SOFAFRAMEWORK_INSTALL_LIB_DIR}"
    ARCHIVE DESTINATION "${SOFAFRAMEWORK_INSTALL_LIB_DIR}"
    )

if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
install(DIRECTORY ${SofaDefaultType_SOURCE_DIR}/ 
        DESTINATION "${SOFAFRAMEWORK_INSTALL_INCLUDE_DIR}/sofa/defaulttype"
        COMPONENT SofaFramework_headers
        FILES_MATCHING 
            PATTERN "*.h"
            PATTERN "*.inl"
            PATTERN "*.hxx"
            PATTERN "*.hpp"
       )
endif()
