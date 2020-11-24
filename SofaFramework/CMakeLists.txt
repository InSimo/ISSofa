cmake_minimum_required(VERSION 2.8.12)
if(POLICY CMP0074)
cmake_policy(SET CMP0074 NEW)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include(CMakeSofaCreateTarget)

set(SOLUTION_NAME "SofaFramework")
project(${SOLUTION_NAME})

# Temporary variable until we move the sub projects
get_filename_component(SofaFramework_PARENT_SOURCE_DIR ${SofaFramework_SOURCE_DIR} DIRECTORY)
set( SofaFramework_ACTUAL_SOURCE_DIR ${SofaFramework_PARENT_SOURCE_DIR}/framework)
set(SOFAFRAMEWORK_SOURCE_INCLUDE_DIR ${SofaFramework_ACTUAL_SOURCE_DIR})

##
## SofaFramework options
##
option(SOFAFRAMEWORK_DETECTIONOUTPUT_FREEMOTION      "Activate DETECTIONOUTPUT_FREEMOTION option" OFF)
option(SOFAFRAMEWORK_DETECTIONOUTPUT_BARYCENTRICINFO "Activate DETECTIONOUTPUT_BARYCENTRICINFO option" OFF)
option(SOFAFRAMEWORK_CHECK_CONTAINER_ACCESS  "Compile SofaHelper library with with SOFA_CONTAINER_ACCESS_FAILURE preprocessor definition" OFF)

option(SOFAFRAMEWORK_USECRSCONSTRAINT                 "Activate SOFA_USECRSCONSTRAINT option" ON)
option(SOFAFRAMEWORK_CRS_POLICY_CHECK                 "Activate check policy flag for CRSMatrix" OFF)
option(SOFAFRAMEWORK_CRS_POLICY_VERBOSE               "Activate verbose policy flag for CRSMatrix" OFF)
option(SOFAFRAMEWORK_CRS_POLICY_LOGTRACE              "Activate log trace policy flag for CRSMatrix" OFF)
option(SOFAFRAMEWORK_CRS_POLICY_PRINTTRACE            "Activate print trace policy flag for CRSMatrix" OFF)

option(SOFAFRAMEWORK_MAPPING_APPLY_HANDLETOPOLOGYCHANGE  "Enable apply from mappings in handleTopologyChange method" OFF)

set(DETECTIONOUTPUT_FREEMOTION ${SOFAFRAMEWORK_DETECTIONOUTPUT_FREEMOTION} )
set(DETECTIONOUTPUT_BARYCENTRICINFO ${SOFAFRAMEWORK_DETECTIONOUTPUT_BARYCENTRICINFO} )

set(SOFA_USECRSCONSTRAINT ${SOFAFRAMEWORK_USECRSCONSTRAINT} )
set(SOFA_CRS_POLICY_CHECK ${SOFAFRAMEWORK_CRS_POLICY_CHECK} )
set(SOFA_CRS_POLICY_VERBOSE ${SOFAFRAMEWORK_CRS_POLICY_VERBOSE} )
set(SOFA_CRS_POLICY_LOGTRACE ${SOFAFRAMEWORK_CRS_POLICY_LOGTRACE} )
set(SOFA_CRS_POLICY_PRINTTRACE ${SOFAFRAMEWORK_CRS_POLICY_PRINTTRACE} )

set(SOFA_MAPPING_APPLY_HANDLETOPOLOGYCHANGE  ${SOFAFRAMEWORK_MAPPING_APPLY_HANDLETOPOLOGYCHANGE})

# Assertion level
set(SOFA_ASSERT_LEVEL 2 CACHE STRING "Level of runtime assertion in Debug and RelWithDebInfo (0=no assertion, 1=fast assertions, 2=all assertions).")

set(SOFA_FAST_ASSERTION_ENABLED FALSE CACHE INTERNAL "")
set(SOFA_ASSERTION_ENABLED      FALSE CACHE INTERNAL "")

if (SOFA_ASSERT_LEVEL EQUAL 1)
    set(SOFA_FAST_ASSERTION_ENABLED TRUE CACHE INTERNAL "")
elseif(SOFA_ASSERT_LEVEL EQUAL 2)
    set(SOFA_ASSERTION_ENABLED      TRUE CACHE INTERNAL "")
endif()


##
## SofaFramework version
##
set( SOFAFRAMEWORK_MAJOR_VERSION 0 )
set( SOFAFRAMEWORK_MINOR_VERSION 0 )
set( SOFAFRAMEWORK_BUILD_VERSION 1 )

set( SOFAFRAMEWORK_VERSION ${SOFAFRAMEWORK_MAJOR_VERSION}.${SOFAFRAMEWORK_MINOR_VERSION}.${SOFAFRAMEWORK_BUILD_VERSION} CACHE INTERNAL "Set to internal so it can be changed by sub directories.")

set( SOFAFRAMEWORK_EXTLIBS_DIR "${SofaFramework_PARENT_SOURCE_DIR}/extlibs" CACHE PATH "")

set(SOFAFRAMEWORK_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(SOFAFRAMEWORK_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(SOFAFRAMEWORK_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(SOFAFRAMEWORK_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/SofaFramework)

# Offer the user the choice of overriding the installation directories
set(SOFAFRAMEWORK_INSTALL_LIB_DIR     ${CMAKE_INSTALL_LIBDIR}     CACHE PATH "Installation directory for libraries relative to CMAKE_INSTALL_PREFIX variable")
set(SOFAFRAMEWORK_INSTALL_BIN_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for executables relative to CMAKE_INSTALL_PREFIX variable")
set(SOFAFRAMEWORK_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files relative to CMAKE_INSTALL_PREFIX variable")
set(SOFAFRAMEWORK_INSTALL_CMAKE_DIR   ${SOFAFRAMEWORK_INSTALL_LIB_DIR}/cmake/SofaFramework  )

set(SOFAFRAMEWORK_DEPENDENCY_LINK)
set(SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIRECTORIES)

##
## Dependencies lookup 
##
# OpenGL ( glu )
find_package(OpenGL REQUIRED)
# no target support for OpenGL on windows yet see https://github.com/Kitware/CMake/commit/02e34de2b6c27ee5d367be4fa280ffd3ad1965ba
# it is not too harmful since this particular library gives symbolic names that can be consumed upon install on the target system.
list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK ${OPENGL_LIBRARIES})
list(APPEND SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIRECTORIES ${OPENGL_INCLUDE_DIR})

#tinyxml
find_package(TinyXML REQUIRED)
list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK tinyxml )
set(SOFA_HAVE_TINYXML 1 CACHE INTERNAL "")

#zlib
find_package(ZLIB)
if(ZLIB_FOUND)
    list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK ZLIB::ZLIB )
endif()
set(SOFA_HAVE_ZLIB ${ZLIB_FOUND} CACHE INTERNAL "")

# PNG
find_package(PNG)
if( PNG_FOUND)
	list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK PNG::PNG )
endif()
set(SOFA_HAVE_PNG ${PNG_FOUND} CACHE INTERNAL "")

# GLEW
find_package(GLEW)
if( GLEW_FOUND)
	list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK GLEW::GLEW )
endif()
set(SOFA_HAVE_GLEW ${GLEW_FOUND} CACHE INTERNAL "")

# GLUT
find_package(GLUT)
if( GLUT_FOUND )
	list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK GLUT::GLUT )
endif()
set(SOFA_HAVE_GLUT ${GLUT_FOUND} CACHE INTERNAL "")

# X11
if(UNIX AND NOT APPLE)
    find_package(X11 REQUIRED)
    sofa_create_target(X11 SofaFramework  "${X11_X11_LIB}" "${X11_INCLUDE_DIR}" )
    list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK ${X11_Target} )
    set(SOFA_HAVE_X11 ${X11_FOUND} CACHE INTERNAL "")
else()
    set(SOFA_HAVE_X11 FALSE CACHE INTERNAL "")
endif()


set(SOFA_HAVE_BOOST FALSE CACHE INTERNAL "")

if(SOFA_USE_MINIBOOST)
    find_package(SofaBoost COMPONENTS miniBoost REQUIRED)
    find_package(Threads REQUIRED)
    list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK SofaBoost::miniBoost Threads::Threads)
else()
    # Boost
    find_package(Boost COMPONENTS thread system date_time chrono REQUIRED)

    set(SOFA_HAVE_BOOST_SYSTEM ${Boost_SYSTEM_FOUND} )
    set(SOFA_HAVE_BOOST_THREAD ${Boost_THREAD_FOUND} )
    set(SOFA_HAVE_BOOST_DATE_TIME ${Boost_DATE_TIME_FOUND} )
    set(SOFA_HAVE_BOOST_CHRONO ${Boost_CHRONO_FOUND} )

    list(APPEND Boost_INCLUDE_DIRS ${Boost_INCLUDE_DIR} )

    if(SOFA_HAVE_BOOST_SYSTEM AND 
       SOFA_HAVE_BOOST_THREAD AND 
       SOFA_HAVE_BOOST_DATE_TIME AND 
       SOFA_HAVE_BOOST_CHRONO)
        set(SOFA_HAVE_BOOST TRUE CACHE INTERNAL "")
        list(APPEND SOFAFRAMEWORK_DEPENDENCY_LINK Boost::system Boost::thread Boost::date_time Boost::chrono)
    endif()
endif()

#Eigen
find_package(Eigen3 REQUIRED)
list(APPEND SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
set(SOFA_HAVE_EIGEN2 1 CACHE INTERNAL "")

#Rapidjson
list(APPEND SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIRECTORIES "${SOFAFRAMEWORK_EXTLIBS_DIR}/rapidjson/include")
##
##
##

set(SOFA_HAVE_EASTL ${SOFA_ENABLE_EASTL} CACHE INTERNAL "")

set(SOFA_HAVE_OPTIONAL ${SOFA_OPTIONAL} CACHE INTERNAL "")

configure_file( ${SofaFramework_SOURCE_DIR}/SofaFramework.h.in
                ${CMAKE_CURRENT_BINARY_DIR}/sofa/SofaFramework.h )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/sofa/SofaFramework.h
         DESTINATION "${SOFAFRAMEWORK_INSTALL_INCLUDE_DIR}/sofa")
# make sure everyone in the build tree can see <sofa/SofaFramework.h>
list(APPEND SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR})

list(REMOVE_DUPLICATES SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIRECTORIES)

add_subdirectory( ${SofaFramework_ACTUAL_SOURCE_DIR}/sofa/helper      ${CMAKE_CURRENT_BINARY_DIR}/SofaHelper )
add_subdirectory( ${SofaFramework_ACTUAL_SOURCE_DIR}/sofa/defaulttype ${CMAKE_CURRENT_BINARY_DIR}/SofaDefaultType )
add_subdirectory( ${SofaFramework_ACTUAL_SOURCE_DIR}/sofa/core        ${CMAKE_CURRENT_BINARY_DIR}/SofaCore )
if(TARGET gtest)
    add_subdirectory( ${SofaFramework_ACTUAL_SOURCE_DIR}/framework_test        ${CMAKE_CURRENT_BINARY_DIR}/framework_test )
endif()
if(SOFA_ENABLE_BENCHMARKS)
    add_subdirectory(${SofaFramework_ACTUAL_SOURCE_DIR}/framework_bench        ${CMAKE_CURRENT_BINARY_DIR}/framework_bench )
endif()

# set IDE project filter
if(MSVC)
    #message("${PROJECT_NAME} in ${GLOBAL_PROJECT_OPTION_FOLDER_${PROJECT_NAME}}")
    set_target_properties(SofaHelper      PROPERTIES FOLDER "SofaFramework")
    set_target_properties(SofaDefaultType PROPERTIES FOLDER "SofaFramework")
    set_target_properties(SofaCore        PROPERTIES FOLDER "SofaFramework")
endif()

include(CMakePackageConfigHelpers)

# SofaFrameworkConfigVersion.cmake file (same for build tree and intall)
write_basic_package_version_file(${SOFAFRAMEWORK_BUILD_CMAKE_DIR}/SofaFrameworkConfigVersion.cmake
                                 VERSION ${SOFAFRAMEWORK_VERSION}
                                 COMPATIBILITY AnyNewerVersion)
install(FILES ${SOFAFRAMEWORK_BUILD_CMAKE_DIR}/SofaFrameworkConfigVersion.cmake
        DESTINATION ${SOFAFRAMEWORK_INSTALL_CMAKE_DIR})


# SofaFrameworkConfig.cmake (build tree)
set(SOFAFRAMEWORK_INCLUDE_DIR "${SOFAFRAMEWORK_BUILD_INCLUDE_DIR}")
configure_package_config_file(SofaFrameworkConfig.cmake.in
                              ${SOFAFRAMEWORK_BUILD_CMAKE_DIR}/SofaFrameworkConfig.cmake
                              INSTALL_DESTINATION ${SOFAFRAMEWORK_BUILD_CMAKE_DIR}
                              INSTALL_PREFIX ${CMAKE_BINARY_DIR}
                              PATH_VARS SOFAFRAMEWORK_INCLUDE_DIR)

# SofaFrameworkConfig.cmake (installed)
set(SOFAFRAMEWORK_INCLUDE_DIR "${SOFAFRAMEWORK_INSTALL_INCLUDE_DIR}")
configure_package_config_file(SofaFrameworkConfig.cmake.in
                              ${SOFAFRAMEWORK_BUILD_CMAKE_DIR}/SofaFrameworkConfigInstall.cmake
                              INSTALL_DESTINATION ${SOFAFRAMEWORK_INSTALL_CMAKE_DIR}
                              PATH_VARS SOFAFRAMEWORK_INCLUDE_DIR)

install(FILES ${SOFAFRAMEWORK_BUILD_CMAKE_DIR}/SofaFrameworkConfigInstall.cmake
        DESTINATION ${SOFAFRAMEWORK_INSTALL_CMAKE_DIR}
        RENAME SofaFrameworkConfig.cmake)

# CMakeSofaCreateTarget (build)
configure_file( ${SofaFramework_SOURCE_DIR}/CMakeSofaCreateTarget.cmake
                ${SOFAFRAMEWORK_BUILD_CMAKE_DIR}/CMakeSofaCreateTarget.cmake COPYONLY)
        
# CMakeSofaCreateTarget (installed)
install(FILES ${SofaFramework_SOURCE_DIR}/CMakeSofaCreateTarget.cmake 
        DESTINATION ${SOFAFRAMEWORK_INSTALL_CMAKE_DIR} )

# CMakeParseLibraryList (build)
configure_file( ${SofaFramework_SOURCE_DIR}/CMakeParseLibraryList.cmake
                ${SOFAFRAMEWORK_BUILD_CMAKE_DIR}/CMakeParseLibraryList.cmake COPYONLY)
        
# CMakeParseLibraryList (installed)
install(FILES ${SofaFramework_SOURCE_DIR}/CMakeParseLibraryList.cmake 
        DESTINATION ${SOFAFRAMEWORK_INSTALL_CMAKE_DIR} )
        
# SofaFrameworkTargets.cmake (installed)
install(EXPORT SofaFrameworkTargets
        DESTINATION ${SOFAFRAMEWORK_INSTALL_CMAKE_DIR} )

# compatibility with find_package
set(SofaFramework_LIBRARIES SofaHelper SofaDefaultType SofaCore CACHE INTERNAL "")
set(SOFAFRAMEWORK_INCLUDE_DIR "$<BUILD_INTERFACE:${SOFAFRAMEWORK_BUILD_INCLUDE_DIR}>$<INSTALL_INTERFACE:${SOFAFRAMEWORK_INSTALL_INCLUDE_DIR}>" CACHE INTERNAL "")
set(SofaFramework_INCLUDE_DIRS ${SOFAFRAMEWORK_INCLUDE_DIR} ${SOFAFRAMEWORK_DEPENDENCY_INCLUDE_DIR} CACHE INTERNAL "")
