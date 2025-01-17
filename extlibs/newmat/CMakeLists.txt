cmake_minimum_required(VERSION 2.8.12)

project(newmat)

set(NEWMAT_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(NEWMAT_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(NEWMAT_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(NEWMAT_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/Newmat)

# Offer the user the choice of overriding the installation directories
set(NEWMAT_INSTALL_LIB_DIR     ${CMAKE_INSTALL_LIBDIR}     CACHE PATH "Installation directory for libraries relative to CMAKE_INSTALL_PREFIX variable")
set(NEWMAT_INSTALL_BIN_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for executables relative to CMAKE_INSTALL_PREFIX variable")
set(NEWMAT_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files relative to CMAKE_INSTALL_PREFIX variable")
set(NEWMAT_INSTALL_CMAKE_DIR   ${NEWMAT_INSTALL_LIB_DIR}/cmake/Newmat )

set(HEADER_FILES

    newmat/include.h
    newmat/newmat.h
    newmat/newmatrc.h
    newmat/boolean.h
    newmat/myexcept.h
    newmat/controlw.h 
    newmat/newmatap.h
    newmat/newmatrm.h
    newmat/precisio.h 
	)

set(SOURCE_FILES

    newmat/newmat1.cpp
    newmat/newmat2.cpp
    newmat/newmat3.cpp
    newmat/newmat4.cpp
    newmat/newmat5.cpp
    newmat/newmat6.cpp
    newmat/newmat7.cpp
    newmat/newmat8.cpp
    newmat/newmat9.cpp
    newmat/newmatex.cpp
    newmat/newmatrm.cpp
    newmat/bandmat.cpp
    newmat/submat.cpp
    newmat/myexcept.cpp
    newmat/cholesky.cpp
    newmat/evalue.cpp
    newmat/fft.cpp
    newmat/hholder.cpp
    newmat/jacobi.cpp
    newmat/newfft.cpp
    newmat/sort.cpp
    newmat/svd.cpp 
	)

add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:${NEWMAT_INSTALL_INCLUDE_DIR}>")
target_compile_definitions(${PROJECT_NAME} PRIVATE "use_namespace")

# The code must be relocatable if we want to link a shared library against it
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
        OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE "-fPIC")
endif()

# disable annoying warnings from gcc
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_ARG1)
        # in the case of CXX="ccache g++"
        string(STRIP ${CMAKE_CXX_COMPILER_ARG1} CMAKE_CXX_COMPILER_ARG1_stripped)
        execute_process(COMMAND ${CMAKE_CXX_COMPILER_ARG1_stripped} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCXX_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
        execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCXX_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
    message("g++ version: ${GCXX_VERSION}")
    if(GCXX_VERSION VERSION_EQUAL 7 OR GCXX_VERSION VERSION_GREATER 7)
        target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-misleading-indentation" "-Wno-terminate")
    endif()
endif()

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

include(CMakePackageConfigHelpers)

# NewmatConfig.cmake (build tree)
set(NEWMAT_INCLUDE_DIR "${NEWMAT_BUILD_INCLUDE_DIR}")
configure_package_config_file(NewmatConfig.cmake.in
                              ${NEWMAT_BUILD_CMAKE_DIR}/NewmatConfig.cmake
                              INSTALL_DESTINATION ${NEWMAT_BUILD_CMAKE_DIR}
                              INSTALL_PREFIX ${CMAKE_BINARY_DIR}
                              PATH_VARS NEWMAT_INCLUDE_DIR)

# NewmatConfig.cmake (installed)
set(NEWMAT_INCLUDE_DIR "${NEWMAT_INSTALL_INCLUDE_DIR}")
configure_package_config_file(NewmatConfig.cmake.in
                              ${NEWMAT_BUILD_CMAKE_DIR}/NewmatConfigInstall.cmake
                              INSTALL_DESTINATION ${NEWMAT_INSTALL_CMAKE_DIR}
                              PATH_VARS NEWMAT_INCLUDE_DIR)

install(FILES ${NEWMAT_BUILD_CMAKE_DIR}/NewmatConfigInstall.cmake
        DESTINATION ${NEWMAT_INSTALL_CMAKE_DIR}
        RENAME NewmatConfig.cmake)

install(TARGETS ${PROJECT_NAME}
    EXPORT  NewmatTargets
    RUNTIME DESTINATION "${NEWMAT_INSTALL_BIN_DIR}" COMPONENT Newmat_libraries
    LIBRARY DESTINATION "${NEWMAT_INSTALL_LIB_DIR}" COMPONENT Newmat_libraries
    ARCHIVE DESTINATION "${NEWMAT_INSTALL_LIB_DIR}" COMPONENT Newmat_libraries
    PUBLIC_HEADER DESTINATION "${NEWMAT_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT Newmat_headers
    )
        
# NewmatTargets.cmake (installed)
install(EXPORT NewmatTargets
        DESTINATION ${NEWMAT_INSTALL_CMAKE_DIR} )

