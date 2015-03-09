cmake_minimum_required(VERSION 2.8.12)

project(csparse)

set(CSPARSE_BUILD_LIB_DIR     ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CSPARSE_BUILD_BIN_DIR     ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CSPARSE_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(CSPARSE_BUILD_CMAKE_DIR   ${CMAKE_BINARY_DIR}/Csparse)

# Offer the user the choice of overriding the installation directories
set(CSPARSE_INSTALL_LIB_DIR     lib                      CACHE PATH "Installation directory for libraries relative to CMAKE_INSTALL_PREFIX variable")
set(CSPARSE_INSTALL_BIN_DIR     bin                      CACHE PATH "Installation directory for executables relative to CMAKE_INSTALL_PREFIX variable")
set(CSPARSE_INSTALL_INCLUDE_DIR include/csparse          CACHE PATH "Installation directory for header files relative to CMAKE_INSTALL_PREFIX variable")
set(CSPARSE_INSTALL_CMAKE_DIR   ${CSPARSE_INSTALL_LIB_DIR}/cmake/Csparse )


set(HEADER_FILES

    csparse.h 
    ldl.h 
    UFconfig.h 
	)

set(SOURCE_FILES

    csparse.c 
    ldl.c 
	)

add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:${CSPARSE_INSTALL_INCLUDE_DIR}>")

# The code must be relocatable if we want to link a shared library against it
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU"
        OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE "-fPIC")
endif()


set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}"  )

include(CMakePackageConfigHelpers)

# CsparseConfig.cmake (build tree)
set(CSPARSE_INCLUDE_DIR "${CSPARSE_BUILD_INCLUDE_DIR}")
configure_package_config_file(CsparseConfig.cmake.in
                              ${CSPARSE_BUILD_CMAKE_DIR}/CsparseConfig.cmake
                              INSTALL_DESTINATION ${CSPARSE_BUILD_CMAKE_DIR}
                              PATH_VARS CSPARSE_INCLUDE_DIR)

# CsparseConfig.cmake (installed)
set(CSPARSE_INCLUDE_DIR "${CSPARSE_INSTALL_INCLUDE_DIR}")
configure_package_config_file(CsparseConfig.cmake.in
                              ${CSPARSE_BUILD_CMAKE_DIR}/CsparseConfigInstall.cmake
                              INSTALL_DESTINATION ${CSPARSE_INSTALL_CMAKE_DIR}
                              PATH_VARS CSPARSE_INCLUDE_DIR)

install(FILES ${CSPARSE_BUILD_CMAKE_DIR}/CsparseConfigInstall.cmake
        DESTINATION ${CSPARSE_INSTALL_CMAKE_DIR}
        RENAME CsparseConfig.cmake)

install(TARGETS ${PROJECT_NAME}
    EXPORT  CsparseTargets
    RUNTIME DESTINATION "${CSPARSE_INSTALL_BIN_DIR}" COMPONENT Csparse_libraries
    LIBRARY DESTINATION "${CSPARSE_INSTALL_LIB_DIR}" COMPONENT Csparse_libraries
    ARCHIVE DESTINATION "${CSPARSE_INSTALL_LIB_DIR}" COMPONENT Csparse_libraries
    PUBLIC_HEADER DESTINATION "${CSPARSE_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT Csparse_headers
    )
        
# CsparseTargets.cmake (installed)
install(EXPORT CsparseTargets
        DESTINATION ${CSPARSE_INSTALL_CMAKE_DIR} )


