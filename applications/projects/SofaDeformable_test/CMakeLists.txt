cmake_minimum_required(VERSION 2.8)

if (NOT SOFA-MISC_TESTS)
    message(STATUS "  > Unit tests are disabled, exiting")
    message(STATUS "")
    return()
endif()

project("SofaDeformable_test")

include(${SOFA_CMAKE_DIR}/preProject.cmake)

file(GLOB HEADER_FILES "*.h")
file(GLOB SOURCE_FILES "*.cpp")

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

if(SOFA-MISC_BUILD_GTEST OR WIN32)
    include_directories("${SOFA_EXTLIBS_DIR}/gtest/include")
endif()

AddCompilerDefinitions("SOFA_PROJECT_TEST")
AddLinkerDependencies(gtest gtest_main SofaDeformable)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${PROJECT_NAME} COMMENT "-> Running unit tests (project '${PROJECT_NAME}') ..." VERBATIM)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
