cmake_minimum_required(VERSION 2.8.12)

project(SofaTopologyMapping_test)

set(SOURCE_FILES
    Mesh2PointTopologicalMapping_test.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
include_directories(${gtest_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} gtest_main SofaTopologyMapping SofaSimulationTree SofaBaseMechanics)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${PROJECT_NAME} COMMENT "-> Running unit tests..." VERBATIM)
