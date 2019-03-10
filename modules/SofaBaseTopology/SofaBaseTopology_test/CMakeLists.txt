cmake_minimum_required(VERSION 2.8.12)
project(SofaBaseTopology_test)

set( HEADER_FILES
  TopologyData_test.h
)

set( SOURCE_FILES
  PointSetTopology_test.cpp
  EdgeSetTopology_test.cpp
  TriangleSetTopology_test.cpp
  TopologyData_test.cpp
)

include_directories(${gtest_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} sofa_gtest_main SofaBaseTopology SofaSimulationTree)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${PROJECT_NAME} "--output-on-failure" COMMENT "-> Running unit tests..." VERBATIM)
