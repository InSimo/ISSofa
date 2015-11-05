cmake_minimum_required(VERSION 2.8.12)
project(SofaBaseTopology_test)

set( SOURCE_FILES
  PointSetTopology_test.cpp
)

include_directories(${gtest_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} gtest_main SofaBaseTopology)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${PROJECT_NAME} COMMENT "-> Running unit tests..." VERBATIM)
