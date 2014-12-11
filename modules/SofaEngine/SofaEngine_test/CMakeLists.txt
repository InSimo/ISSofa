cmake_minimum_required(VERSION 2.8)
project(SofaEngine_test)

set(SOURCE_FILES
    TestEngine_test.cpp
	TransformEngine_test.cpp
    )

include_directories(${gtest_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} gtest_main SofaEngine)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${PROJECT_NAME} COMMENT "-> Running unit tests..." VERBATIM)
