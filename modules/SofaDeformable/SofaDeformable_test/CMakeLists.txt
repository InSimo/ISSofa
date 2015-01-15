cmake_minimum_required(VERSION 2.8)

project("SofaDeformable_test")

file(GLOB HEADER_FILES "*.h")
file(GLOB SOURCE_FILES "*.cpp")

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC ${gtest_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} gtest gtest_main SofaDeformable)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${PROJECT_NAME} COMMENT "-> Running unit tests (project '${PROJECT_NAME}') ..." VERBATIM)
