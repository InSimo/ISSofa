cmake_minimum_required(VERSION 3.0)

project(SofaExampleDDGNodeMT)

find_package(SofaFramework REQUIRED)

set(SofaExampleDDGNodeMT_DEPENDENCY_INCLUDE_DIR ${SofaFramework_INCLUDE_DIRS})
set(SofaExampleDDGNodeMT_DEPENDENCY_LIBS SofaCore)

set(HEADER_FILES
    AccumulateDataEngine.h
)

set(SOURCE_FILES
    AccumulateDataEngine.cpp
    Main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries( ${PROJECT_NAME} ${SofaExampleDDGNodeMT_DEPENDENCY_LIBS} ) 

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SofaExampleDDGNodeMT_DEPENDENCY_INCLUDE_DIR}>")


# make the name of debug applications end in _d.
set_target_properties( ${PROJECT_NAME}  PROPERTIES DEBUG_POSTFIX "_d" )

