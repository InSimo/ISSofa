cmake_minimum_required(VERSION 3.1)
project(SofaComponentMisc)

set(HEADER_FILES
    config.h
    initComponentMisc.h
)

set(SOURCE_FILES
    initComponentMisc.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaMisc SofaMiscCollision SofaMiscEngine SofaMiscFem SofaMiscForceField SofaMiscMapping SofaMiscSolver SofaMiscTopology)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_COMPONENT_MISC")
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
endif()

sofa_install_targets(SofaMisc ${PROJECT_NAME} ${PROJECT_NAME})
