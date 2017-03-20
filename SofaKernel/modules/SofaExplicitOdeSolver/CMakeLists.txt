cmake_minimum_required(VERSION 3.1)
project(SofaExplicitOdeSolver)

set(HEADER_FILES
    EulerSolver.h
    config.h
    initExplicitODESolver.h
)

set(SOURCE_FILES
    EulerSolver.cpp
    initExplicitODESolver.cpp
)


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaSimulationTree)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_EXPLICIT_ODE_SOLVER")
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
endif()

sofa_install_targets(SofaCommon ${PROJECT_NAME} ${PROJECT_NAME})
