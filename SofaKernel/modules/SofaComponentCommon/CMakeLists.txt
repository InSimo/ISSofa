cmake_minimum_required(VERSION 3.1)
project(SofaComponentCommon)

set(HEADER_FILES
    config.h
    initComponentCommon.h
)

set(SOURCE_FILES
    initComponentCommon.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaEngine SofaEigen2Solver SofaDeformable SofaExplicitOdeSolver SofaImplicitOdeSolver SofaLoader SofaMeshCollision SofaObjectInteraction SofaRigid SofaSimpleFem)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_COMPONENT_COMMON")
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
endif()

sofa_install_targets(SofaCommon ${PROJECT_NAME} ${PROJECT_NAME})
