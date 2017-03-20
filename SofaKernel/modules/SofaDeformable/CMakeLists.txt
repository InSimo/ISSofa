cmake_minimum_required(VERSION 3.1)
project(SofaDeformable)

set(HEADER_FILES
    MeshSpringForceField.h
    MeshSpringForceField.inl
    RestShapeSpringsForceField.h
    RestShapeSpringsForceField.inl
    SpringForceField.h
    SpringForceField.inl
    StiffSpringForceField.h
    StiffSpringForceField.inl
    config.h
    initDeformable.h
)

set(SOURCE_FILES
    MeshSpringForceField.cpp
    RestShapeSpringsForceField.cpp
    SpringForceField.cpp
    StiffSpringForceField.cpp
    initDeformable.cpp
)


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaBaseTopology)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_DEFORMABLE")
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
endif()

sofa_install_targets(SofaCommon ${PROJECT_NAME} ${PROJECT_NAME})
