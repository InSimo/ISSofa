cmake_minimum_required(VERSION 3.1)
project(SofaMiscEngine)

set(HEADER_FILES
    DisplacementMatrixEngine.h
    DisplacementMatrixEngine.inl
    Distances.h
    Distances.inl
    ProjectiveTransformEngine.h
    ProjectiveTransformEngine.inl
    config.h
    initMiscEngine.h
)

set(SOURCE_FILES
    DisplacementMatrixEngine.cpp
    Distances.cpp
    ProjectiveTransformEngine.cpp
    initMiscEngine.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaNonUniformFem)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_MISC_ENGINE")
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
endif()

sofa_install_targets(SofaMisc ${PROJECT_NAME} ${PROJECT_NAME})
