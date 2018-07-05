cmake_minimum_required(VERSION 2.8.12)
project("Sensable")


# Offer the user the choice of overriding the installation directories
set(SENSABLE_INSTALL_LIB_DIR     lib                        CACHE PATH "Installation directory for libraries")
set(SENSABLE_INSTALL_BIN_DIR     bin                        CACHE PATH "Installation directory for executables")
set(SENSABLE_INSTALL_INCLUDE_DIR include                    CACHE PATH "Installation directory for header files")
set(SENSABLE_INSTALL_CMAKE_DIR   ${SENSABLE_INSTALL_LIB_DIR}/cmake/Sensable )

set(HEADER_FILES
    EnslavementForceFeedback.h
    OmniDriver.h)

set(SOURCE_FILES
    initSensable.cpp
	EnslavementForceFeedback.cpp
    OmniDriver.cpp)

find_package(SofaGeneral REQUIRED)

find_package(OpenHaptics REQUIRED)

include_directories(${OPENHAPTICS_INCLUDE_DIR})
include_directories(${OPENHAPTICS_UTIL_INCLUDE_DIR})

set(SENSABLE_DEPENDENCIES
    SofaGuiQt SofaComponentGeneral SofaBaseCollision SofaHaptics ${OPENHAPTICS_HDU_LIBRARY} ${OPENHAPTICS_HD_LIBRARY} ${OPENHAPTICS_HL_LIBRARY})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${SENSABLE_DEPENDENCIES} )

set(SENSABLE_COMPILER_FLAGS "-DSOFA_BUILD_SENSABLEPLUGIN")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SENSABLE_COMPILER_FLAGS}"  )
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
set_target_properties( ${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}" )
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT    SensableTargets
        RUNTIME DESTINATION "${SENSABLE_INSTALL_BIN_DIR}" COMPONENT Sensable_libraries
        LIBRARY DESTINATION "${SENSABLE_INSTALL_LIB_DIR}" COMPONENT Sensable_libraries
        ARCHIVE DESTINATION "${SENSABLE_INSTALL_LIB_DIR}" COMPONENT Sensable_libraries
        PUBLIC_HEADER DESTINATION "${SENSABLE_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT Sensable_headers
)