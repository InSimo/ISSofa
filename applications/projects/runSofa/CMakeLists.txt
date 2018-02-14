cmake_minimum_required(VERSION 2.8)

# Offer the user the choice of overriding the installation directories
set(RUNSOFA_INSTALL_BIN_DIR     bin                      CACHE PATH "Installation directory for executables relative to CMAKE_INSTALL_PREFIX variable")

find_package(SofaGeneral REQUIRED)
find_package(SofaMisc    REQUIRED)

project(runSofa)

set(SOURCE_FILES

	Main.cpp
	)

#UseQt()

if(APPLE)
    set(RC_FILES "runSOFA.icns")
else()
    set(RC_FILES "sofa.rc")
endif()

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${RC_FILES})

# since "SofaComponentMain" does not exist any more we link to every components collection
# this is temporary, until every module is switched to plugins
target_link_libraries( ${PROJECT_NAME} SofaComponentBase SofaComponentCommon SofaComponentGeneral SofaComponentAdvanced SofaComponentMisc )

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAGENERAL_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFAMISC_BUILD_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")

# make the name of debug applications end in _d.
set_target_properties( ${PROJECT_NAME}  PROPERTIES DEBUG_POSTFIX "_d" )

install(TARGETS ${PROJECT_NAME}
        COMPONENT SofaApplications
        RUNTIME DESTINATION "${RUNSOFA_INSTALL_BIN_DIR}"
        ARCHIVE DESTINATION "${RUNSOFA_INSTALL_BIN_DIR}"
        )
