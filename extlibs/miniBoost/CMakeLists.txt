cmake_minimum_required(VERSION 3.10)

project(SofaBoost LANGUAGES CXX)

add_library( miniBoost INTERFACE )

target_include_directories( miniBoost INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include> )

include(GNUInstallDirs)

install(TARGETS miniBoost
        EXPORT SofaBoostTargets
        INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

include(CMakePackageConfigHelpers)

configure_package_config_file( SofaBoostConfig.cmake.in
    "${CMAKE_BINARY_DIR}/SofaBoost/SofaBoostConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SofaBoost")

install(FILES ${CMAKE_BINARY_DIR}/SofaBoost/SofaBoostConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SofaBoost")
    
install(EXPORT SofaBoostTargets
    NAMESPACE "SofaBoost::"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SofaBoost")

if( NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/boost
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
            FILES_MATCHING 
                PATTERN "*.h"
                PATTERN "*.inl"
                PATTERN "*.hxx"
                PATTERN "*.hpp"
                PATTERN "*.ipp"
                PATTERN ".svn" EXCLUDE
            )
endif()

add_library( SofaBoost::miniBoost ALIAS miniBoost )
