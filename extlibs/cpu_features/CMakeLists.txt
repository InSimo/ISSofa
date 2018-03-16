cmake_minimum_required(VERSION 3.0)

project(cpu_features VERSION 0.1.0)

set(CPU_FEATURES_BUILD_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CPU_FEATURES_BUILD_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CPU_FEATURES_BUILD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CPU_FEATURES_BUILD_CMAKE_DIR ${CMAKE_BINARY_DIR}/cpu_features/)

# Offer the user the choice of overriding the installation directories
set(CPU_FEATURES_INSTALL_LIB_DIR     lib     CACHE PATH "Installation directory for libraries")
set(CPU_FEATURES_INSTALL_BIN_DIR     bin     CACHE PATH "Installation directory for executables")
set(CPU_FEATURES_INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(CPU_FEATURES_INSTALL_CMAKE_DIR   ${CPU_FEATURES_INSTALL_LIB_DIR}/cmake/cpu_features)

# BUILD_TESTING is a standard CMake variable, but we declare it here to make it
# prominent in the GUI.
option(BUILD_TESTING "Enable test (depends on googletest)." OFF)
# BUILD_SHARED_LIBS is a standard CMake variable, but we declare it here to make
# it prominent in the GUI.
# The use of shared libraries is discouraged.
# For API / ABI compatibility reasons, it is recommended to build and use
# cpu_features in a subdirectory of your project or as an embedded dependency.
option(BUILD_SHARED_LIBS "Build library as shared." OFF)
#
# library : cpu_features
#

set(_HDRS
  include/cpuinfo_aarch64.h
  include/cpuinfo_arm.h
  include/cpuinfo_mips.h
  include/cpuinfo_x86.h
)

set(HEADER_FILES
  ${_HDRS}
  include/internal/bit_utils.h
  include/internal/linux_features_aggregator.h
  include/internal/cpuid_x86.h
  include/internal/filesystem.h
  include/internal/hwcaps.h
  include/internal/stack_line_reader.h
  include/internal/string_view.h
  include/cpu_features_macros.h
)

set(SOURCE_FILES
  src/linux_features_aggregator.c
  src/cpuid_x86_clang_gcc.c
  src/cpuid_x86_msvc.c
  src/cpuinfo_aarch64.c
  src/cpuinfo_arm.c
  src/cpuinfo_mips.c
  src/cpuinfo_x86.c
  src/filesystem.c
  src/hwcaps.c
  src/stack_line_reader.c
  src/string_view.c
)

add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})


target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
  include/internal
)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${_HDRS}")
target_compile_definitions(${PROJECT_NAME}
  PUBLIC STACK_LINE_READER_BUFFER_SIZE=1024)
target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_DL_LIBS})

# The code must be relocatable if we want to link a shared library against it
set_property(TARGET cpu_features PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(CpuFeature::cpu_features ALIAS cpu_features)

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )

#
# tests
#

include(CTest)
if(BUILD_TESTING)
  if(TARGET gtest OR TARGET gmock_main)
    add_subdirectory(test)
  endif()
endif()

#
# Install
#

install(TARGETS cpu_features
        EXPORT  cpu_featuresTargets
        RUNTIME DESTINATION "${CPU_FEATURES_INSTALL_BIN_DIR}" 
        LIBRARY DESTINATION "${CPU_FEATURES_INSTALL_LIB_DIR}" 
        ARCHIVE DESTINATION "${CPU_FEATURES_INSTALL_LIB_DIR}" 
        PUBLIC_HEADER DESTINATION "${CPU_FEATURES_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" )
include(CMakePackageConfigHelpers)

# CpuFeaturesConfig.cmake (build tree)
set(CPU_FEATURES_INCLUDE_DIR "${CPU_FEATURES_BUILD_INCLUDE_DIR}")
configure_package_config_file(CpuFeaturesConfig.cmake.in 
                              ${CPU_FEATURES_BUILD_CMAKE_DIR}/CpuFeaturesConfig.cmake
                              INSTALL_DESTINATION ${CPU_FEATURES_BUILD_CMAKE_DIR}
                              PATH_VARS CPU_FEATURES_INCLUDE_DIR)

# CpuFeaturesConfig.cmake (installed)
set(CPU_FEATURES_INCLUDE_DIR "${CPU_FEATURES_INSTALL_INCLUDE_DIR}")
configure_package_config_file(CpuFeaturesConfig.cmake.in 
                              ${CPU_FEATURES_BUILD_CMAKE_DIR}/CpuFeaturesConfigInstall.cmake
                              INSTALL_DESTINATION ${CPU_FEATURES_INSTALL_CMAKE_DIR}
                              PATH_VARS CPU_FEATURES_INCLUDE_DIR)

install(FILES ${CPU_FEATURES_BUILD_CMAKE_DIR}/CpuFeaturesConfigInstall.cmake
        DESTINATION ${CPU_FEATURES_INSTALL_CMAKE_DIR}
        RENAME CpuFeaturesConfig.cmake)

# CpuFeaturesTargets.cmake (installed)
install(EXPORT cpu_featuresTargets
        DESTINATION ${CPU_FEATURES_INSTALL_CMAKE_DIR})
