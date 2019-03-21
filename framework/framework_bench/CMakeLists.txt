cmake_minimum_required(VERSION 2.8.12)

if (POLICY CMP0076)
  # silence warning about interface sources path conversion to absolute
  cmake_policy(SET CMP0076 OLD)
endif()

project(SofaFramework_bench)

# 1 file = 1 executable
set(BENCHMARKS_FILES
    CompressedRowSparseMatrixConstraint_bench.cpp
    CompressedRowSparseMatrixMechanical_bench.cpp
   )

set(BENCHMARKS_HEADERS
    CompressedRowSparseMatrixHelper_bench.h
    CompressedRowSparseMatrixOld.h
   )

# Boost
find_package(Boost REQUIRED COMPONENTS filesystem QUIET)

add_library(${PROJECT_NAME} INTERFACE)
target_sources(${PROJECT_NAME} INTERFACE ${BENCHMARKS_HEADERS})

foreach(SOURCE_FILE ${BENCHMARKS_FILES})
    get_filename_component(TARGET_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${TARGET_NAME} ${SOURCE_FILE})
    target_link_libraries(${TARGET_NAME} SofaSparseSolver Boost::filesystem)
    install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${SOFAFRAMEWORK_INSTALL_BIN_DIR} COMPONENT SofaFramework_bench)
endforeach()

