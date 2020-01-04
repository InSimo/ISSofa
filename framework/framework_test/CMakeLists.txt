cmake_minimum_required(VERSION 2.8)

project(sofa_gtest_main)

set(MAIN_SOURCE_FILES
    main.cpp
    )

include_directories(${gtest_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} STATIC ${MAIN_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC gtest SofaHelper)

project(SofaFramework_test)

set(SOURCE_FILES
    defaulttype/CompressedRowSparseMatrixConstraintEigenUtils_test.cpp
    defaulttype/ConstraintSparseMatrix_test.cpp
    defaulttype/MapMapSparseMatrix_test.cpp
    defaulttype/MapMapSparseMatrixEigenUtils_test.cpp
    defaulttype/VecTypesTest.cpp
    #defaulttype/MatTypesTest.cpp
    defaulttype/RigidMass_test.cpp
    helper/DataParserRegistryTest.cpp
    core/CppDataParserTest.cpp
    core/JsonDataParserTest.cpp
    helper/map_ptr_stable_compare_test.cpp
    helper/FactoryTest.cpp
    helper/preprocessorTest.cpp
    helper/SSOBufferTest.cpp
    helper/system/atomicTest.cpp
    helper/system/FileSystemTest.cpp
    helper/system/UtilsTest.cpp
    #core/PluginTest.cpp
    core/objectmodel/AspectPoolTest.cpp
    core/objectmodel/DataTest.cpp
    core/objectmodel/DataTypeInfoTest.cpp
    core/objectmodel/PairTypeInfoTest.cpp
    core/objectmodel/StructTypeInfoTest.cpp
    core/objectmodel/EnumTypeInfoTest.cpp
    defaulttype/QuatTest.cpp
    defaulttype/UnitsTest.cpp
    defaulttype/DataMetadataTest.cpp
    #plugin-system/PluginSystemTest.cpp
    )

add_definitions("-DFRAMEWORK_TEST_RESOURCES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/resources\"")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} sofa_gtest_main SofaCore)
#add_dependencies(${PROJECT_NAME} PluginA PluginB PluginC PluginD PluginE PluginF)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${PROJECT_NAME} "--output-on-failure" COMMENT "-> Running unit tests..." VERBATIM)

#add_subdirectory(defaulttype)
