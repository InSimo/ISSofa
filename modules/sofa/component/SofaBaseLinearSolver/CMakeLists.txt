include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    ../initBaseLinearSolver.h 
    ../linearsolver/CGLinearSolver.h 
    ../linearsolver/CGLinearSolver.inl 
    ../linearsolver/CholeskySolver.h 
    ../linearsolver/CholeskySolver.inl 
    ../linearsolver/BTDLinearSolver.h 
    ../linearsolver/BTDLinearSolver.inl 
    ../linearsolver/FullVector.h 
    ../linearsolver/FullMatrix.h 
    ../linearsolver/DiagonalMatrix.h 
    ../linearsolver/SingleMatrixAccessor.h 
    ../linearsolver/SparseMatrix.h 
    ../linearsolver/CompressedRowSparseMatrix.h 
    ../linearsolver/CompressedRowSparseMatrix.inl 
    ../linearsolver/GraphScatteredTypes.h 
    ../linearsolver/DefaultMultiMatrixAccessor.h 
    ../linearsolver/MatrixLinearSolver.h 
    ../linearsolver/MatrixLinearSolver.inl
    ../linearsolver/MatrixExpr.h 
    ../linearsolver/matrix_bloc_traits.h 
    ../linearsolver/BlocMatrixWriter.h
    ../linearsolver/MinResLinearSolver.h
    ../linearsolver/MinResLinearSolver.inl

    )
    
set(SOURCE_FILES

    ../initBaseLinearSolver.cpp 
    ../linearsolver/CGLinearSolver.cpp 
    ../linearsolver/CholeskySolver.cpp 
    ../linearsolver/BTDLinearSolver.cpp 
    ../linearsolver/FullVector.cpp 
    ../linearsolver/GraphScatteredTypes.cpp 
    ../linearsolver/DefaultMultiMatrixAccessor.cpp 
    ../linearsolver/MatrixLinearSolver.cpp 
    ../linearsolver/SingleMatrixAccessor.cpp
    ../linearsolver/MinResLinearSolver.cpp

    )

set(GROUP_BASE_DIR "..")



include( CMakeDependentOption )
cmake_dependent_option(UNITY_BUILD_${PROJECT_NAME} "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." ON  SOFA_ALLOW_UNITY_BUILDS OFF)

if( UNITY_BUILD_${PROJECT_NAME} )
  # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
  # source files.
  set( UNITYBUILD_INCLUDES "" )
  foreach(filename ${SOURCE_FILES})
    SET( UNITYBUILD_INCLUDES "${UNITYBUILD_INCLUDES}#include \"${filename}\"\n" )
  endforeach(filename)

  file(WRITE UnityBuild.cpp ${UNITYBUILD_INCLUDES} )

  # overwrite the source files to only include the generated unity build file.
  set( SOURCE_FILES UnityBuild.cpp )
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

AddCompilerDefinitions("SOFA_BUILD_BASE_LINEAR_SOLVER")
AddLinkerDependencies(SofaSimulationTree)

include(${SOFA_CMAKE_DIR}/postProject.cmake)
