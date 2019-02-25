This document give instructions for CompressedRowSparseMatrix trace generation and benchmarking.

Generation : 

First of all, you need to actiave LogTrace Policy in CompressedRowSparseMatrix.cpp and do a compilation.

Now you are able to log calls of methods of CompressedRowSparseMatrix into binary files.
Choose the simulated object that you want to examinate. Associated solver need to be a BSCLDLSolver.
Into solver you can choose to log Mechanical Matrix (logMechanicalMatrixTrace) or Constraint Jacobian
Matrix (data logConstraintMatrixTrace). You have also to specify begin step with data 
logMatrixTraceStartStep and end step with data logMatrixTraceStopStep.

Binary files will be generated with a check matrix file where you launched runSofa. You have to copy it
into a subfolder of ISSofa/share/benchmarks/CRSRessources.


Run benchmark :

You have to activate cmake flag SOFA_ENABLE_BENCHMARKS.
 
Use CompressedRowSparseMatrixMechanical_bench.cpp or CompressedRowSparseMatrixConstraint.cpp according to
your matrix trace.

For help on benchmarks arguments, try CompressedRowSparseMatrixMechanical_bench.cpp --help.
