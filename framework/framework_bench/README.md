This document give instructions for CompressedRowSparseMatrix trace generation and benchmarking.

Generation : 

First of all, you need to activate the LogTrace Policy in CompressedRowSparseMatrix.h and do a compilation.

Now you are able to log calls of methods of CompressedRowSparseMatrix into binary files.
Choose the simulated object that you want to examine. The associated solver needs to be a BSCLDLSolver.
Into solver you can choose to log Mechanical Matrix (logMechanicalMatrixTrace) or Constraint Jacobian
Matrix (data logConstraintMatrixTrace). You have also to specify the begin step with the data 
logMatrixTraceStartStep and the end step with the data logMatrixTraceStopStep.

Binary files will be generated with a check matrix file where you launched runSofa. You have to copy it
into a subfolder of ISSofa/share/benchmarks/CRSResources.
Optionally, you can compress them with the gzip command to reduce disk usage before committing them.


Run benchmark :

You have to activate the cmake flag SOFA_ENABLE_BENCHMARKS.
 
Use CompressedRowSparseMatrixMechanical_bench.cpp or CompressedRowSparseMatrixConstraint.cpp according to
your matrix trace.

For help on benchmarks arguments, try CompressedRowSparseMatrixMechanical_bench.cpp --help.
