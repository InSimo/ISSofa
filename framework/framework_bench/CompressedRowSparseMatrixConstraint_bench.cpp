// This is benchmarking code, enforce no debug code and no container access checks in this binary
#ifdef SOFA_CHECK_CONTAINER_ACCESS
#undef SOFA_CHECK_CONTAINER_ACCESS
#endif
#ifndef NDEBUG
#define NDEBUG
#endif

#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/FileSystem.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/thread/CTime.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/defaulttype/MapMapSparseMatrix.h>

#include <chrono>

using namespace sofa::defaulttype;
using namespace sofa::helper::system::thread;

/// Number of execution to pre load CPU
static constexpr std::size_t nPreload = 5u;

/// Specific policy for benchmark on CRSMatrixConstraint
class CRSBenchConstraintPolicyA : public CRSConstraintPolicy
{
public:

    static constexpr bool OrderedInsertion = false;
    static constexpr bool CompressZeros = true;
    static constexpr bool ClearZeros = true;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

/// Specific policy for benchmark on CRSMatrixConstraint
class CRSBenchConstraintPolicyB : public CRSConstraintPolicy
{
public:

    static constexpr bool OrderedInsertion = false;
    static constexpr bool CompressZeros = false;
    static constexpr bool ClearZeros = false;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

/// Specific policy for benchmark on CRSMatrixConstraint
class CRSBenchConstraintPolicyC : public CRSConstraintPolicy
{
public:  

    static constexpr bool OrderedInsertion = true;
    static constexpr bool CompressZeros = true;
    static constexpr bool ClearZeros = true;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

/// Specific policy for benchmark on CRSMatrixConstraint
class CRSBenchConstraintPolicyD : public CRSConstraintPolicy
{
public:

    static constexpr bool OrderedInsertion = true;
    static constexpr bool CompressZeros = false;
    static constexpr bool ClearZeros = false;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

template <class TMapMap, class TBloc>
void callMapMapFn(int fnId, TMapMap& mat, unsigned int rowId, unsigned int colId, const TBloc& bloc)
{
    switch (fnId)
    {
        case FnEnum::addCol          : mat.writeLine(rowId).addCol(colId, bloc); break;
        case FnEnum::setCol          : mat.writeLine(rowId).setCol(colId, bloc); break;
        case FnEnum::compress        : mat.compress(); break;
        case FnEnum::clear           : mat.clear(); break;
        default : std::cerr << "callMapMapFn unsupported fnId: " << fnId << std::endl;; return;
    }
}

template<class TMatrix, int Type>
double benchStep(const std::string& logTraceFileName, const std::size_t nbExec)
{
    CRSTraceReader<TMatrix, Type>  reader;
    
    reader.readInstructions(logTraceFileName);

    for (unsigned int i = 0; i < nPreload; ++i)
    {
        TMatrix matrix;
        reader.playInstructions(matrix);
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    for (unsigned int i = 0; i < nbExec; ++i)
    {
        TMatrix matrix;
        reader.playInstructions(matrix);
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = endTime - startTime;
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    double milli = double(microseconds) / 1000.;

    double time = milli / double(nbExec);

    return time;
}

template<class TMatrix, int Type>
double benchReadStep(const std::string& logTraceFileName, const std::size_t nbExec)
{
    CRSTraceReader<TMatrix, Type>  reader;
    reader.readInstructions(logTraceFileName);

    TMatrix matrix;
    reader.playInstructions(matrix);

    using TBloc = typename TMatrix::Bloc;


    for (unsigned int i = 0; i < nPreload; ++i)
    {
        TBloc bloc;

        auto rowItEnd = matrix.end();

        for (auto rowIt = matrix.begin(); rowIt != rowItEnd; ++rowIt)
        {
            auto colIt = rowIt.begin();
            auto colItEnd = rowIt.end();

            while (colIt != colItEnd)
            {
                bloc += colIt.val();
                ++colIt;
            }
        }
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    for (unsigned int i = 0; i < nbExec; ++i)
    {
        TBloc bloc;

        auto rowItEnd = matrix.end();

        for (auto rowIt = matrix.begin(); rowIt != rowItEnd; ++rowIt)
        {
            auto colIt = rowIt.begin();
            auto colItEnd = rowIt.end();

            while (colIt != colItEnd)
            {
                bloc += colIt.val();
                ++colIt;
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = endTime - startTime;
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    double milli = double(microseconds) / 1000.;

    double time = milli / double(nbExec);

    return time;
}


template< class TBloc >
double benchStepMapMap(const std::string& logTraceFileName, const std::size_t nbExec)
{
    typedef MapMapSparseMatrix<TBloc>                                             TMapMap;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyA> TMatrixA;

    // instantiate a reader just to have access to the instructions
    CRSTraceReader<TMatrixA, TMatrixA::Policy::matrixType>  reader;
    reader.readInstructions(logTraceFileName);
    const std::vector<int>& fnIds   = reader.getFnIds();
    const auto& fnArgs        = reader.getFnArgs();
    std::size_t nbInstruction = fnIds.size();

    for (unsigned int i = 0; i < nPreload; ++i)
    {
        TMapMap matrix;
        for (std::size_t j = 0; j < nbInstruction; ++j)
        {
            callMapMapFn(fnIds[j], matrix, fnArgs[j].i, fnArgs[j].j, fnArgs[j].b);
        }
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    for (unsigned int i = 0; i < nbExec; ++i)
    {
        TMapMap matrix;
        for (std::size_t j = 0; j < nbInstruction; ++j)
        {
            callMapMapFn(fnIds[j], matrix, fnArgs[j].i, fnArgs[j].j, fnArgs[j].b);
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = endTime - startTime;
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    double milli = double(microseconds) / 1000.;

    double time = milli / double(nbExec);

    return time;

}

template< class TBloc >
double benchReadStepMapMap(const std::string& logTraceFileName, const std::size_t nbExec)
{
    typedef MapMapSparseMatrix<TBloc>                                             TMapMap;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyA> TMatrixA;

    // instantiate a reader just to have access to the instructions
    CRSTraceReader<TMatrixA, TMatrixA::Policy::matrixType>  reader;
    reader.readInstructions(logTraceFileName);
    const std::vector<int>& fnIds = reader.getFnIds();
    const auto& fnArgs      = reader.getFnArgs();
    std::size_t nbInstruction = fnIds.size();

    TMapMap matrix;
    for (std::size_t j = 0; j < nbInstruction; j++)
    {
        callMapMapFn(fnIds[j], matrix, fnArgs[j].i, fnArgs[j].j, fnArgs[j].b);
    }

    for (unsigned int i = 0; i < nPreload; ++i)
    {
        TBloc bloc;
        auto rowItEnd = matrix.end();

        for (auto rowIt = matrix.begin(); rowIt != rowItEnd; ++rowIt)
        {
            auto colIt = rowIt.begin();
            auto colItEnd = rowIt.end();

            while (colIt != colItEnd)
            {
                bloc += colIt.val();
                ++colIt;
            }
        }
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    for (unsigned int i = 0; i < nbExec; ++i)
    {
        TBloc bloc;
        auto rowItEnd = matrix.end();

        for (auto rowIt = matrix.begin(); rowIt != rowItEnd; ++rowIt)
        {
            auto colIt = rowIt.begin();
            auto colItEnd = rowIt.end();

            while (colIt != colItEnd)
            {
                bloc += colIt.val();
                ++colIt;
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = endTime - startTime;
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    double milli = double(microseconds) / 1000.;

    double time = milli / double(nbExec);

    return time;

}

template<class TBloc>
void benchFast(const std::vector<std::string>& matrixTraceFiles, const std::size_t nbExec)
{
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyA> TMatrixA;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyB> TMatrixB;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyC> TMatrixC;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyD> TMatrixD;
    //typedef MapMapSparseMatrix<TBloc>                                             TMapMap;


    const std::size_t numLogTraceFiles = matrixTraceFiles.size();

    std::cout << "Run benchByStep on " << nbExec << " executions  with " << numLogTraceFiles << " log trace files" << std::endl;

    for (const std::string& file : matrixTraceFiles)
    {
        {
            const double benchMapMap = benchStepMapMap<TBloc>(file, nbExec);
            const double benchA = benchStep<TMatrixA, TMatrixA::Policy::matrixType>(file, nbExec);
            const double benchB = benchStep<TMatrixB, TMatrixB::Policy::matrixType>(file, nbExec);
            const double benchC = benchStep<TMatrixC, TMatrixC::Policy::matrixType>(file, nbExec);
            const double benchD = benchStep<TMatrixD, TMatrixD::Policy::matrixType>(file, nbExec);

            std::cout << "File: " << file << std::endl;
            std::cout << "MapMap WRITE mean : " << benchMapMap << " ms" << std::endl;
            std::cout << "Policy A WRITE mean : " << benchA << " ms" << std::endl;
            std::cout << "Policy B WRITE mean : " << benchB << " ms" << std::endl;
            std::cout << "Policy C WRITE mean : " << benchC << " ms" << std::endl;
            std::cout << "Policy D WRITE mean : " << benchD << " ms" << std::endl;

            std::cout << "Policy A WRITE SpeedUp : " << 100.0 * (benchMapMap - benchA) / benchMapMap << " %" << std::endl;
            std::cout << "Policy B WRITE SpeedUp : " << 100.0 * (benchMapMap - benchB) / benchMapMap << " %" << std::endl;
            std::cout << "Policy C WRITE SpeedUp : " << 100.0 * (benchMapMap - benchC) / benchMapMap << " %" << std::endl;
            std::cout << "Policy D WRITE SpeedUp : " << 100.0 * (benchMapMap - benchD) / benchMapMap << " %" << std::endl;
        }
        {
            const double benchMapMap = benchReadStepMapMap<TBloc>(file, nbExec);
            const double benchA = benchReadStep<TMatrixA, TMatrixA::Policy::matrixType>(file, nbExec);
            const double benchB = benchReadStep<TMatrixB, TMatrixB::Policy::matrixType>(file, nbExec);
            const double benchC = benchReadStep<TMatrixC, TMatrixC::Policy::matrixType>(file, nbExec);
            const double benchD = benchReadStep<TMatrixD, TMatrixD::Policy::matrixType>(file, nbExec);

            std::cout << "MapMap READ mean : " << benchMapMap << " ms" << std::endl;
            std::cout << "Policy A READ mean : " << benchA << " ms" << std::endl;
            std::cout << "Policy B READ mean : " << benchB << " ms" << std::endl;
            std::cout << "Policy C READ mean : " << benchC << " ms" << std::endl;
            std::cout << "Policy D READ mean : " << benchD << " ms" << std::endl;

            std::cout << "Policy A READ SpeedUp : " << 100.0 * (benchMapMap - benchA) / benchMapMap << " %" << std::endl;
            std::cout << "Policy B READ SpeedUp : " << 100.0 * (benchMapMap - benchB) / benchMapMap << " %" << std::endl;
            std::cout << "Policy C READ SpeedUp : " << 100.0 * (benchMapMap - benchC) / benchMapMap << " %" << std::endl;
            std::cout << "Policy D READ SpeedUp : " << 100.0 * (benchMapMap - benchD) / benchMapMap << " %" << std::endl;
        }
    }
}

void showHelp()
{
    std::cout<<"Usage : CompressedRowSparseMatrixMechanicalCompare_bench [option] ... [arg] ..."<<std::endl;
    std::cout<<"-h   / --help      : print this help message and exit"<<std::endl;
    std::cout<<"-n   / --nbExec    : number of execution, default 50"<<std::endl;
    std::cout<<"     / --subfolder : name of subfolder of ISSofa/share/benchmarks/CRSResources where trace to bench is located"<<std::endl;
    std::cout<<"arg ...            : arguments passed to program in sys.argv[1:]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"For trace generation help please refer to README in ISSofa/framework/framework_bench"<<std::endl;
}

int main(int argc, char* argv[])
{
    long nbExec = 50;
    std::string folderName = "CRSConstraint_Reference";

    for(int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "-h" || arg == "--help")
        {
            showHelp();
            return 0;
        }
        else if (arg == "-n" || arg == "--nbExec")
        {
            i++; nbExec = strtol(argv[i], nullptr, 10);
        }
        else if (arg == "--subfolder")
        {
            i++; folderName = std::string(argv[i]);
        }
        else
        {
            std::cout<<"ERROR: not recognized option : "<<std::string(argv[i])<<" , please use --help to se available options"<<std::endl;
            return -1;
        }
    }

    std::string resourcesDirectory = "/benchmarks/CRSResources";
    std::string matrixTraceDirectory = resourcesDirectory + "/" + folderName + "/";
    const std::vector< std::string >& paths = sofa::helper::system::DataRepository.getPaths();

    bool foundPath = false;
    for (std::size_t i = 0; i < paths.size(); i++)
    {
        if (sofa::helper::system::FileSystem::isDirectory(paths[i] + matrixTraceDirectory))
        {
            foundPath = true;
            matrixTraceDirectory = paths[i] + matrixTraceDirectory;
            break;
        }
    }

    if (!foundPath)
    {
        std::cout << "Error could not find " << matrixTraceDirectory << " in all environnement paths" << std::endl;
        return -1;
    }
    
    std::vector<std::string> matrixTraceFiles;
#ifdef SOFA_HAVE_ZLIB
    { // first try compressed files
        sofa::helper::system::FileSystem::listDirectory(matrixTraceDirectory, matrixTraceFiles, ".gz");
    }
    if (matrixTraceFiles.empty())
#endif
    { // try uncompressed files
        sofa::helper::system::FileSystem::listDirectory(matrixTraceDirectory, matrixTraceFiles, ".bin");
    }
    if (matrixTraceFiles.empty())
    {
        std::cout << "ERROR: could not find trace into specified folder " << matrixTraceDirectory << std::endl;
        return -1;
    }
    else
    {
        for (auto& file : matrixTraceFiles)
        {
            file = matrixTraceDirectory + file;
        }
    }

    const std::string fileName = sofa::helper::system::SetDirectory::GetFileName(matrixTraceFiles[0].c_str());

    if      (fileName.find("_V1f_") != std::string::npos) benchFast<Vec1f>(matrixTraceFiles, nbExec);
    else if (fileName.find("_V1d_") != std::string::npos) benchFast<Vec1d>(matrixTraceFiles, nbExec);
    else if (fileName.find("_V3f_") != std::string::npos) benchFast<Vec3f>(matrixTraceFiles, nbExec);
    else if (fileName.find("_V3d_") != std::string::npos) benchFast<Vec3d>(matrixTraceFiles, nbExec);
    else if (fileName.find("_V6f_") != std::string::npos) benchFast<Vec6f>(matrixTraceFiles, nbExec);
    else if (fileName.find("_V6d_") != std::string::npos) benchFast<Vec6d>(matrixTraceFiles, nbExec);
    else if (fileName.find("_RigidDeriv<2,float>_") != std::string::npos) benchFast<RigidDeriv<2, float>>(matrixTraceFiles, nbExec);
    else if (fileName.find("_RigidDeriv<2,double>_") != std::string::npos) benchFast<RigidDeriv<2, double>>(matrixTraceFiles, nbExec);
    else if (fileName.find("_RigidDeriv<3,float>_") != std::string::npos) benchFast<RigidDeriv<3, float>>(matrixTraceFiles, nbExec);
    else if (fileName.find("_RigidDeriv<3,double>_") != std::string::npos) benchFast<RigidDeriv<3, double>>(matrixTraceFiles, nbExec);
    else
    {
        std::cout<<"ERROR: CompressedRowSparseMatrixConstraint_bench bloc template not handled"<<std::endl;
        return -1;
    }

    return 0;
} 