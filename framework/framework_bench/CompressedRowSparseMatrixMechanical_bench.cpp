// This is benchmarking code, enforce no debug code and no container access checks in this binary
#ifdef SOFA_CHECK_CONTAINER_ACCESS
#undef SOFA_CHECK_CONTAINER_ACCESS
#endif
#ifndef NDEBUG
#define NDEBUG
#endif

#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/thread/CTime.h>

#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.inl>

#include "CompressedRowSparseMatrixHelper_bench.h"
#include "CompressedRowSparseMatrixOld.h"

#include <chrono>

using namespace sofa::defaulttype;
using namespace sofa::helper::system::thread;

/// Number of execution to pre load CPU
static constexpr std::size_t nPreload = 5u;

/// Specific policy for benchmark on CRSMatrixMechanical
class CRSBenchMechanicalPolicyA : public CRSMechanicalPolicy
{
public:

    static constexpr bool OrderedInsertion = false;
    static constexpr bool StoreTouchFlags = false;
    static constexpr bool StoreLowerTriangularBloc = true;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};


template class CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyA>;
template class CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyA>;
template class CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyA>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyA>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyA>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyA>));

template class CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyA>;
template class CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyA>;
template class CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyA>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyA>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyA>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyA>));

/// Specific policy for benchmark on CRSMatrixMechanical
class CRSBenchMechanicalPolicyB : public CRSMechanicalPolicy
{
public:

    static constexpr bool OrderedInsertion = false;
    static constexpr bool StoreTouchFlags = false;
    static constexpr bool StoreLowerTriangularBloc = false;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

template class CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyB>;
template class CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyB>;
template class CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyB>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyB>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyB>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyB>));

template class CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyB>;
template class CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyB>;
template class CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyB>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyB>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyB>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyB>));

/// Specific policy for benchmark on CRSMatrixMechanical
class CRSBenchMechanicalPolicyC : public CRSMechanicalPolicy
{
public:

    static constexpr bool OrderedInsertion = true;
    static constexpr bool StoreTouchFlags = false;
    static constexpr bool StoreLowerTriangularBloc = true;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

template class CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyC>;
template class CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyC>;
template class CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyC>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyC>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyC>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyC>));

template class CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyC>;
template class CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyC>;
template class CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyC>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyC>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyC>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyC>));

/// Specific policy for benchmark on CRSMatrixMechanical
class CRSBenchMechanicalPolicyD : public CRSMechanicalPolicy
{
public:

    static constexpr bool OrderedInsertion = true;
    static constexpr bool StoreTouchFlags = false;
    static constexpr bool StoreLowerTriangularBloc = false;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

template class CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyD>;
template class CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyD>;
template class CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyD>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<float, CRSBenchMechanicalPolicyD>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1f, CRSBenchMechanicalPolicyD>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3f, CRSBenchMechanicalPolicyD>));

template class CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyD>;
template class CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyD>;
template class CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyD>;

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<double, CRSBenchMechanicalPolicyD>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1d, CRSBenchMechanicalPolicyD>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3d, CRSBenchMechanicalPolicyD>));


template<class TMatrix, int Type>
double bench(CRSTraceReader<TMatrix, Type>& reader, const std::size_t nbExec)
{
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
double benchStep(const std::string& logTraceFileName, const std::size_t nbExec)
{
    CRSTraceReader<TMatrix, Type>  reader;   
    reader.readInstructions(logTraceFileName);

    return bench<TMatrix, Type>(reader, nbExec);
}

template<class TMatrix, int Type>
double benchAllSteps(const std::vector<std::string>& logTraceFiles, const std::size_t nbExec)
{
    CRSTraceReader<TMatrix, Type>  reader;
    for (const std::string& file : logTraceFiles)
    {
        reader.readInstructions(file);
    }
    return bench<TMatrix, Type>(reader, nbExec);
}


template<class TBloc>
void benchByStep(const std::vector<std::string>& matrixTraceFiles, const std::size_t nbExec)
{
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyA> TMatrixA;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyB> TMatrixB;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyC> TMatrixC;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyD> TMatrixD;
    typedef CompressedRowSparseMatrixOld<TBloc>                                 TMatrixOld;

    const std::size_t numLogTraceFiles = matrixTraceFiles.size();

    std::cout<<"Run benchByStep "<<nbExec<<" executions on "<< numLogTraceFiles << " log trace files" << std::endl;


    // -----
    // execute benchmark
    // -----

    std::vector< double > benchOld;
    std::vector< double > benchA;
    std::vector< double > benchB;
    std::vector< double > benchC;
    std::vector< double > benchD;

    for (const std::string& file : matrixTraceFiles)
    {
        std::cout << "File: " << file << std::endl;

        benchOld.push_back( benchStep< TMatrixOld, 1 >(file, nbExec) );
        benchA.push_back( benchStep< TMatrixA, TMatrixA::Policy::matrixType>(file, nbExec) );
        benchB.push_back( benchStep< TMatrixB, TMatrixB::Policy::matrixType>(file, nbExec) );
        benchC.push_back( benchStep< TMatrixC, TMatrixC::Policy::matrixType>(file, nbExec) );
        benchD.push_back( benchStep< TMatrixD, TMatrixD::Policy::matrixType>(file, nbExec) );


        std::cout << "CRS Old  mean : " << benchOld.back() << " ms" << std::endl;
        std::cout << "Policy A mean : " << benchA.back() << " ms" << std::endl;
        std::cout << "Policy B mean : " << benchB.back() << " ms" << std::endl;
        std::cout << "Policy C mean : " << benchC.back() << " ms" << std::endl;
        std::cout << "Policy D mean : " << benchD.back() << " ms" << std::endl;

        std::cout << "Policy A SpeedUp : " << 100.0 * (benchOld.back() - benchA.back() ) / benchOld.back() << " %" << std::endl;
        std::cout << "Policy B SpeedUp : " << 100.0 * (benchOld.back() - benchB.back()) / benchOld.back() << " %" << std::endl;
        std::cout << "Policy C SpeedUp : " << 100.0 * (benchOld.back() - benchC.back()) / benchOld.back() << " %" << std::endl;
        std::cout << "Policy D SpeedUp : " << 100.0 * (benchOld.back() - benchD.back()) / benchOld.back() << " %" << std::endl;

    }

}

template<class TBloc>
void benchAll(const std::vector<std::string>& matrixTraceFiles, const std::size_t nbExec)
{
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyA> TMatrixA;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyB> TMatrixB;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyC> TMatrixC;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyD> TMatrixD;
    typedef CompressedRowSparseMatrixOld<TBloc>                                 TMatrixOld;

    const std::size_t numLogTraceFiles = matrixTraceFiles.size();

    std::cout << "Run benchAll " << nbExec << " executions on " << numLogTraceFiles << " log trace files" << std::endl;

    const double benchOld = benchAllSteps< TMatrixOld, 1 >(matrixTraceFiles, nbExec);
    const double benchA = benchAllSteps< TMatrixA, TMatrixA::Policy::matrixType >(matrixTraceFiles, nbExec);
    const double benchB = benchAllSteps< TMatrixB, TMatrixB::Policy::matrixType >(matrixTraceFiles, nbExec);
    const double benchC = benchAllSteps< TMatrixC, TMatrixC::Policy::matrixType >(matrixTraceFiles, nbExec);
    const double benchD = benchAllSteps< TMatrixD, TMatrixD::Policy::matrixType >(matrixTraceFiles, nbExec);

    std::cout << "CRS Old  mean : " << benchOld << " ms" << std::endl;
    std::cout << "Policy A mean : " << benchA << " ms" << std::endl;
    std::cout << "Policy B mean : " << benchB << " ms" << std::endl;
    std::cout << "Policy C mean : " << benchC << " ms" << std::endl;
    std::cout << "Policy D mean : " << benchD << " ms" << std::endl;

    std::cout << "Policy A SpeedUp : " << 100.0 * (benchOld - benchA) / benchOld << " %" << std::endl;
    std::cout << "Policy B SpeedUp : " << 100.0 * (benchOld - benchB) / benchOld << " %" << std::endl;
    std::cout << "Policy C SpeedUp : " << 100.0 * (benchOld - benchC) / benchOld << " %" << std::endl;
    std::cout << "Policy D SpeedUp : " << 100.0 * (benchOld - benchD) / benchOld << " %" << std::endl;

}

void showHelp()
{
    std::cout<<"Usage : CompressedRowSparseMatrixMechanicalCompare_bench [option] ... [arg] ..."<<std::endl;
    std::cout<<"-a   / --accuracy  : type of bench you want to launch, default benchByStep"<<std::endl;
    std::cout<<"                     0 : benchByStep = Each trace file is treated separetly ( step by step )" << std::endl;
    std::cout<<"                     1 : benchAll    = Trace files are processed together ( read all then benchmark )"<<std::endl;
    std::cout<<"-h   / --help      : print this help message and exit"<<std::endl;
    std::cout<<"-n   / --nbExec    : number of execution, default 25"<<std::endl;
    std::cout<<"     / --subfolder : name of subfolder of ISSofa/framework/framework_bench/ressources where trace to bench is located (default /benchmarks/CRSResources)"<<std::endl;
    std::cout<<"arg ...            : arguments passed to program in sys.argv[1:]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"For trace generation help please refer to README in ISSofa/framework/framework_bench"<<std::endl;
}

int main(int argc, char* argv[])
{
    long accuracy = 0;
    long nbExec = 25;
    std::string folderName = "CRSMechanical_Reference";

    for(int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "-h" || arg == "--help")
        {
            showHelp();
            return 0;
        }
        else if (arg == "-a" || arg == "--accuracy")
        {
            i++; accuracy = strtol(argv[i], nullptr, 10);
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
        if (filesys::exists(paths[i] + matrixTraceDirectory))
        {
            foundPath = true;
            matrixTraceDirectory = paths[i] + matrixTraceDirectory;
            break;
        }
    }

    if (!foundPath)
    {
        std::cout<<"Error could not find "<<matrixTraceDirectory<<" in all environnement paths"<<std::endl;
        return -1;
    }

    std::vector<std::string> matrixTraceFiles;
#ifdef SOFA_HAVE_ZLIB
    { // first try compressed files
        matrixTraceFiles = getAllFilesInDir(matrixTraceDirectory, {}, ".gz");
    }
    if (matrixTraceFiles.empty())
#endif
    { // try uncompressed files
        matrixTraceFiles = getAllFilesInDir(matrixTraceDirectory, {}, ".bin");
    }
    if (matrixTraceFiles.empty())
    {
        std::cout<<"ERROR: could not find trace into specified folder "<<matrixTraceDirectory << std::endl;
        return -1;
    }
    
    std::string fileName = sofa::helper::system::SetDirectory::GetFileName(matrixTraceFiles[0].c_str());

    if      (fileName.find("_double_") != std::string::npos)
    {
        if (accuracy == 0) benchByStep<double>(matrixTraceFiles, nbExec);
        else benchAll<double>(matrixTraceFiles, nbExec);
    }
    else if ((fileName.find("_float_") != std::string::npos))
    {
        if (accuracy == 0) benchByStep<float>(matrixTraceFiles, nbExec);
        else benchAll<float>(matrixTraceFiles, nbExec);
    }
    else if ((fileName.find("_1f_") != std::string::npos))
    {
        if (accuracy == 0) benchByStep<Mat1x1f>(matrixTraceFiles, nbExec);
        else benchAll<Mat1x1f>(matrixTraceFiles, nbExec);
    }
    else if ((fileName.find("_1d_") != std::string::npos))
    {
        if (accuracy == 0) benchByStep<Mat1x1d>(matrixTraceFiles, nbExec);
        else benchAll<Mat1x1d>(matrixTraceFiles, nbExec);
    }
    else if ((fileName.find("_3f_") != std::string::npos))
    {
        if (accuracy == 0) benchByStep<Mat3x3f>(matrixTraceFiles, nbExec);
        else benchAll<Mat3x3f>(matrixTraceFiles, nbExec);
    }
    else if ((fileName.find("_3d_") != std::string::npos))
    {
        if (accuracy == 0) benchByStep<Mat3x3d>(matrixTraceFiles, nbExec);
        else benchAll<Mat3x3d>(matrixTraceFiles, nbExec);
    }
    else
    {
        std::cout<<"ERROR: CompressedRowSparseMatrixMechanicalCompare_bench bloc template not handled"<<std::endl;
        return -1;
    }

    return 0;
}
