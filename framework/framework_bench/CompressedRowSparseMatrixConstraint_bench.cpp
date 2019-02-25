// This is benchmarking code, enforce no debug code and no container access checks in this binary
#ifdef SOFA_CHECK_CONTAINER_ACCESS
#undef SOFA_CHECK_CONTAINER_ACCESS
#endif
#ifndef NDEBUG
#define NDEBUG
#endif

#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/thread/CTime.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include "CompressedRowSparseMatrixHelper_bench.h"
#include <sofa/defaulttype/MapMapSparseMatrix.h>


using namespace sofa::defaulttype;
using namespace sofa::helper::system::thread;

/// Number of execution to pre load CPU
static constexpr std::size_t nPreload = 100u;

/// Specific policy for benchmark on CRSMatrixConstraint
template<typename TBloc>
class CRSBenchConstraintPolicyA : public CRSConstraintPolicy<TBloc>
{
public:

    static constexpr bool OrderedInsertion = false;
    static constexpr bool CompressZeros = true;
    static constexpr bool ClearZeros = true;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

/// Specific policy for benchmark on CRSMatrixConstraint
template<typename TBloc>
class CRSBenchConstraintPolicyB : public CRSConstraintPolicy<TBloc>
{
public:

    static constexpr bool OrderedInsertion = false;
    static constexpr bool CompressZeros = false;
    static constexpr bool ClearZeros = false;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

/// Specific policy for benchmark on CRSMatrixConstraint
template<typename TBloc>
class CRSBenchConstraintPolicyC : public CRSConstraintPolicy<TBloc>
{
public:

    static constexpr bool OrderedInsertion = true;
    static constexpr bool CompressZeros = true;
    static constexpr bool ClearZeros = true;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

/// Specific policy for benchmark on CRSMatrixConstraint
template<typename TBloc>
class CRSBenchConstraintPolicyD : public CRSConstraintPolicy<TBloc>
{
public:

    static constexpr bool OrderedInsertion = true;
    static constexpr bool CompressZeros = false;
    static constexpr bool ClearZeros = false;

    static constexpr bool PrintTrace = false;
    static constexpr bool LogTrace = false;
};

template <class TBloc, class TPolicy>
bool checkMatrix(const std::string& checkMatrixFileName, const std::vector<std::string>& listOfBinFiles, const bool verbose)
{
    typedef CompressedRowSparseMatrixConstraint<TBloc> TMatrixRef;
    typedef CompressedRowSparseMatrixConstraint<TBloc, TPolicy> TMatrix;

    if (TPolicy::CompressZeros)
    {
        std::cout<<"Policy CompressZero change matrix format, check not available"<<std::endl;
        return true;
    }

    FnList<TMatrix> instructions = readInstructionsFromFiles<TMatrix>(listOfBinFiles);

    /// Check Matrix consistency
    TMatrixRef mref;
    TMatrix mloaded;
    playAllInstructions<TMatrix>(mloaded, instructions);
    return checkMatrixConsistency<TMatrixRef>(checkMatrixFileName, mloaded, verbose);
}

template <class TMapMap, class TBloc>
void callMapMapFn(int fnId, TMapMap& mat, unsigned int rowId, unsigned int colId, TBloc bloc)
{
    switch (fnId)
    {
        case FnEnum::addCol          : mat.writeLine(rowId).addCol(colId, bloc); break;
        case FnEnum::setCol          : mat.writeLine(rowId).setCol(colId, bloc); break;
        default : assert(false); return;
    }
}

template<class TBloc>
void benchFast(std::string& checkMatrixFile, const std::vector<std::string>& listOfBinFiles,
               const std::size_t start, const std::size_t stop, const std::size_t nbExec, const int nbPolicy, const bool verbose)
{
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyA<TBloc>> TMatrixA;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyB<TBloc>> TMatrixB;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyC<TBloc>> TMatrixC;
    typedef CompressedRowSparseMatrixConstraint<TBloc, CRSBenchConstraintPolicyD<TBloc>> TMatrixD;
    typedef MapMapSparseMatrix<TBloc>                                                    TMapMap;

    std::cout<<"Run benchFast on "<<nbExec<<" executions with "<<nbPolicy<<" policies between step "<<start<<" and step "<<stop<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Policy A : ";
    bool checkA = checkMatrix<TBloc, CRSBenchConstraintPolicyA<TBloc>>(checkMatrixFile, listOfBinFiles, verbose);
    bool checkB = false;
    bool checkC = false;
    bool checkD = false;
    if (nbPolicy > 1)
    {
        std::cout<<"Policy B : ";
        checkB = checkMatrix<TBloc, CRSBenchConstraintPolicyB<TBloc>>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 2)
    {
        std::cout<<"Policy C : ";
        checkC = checkMatrix<TBloc, CRSBenchConstraintPolicyC<TBloc>>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 3)
    {
        std::cout<<"Policy D : ";
        checkD = checkMatrix<TBloc, CRSBenchConstraintPolicyD<TBloc>>(checkMatrixFile, listOfBinFiles, verbose);
    }
    std::cout<<std::endl;

    // -----
    // pre load cpu
    // -----

    FnList<TMatrixA> instructions = readInstructionsFromFiles<TMatrixA>(listOfBinFiles);
    for (unsigned int i = 0; i < nPreload; i ++)
    {
        TMatrixA matrixPreLoad;
        playAllInstructions<TMatrixA>(matrixPreLoad, instructions);
    }

    // -----
    // execute benchmark
    // -----

    /// MapMap implementation

    /// Write test
    ctime_t tWriteMapMap0 = CTime::getFastTime();

    for (std::size_t i = 0; i < nbExec; i++)
    {
        TMapMap matrix;
        for (std::size_t step = start; step <= stop; step++)
        {
            std::vector<int>& fnIds = instructions.fnIdbyStep[step];
            auto& fnArgs = instructions.fnArgsbyStep[step];
            std::size_t nbInstruction = fnIds.size();

            for (std::size_t j = 0; j < nbInstruction; j++)
            {
                callMapMapFn(fnIds[j], matrix, fnArgs[j].i, fnArgs[j].j, fnArgs[j].b);
            }
        }
    }

    ctime_t tWriteMapMap1 = CTime::getFastTime();
    double timeusWriteMapMap = 0.001 * (((tWriteMapMap1 - tWriteMapMap0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
    std::cout << "MapMap   Write mean duration : " << timeusWriteMapMap << " ms" << std::endl;


    /// Read test
    TMapMap matrixMapMap;
    for (std::size_t step = start; step <= stop; step++)
    {
        std::vector<int>& fnIds = instructions.fnIdbyStep[step];
        auto& fnArgs = instructions.fnArgsbyStep[step];
        std::size_t nbInstruction = fnIds.size();

        for (std::size_t j = 0; j < nbInstruction; j++)
        {
            callMapMapFn(fnIds[j], matrixMapMap, fnArgs[j].i, fnArgs[j].j, fnArgs[j].b);
        }
    }

    TBloc DataMapMap;

    ctime_t tReadMapMap0 = CTime::getFastTime();
    for (std::size_t i = 0; i < nbExec; i++)
    {
        auto rowItEnd = matrixMapMap.end();

        for (auto rowIt = matrixMapMap.begin(); rowIt != rowItEnd; ++rowIt)
        {
            auto colIt = rowIt.begin();
            auto colItEnd = rowIt.end();

            while (colIt != colItEnd)
            {
                DataMapMap += colIt.val();
                ++colIt;
            }
        }
    }
    ctime_t tReadMapMap1 = CTime::getFastTime();
    double timeusReadMapMap = 0.001 * (((tReadMapMap1 - tReadMapMap0) * 1000000000) / (CTime::getTicksPerSec() * nbExec));
    std::cout << "MapMap   Read  mean duration : " << timeusReadMapMap << " micro second" << std::endl;

    /// Policy A
    if (checkA)
    {
        /// Write Test
        ctime_t ta0 = CTime::getFastTime();
        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixA matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixA, TMatrixA::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
            }
        }

        ctime_t ta1 = CTime::getFastTime();
        double timeusAWrite = 0.001 * (((ta1 - ta0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy A Write Mean duration : " << timeusAWrite << " ms" << std::endl;
        std::cout << "Policy A Write SpeedUp       : " << 100.0 * (timeusWriteMapMap - timeusAWrite) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;

        /// ReadTest
        TMatrixA matrixCRS;
        playAllInstructions<TMatrixA>(matrixCRS, instructions);
        TBloc DataCRS;

        ctime_t ta2 = CTime::getFastTime();
        for (std::size_t i = 0; i < nbExec; i++)
        {
            auto rowItEnd = matrixCRS.end();

            for (auto rowIt = matrixCRS.begin(); rowIt != rowItEnd; ++rowIt)
            {
                auto colIt = rowIt.begin();
                auto colItEnd = rowIt.end();

                while (colIt != colItEnd)
                {
                    DataCRS += colIt.val();
                    ++colIt;
                }
            }
        }
        ctime_t ta3 = CTime::getFastTime();
        double timeusARead = 0.001 * (((ta3 - ta2) * 1000000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy A Read  Mean duration : " << timeusARead << " micro second" << std::endl;
        std::cout << "Policy A Read  SpeedUp       : " << 100.0 * (timeusReadMapMap - timeusARead) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;
    }

    /// Policy B
    if (nbPolicy > 1 && checkB)
    {
        /// Write test
        ctime_t tb0 = CTime::getFastTime();

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixB matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixB, TMatrixB::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
            }
        }

        ctime_t tb1 = CTime::getFastTime();
        double timeusBWrite = 0.001 * (((tb1 - tb0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy B Write Mean duration : " << timeusBWrite << " ms" << std::endl;
        std::cout << "Policy B Write SpeedUp       : " << 100.0 * (timeusWriteMapMap - timeusBWrite) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;

        /// ReadTest
        TMatrixB matrixCRS;
        FnList<TMatrixB> instructionsB = readInstructionsFromFiles<TMatrixB>(listOfBinFiles);
        playAllInstructions<TMatrixB>(matrixCRS, instructionsB);
        TBloc DataCRS;

        ctime_t tb2 = CTime::getFastTime();
        for (std::size_t i = 0; i < nbExec; i++)
        {
            auto rowItEnd = matrixCRS.end();

            for (auto rowIt = matrixCRS.begin(); rowIt != rowItEnd; ++rowIt)
            {
                auto colIt = rowIt.begin();
                auto colItEnd = rowIt.end();

                while (colIt != colItEnd)
                {
                    DataCRS += colIt.val();
                    ++colIt;
                }
            }
        }
        ctime_t tb3 = CTime::getFastTime();
        double timeusBRead = 0.001 * (((tb3 - tb2) * 1000000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy B Read  Mean duration : " << timeusBRead << " micro second" << std::endl;
        std::cout << "Policy B Read  SpeedUp       : " << 100.0 * (timeusReadMapMap - timeusBRead) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;
    }

    /// Policy C
    if (nbPolicy > 2 && checkC)
    {
        /// Write Test
        ctime_t tc0 = CTime::getFastTime();

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixC matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixC, TMatrixC::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
            }
        }

        ctime_t tc1 = CTime::getFastTime();
        double timeusCWrite = 0.001 * (((tc1 - tc0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy C Write Mean duration : " << timeusCWrite << " ms" << std::endl;
        std::cout << "Policy C Write SpeedUp       : " << 100.0 * (timeusWriteMapMap - timeusCWrite) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;

        /// ReadTest
        TMatrixC matrixCRS;
        FnList<TMatrixC> instructionsC = readInstructionsFromFiles<TMatrixC>(listOfBinFiles);
        playAllInstructions<TMatrixC>(matrixCRS, instructionsC);
        TBloc DataCRS;

        ctime_t tc2 = CTime::getFastTime();
        for (std::size_t i = 0; i < nbExec; i++)
        {
            auto rowItEnd = matrixCRS.end();

            for (auto rowIt = matrixCRS.begin(); rowIt != rowItEnd; ++rowIt)
            {
                auto colIt = rowIt.begin();
                auto colItEnd = rowIt.end();

                while (colIt != colItEnd)
                {
                    DataCRS += colIt.val();
                    ++colIt;
                }
            }
        }
        ctime_t tc3 = CTime::getFastTime();
        double timeusCRead = 0.001 * (((tc3 - tc2) * 1000000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy C Read  Mean duration : " << timeusCRead << " micro second" << std::endl;
        std::cout << "Policy C Read  SpeedUp       : " << 100.0 * (timeusReadMapMap - timeusCRead) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;
    }

    /// Policy D
    if (nbPolicy > 3 && checkD)
    {
        ctime_t td0 = CTime::getFastTime();

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixD matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixD, TMatrixD::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
            }
        }

        ctime_t td1 = CTime::getFastTime();
        double timeusDWrite = 0.001 * (((td1 - td0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy D Write Mean duration : " << timeusDWrite << " ms" << std::endl;
        std::cout << "Policy D Write SpeedUp       : " << 100.0 * (timeusWriteMapMap - timeusDWrite) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;

        /// ReadTest
        TMatrixD matrixCRS;
        FnList<TMatrixD> instructionsD = readInstructionsFromFiles<TMatrixD>(listOfBinFiles);
        playAllInstructions<TMatrixD>(matrixCRS, instructionsD);
        TBloc DataCRS;

        ctime_t td2 = CTime::getFastTime();
        for (std::size_t i = 0; i < nbExec; i++)
        {
            auto rowItEnd = matrixCRS.end();

            for (auto rowIt = matrixCRS.begin(); rowIt != rowItEnd; ++rowIt)
            {
                auto colIt = rowIt.begin();
                auto colItEnd = rowIt.end();

                while (colIt != colItEnd)
                {
                    DataCRS += colIt.val();
                    ++colIt;
                }
            }
        }
        ctime_t td3 = CTime::getFastTime();
        double timeusDRead = 0.001 * (((td3 - td2) * 1000000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout<<std::endl;
        std::cout << "Policy D Read  Mean duration : " << timeusDRead << " micro second" << std::endl;
        std::cout << "Policy D Read  SpeedUp       : " << 100.0 * (timeusReadMapMap - timeusDRead) / timeusWriteMapMap << " %" <<std::endl;
        std::cout<<std::endl;
    }
}

void showHelp()
{
    std::cout<<"Usage : CompressedRowSparseMatrixMechanicalCompare_bench [option] ... [arg] ..."<<std::endl;
    std::cout<<"-h   / --help      : print this help message and exit"<<std::endl;
    std::cout<<"-n   / --nbExec    : number of execution, default 1000"<<std::endl;
    std::cout<<"-nbp / --nbpolicy  : number of policy to bench, max 4, default 1"<<std::endl;
    std::cout<<"                     policies could be specified directly in this cpp file, compilation needed"<<std::endl;
    std::cout<<"     / --start     : start step for bench, default 0"<<std::endl;
    std::cout<<"     / --stop      : stop  step for bench, default max of trace"<<std::endl;
    std::cout<<"     / --subfolder : name of subfolder of ISSofa/framework/framework_bench/ressources where trace to bench is located"<<std::endl;
    std::cout<<"     / --verbose   : when check matrix consistency failed, verbose give more outputs for debug"<<std::endl;
    std::cout<<"arg ...            : arguments passed to program in sys.argv[1:]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"For trace generation help please refer to README in ISSofa/framework/framework_bench"<<std::endl;
}

int main(int argc, char* argv[])
{
    long nbExec = 1000;
    long startStep = 0;
    long endStep = -1;
    long nbPolicy = 1;
    bool verbose = false;
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
        else if (arg == "--start")
        {
            i++; startStep = strtol(argv[i], nullptr, 10);
        }
        else if (arg == "--stop")
        {
            i++; endStep = strtol(argv[i], nullptr, 10);
        }
        else if (arg == "-nbp" || arg == "--nbpolicy")
        {
            i++; nbPolicy = strtol(argv[i], nullptr, 10);
        }
        else if (arg == "--verbose")
        {
            verbose = true;
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

    std::string resourcesDirectory = "/benchmarks/CRSRessources";
    std::string matrixTraceDirectory = resourcesDirectory + "/" + folderName + "/";
    const std::vector< std::string > paths = sofa::helper::system::DataRepository.getPaths();

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
        std::cout<<"Error could not found "<<matrixTraceDirectory<<" in all environnement paths"<<std::endl;
        return -1;
    }

    std::vector<std::string> listOfBinFiles = getAllFilesInDir(matrixTraceDirectory, {}, ".bin");
    if (listOfBinFiles.empty())
    {
        std::cout<<"ERROR: could not find trace into specified folder name"<<std::endl;
        return -1;
    }

    const char delim = '_';
    std::string filesName = getFileName(listOfBinFiles[0]);
    std::stringstream ss(filesName);

    std::string matrixType, tbloc;
    std::getline(ss, matrixType, delim);
    std::getline(ss, tbloc, delim);

    std::string checkFile = matrixTraceDirectory + matrixType + "_" + tbloc + "_Check.txt";

    if (endStep == -1) endStep = static_cast<long>(listOfBinFiles.size() - 1);

    if (matrixType != "CRSConstraintMatrix")
    {
        std::cout<<"ERROR: CompressedRowSparseMatrixConstraint_bench could be only instanciated with CRSConstraint matrix trace"<<std::endl;
        return -1;
    }
    else
    {
        std::cout<<"Execute CRSMatrixConstraintBench on trace stored in subfolder "<<folderName<< " with template "<<tbloc<<std::endl;
    }

    if      (tbloc == "V1f") benchFast<Vec1f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "V1d") benchFast<Vec1d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "V3f") benchFast<Vec3f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "V3d") benchFast<Vec3d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "V6f") benchFast<Vec6f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "V6d") benchFast<Vec6d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "RigidDeriv<2,float>")  benchFast<RigidDeriv<2,float>>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "RigidDeriv<2,double>") benchFast<RigidDeriv<3,double>>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "RigidDeriv<3,float>")  benchFast<RigidDeriv<3,float>>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else if (tbloc == "RigidDeriv<3,double>") benchFast<RigidDeriv<3,double>>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose);
    else
    {
        std::cout<<"ERROR: CompressedRowSparseMatrixConstraint_bench bloc template not handled"<<std::endl;
        return -1;
    }

    return 0;
}
