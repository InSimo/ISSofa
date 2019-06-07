// This is benchmarking code, enforce no debug code and no container access checks in this binary
#ifdef SOFA_CHECK_CONTAINER_ACCESS
#undef SOFA_CHECK_CONTAINER_ACCESS
#endif
#ifndef NDEBUG
#define NDEBUG
#endif

#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/thread/CTime.h>

#include "CompressedRowSparseMatrixHelper_bench.h"
#include "CompressedRowSparseMatrixOld.h"


using namespace sofa::defaulttype;
using namespace sofa::helper::system::thread;

/// Number of execution to pre load CPU
static constexpr std::size_t nPreload = 100u;

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

template <class TBloc, class TPolicy>
bool checkMatrix(const std::string& checkMatrixFileName, const std::vector<std::string>& listOfBinFiles, const bool verbose)
{
    typedef CompressedRowSparseMatrixMechanical<TBloc> TMatrixRef;
    typedef CompressedRowSparseMatrixMechanical<TBloc, TPolicy> TMatrix;

    FnList<TMatrix> instructions = readInstructionsFromFiles<TMatrix>(listOfBinFiles);

    /// Check Matrix consistency
    TMatrixRef mref;
    TMatrix mloaded;
    playAllInstructions<TMatrix>(mloaded, instructions);
    return checkMatrixConsistency<TMatrixRef>(checkMatrixFileName, mloaded, verbose);
}

template<class TBloc>
void benchFast(std::string& checkMatrixFile, const std::vector<std::string>& listOfBinFiles,
               const std::size_t start, const std::size_t stop, const std::size_t nbExec, const int nbPolicy, const bool verbose)
{
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyA> TMatrixA;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyB> TMatrixB;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyC> TMatrixC;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyD> TMatrixD;
    typedef CompressedRowSparseMatrixOld<TBloc>                                   TMatrixOld;

    FnList<TMatrixA> instructions = readInstructionsFromFiles<TMatrixA>(listOfBinFiles);

    std::cout<<"Run benchFast on "<<nbExec<<" executions with "<<nbPolicy<<" policies between step "<<start<<" and step "<<stop<<std::endl;

    std::cout<<std::endl;
    std::cout<<"Policy A : ";
    bool checkA = checkMatrix<TBloc, CRSBenchMechanicalPolicyA>(checkMatrixFile, listOfBinFiles, verbose);
    bool checkB = false;
    bool checkC = false;
    bool checkD = false;
    if (nbPolicy > 1)
    {
        std::cout<<"Policy B : ";
        checkB = checkMatrix<TBloc, CRSBenchMechanicalPolicyB>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 2)
    {
        std::cout<<"Policy C : ";
        checkC = checkMatrix<TBloc, CRSBenchMechanicalPolicyC>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 3)
    {
        std::cout<<"Policy D : ";
        checkD = checkMatrix<TBloc, CRSBenchMechanicalPolicyD>(checkMatrixFile, listOfBinFiles, verbose);
    }
    std::cout<<std::endl;

    // -----
    // pre load cpu
    // -----

    for (unsigned int i = 0; i < nPreload; i ++)
    {
        TMatrixA matrixPreLoad;
        playAllInstructions<TMatrixA>(matrixPreLoad, instructions);
    }

    // -----
    // execute benchmark
    // -----

    /// Old implementation
    ctime_t told0 = CTime::getFastTime();

    for (std::size_t i = 0; i < nbExec; i++)
    {
        TMatrixOld matrix;
        sofa::defaulttype::CRSTraceReader<TMatrixOld, 1u> traceReader(&matrix);
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
    ctime_t told1 = CTime::getFastTime();
    double timeusOld = 0.001 * (((told1 - told0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
    std::cout << "CRS Old  Mean duration : " << timeusOld << " ms" << std::endl;

    /// Policy A
    if (checkA)
    {
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
        double timeusA = 0.001 * (((ta1 - ta0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout << "Policy A Mean duration : " << timeusA << " ms" << std::endl;
        std::cout << "Policy A SpeedUp       : " << 100.0 * (timeusOld - timeusA) / timeusOld << " %" <<std::endl;
    }

    /// Policy B
    if (nbPolicy > 1 && checkB)
    {
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
        double timeusB = 0.001 * (((tb1 - tb0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout << "Policy B Mean duration : " << timeusB << " ms" << std::endl;
        std::cout << "Policy B SpeedUp       : " << 100.0 * (timeusOld - timeusB) / timeusOld << " %" <<std::endl;
    }

    /// Policy C
    if (nbPolicy > 2 && checkC)
    {
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
        double timeusC = 0.001 * (((tc1 - tc0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout << "Policy C Mean duration : " << timeusC << " ms" << std::endl;
        std::cout << "Policy C SpeedUp       : " << 100.0 * (timeusOld - timeusC) / timeusOld << " %" <<std::endl;
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
        double timeusD = 0.001 * (((td1 - td0) * 1000000) / (CTime::getTicksPerSec() * nbExec));
        std::cout << "Policy D Mean duration : " << timeusD << " ms" << std::endl;
        std::cout << "Policy D SpeedUp       : " << 100.0 * (timeusOld - timeusD) / timeusOld << " %" <<std::endl;
    }
}

template<class TBloc>
void benchByStep(std::string& checkMatrixFile, const std::vector<std::string>& listOfBinFiles,
                 const std::size_t start, const std::size_t stop, const std::size_t nbExec, const int nbPolicy, const bool verbose)
{
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyA> TMatrixA;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyB> TMatrixB;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyC> TMatrixC;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyD> TMatrixD;
    typedef CompressedRowSparseMatrixOld<TBloc>                                 TMatrixOld;

    FnList<TMatrixA> instructions = readInstructionsFromFiles<TMatrixA>(listOfBinFiles);

    std::cout<<"Run benchByStep on "<<nbExec<<" executions with "<<nbPolicy<<" policies between step "<<start<<" and step "<<stop<<std::endl;

    // -----
    // pre load cpu
    // -----

    for (unsigned int i = 0; i < nPreload; i ++)
    {
        TMatrixA matrixPreLoad;
        playAllInstructions<TMatrixA>(matrixPreLoad, instructions);
    }

    // -----
    // execute benchmark
    // -----

    std::vector<ctime_t> timesOld;
    std::vector<ctime_t> timesA;
    std::vector<ctime_t> timesB;
    std::vector<ctime_t> timesC;
    std::vector<ctime_t> timesD;

    std::cout<<"Policy A : ";
    bool checkA = checkMatrix<TBloc, CRSBenchMechanicalPolicyA>(checkMatrixFile, listOfBinFiles, verbose);
    bool checkB = false;
    bool checkC = false;
    bool checkD = false;
    if (nbPolicy > 1)
    {
        std::cout<<"Policy B : ";
        checkB = checkMatrix<TBloc, CRSBenchMechanicalPolicyB>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 2)
    {
        std::cout<<"Policy C : ";
        checkC = checkMatrix<TBloc, CRSBenchMechanicalPolicyC>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 3)
    {
        std::cout<<"Policy D : ";
        checkD = checkMatrix<TBloc, CRSBenchMechanicalPolicyD>(checkMatrixFile, listOfBinFiles, verbose);
    }

    /// Old implementation
    timesOld.resize(stop - start + 1);

    for (std::size_t i = 0; i < nbExec; i++)
    {
        TMatrixOld matrix;
        sofa::defaulttype::CRSTraceReader<TMatrixOld, 1u> traceReader(&matrix);
        for (std::size_t step = start; step <= stop; step++)
        {
            std::vector<int>& fnIds = instructions.fnIdbyStep[step];
            auto& fnArgs = instructions.fnArgsbyStep[step];
            std::size_t nbInstruction = fnIds.size();

            ctime_t told0 = CTime::getFastTime();
            for (std::size_t j = 0; j < nbInstruction; j++)
            {
                traceReader.callFn(fnIds[j], fnArgs[j]);
            }
            ctime_t told1 = CTime::getFastTime();
            timesOld[step - start] += told1 - told0;
        }
    }

    /// Policy A
    if (checkA)
    {
        timesA.resize(stop - start + 1);

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixA matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixA, TMatrixA::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                ctime_t ta0 = CTime::getFastTime();
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
                ctime_t ta1 = CTime::getFastTime();
                timesA[step - start] += ta1 - ta0;
            }
        }
    }

    /// Policy B
    if (nbPolicy > 1 && checkB)
    {
        timesB.resize(stop - start + 1);

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixB matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixB, TMatrixB::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                ctime_t tb0 = CTime::getFastTime();
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
                ctime_t tb1 = CTime::getFastTime();
                timesB[step - start] += tb1 - tb0;
            }
        }
    }

    /// Policy C
    if (nbPolicy > 2 && checkC)
    {
        timesC.resize(stop - start + 1);

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixC matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixC, TMatrixC::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                ctime_t tc0 = CTime::getFastTime();
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
                ctime_t tc1 = CTime::getFastTime();
                timesC[step - start] += tc1 - tc0;
            }
        }
    }

    /// Policy D
    if (nbPolicy > 3 && checkD)
    {
        timesD.resize(stop - start + 1);
        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixD matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixD, TMatrixD::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                ctime_t td0 = CTime::getFastTime();
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                }
                ctime_t td1 = CTime::getFastTime();
                timesD[step - start] += td1 - td0;
            }
        }
    }

    if (!checkA && !checkB && !checkC && !checkD) return;
    else
    {
        ctime_t ticksPerSec = CTime::getTicksPerSec();
        for (std::size_t i = 0; i < timesA.size(); i++)
        {
            double tOld = 0.0;
            double tA = 0.0;
            double tB = 0.0;
            double tC = 0.0;
            double tD = 0.0;

            tOld = 0.001 * (((timesOld[i]) * 1000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
            if (checkA) tA = 0.001 * (((timesA[i]) * 1000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
            if (nbPolicy > 1 && checkB) tB = 0.001 * (((timesB[i]) * 1000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
            if (nbPolicy > 2 && checkC) tC = 0.001 * (((timesC[i]) * 1000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
            if (nbPolicy > 3 && checkD) tD = 0.001 * (((timesD[i]) * 1000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));

            std::cout << std::endl;
            std::cout << "Step : " << start + i << std::endl;
            std::cout << std::endl;
            std::cout << "CRS Old  mean : " << tOld << " ms" << std::endl;
            if (checkA) std::cout << "Policy A mean : " << tA << " ms" << std::endl;
            if (nbPolicy > 1 && checkB) std::cout << "Policy B mean : " << tB << " ms" << std::endl;
            if (nbPolicy > 2 && checkC) std::cout << "Policy C mean : " << tC << " ms" << std::endl;
            if (nbPolicy > 3 && checkD) std::cout << "Policy D mean : " << tD << " ms" << std::endl;
            std::cout << std::endl;
            std::cout << "Policy A SpeedUp : " << 100.0 * (tOld - tA) / timesOld[i] << " %" <<std::endl;
            if (nbPolicy > 1 && checkB) std::cout << "Policy B SpeedUp : " << 100.0 * (tOld - tB) / tOld << " %" <<std::endl;
            if (nbPolicy > 2 && checkC) std::cout << "Policy C SpeedUp : " << 100.0 * (tOld - tC) / tOld << " %" <<std::endl;
            if (nbPolicy > 3 && checkD) std::cout << "Policy D SpeedUp : " << 100.0 * (tOld - tD) / tOld << " %" <<std::endl;
        }
    }
}

template<class TBloc>
void benchAll(std::string& checkMatrixFile, const std::vector<std::string>& listOfBinFiles,
              const std::size_t start, const std::size_t stop, const std::size_t nbExec, const int nbPolicy, const bool verbose)
{
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyA> TMatrixA;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyB> TMatrixB;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyC> TMatrixC;
    typedef CompressedRowSparseMatrixMechanical<TBloc, CRSBenchMechanicalPolicyD> TMatrixD;
    typedef CompressedRowSparseMatrixOld<TBloc>                                 TMatrixOld;

    FnList<TMatrixA> instructions = readInstructionsFromFiles<TMatrixA>(listOfBinFiles);

    std::cout<<"Run benchAll on "<<nbExec<<" executions with "<<nbPolicy<<" policies between step "<<start<<" and step "<<stop<<std::endl;
    // -----
    // pre load cpu
    // -----

    for (unsigned int i = 0; i < nPreload; i ++)
    {
        TMatrixA matrixPreLoad;
        playAllInstructions<TMatrixA>(matrixPreLoad, instructions);
    }

    // -----
    // execute benchmark
    // -----

    std::vector<std::vector<ctime_t>> timesOld;
    std::vector<std::vector<ctime_t>> timesA;
    std::vector<std::vector<ctime_t>> timesB;
    std::vector<std::vector<ctime_t>> timesC;
    std::vector<std::vector<ctime_t>> timesD;

    std::cout<<"Policy A : ";
    bool checkA = checkMatrix<TBloc, CRSBenchMechanicalPolicyA>(checkMatrixFile, listOfBinFiles, verbose);
    bool checkB = false;
    bool checkC = false;
    bool checkD = false;
    if (nbPolicy > 1)
    {
        std::cout<<"Policy B : ";
        checkB = checkMatrix<TBloc, CRSBenchMechanicalPolicyB>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 2)
    {
        std::cout<<"Policy C : ";
        checkC = checkMatrix<TBloc, CRSBenchMechanicalPolicyC>(checkMatrixFile, listOfBinFiles, verbose);
    }
    if (nbPolicy > 3)
    {
        std::cout<<"Policy D : ";
        checkD = checkMatrix<TBloc, CRSBenchMechanicalPolicyD>(checkMatrixFile, listOfBinFiles, verbose);
    }

    /// Old implementation
    timesOld.resize(stop - start + 1);
    for (std::size_t i = 0; i < nbExec; i++)
    {
        TMatrixOld matrix;
        sofa::defaulttype::CRSTraceReader<TMatrixOld, 1u> traceReader(&matrix);
        for (std::size_t step = start; step <= stop; step++)
        {
            std::vector<int>& fnIds = instructions.fnIdbyStep[step];
            auto& fnArgs = instructions.fnArgsbyStep[step];
            std::size_t nbInstruction = fnIds.size();

            timesOld[step - start].resize(nbInstruction);
            for (std::size_t j = 0; j < nbInstruction; j++)
            {
                ctime_t told0 = CTime::getFastTime();
                traceReader.callFn(fnIds[j], fnArgs[j]);
                ctime_t told1 = CTime::getFastTime();
                timesOld[step - start][j] += told1 - told0;
            }
        }
    }

    /// Policy A
    if (checkA)
    {
        timesA.resize(stop - start + 1);

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixA matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixA, TMatrixA::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                timesA[step - start].resize(nbInstruction);
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    ctime_t t0 = CTime::getFastTime();
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                    ctime_t t1 = CTime::getFastTime();
                    timesA[step - start][j] += t1 - t0;
                }
            }
        }
    }

    /// Policy B
    if (nbPolicy > 1 && checkB)
    {
        timesB.resize(stop - start + 1);

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixB matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixB, TMatrixB::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                timesB[step - start].resize(nbInstruction);
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    ctime_t t0 = CTime::getFastTime();
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                    ctime_t t1 = CTime::getFastTime();
                    timesB[step - start][j] += t1 - t0;
                }
            }
        }
    }

    /// Policy C
    if (nbPolicy > 2 && checkC)
    {
        timesC.resize(stop - start + 1);

        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixC matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixC, TMatrixC::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                timesC[step - start].resize(nbInstruction);
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    ctime_t t0 = CTime::getFastTime();
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                    ctime_t t1 = CTime::getFastTime();
                    timesC[step - start][j] += t1 - t0;
                }
            }
        }
    }

    /// Policy D
    if (nbPolicy > 3 && checkD)
    {
        timesD.resize(stop - start + 1);
        for (std::size_t i = 0; i < nbExec; i++)
        {
            TMatrixD matrix;
            sofa::defaulttype::CRSTraceReader<TMatrixD, TMatrixD::Policy::matrixType> traceReader(&matrix);

            for (std::size_t step = start; step <= stop; step++)
            {
                std::vector<int>& fnIds = instructions.fnIdbyStep[step];
                auto& fnArgs = instructions.fnArgsbyStep[step];
                std::size_t nbInstruction = fnIds.size();

                timesD[step - start].resize(nbInstruction);
                for (std::size_t j = 0; j < nbInstruction; j++)
                {
                    ctime_t t0 = CTime::getFastTime();
                    traceReader.callFn(fnIds[j], fnArgs[j]);
                    ctime_t t1 = CTime::getFastTime();
                    timesD[step - start][j] += t1 - t0;
                }
            }
        }
    }

    if (!checkA && !checkB && !checkC && !checkD) return;
    else
    {
        ctime_t ticksPerSec = CTime::getTicksPerSec();
        for (std::size_t i = 0; i < timesA.size(); i++)
        {
            std::vector<int>& fnIds = instructions.fnIdbyStep[i];

            std::cout << std::endl;
            std::cout << "Step : " << start + i << std::endl;
            std::cout << std::endl;
            for (std::size_t j = 0; j < timesOld[i].size(); j++)
            {
                double tOld = 0.0;
                double tA = 0.0;
                double tB = 0.0;
                double tC = 0.0;
                double tD = 0.0;

                /// Process times
                tOld = 0.001 * (((timesOld[i][j]) * 1000000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
                if (checkA) tA = 0.001 * (((timesA[i][j]) * 1000000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
                if (nbPolicy > 1 && checkB) tB = 0.001 * (((timesB[i][j]) * 1000000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
                if (nbPolicy > 2 && checkC) tC = 0.001 * (((timesC[i][j]) * 1000000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));
                if (nbPolicy > 3 && checkD) tD = 0.001 * (((timesD[i][j]) * 1000000000) / (ticksPerSec * static_cast<unsigned long long>(nbExec)));

                std::cout << std::endl;
                std::cout << "FnID : " << fnIds[j] << std::endl;

                std::cout << "CRS Old  mean : " << tOld << " micro second" << std::endl;
                if (checkA) std::cout << "Policy A mean : " << tA << " micro second" << std::endl;
                if (nbPolicy > 1 && checkB) std::cout << "Policy B mean : " << tB << " micro second" << std::endl;
                if (nbPolicy > 2 && checkC) std::cout << "Policy C mean : " << tC << " micro second" << std::endl;
                if (nbPolicy > 3 && checkD) std::cout << "Policy D mean : " << tD << " micro second" << std::endl;
            }
        }
    }
}

void showHelp()
{
    std::cout<<"Usage : CompressedRowSparseMatrixMechanicalCompare_bench [option] ... [arg] ..."<<std::endl;
    std::cout<<"-a   / --accuracy  : type of bench you want to launch, default benchFast"<<std::endl;
    std::cout<<"                     0 : benchFast   = Means of nbExec executions of all instructions of all steps between start step and stop step"<<std::endl;
    std::cout<<"                     1 : benchByStep = Means of nbExec executions of all instructions for each step between start step and stop step"<<std::endl;
    std::cout<<"                     2 : benchAll    = Means of nbExec executions of each instructions for each step between start step and stop step"<<std::endl;
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
    long accuracy = 0;
    long nbExec = 1000;
    long startStep = 0;
    long endStep = -1;
    long nbPolicy = 1;
    bool verbose = false;
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

    std::vector<std::string> listOfBinFiles;
#ifdef SOFA_HAVE_ZLIB
    { // first try compressed files
        listOfBinFiles = getAllFilesInDir(matrixTraceDirectory, {}, ".gz");
    }
    if (listOfBinFiles.empty())
#endif
    { // try uncompressed files
        listOfBinFiles = getAllFilesInDir(matrixTraceDirectory, {}, ".bin");
    }
    if (listOfBinFiles.empty())
    {
        std::cout<<"ERROR: could not find trace into specified folder "<<matrixTraceDirectory << std::endl;
        return -1;
    }

    const char delim = '_';
    std::string filesName = sofa::helper::system::SetDirectory::GetFileName(listOfBinFiles[0].c_str());
    std::stringstream ss(filesName);

    std::string matrixType, tbloc;
    std::getline(ss, matrixType, delim);
    std::getline(ss, tbloc, delim);

    std::string checkFile = matrixTraceDirectory + matrixType + "_" + tbloc + "_Check.txt";

    if (endStep == -1) endStep = static_cast<long>(listOfBinFiles.size() - 1);

    if (matrixType != "CRSMechanicalMatrix")
    {
        std::cout<<"ERROR: CompressedRowSparseMatrixMechanicalCompare_bench could be only instanciated with CRSMechanical matrix trace"<<std::endl;
        return -1;
    }
    else
    {
        std::cout<<"Execute CRSMatrixMechanicalCompareBench on trace stored in subfolder "<<folderName<< " with template "<<tbloc<<std::endl;
    }

    if      (tbloc == "double")
    {
        switch (accuracy)
        {
            case 0  : benchFast  <double>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 1  : benchByStep<double>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 2  : benchAll   <double>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            default : std::cout<<"ERROR: not handled accuracy"<<std::endl; return - 1;
        }
    }
    else if (tbloc == "float")
    {
        switch (accuracy)
        {
            case 0  : benchFast  <float>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 1  : benchByStep<float>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 2  : benchAll   <float>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            default : std::cout<<"ERROR: not handled accuracy"<<std::endl; return - 1;
        }
    }
    else if (tbloc == "1f")
    {
        switch (accuracy)
        {
            case 0  : benchFast  <Mat1x1f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 1  : benchByStep<Mat1x1f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 2  : benchAll   <Mat1x1f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            default : std::cout<<"ERROR: not handled accuracy"<<std::endl; return - 1;
        }
    }
    else if (tbloc == "1d")
    {
        switch (accuracy)
        {
            case 0  : benchFast  <Mat1x1d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 1  : benchByStep<Mat1x1d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 2  : benchAll   <Mat1x1d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            default : std::cout<<"ERROR: not handled accuracy"<<std::endl; return - 1;
        }
    }
    else if (tbloc == "3f")
    {
        switch (accuracy)
        {
            case 0  : benchFast  <Mat3x3f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 1  : benchByStep<Mat3x3f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 2  : benchAll   <Mat3x3f>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            default : std::cout<<"ERROR: not handled accuracy"<<std::endl; return - 1;
        }
    }
    else if (tbloc == "3d")
    {
        switch (accuracy)
        {
            case 0  : benchFast  <Mat3x3d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 1  : benchByStep<Mat3x3d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            case 2  : benchAll   <Mat3x3d>(checkFile, listOfBinFiles, startStep, endStep, nbExec, nbPolicy, verbose); break;
            default : std::cout<<"ERROR: not handled accuracy"<<std::endl; return - 1;
        }
    }
    else
    {
        std::cout<<"ERROR: CompressedRowSparseMatrixMechanicalCompare_bench bloc template not handled"<<std::endl;
        return -1;
    }

    return 0;
}
