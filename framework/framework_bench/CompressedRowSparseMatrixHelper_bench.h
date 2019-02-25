// This is benchmarking code, enforce no debug code and no container access checks in this binary
#ifdef SOFA_CHECK_CONTAINER_ACCESS
#undef SOFA_CHECK_CONTAINER_ACCESS
#endif
#ifndef NDEBUG
#define NDEBUG
#endif

#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.h>
#include <sofa/defaulttype/CompressedRowSparseMatrixConstraint.h>
#include <boost/filesystem.hpp>
#include <boost/range.hpp>

using namespace sofa::defaulttype;
using namespace boost::system;
namespace filesys = boost::filesystem;

std::string getFileName(const std::string& s)
{
   char sep = '/';

#ifdef _WIN32
   sep = '\\';
#endif

   size_t i = s.rfind(sep, s.length());
   if (i != std::string::npos) return(s.substr(i+1, s.length() - i));
   return("");
}

bool compareFunction (std::string a, std::string b) {return a<b;}

std::vector<std::string> getAllFilesInDir(const std::string &dirPath, const std::vector<std::string> fileSkipList = { }, const std::string extension = std::string())
{
    // Create a vector of string
    std::vector<std::string> listOfFiles;
    try
    {
        if(filesys::is_directory(dirPath))
        {
            for(auto& entry : boost::make_iterator_range(filesys::directory_iterator(dirPath), {}))
            {
                filesys::path p(entry);
                if (std::find(fileSkipList.begin(), fileSkipList.end(), p.stem()) == fileSkipList.end())
                    if (extension.empty() || p.extension() == extension)
                        listOfFiles.push_back(entry.path().string());
            }
        }
    }
    catch (std::system_error & e) { std::cerr << "Exception :: " << e.what(); }

    std::sort(listOfFiles.begin(), listOfFiles.end(), compareFunction);
    return listOfFiles;
}

template<typename TMatrix>
struct FnList
{
    std::vector<std::vector<int>> fnIdbyStep;
    std::vector<std::vector<FnArgs<typename TMatrix::Bloc, typename TMatrix::DBloc>>> fnArgsbyStep;
};

template <typename TMatrix>
FnList<TMatrix> readInstructionsFromFiles(const std::vector<std::string>& listOfFiles)
{
    TMatrix temp;
    CRSTraceReader<TMatrix, TMatrix::Policy::matrixType> reader(&temp);
    SOFA_IF_CONSTEXPR (TMatrix::Policy::PrintTrace) reader.setTextFile("BenchMatrixTrace.txt");

    FnList<TMatrix> instructions;

    for (auto str : listOfFiles)
    {
        FILE* currentFile = nullptr;
        currentFile = fopen(str.c_str(), "rb");
        if (currentFile == nullptr)
        {
            std::cout<<"ERROR: when reading trace instructions from file : "<<str<<std::endl;
        }
        else
        {
            reader.setFile(currentFile);
            std::vector<int> fnIds;
            std::vector<FnArgs<typename TMatrix::Bloc, typename TMatrix::DBloc>> fnArgs;

            int fnId;
            while (reader.readFn(fnId))
            {
                fnIds.push_back(fnId);
                fnArgs.push_back(reader.readArgs(fnIds.back()));
            }
            instructions.fnIdbyStep.push_back(fnIds);
            instructions.fnArgsbyStep.push_back(fnArgs);
        }
    }
    reader.endPrint();
    return instructions;
}

template <typename TMatrix>
void playAllInstructions(TMatrix& m, FnList<TMatrix>& instructions)
{
    CRSTraceReader<TMatrix, TMatrix::Policy::matrixType> traceReader(&m);
    for (std::size_t step = 0; step < instructions.fnIdbyStep.size(); step++)
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

void readVecIndex(std::string& line, sofa::helper::vector<int>& d)
{
    std::istringstream ss(line);
    std::string temp;
    while(getline(ss, temp, ';'))
    {
        d.push_back(std::stoi(temp));
    }
}

template <typename TBloc>
void readVecBloc(std::string& line, sofa::helper::vector<TBloc>& d)
{
    std::istringstream ss(line);
    std::string temp;
    while(getline(ss, temp, ';'))
    {
        std::istringstream bstring(temp);
        TBloc b;
        bstring >> b;
        d.push_back(b);
    }
}

bool double_equals(double a, double b, double epsilon = 0.0001)
{
    return std::abs(a - b) < epsilon;
}

template< class TMatrixRef, class TMatrix>
bool checkMatrixConsistency(const std::string& checkMatrixFileName, TMatrix& mToCheck, bool verbose = false)
{
    TMatrixRef mReference;

    enum { NL = TMatrixRef::NL };  ///< Number of rows of a block
    enum { NC = TMatrixRef::NC };  ///< Number of columns of a block

    std::ifstream checkFile;
    checkFile.open(checkMatrixFileName);

    if (!checkFile.is_open())
    {
        std::cout<<"check matrix consistency cannot open check matrix file"<<std::endl;
        return true;
    }

    /// Read check file and fill mReference;
    std::string line;
    mReference.resizeBloc(mToCheck.nBlocRow, mToCheck.nBlocCol);
    getline(checkFile, line);
    readVecIndex(line, mReference.rowIndex);
    getline(checkFile, line);
    readVecIndex(line, mReference.rowBegin);
    getline(checkFile, line);
    readVecIndex(line, mReference.colsIndex);
    getline(checkFile, line);
    readVecBloc(line, mReference.colsValue);
    mReference.compress();

    for (std::size_t xi = 0; xi < mReference.rowIndex.size(); ++xi)
    {
        typename TMatrixRef::Range rowRange( mReference.rowBegin[xi], mReference.rowBegin[xi+1] );
        for (typename TMatrixRef::Index xj = rowRange.begin() ; xj < rowRange.end() ; ++xj)
        {
            SOFA_IF_CONSTEXPR (!TMatrix::StoreLowerTriangularBloc)
            {
                std::cout<<"For the moment check of matrix with StoreLowerTriangularBloc policy is not implemented"<<std::endl;
                return true;
            }
            else
            {
                const typename TMatrixRef::Bloc bRef = mReference.colsValue[xj];
                const typename TMatrix::Bloc bCheck = mToCheck.colsValue[xj];
                for (int bi = 0; bi < NL; bi++)
                {
                    for (int bj = 0; bj < NC; bj++)
                    {
                        if (!double_equals(TMatrixRef::traits::v(bRef, bi, bj), TMatrix::traits::v(bCheck, bi, bj)))
                        {
                            std::cerr << "Check matrix error" << std::endl;
                            if (verbose)
                            {
                                std::cout<<"RowIndex Ref   : "<<mReference.rowIndex<<std::endl;
                                std::cout<<"RowIndex Check : "<<mToCheck.rowIndex<<std::endl;
                                std::cout<<"ColIndex Ref   : "<<mReference.colsIndex<<std::endl;
                                std::cout<<"ColIndex Check : "<<mToCheck.colsIndex<<std::endl;
                            }
                            return false;
                        }
                    }
                }
            }
        }
    }

    std::cerr << "Check matrix consistency passed successfully" << std::endl;
    return true;
}
