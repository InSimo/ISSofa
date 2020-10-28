// This is benchmarking code, enforce no debug code and no container access checks in this binary
#ifdef SOFA_CHECK_CONTAINER_ACCESS
#undef SOFA_CHECK_CONTAINER_ACCESS
#endif
#ifndef NDEBUG
#define NDEBUG
#endif

#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.h>
#include <sofa/defaulttype/CompressedRowSparseMatrixConstraint.h>
#include <sofa/helper/system/SetDirectory.h>
#include <boost/filesystem.hpp>
#include <boost/range.hpp>

using namespace sofa::defaulttype;
using namespace boost::system;
namespace filesys = boost::filesystem;

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
bool checkMatrixConsistency(const TMatrixRef& mReference, TMatrix& mToCheck, bool verbose = false)
{
    enum { NL = TMatrixRef::NL };  ///< Number of rows of a block
    enum { NC = TMatrixRef::NC };  ///< Number of columns of a block

    for (std::size_t xi = 0; xi < mReference.rowIndex.size(); ++xi)
    {
        typename TMatrixRef::Range rowRange( mReference.rowBegin[xi], mReference.rowBegin[xi+1] );
        
        const int i = mReference.rowIndex[i];
        
        for (typename TMatrixRef::Index xj = rowRange.begin() ; xj < rowRange.end() ; ++xj)
        {
            const int j = mReference.colsIndex[xj];
            const typename TMatrixRef::Bloc& bRef = mReference.colsValue[xj];
            const typename TMatrix::Bloc   bCheck = mToCheck.bloc(i, j);

            for (int bi = 0; bi < NL; bi++)
            {
                for (int bj = 0; bj < NC; bj++)
                {
                    if (!double_equals(TMatrixRef::traits::v(bRef, bi, bj), TMatrix::traits::v(bCheck, bi, bj)))
                    {
                        std::cerr << "Check matrix error" << std::endl;
                        if (verbose)
                        {
                            std::cerr << "RowIndex Ref   : " << mReference.rowIndex << std::endl;
                            std::cerr << "RowIndex Check : " << mToCheck.rowIndex << std::endl;
                            std::cerr << "ColIndex Ref   : " << mReference.colsIndex << std::endl;
                            std::cerr << "ColIndex Check : " << mToCheck.colsIndex << std::endl;
                        }
                        return false;
                    }
                }
            }

        }
    }

    std::cout << "Check matrix consistency passed successfully" << std::endl;
    return true;
}
