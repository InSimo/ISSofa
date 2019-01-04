#include "CompressedRowSparseMatrixConstraint.h"
#include <Eigen/Sparse>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <cassert>
#include <type_traits>
#include <cstdlib>

namespace sofa
{
namespace defaulttype
{


template< class TBloc >
struct CompressedRowSparseMatrixToEigenSparse
{

};

template <typename TVec>
struct CompressedRowSparseMatrixToEigenSparseVec
{
    typedef typename TVec::Real Real;
    typedef CompressedRowSparseMatrixConstraint< TVec > TCompressedRowSparseMatrix;
    typedef Eigen::SparseMatrix<Real, Eigen::RowMajor> EigenSparseMatrix;


    EigenSparseMatrix operator() (const TCompressedRowSparseMatrix& mat, std::size_t size)
    {
        std::size_t eigenMatSize = size * TVec::size();
        EigenSparseMatrix eigenMat(eigenMatSize, eigenMatSize);

        std::vector<Eigen::Triplet<Real> > triplets;

        for (auto row = mat.begin(); row != mat.end(); ++row)
        {
            for (auto col = row.begin(), colend = row.end(); col !=colend; ++col)
            {
                const TVec& vec = col.val();
                int   colIndex  = col.index() * TVec::size();

                for (std::size_t i = 0; i < TVec::size(); ++i)
                {
                    triplets.emplace_back(Eigen::Triplet<Real>( row.index(), colIndex + i, vec[i]) );
                }

            }
        }

        eigenMat.setFromTriplets(triplets.begin(), triplets.end());;
        eigenMat.makeCompressed();

        return eigenMat;
    }

};

template< int N, typename Real >
class CompressedRowSparseMatrixToEigenSparse< sofa::defaulttype::Vec<N,Real> >
    : public  CompressedRowSparseMatrixToEigenSparseVec< sofa::defaulttype::Vec<N, Real> >
{

};

template< int N, typename Real >
class CompressedRowSparseMatrixToEigenSparse< sofa::defaulttype::RigidDeriv<N, Real > >
    : public CompressedRowSparseMatrixToEigenSparseVec<sofa::defaulttype::RigidDeriv<N, Real>>
{

};


template< class TBloc >
struct EigenSparseToCompressedRowSparseMatrix
{

};


template <typename TVec>
struct EigenSparseToCompressedRowSparseMatrixVec
{
    typedef typename TVec::Real Real;
    typedef CompressedRowSparseMatrixConstraint< TVec > TCompressedRowSparseMatrix;
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> EigenSparseMatrix;


    TCompressedRowSparseMatrix operator() (const EigenSparseMatrix& eigenMat)
    {
        TCompressedRowSparseMatrix mat;

        const int* outerIndexPtr  = eigenMat.outerIndexPtr();
        const int* innerIndexPtr  = eigenMat.innerIndexPtr();
        const double* valuePtr      = eigenMat.valuePtr();

        for (int rowIndex = 0; rowIndex < eigenMat.outerSize(); ++rowIndex)
        {
            int offset      = *(outerIndexPtr + rowIndex);
            int rowNonZeros = *(outerIndexPtr + rowIndex + 1) - *(outerIndexPtr + rowIndex);

            if (rowNonZeros != 0)
            {
                auto rowIterator = mat.writeLine(rowIndex);

                int i = 0;
                const int*  colPtr = innerIndexPtr + offset;
                //const Real* valPtr = valuePtr + offset;
                int   blockIndex   = *colPtr / TVec::size();
                int   blockOffset  = *colPtr - (blockIndex * TVec::size());


                while (i != rowNonZeros)
                {
                    TVec val;
                    int currentBlockIndex = blockIndex;
                    //int currentCol   = *colPtr;
                    while (currentBlockIndex == blockIndex && i != rowNonZeros)
                    {
                        val[blockOffset] = *valuePtr; // TODO: valPtr ?
                        ++i;
                        ++colPtr;
                        ++valuePtr; // TODO: valPtr ?
                        blockIndex = *colPtr / TVec::size();
                        blockOffset = *colPtr - (blockIndex * TVec::size());
                    }

                    rowIterator.addCol(currentBlockIndex, val);
                }
            }
        }

        return mat;
    }
};

template< int N, typename Real>
class EigenSparseToCompressedRowSparseMatrix< sofa::defaulttype::Vec<N, Real> > :
    public EigenSparseToCompressedRowSparseMatrixVec<sofa::defaulttype::Vec<N, Real> >
{

};

template< int N, typename Real>
class EigenSparseToCompressedRowSparseMatrix< sofa::defaulttype::RigidDeriv<N, Real> > :
    public EigenSparseToCompressedRowSparseMatrixVec<sofa::defaulttype::RigidDeriv<N, Real> >
{

};



/// Computes lhs += jacobian^T * rhs
template< typename LhsMatrixDeriv, typename RhsMatrixDeriv, typename Real >
void addMultTransposeEigen(LhsMatrixDeriv& lhs, const Eigen::SparseMatrix<Real, Eigen::RowMajor>& jacobian, const RhsMatrixDeriv& rhs)
{
    auto rhsRowIt    = rhs.begin();
    auto rhsRowItEnd = rhs.end();

    typedef typename LhsMatrixDeriv::Data LhsDeriv;
    typedef typename RhsMatrixDeriv::Data RhsDeriv;
    typedef Eigen::SparseMatrix<Real, Eigen::RowMajor> EigenSparseMatrix;
    const EigenSparseMatrix jacobianT = jacobian.transpose();

    typedef Eigen::Matrix<typename RhsDeriv::value_type, RhsDeriv::total_size, 1> EigenRhsVector;
    typedef Eigen::Matrix<typename LhsDeriv::value_type, LhsDeriv::total_size, 1> EigenLhsVector;

    // must be passed a valid iterator
    auto isEigenSparseIteratorInsideBlock = [](const typename EigenSparseMatrix::InnerIterator& it,
                                               int bBegin, int bEnd) -> bool
    {
        assert(it);
        return (it.col() >= bBegin && it.col() < bEnd);
    };


    while (rhsRowIt != rhsRowItEnd)
    {
        auto rhsColIt = rhsRowIt.begin();
        auto rhsColItEnd = rhsRowIt.end();

        if (rhsColIt != rhsColItEnd)
        {
            auto lhsRowIt = lhs.writeLine(rhsRowIt.index());
            while (rhsColIt != rhsColItEnd)
            {
                const int bColBegin = rhsColIt.index() * RhsDeriv::total_size;
                const int bColEnd   = bColBegin + RhsDeriv::total_size;

                // read jacobianT rows, block by block
                for (int k = 0; k < jacobianT.outerSize(); k+= LhsDeriv::total_size)
                {
                    // check the next LhsDeriv::total_size rows for potential non zero values
                    // inside the block [k, bCol, k+LhsDeriv::total_size, bCol+RhsDeriv::total_size]
                    bool blockEmpty = true;
                    for (int j = 0; j < LhsDeriv::total_size; ++j)
                    {
                        typename EigenSparseMatrix::InnerIterator it(jacobianT, k+j);
                        // advance until we are either invalid or inside the block
                        while (it && 
                               !isEigenSparseIteratorInsideBlock(it,bColBegin, bColEnd) )
                        {
                            ++it;
                        }

                        if (it)
                        {
                            blockEmpty = false;
                            break;
                        }
                    }

                    if(!blockEmpty)
                    {
                        auto b = jacobianT.block(k, bColBegin, LhsDeriv::total_size, RhsDeriv::total_size);

                        LhsDeriv lhsToInsert;
                        Eigen::Map< EigenLhsVector >       lhs(lhsToInsert.ptr());
                        Eigen::Map<const EigenRhsVector >  rhs(rhsColIt.val().ptr());
                        lhs = b * rhs;

                        lhsRowIt.addCol( k / LhsDeriv::total_size, lhsToInsert);
                        
                    }

                }

                ++rhsColIt;
            }
        }

        ++rhsRowIt;
    }
}

}

}
