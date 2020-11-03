#include "EigenLLTDomainSolver.h"

namespace sofa
{
namespace component
{

namespace linearsolver
{

template< class DataTypes>
EigenLLTDomainSolver< DataTypes >::EigenLLTDomainSolver()
:l_mstate(initLink("mstate", "Path to the mechanical state associated with this domain"))
{
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::resetSystem()
{
    m_rhsId = sofa::core::ConstVecDerivId::null();
    m_lhsId = sofa::core::VecDerivId::null();
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::setSystemMBKMatrix(const sofa::core::MechanicalParams* mparams)
{
    m_matrixAccessor.clear();
    m_matrixAccessor.addMechanicalState(l_mstate.get());
    m_matrixAccessor.setGlobalMatrix(&m_MBK);
    m_matrixAccessor.setupMatrices();
    m_MBK.resize(m_matrixAccessor.getGlobalDimension(), m_matrixAccessor.getGlobalDimension());

    sofa::simulation::common::MechanicalOperations mops(mparams, this->getContext());
    mops.addMBK_ToMatrix(&m_matrixAccessor, mparams->mFactor(), mparams->bFactor(), mparams->kFactor());

    m_MBK.compress();

    setSystemEigenMatrixFromMBK();
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::setSystemMBKMatrix(int scalarSize, const VecIndex& rowIndex, const VecIndex& rowBegin, const VecIndex& colsIndex, const VecBlock& colsValue)
{
    m_MBK.clear();
    m_MBK.resize(scalarSize, scalarSize);
    m_MBK.rowIndex = rowIndex;
    m_MBK.rowBegin = rowBegin;
    m_MBK.colsIndex = colsIndex;
    m_MBK.colsValue = colsValue;

    setSystemEigenMatrixFromMBK();

}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::setSystemRHVector(sofa::core::MultiVecDerivId v)
{
    m_rhsId = v.getId(l_mstate.get());
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::setSystemLHVector(sofa::core::MultiVecDerivId v)
{
    m_lhsId = v.getId(l_mstate.get());
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::solveSystem()
{
    invertSystem();

    sofa::Data<VecDeriv>* d_lhs = l_mstate->write(m_lhsId);
    const sofa::Data<VecDeriv>* d_rhs = l_mstate->read(m_rhsId);

    auto lhs = sofa::helper::write(*d_lhs);
    auto rhs = sofa::helper::read(*d_rhs);

    double* lhsPtr       = lhs.wref()[0].ptr();
    const double* rhsPtr = rhs.ref()[0].ptr();

    solve(rhsPtr, lhsPtr);
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::writeSolution()
{
    // nothing to do, already done by solveSystem()
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::invertSystem()
{
    if (needFactorizeSymbolic())
    {
        analizePattern();
        m_pattern.rowBegin   = m_MBK.getRowBegin();
        m_pattern.colsIndex  = m_MBK.getColsIndex();
        m_pattern.size       = m_MBK.colBSize();
    }
    
    factorize();
}


template< class DataTypes >
bool EigenLLTDomainSolver< DataTypes >::buildComplianceMatrix(const sofa::core::ConstraintParams* cparams, defaulttype::BaseMatrix* result, SReal fact)
{
    sofa::Data<MatrixDeriv>* d_j = cparams->j()[l_mstate.get()].write();
    {
        auto J = sofa::helper::write(*d_j);
        J.wref().compress();
    }
    const MatrixDeriv& J = d_j->getValue(cparams);

    if (J.colsValue.empty()) return true;

    solveTriangular(J);

    assembleCompliance(result, fact);

    return true;
}

template< class DataTypes >
void EigenLLTDomainSolver< DataTypes >::applyConstraintForce(const sofa::core::ConstraintParams* cparams, sofa::core::MultiVecDerivId dxId, const sofa::defaulttype::BaseVector* f)
{
    const sofa::Data<MatrixDeriv>* d_j    = cparams->j()[l_mstate.get()].read();
    sofa::Data<VecDeriv>* d_lambda        = cparams->lambda()[l_mstate.get()].write();
    sofa::Data<VecDeriv>* d_dx            = dxId[l_mstate.get()].write();

    const MatrixDeriv& J = d_j->getValue(cparams);

    auto lambda = sofa::helper::write(*d_lambda);
    auto dx = sofa::helper::write(*d_dx);

    if (l_mstate->getSize() == 0) return;

    lambda.resize(l_mstate->getSize());

    if (J.colsValue.empty())
    {
        // make sure the corrective motion is zero
        std::fill(dx.wref().begin(), dx.wref().end(), Deriv());
        std::fill(lambda.wref().begin(), lambda.wref().end(), Deriv());
        return;
    }

    J.multTransposeBaseVector(lambda, f);


    double* lhsPtr = dx.wref()[0].ptr();
    const double* rhsPtr = lambda.ref()[0].ptr();

    solve(rhsPtr, lhsPtr);
}

template< class DataTypes>
const sofa::core::behavior::MultiMatrixAccessor* EigenLLTDomainSolver< DataTypes >::getSystemMultiMatrixAccessor() const
{
    return &m_matrixAccessor;
}

template< class DataTypes>
bool EigenLLTDomainSolver< DataTypes >::analizePattern()
{
    m_decomposition.analyzePattern(m_matrix);
    return m_decomposition.info() == Eigen::Success;
}

template< class DataTypes>
bool EigenLLTDomainSolver< DataTypes >::factorize()
{
    m_decomposition.factorize(m_matrix);
    return m_decomposition.info() == Eigen::Success;
}

template< class DataTypes>
bool EigenLLTDomainSolver< DataTypes >::solve(const double* b, double* x) const
{
    Eigen::Map<const Eigen::VectorXd> bMap(b, m_matrix.cols());
    Eigen::Map<Eigen::VectorXd>       xMap(x, m_matrix.cols());
    xMap = m_decomposition.solve(bMap);

    return m_decomposition.info() == Eigen::Success;
}


template< class DataTypes >
void EigenLLTDomainSolver< DataTypes >::solveTriangular(const MatrixDeriv& J)
{
    
    const int numConstraints = J.rowIndex.size();
    const int mstateMatrixSize = m_matrix.cols();

    m_triangularSolve.resize(numConstraints, mstateMatrixSize);

    if (mstateMatrixSize == 0) return;

    VecIndex& indexMap     = m_triangularSolve.indexMap;
    EigenRowMatrix& LinvJ  = m_triangularSolve.LinvJ;
    LinvJ.setZero();

    Eigen::VectorXd Jperm;
    Jperm.resize(mstateMatrixSize);

    for (int r = 0; r < J.rowIndex.size(); ++r)
    {
        const int i = J.rowIndex[r];
        indexMap[r]             = i;
        Jperm.setZero();

        // fill the the r-th row of LinvJ with the permutation of of the r-th row of J
        for (int colPtr = J.rowBegin[r]; colPtr < J.rowBegin[r + 1]; ++colPtr)
        {
            const int colIndexBlock = J.colsIndex[colPtr];
            const Deriv& b = J.colsValue[colPtr];

            for (int bj = 0; bj < MatrixDeriv::NC; ++bj)
            {
                const int j = colIndexBlock * MatrixDeriv::NC + bj;
                const int jperm = m_decomposition.permutationP().indices()[j];
                //LinvJ(jperm, r) = sofa::defaulttype::matrix_bloc_traits<Deriv>::v(b, 0, bj);
                Jperm[jperm] = sofa::defaulttype::matrix_bloc_traits<Deriv>::v(b, 0, bj);
            }
        }

        // perform the triangular solve on the row
        LinvJ.row(r) = m_decomposition.matrixL().solve(Jperm);
    }

    //m_decomposition.matrixL().solveInPlace(LinvJ);
    
}


template< class DataTypes >
void EigenLLTDomainSolver< DataTypes >::assembleCompliance(sofa::defaulttype::BaseMatrix* result, SReal fact) const
{
    if (m_triangularSolve.empty()) return;

    const VecIndex& indexMap = m_triangularSolve.indexMap;
    const EigenRowMatrix& LinvJ = m_triangularSolve.LinvJ;

    // JMinvJt = LinvJ.(LinvJ)^T
    // it is symmetric, so let's do the upper triangular part
    for (int i = 0; i < LinvJ.rows(); ++i)
    {
        const int irow = indexMap[i];
        result->add(irow,irow, LinvJ.row(i).dot(LinvJ.row(i)) * fact);

        for (int j = i+1; j < LinvJ.rows(); ++j)
        {
            const int jcol = indexMap[j];
            const double v = LinvJ.row(i).dot(LinvJ.row(j)) * fact;
            result->add(irow, jcol, v);
            result->add(jcol, irow, v);
        }
    }
}

template< class DataTypes>
void EigenLLTDomainSolver< DataTypes >::setSystemEigenMatrixFromMBK()
{
    m_matrix.setZero();
    m_matrix.resize(m_MBK.rowSize(), m_MBK.colSize());
    m_matrix.reserve(m_MBK.colsValue.size() * BSRMatrix::NL * BSRMatrix::NC);

    // The Eigen SparseMatrix is stored in column major format
    // Since MBK is symmetric and row major,
    // and use them to fill each column of the Eigen SparseMatrix

    for (int r = 0; r < (int)m_MBK.rowIndex.size(); ++r)
    {
        int iblock = m_MBK.rowIndex[r];
        // we fill the Eigen SparseMatrix in an orderly fashion so that it is already compressed
        // we need to scan each row NL times to access all the scalar values for this block row
        for (int bi = 0; bi < BSRMatrix::NL; ++bi)
        {
            const int i = iblock * BSRMatrix::NL + bi;
            m_matrix.startVec(i); // start column i of the Eigen SparseMatrix

            for (int colPtr = m_MBK.rowBegin[r]; colPtr < m_MBK.rowBegin[r + 1]; ++colPtr)
            {
                int jblock = m_MBK.colsIndex[colPtr];

                if (iblock > jblock) continue; // fill only the lower part of the Eigen SparseMatrix

                const Block& b = m_MBK.colsValue[colPtr];

                for (int bj = 0; bj < BSRMatrix::NC; ++bj)
                {
                    const int j = jblock * BSRMatrix::NC + bj;
                    m_matrix.insertBack(j, i) = sofa::defaulttype::matrix_bloc_traits<Block>::v(b, bi, bj);
                }
            }
        }
    }

    m_matrix.finalize();
}

template< class DataTypes>
bool EigenLLTDomainSolver< DataTypes >::needFactorizeSymbolic() const
{
    const auto& M_rowptr = m_MBK.getRowBegin();
    const auto& M_colind = m_MBK.getColsIndex();
    const int   M_size   = m_MBK.colBSize();

    const auto& P_rowptr = m_pattern.rowBegin;
    const auto& P_colind = m_pattern.colsIndex;
    const int   P_size   = m_pattern.size;

    if (M_size != P_size) return true; // not the same number of columns

    if (M_rowptr[M_size] != P_rowptr[P_size]) return true; // not the same number of nnz

    for (int i = 0; i < M_size; ++i)// test if the number of nnz of each row differ
    {
        if (M_rowptr[i] != P_rowptr[i]) return true;
    }

    for (int i = 0; i < M_rowptr[M_size]; ++i) // test if the actual column indices of each row differ
    {
        if (M_colind[i] != P_colind[i]) return true;
    }

    return false;
}



}

}

}
