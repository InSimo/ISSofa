/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_LINEARSOLVER_SparseLDLSolver_H
#define SOFA_COMPONENT_LINEARSOLVER_SparseLDLSolver_H

#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/defaulttype/FullMatrix.h>
#include <SofaBaseLinearSolver/SparseMatrix.h>
#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.h>
#include <sofa/helper/map.h>
#include <math.h>
#include <SofaSparseSolver/SparseLDLSolverImpl.h>
#include <sofa/defaulttype/BaseMatrix.h>
#include <sofa/SofaGeneral.h>

namespace sofa
{

namespace component
{

namespace linearsolver
{

/// Direct linear solver based on Sparse LDL^T factorization, implemented with the CSPARSE library
template<class TMatrix, class TVector, class TThreadManager = NoThreadManager>
class SparseLDLSolver : public sofa::component::linearsolver::SparseLDLSolverImpl<TMatrix,TVector, TThreadManager>
{
public :
    SOFA_CLASS(SOFA_TEMPLATE3(SparseLDLSolver,TMatrix,TVector,TThreadManager),SOFA_TEMPLATE3(sofa::component::linearsolver::SparseLDLSolverImpl,TMatrix,TVector,TThreadManager));

    typedef TMatrix Matrix;
    typedef TVector Vector;
    typedef typename Matrix::Real Real;
    typedef sofa::component::linearsolver::SparseLDLSolverImpl<TMatrix,TVector,TThreadManager> Inherit;
    typedef typename Inherit::ResMatrixType ResMatrixType;
    typedef typename Inherit::JMatrixType JMatrixType;
    typedef SparseLDLImplInvertData<helper::vector<int>, helper::vector<Real> > InvertData;

    template<class T>
    using FullMatrix = sofa::defaulttype::FullMatrix<T>;

    void solve (Matrix& M, Vector& x, Vector& b);
    void invert(Matrix& M);
    bool addJMInvJtLocal(TMatrix * M, ResMatrixType * result,const JMatrixType * J, double fact);
    int numStep;

    Data<bool> f_saveMatrixToFile;

    MatrixInvertData * createInvertData() {
        return new InvertData();
    }

    // /////////////////////////////////////////////////////////////
    //  API for the factorization of the bilateral constraints along with the mechanics

    virtual void solveSystem() override;
    
    virtual void buildConstitutiveConstraintsJMatrix(const sofa::core::ConstraintParams* cparams, std::size_t numConstitutiveConstraints);

    virtual void buildConstitutiveConstraintsSystemMatrix(const sofa::core::ConstraintParams* cparams);

    virtual void buildConstitutiveConstraintsHVectors(const core::ConstraintParams* cparams);

    //  end of the API for the factorization of the bilateral constraints along with the mechanics
    // /////////////////////////////////////////////////////////////

protected :
    SparseLDLSolver();

    helper::vector<int> Jlocal2global;
    FullMatrix<Real> JLinvDinv,JLinv;
    sofa::defaulttype::CompressedRowSparseMatrixMechanical<Real> Mfiltered;
//    helper::vector<Real> line,res;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_SPARSE_SOLVER)
extern template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< CompressedRowSparseMatrixMechanical< double>,FullVector<double> >;
extern template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< CompressedRowSparseMatrixMechanical< defaulttype::Mat<3,3,double> >,FullVector<double> >;
extern template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< CompressedRowSparseMatrixMechanical< float>,FullVector<float> >;
extern template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< CompressedRowSparseMatrixMechanical< defaulttype::Mat<3,3,float> >,FullVector<float> >;
#endif


} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
