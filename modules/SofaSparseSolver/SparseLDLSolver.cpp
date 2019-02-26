/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
// Author: Hadrien Courtecuisse

#include <SofaSparseSolver/SparseLDLSolver.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace linearsolver
{

SOFA_DECL_CLASS(SparseLDLSolver)

int SparseLDLSolverClass = core::RegisterObject("Direct linear solver based on Sparse LDL^T factorization")
        .add< SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical<double>,sofa::defaulttype::FullVector<double> > >()
        .add< SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical<defaulttype::Mat<3,3,double> >,sofa::defaulttype::FullVector<double> > >(true)
        .add< SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical<float>,sofa::defaulttype::FullVector<float> > >()
        .add< SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical<defaulttype::Mat<3,3,float> >,sofa::defaulttype::FullVector<float> > >()
        ;

template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical<double>,sofa::defaulttype::FullVector<double> >;
template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical< defaulttype::Mat<3,3,double> >,sofa::defaulttype::FullVector<double> >;
template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical<float>,sofa::defaulttype::FullVector<float> >;
template class SOFA_SPARSE_SOLVER_API SparseLDLSolver< sofa::defaulttype::CompressedRowSparseMatrixMechanical< defaulttype::Mat<3,3,float> >,sofa::defaulttype::FullVector<float> >;

} // namespace linearsolver

} // namespace component

} // namespace sofa
