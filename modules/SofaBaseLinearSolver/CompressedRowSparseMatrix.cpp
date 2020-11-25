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

#include "CompressedRowSparseMatrix.h"

namespace sofa
{

namespace component
{

namespace linearsolver
{

// FIXME: Quick and dirty way to ensure the class shared pointer creation (see TClass::CreateUnique) is not called in a multithreaded context
const sofa::core::objectmodel::BaseClassInfo* CompressedRowSparseMatrixClasses[] = {
#ifndef SOFA_FLOAT
    CompressedRowSparseMatrix<double>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<1, 1, double>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<2, 2, double>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<3, 1, double>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<3, 3, double>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<3, 6, double>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<4, 4, double>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<6, 6, double>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<8, 8, double>>::GetClass(),
#endif
#ifndef SOFA_DOUBLE
    CompressedRowSparseMatrix<float>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<1, 1, float>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<2, 2, float>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<3, 1, float>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<3, 3, float>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<3, 6, float>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<4, 4, float>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<6, 6, float>>::GetClass(),
    CompressedRowSparseMatrix<defaulttype::Mat<8, 8, float>>::GetClass(),
#endif
    NULL
};

} // namespace linearsolver

} // namespace component

} // namespace sofa