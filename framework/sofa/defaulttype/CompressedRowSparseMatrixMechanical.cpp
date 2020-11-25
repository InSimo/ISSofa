/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include "CompressedRowSparseMatrixMechanical.inl"
#include "Mat.h"

namespace sofa
{
namespace defaulttype
{


SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<float>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1f>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat2x2f>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3f>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat4x4f>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<6, 6, float> >));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<8, 8, float> >));

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<double>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1d>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat2x2d>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3d>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat4x4d>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<6,6,double> >));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<8,8,double> >));

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<float, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1f, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat2x2f, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3f, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat4x4f, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<6, 6, float>, CRSMechanicalStoreTouchedPolicy >));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<8, 8, float>, CRSMechanicalStoreTouchedPolicy >));

SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<double, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat1x1d, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat2x2d, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat3x3d, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat4x4d, CRSMechanicalStoreTouchedPolicy>));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<6, 6, double>, CRSMechanicalStoreTouchedPolicy >));
SOFA_TEMPLATE_MATRIX_CLASS_IMPL((CompressedRowSparseMatrixMechanical<Mat<8, 8, double>, CRSMechanicalStoreTouchedPolicy >));


template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<float>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat1x1f>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat2x2f>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat3x3f>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat4x4f>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<6,6,float> >;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<8,8,float> >;

template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<double>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat1x1d>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat2x2d>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat3x3d>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat4x4d>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<6, 6, double> >;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<8, 8, double> >;

template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<float, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat1x1f, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat2x2f, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat3x3f, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat4x4f, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<6, 6, float>, CRSMechanicalStoreTouchedPolicy >;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<8, 8, float>, CRSMechanicalStoreTouchedPolicy >;

template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<double, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat1x1d, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat2x2d, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat3x3d, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat4x4d, CRSMechanicalStoreTouchedPolicy>;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<6, 6, double>, CRSMechanicalStoreTouchedPolicy >;
template class SOFA_DEFAULTTYPE_API CompressedRowSparseMatrixMechanical<Mat<8, 8, double>, CRSMechanicalStoreTouchedPolicy >;

}

}
