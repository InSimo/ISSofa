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
#include "Polynomial_LD.inl"

namespace sofa
{

namespace helper
{

using namespace sofa::defaulttype;
using namespace std;

#ifndef SOFA_FLOAT
template class Monomial_LD<double,1>;
template class Monomial_LD<double,2>;
template class Monomial_LD<double,3>;
template class Monomial_LD<double,4>;
template class Monomial_LD<double,5>;

template class SOFA_HELPER_API Polynomial_LD<double,1>;
template class SOFA_HELPER_API Polynomial_LD<double,2>;
template class SOFA_HELPER_API Polynomial_LD<double,3>;
template class SOFA_HELPER_API Polynomial_LD<double,4>;
template class SOFA_HELPER_API Polynomial_LD<double,5>;
#endif


//#ifndef SOFA_DOUBLE
//template class Monomial_LD<float,1>;
//template class Monomial_LD<float,2>;
//template class Monomial_LD<float,3>;
//template class Monomial_LD<float,4>;
//template class Monomial_LD<float,5>;
//
//template class SOFA_HELPER_API Polynomial_LD<float,1>;
//template class SOFA_HELPER_API Polynomial_LD<float,2>;
//template class SOFA_HELPER_API Polynomial_LD<float,3>;
//template class SOFA_HELPER_API Polynomial_LD<float,4>;
//template class SOFA_HELPER_API Polynomial_LD<float,5>;
//#endif



} // namespace helper

} // namespace sofa

