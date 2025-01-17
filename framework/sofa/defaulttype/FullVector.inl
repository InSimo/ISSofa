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
#ifndef SOFA_DEFAULTTYPE_FULLVECTOR_INL
#define SOFA_DEFAULTTYPE_FULLVECTOR_INL
#include "FullVector.h"

namespace sofa
{

namespace defaulttype
{

template<> void FullVector<bool>::set(Index i, SReal v)
{
    data[i] = (v!=0);
}

template<> void FullVector<bool>::add(Index i, SReal v)
{
    data[i] |= (v!=0);
}

template<> bool FullVector<bool>::dot(const FullVector<Real>& a) const
{
    Real r = false;
    for(Index i=0; i<cursize && !r; ++i)
        r = (*this)[i] && a[i];
    return r;
}

template<> double FullVector<bool>::norm() const
{
    double r = 0.0;
    for(Index i=0; i<cursize; ++i)
        r += (*this)[i] ? 1.0 : 0.0;
    return helper::rsqrt(r);
}

} // namespace defaulttype

} // namespace sofa

#endif
