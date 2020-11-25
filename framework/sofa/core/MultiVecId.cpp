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

#include "MultiVecId.h"
#include "State.inl"

namespace sofa
{

namespace core
{

template class SOFA_CORE_API  TMultiVecId<V_COORD, V_READ>;
template class SOFA_CORE_API  TMultiVecId<V_COORD, V_WRITE>;
template class SOFA_CORE_API  TMultiVecId<V_DERIV, V_READ>;
template class SOFA_CORE_API  TMultiVecId<V_DERIV, V_WRITE>;
template class SOFA_CORE_API  TMultiVecId<V_MATDERIV, V_READ>;
template class SOFA_CORE_API  TMultiVecId<V_MATDERIV, V_WRITE>;
template class SOFA_CORE_API  TMultiVecId<V_ALL, V_READ>;
template class SOFA_CORE_API  TMultiVecId<V_ALL, V_WRITE>;

}

}

