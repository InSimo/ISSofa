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
#ifndef SOFA_COMPONENT_COLLISION_MOUSEINTERACTOR_INL
#define SOFA_COMPONENT_COLLISION_MOUSEINTERACTOR_INL

#include <SofaUserInteraction/MouseInteractor.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/gl.h>

#include <map>
namespace sofa
{

namespace component
{

namespace collision
{

template <class DataTypes>
void MouseInteractor<DataTypes>::init()
{
    BaseMouseInteractor::init();
    mouseInSofa = MouseContainer::DynamicCast(this->getContext()->getMechanicalState());
    assert(mouseInSofa);
}

}
}
}
#endif
