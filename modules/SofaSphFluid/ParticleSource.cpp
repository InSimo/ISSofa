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
#define SOFA_COMPONENT_MISC_PARTICLESOURCE_CPP
#include "ParticleSource.h"
#include "sofa/core/behavior/Constraint.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>
#include "sofa/defaulttype/Vec3Types.h"

namespace sofa
{

namespace component
{

namespace misc
{

SOFA_DECL_CLASS(ParticleSource)

int ParticleSourceClass = core::RegisterObject("Parametrable particle generator")
#ifndef SOFA_FLOAT
        .add< ParticleSource<defaulttype::Vec3dTypes> >()
        .add< ParticleSource<defaulttype::Vec2dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< ParticleSource<defaulttype::Vec3fTypes> >()
        .add< ParticleSource<defaulttype::Vec2fTypes> >()
#endif
        ;

#ifndef SOFA_FLOAT
template class SOFA_SPH_FLUID_API ParticleSource<defaulttype::Vec3dTypes>;
template class SOFA_SPH_FLUID_API ParticleSource<defaulttype::Vec2dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_SPH_FLUID_API ParticleSource<defaulttype::Vec3fTypes>;
template class SOFA_SPH_FLUID_API ParticleSource<defaulttype::Vec2fTypes>;
#endif

}
}
}
