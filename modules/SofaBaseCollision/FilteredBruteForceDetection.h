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
#ifndef ISPHYSICS_BASE_FILTEREDBRUTEFORCEDETECTION_H
#define ISPHYSICS_BASE_FILTEREDBRUTEFORCEDETECTION_H

#include <SofaBaseCollision/BruteForceDetection.h>

#include <sofa/core/collision/ContactManager.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/SofaBase.h>

namespace sofa
{
namespace collision
{

class SOFA_BASE_COLLISION_API FilteredBruteForceDetection : public sofa::component::collision::BruteForceDetection
{
public:
    SOFA_CLASS(FilteredBruteForceDetection, sofa::component::collision::BruteForceDetection);
	
	typedef sofa::component::collision::BruteForceDetection Inherit;

    typedef sofa::SingleLink<FilteredBruteForceDetection, sofa::core::collision::ContactManager, sofa::BaseLink::FLAG_STRONGLINK|sofa::BaseLink::FLAG_STOREPATH> LinkContactManager;

protected:
    FilteredBruteForceDetection();

    virtual ~FilteredBruteForceDetection(){};

    void init();

    virtual bool keepCollisionBetween(sofa::core::CollisionModel *cm1, sofa::core::CollisionModel *cm2);

    LinkContactManager l_contactManager;
};

} // namespace collision
} // namespace sofa

#endif // ISPHYSICS_BASE_FILTEREDBRUTEFORCEDETECTION_H
