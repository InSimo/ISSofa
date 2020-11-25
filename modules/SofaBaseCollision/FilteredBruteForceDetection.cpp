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
#include "FilteredBruteForceDetection.h"

#include <sofa/core/CollisionModel.h>
#include <sofa/core/ObjectFactory.h>


namespace sofa
{
namespace collision
{

SOFA_DECL_CLASS(FilteredBruteForceDetection)

int FilteredBruteForceDetectionClass = sofa::core::RegisterObject("A BruteForce narrow phase detection filtered on collision response exclusions").add< FilteredBruteForceDetection >();


FilteredBruteForceDetection::FilteredBruteForceDetection()
    : l_contactManager(initLink("contactManager","Path to the Contact Manager"))
{
}


void FilteredBruteForceDetection::init()
{
    if (!l_contactManager)
    {
        l_contactManager.set(this->getContext()->get< sofa::core::collision::ContactManager >());
        l_contactManager.updateLinks();
    }

    if (!l_contactManager)
    {
        serr << l_contactManager.getName() << " not found!" << sendl;
    }

    Inherit::init();
}


bool FilteredBruteForceDetection::keepCollisionBetween(sofa::core::CollisionModel *cm1, sofa::core::CollisionModel *cm2)
{
    std::string response = l_contactManager->getContactResponse(cm1, cm2);

    if (!response.compare("null"))
    {
		return false;
    }

	return Inherit::keepCollisionBetween(cm1, cm2);
}


} // namespace collision

} // namespace sofa