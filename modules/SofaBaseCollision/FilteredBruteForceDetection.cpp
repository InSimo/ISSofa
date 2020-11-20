/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
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