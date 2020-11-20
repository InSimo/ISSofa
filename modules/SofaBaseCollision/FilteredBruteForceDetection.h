/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
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
