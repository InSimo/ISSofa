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
#include <sofa/core/collision/Intersection.inl>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/helper/FnDispatcher.h>

namespace sofa
{

namespace core
{

namespace collision
{

using namespace sofa::defaulttype;

IntersectorMap::~IntersectorMap()
{
    for(InternalMap::const_iterator it = intersectorsMap.begin(), itEnd = intersectorsMap.end(); it != itEnd; ++it)
    {
        delete it->second;
    }
}

const objectmodel::BaseClass* IntersectorMap::getType(core::CollisionModel* model)
{
    const objectmodel::BaseClass* t = model->getClass();
    const std::map<const objectmodel::BaseClass*,const objectmodel::BaseClass*>::iterator it = castMap.find(t);
    if (it == castMap.end())
    {
        const objectmodel::BaseClass* t2 = t;
        for (std::set<const objectmodel::BaseClass* >::iterator it = classes.begin(); it != classes.end(); ++it)
        {
            if ((*it)->isInstance(model))
            {
                t2 = (*it);
                break;
            }
        }
        castMap.insert(std::make_pair(t,t2));
        return t2;
    }
    else return it->second;
}

ElementIntersector* IntersectorMap::get(core::CollisionModel* model1, core::CollisionModel* model2, bool& swapModels)
{
    const objectmodel::BaseClass* t1 = getType(model1);
    const objectmodel::BaseClass* t2 = getType(model2);
    InternalMap::iterator it = intersectorsMap.find(std::make_pair(t1,t2));
    if (it != intersectorsMap.end())
    {
        swapModels = false;
        return it->second;
    }

    it = intersectorsMap.find(std::make_pair(t2,t1));
    if (it != intersectorsMap.end())
    {
        swapModels = true;
        return it->second;
    }

    std::cerr << "ERROR: Element Intersector "
            << t1->className << "-"
            << t2->className << " NOT FOUND within :" << std::endl;
    for(InternalMap::const_iterator it = intersectorsMap.begin(), itEnd = intersectorsMap.end(); it != itEnd; ++it)
    {
        const objectmodel::BaseClass* t1 = it->first.first;
        const objectmodel::BaseClass* t2 = it->first.second;
        std::cerr << "  "
                << t1->className << "-"
                << t2->className;
        ElementIntersector* i = it->second;
        if (!i) std::cout << "  NULL";
        else
            std::cout << "  " << i->name();
        std::cout << std::endl;
    }
    std::cout << std::endl;

    insert(t1, t2, 0);
    return 0;
}

void IntersectorMap::add_impl(const objectmodel::BaseClass* c1,
        const objectmodel::BaseClass* c2,
        ElementIntersector* intersector)
{
    classes.insert(c1);
    classes.insert(c2);
    castMap.clear();
    // rebuild castMap
    for (std::set<const objectmodel::BaseClass* >::iterator it = classes.begin(); it != classes.end(); ++it)
    {
        castMap.insert(std::make_pair((*it),(*it)));
    }

    insert(c1, c2, intersector);
}

void IntersectorMap::insert(const objectmodel::BaseClass* t1, const objectmodel::BaseClass* t2, ElementIntersector* intersector)
{
    const MapValue mapValue(MapKey(t1, t2), intersector);
    InternalMap::iterator it = intersectorsMap.find(mapValue.first);
    if(it != intersectorsMap.end())
    {
        delete it->second;
        it->second = mapValue.second;
    }
    else
    {
        intersectorsMap.insert(mapValue);
    }
}

Intersection::~Intersection()
{
}

/// Test if intersection between 2 types of elements is supported, i.e. an intersection test is implemented for this combinaison of types.
/// Note that this method is deprecated in favor of findIntersector
bool Intersection::isSupported(core::CollisionElementIterator elem1, core::CollisionElementIterator elem2)
{
    bool swap;
    ElementIntersector* i = findIntersector(elem1.getCollisionModel(), elem2.getCollisionModel(), swap);
    return i != NULL;
}

} // namespace collision

} // namespace core

} // namespace sofa

