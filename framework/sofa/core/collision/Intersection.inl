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
#ifndef SOFA_CORE_COLLISION_INTERSECTION_INL
#define SOFA_CORE_COLLISION_INTERSECTION_INL

#include <sofa/core/collision/Intersection.h>
#include <sofa/helper/Factory.h>

namespace sofa
{

namespace core
{

namespace collision
{

template<class Elem1, class Elem2, class T>
class MemberElementIntersector : public ElementIntersector
{
public:
    typedef typename Elem1::Model Model1;
    typedef typename Elem2::Model Model2;
    MemberElementIntersector(T* ptr) : impl(ptr) {}
    /// Test if 2 elements can collide. Note that this can be conservative (i.e. return true even when no collision is present)
    bool canIntersect(core::CollisionElementIterator elem1, core::CollisionElementIterator elem2)
    {
        Model1* m1 = static_cast<Model1*>(elem1.getCollisionModel());
        Model2* m2 = static_cast<Model2*>(elem2.getCollisionModel());
        Elem1 e1(m1,elem1.getIndex());
        Elem2 e2(m2,elem2.getIndex());
        return impl->testIntersection(e1, e2);
    }

    /// Begin intersection tests between two collision models. Return the number of contacts written in the first (current) contacts vector.
    /// If any of the given contacts vector is NULL, then this method should allocate it.
    int beginIntersect(core::CollisionModel* model1, core::CollisionModel* model2, std::pair<DetectionOutputContainer*, DetectionOutputContainer*>& contacts)
    {
        Model1* m1 = static_cast<Model1*>(model1);
        Model2* m2 = static_cast<Model2*>(model2);
        if (contacts.first == nullptr)
        {
            contacts.first = createOutputContainer(m1, m2);
        }
        if (contacts.second == nullptr)
        {
            contacts.second = createOutputContainer(m1, m2);
        }
        return impl->beginIntersection(m1, m2, getOutputContainer(m1, m2, contacts.first));
    }

    /// Compute the intersection between 2 elements.
    int intersect(core::CollisionElementIterator elem1, core::CollisionElementIterator elem2,  DetectionOutputContainer* contacts)
    {
        Model1* m1 = static_cast<Model1*>(elem1.getCollisionModel());
        Model2* m2 = static_cast<Model2*>(elem2.getCollisionModel());
        Elem1 e1(m1, elem1.getIndex());
        Elem2 e2(m2, elem2.getIndex());
        return impl->computeIntersection(e1, e2, getOutputContainer(m1, m2, contacts));
    }

    std::string name() const
    {
        return sofa::helper::gettypename(typeid(Elem1))+std::string("-")+sofa::helper::gettypename(typeid(Elem2));
    }

    /// End intersection tests between two collision models. Return the number of contacts written in the contacts vector.
    int endIntersect(core::CollisionModel* model1, core::CollisionModel* model2, DetectionOutputContainer* contacts)
    {
        Model1* m1 = static_cast<Model1*>(model1);
        Model2* m2 = static_cast<Model2*>(model2);
        return impl->endIntersection(m1, m2, getOutputContainer(m1, m2, contacts));
    }

protected:
    T* impl;
};

template<class Model1, class Model2, class T>
void IntersectorMap::add(T* ptr)
{
    add_impl<Model1, Model2>(new MemberElementIntersector<typename Model1::Element, typename Model2::Element, T>(ptr));
}

template<class Model1, class Model2>
void IntersectorMap::ignore()
{
    add_impl<Model1, Model2>(0);
}

template<class Model1, class Model2>
void IntersectorMap::add_impl(ElementIntersector* intersector)
{
    add_impl(Model1::GetClass(), Model2::GetClass(), intersector);
}

} // namespace collision

} // namespace core

} // namespace sofa

#endif
