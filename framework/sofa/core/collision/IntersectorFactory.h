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
#ifndef SOFA_CORE_COLLISION_INTERSECTORFACTORY_H
#define SOFA_CORE_COLLISION_INTERSECTORFACTORY_H

#include <sofa/core/CollisionModel.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/helper/FnDispatcher.h>

namespace sofa
{

namespace core
{

namespace collision
{

template<class TIntersectionClass>
class BaseIntersectorCreator
{
public:
    virtual ~BaseIntersectorCreator() {}

    virtual void addIntersectors(TIntersectionClass* object) = 0;

    virtual std::string name() const = 0;
};

template<class TIntersectionClass>
class IntersectorFactory
{
protected:
    typedef BaseIntersectorCreator<TIntersectionClass> Creator;
    typedef std::vector<Creator*> CreatorVector;
    CreatorVector creatorVector;

public:

    bool registerCreator(Creator* creator)
    {
        creatorVector.push_back(creator);
        return true;
    }

    void addIntersectors(TIntersectionClass* object)
    {
        for (Creator* creator : creatorVector)
        {
            creator->addIntersectors(object);
        }
    }

    static IntersectorFactory<TIntersectionClass>* getInstance();
};

template<class TIntersectionClass, class TIntersectorClass>
class IntersectorCreator : public BaseIntersectorCreator<TIntersectionClass>
{
public:
    IntersectorCreator(std::string name) : m_name(name)
    {
        IntersectorFactory<TIntersectionClass>::getInstance()->registerCreator(this);
    }
    virtual ~IntersectorCreator() {}

    virtual void addIntersectors(TIntersectionClass* object)
    {
        new TIntersectorClass(object);
    }

    virtual std::string name() const { return m_name; }
protected:
    std::string m_name;
};

} // namespace collision

} // namespace core

} // namespace sofa

#endif
