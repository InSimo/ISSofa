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
#ifndef SOFA_COMPONENT_COLLISION_ALGORITHM_H
#define SOFA_COMPONENT_COLLISION_ALGORITHM_H

#include "initMeshCollision.h"
#include <SofaBaseCollision/CubeModel.h>
#include <sofa/core/CollisionModel.h>

#include <stack>

namespace sofa
{

namespace component
{

namespace collision
{

namespace algorithm
{

/// Calls the given functor with each final CollisionElement lying nearby the given position.
///
/// The distance between the given position and the CollisionElements is approximated using the bounding tree of the CollisionModel:
/// the CubeModel hierarchy is explored and Cube elements which are too far from the given position are pruned.
template<class Fn>
void forEachCollisionElementNearbyPosition(const core::CollisionModel* cm, const defaulttype::Vector3& position, SReal maxDist, Fn&& functor)
{
    using CollisionElementPair = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>;

    auto isCubeCloseToPosition = [&position, maxDist](sofa::component::collision::Cube&& cube)
    {
        const defaulttype::Vector3& minVect = cube.minVect();
        const defaulttype::Vector3& maxVect = cube.maxVect();
        for (std::size_t i = 0; i < 3; i++)
        {
            if (minVect[i] > position[i] + maxDist || position[i] > maxVect[i] + maxDist)
            {
                return false;
            }
        }
        return true;
    };

    auto callFunctorWithEachExternalChild = [&functor](core::CollisionElementIterator it)
    {
        CollisionElementPair p = it.getExternalChildren();
        for (core::CollisionElementIterator itExt = p.first; itExt != p.second; ++itExt)
        {
            functor(itExt);
        }
    };

    const sofa::core::CollisionElementIterator root = const_cast<core::CollisionModel*>(cm)->getFirst()->begin(); // FIXME: these methods should be marked const in CollisionModel
    if (root.isLeaf())
    {
        callFunctorWithEachExternalChild(root);
    }
    else
    {
        std::stack<CollisionElementPair> stack;
        stack.emplace(root.getInternalChildren());

        while (!stack.empty())
        {
            CollisionElementPair p = stack.top();
            stack.pop();

            for (core::CollisionElementIterator it = p.first; it != p.second; ++it)
            {
                if (isCubeCloseToPosition(sofa::component::collision::Cube(it)))
                {
                    if (it.isLeaf())
                    {
                        callFunctorWithEachExternalChild(it);
                    }
                    else
                    {
                        stack.emplace(it.getInternalChildren());
                    }
                }
            }
        }
    }
}

} // namespace algorithm

} // namespace collision

} // namespace component

} // namespace sofa

#endif
