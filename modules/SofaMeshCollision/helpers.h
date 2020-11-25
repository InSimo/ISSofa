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
#ifndef SOFA_COMPONENT_COLLISION_HELPERS_H
#define SOFA_COMPONENT_COLLISION_HELPERS_H

#include "initMeshCollision.h"
#include <SofaBaseCollision/CubeModel.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/defaulttype/BoundingBox.h>

namespace sofa
{

namespace component
{

namespace collision
{

namespace helpers
{

template<class TCollisionModel, class TComputeElementBBoxFn>
void doComputeBoundingTree(TCollisionModel* cm, int maxDepth, bool forceUpdate, TComputeElementBBoxFn&& computeElementBBoxFn)
{
    static_assert(std::is_base_of<sofa::core::CollisionModel, TCollisionModel>::value, "Method must be called with a CollisionModel");

    CubeModel* cubeModel = cm->template createPrevious<CubeModel>();

    if (!cm->isMoving() && !cubeModel->empty() && !forceUpdate) return; // no need to recompute BBox if immobile

    const int size = cm->getSize();

    if (cm->empty())
    {
        // no hierarchy
        cubeModel->resize(0); // clear the previous bounding tree
        return;
    }

    const SReal distance = cm->getProximity();
    if (maxDepth == 0)
    {
        cubeModel->resize(1);
        defaulttype::BoundingBox bbox;
        for (int i = 0; i < size; i++)
        {
            bbox.include(computeElementBBoxFn(i, distance));
        }
        cubeModel->setLeafCube(0, std::make_pair(cm->begin(), cm->end()), bbox.minBBox(), bbox.maxBBox());
    }
    else
    {
        cubeModel->resize(size);
        for (int i = 0; i < size; i++)
        {
            defaulttype::BoundingBox bbox = computeElementBBoxFn(i, distance);
            cubeModel->setParentOf(i, bbox.minBBox(), bbox.maxBBox()); // define the bounding box of the current triangle
        }
        cubeModel->computeBoundingTree(maxDepth);
    }
}

template<class TCollisionModel>
void computeBoundingTree(TCollisionModel* cm, int maxDepth, bool forceUpdate)
{
    doComputeBoundingTree(cm, maxDepth, forceUpdate, [cm](int index, SReal distance){ return cm->computeElementBBox(index, distance); });
}

template<class TCollisionModel>
void computeContinuousBoundingTree(TCollisionModel* cm, int maxDepth, double dt, bool forceUpdate)
{
    doComputeBoundingTree(cm, maxDepth, forceUpdate, [cm, dt](int index, SReal distance) { return cm->computeElementBBox(index, distance, dt); });
}

} // namespace helpers

} // namespace collision

} // namespace component

} // namespace sofa

#endif
