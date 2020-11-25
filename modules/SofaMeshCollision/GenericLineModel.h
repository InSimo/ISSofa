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
#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H

#include "initMeshCollision.h"
#include "BaseLineModel.h"
#include <sofa/defaulttype/BoundingBox.h>

namespace sofa
{

namespace core
{
namespace behavior
{
template<class DataTypes>
class MechanicalState;
}
}

namespace component
{

namespace collision
{

template<class TCollisionModel, class TDataTypes = typename TCollisionModel::DataTypes>
class GenericLineModel : public BaseLineModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericLineModel<TCollisionModel, TDataTypes>), ((BaseLineModel)));

    typedef TCollisionModel FinalCollisionModel;
    typedef TDataTypes DataTypes;

    void init() override;

    virtual void computeBoundingTree(int maxDepth = 0) override;
    virtual void computeContinuousBoundingTree(double dt, int maxDepth = 0) override;

    /// \brief Returns the bounding box of an element.
    //virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance) = 0;
    /// \brief Returns the bounding box of an element, accounting for motions within the given timestep.
    //virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt) = 0;

    sofa::core::behavior::MechanicalState<TDataTypes>* getMechanicalState() const { return this->m_mstate; }

protected:
    GenericLineModel();

    virtual ~GenericLineModel();

    core::behavior::MechanicalState<TDataTypes>* m_mstate = nullptr;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
