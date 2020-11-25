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
#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_INL
#define SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_INL

#include "GenericLineModel.h"
#include "helpers.h"
#include <cassert>

namespace sofa
{

namespace component
{

namespace collision
{

template<class TCollisionModel, class TDataTypes>
GenericLineModel<TCollisionModel,TDataTypes>::GenericLineModel()
{
}

template<class TCollisionModel, class TDataTypes>
GenericLineModel<TCollisionModel,TDataTypes>::~GenericLineModel()
{
}

template<class TCollisionModel, class TDataTypes>
void GenericLineModel<TCollisionModel,TDataTypes>::init()
{
    Inherit1::init();

    m_mstate = sofa::core::behavior::MechanicalState<DataTypes>::DynamicCast(this->getContext()->getMechanicalState());

    if (!m_mstate)
    {
        serr << "No MechanicalState found" << sendl;
    }
}

template<class TCollisionModel, class TDataTypes>
void GenericLineModel<TCollisionModel,TDataTypes>::computeBoundingTree(int maxDepth)
{
    if (this->getSize() != this->getTopology()->getNbEdges())
    {
        if (m_hasTopologicalChange)
        {
            serr << "Something went wrong during topology changes, the size of the collision model does not match the size of the topology" << sendl;
        }
        else
        {
            serr << "No complete set of topology changes detected, but the size of the collision model does not match the size of the topology. Is something updating the topology directly or is the ending event missing ?" << sendl;
        }
        serr << "Collision model size = " << this->getSize() << ", topology size = " << this->getTopology()->getNbEdges() << sendl;
    }

    FinalCollisionModel* cm = static_cast<FinalCollisionModel*>(this);
    helpers::computeBoundingTree(cm, maxDepth, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

template<class TCollisionModel, class TDataTypes>
void GenericLineModel<TCollisionModel,TDataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
{
    FinalCollisionModel* cm = static_cast<FinalCollisionModel*>(this);
    helpers::computeContinuousBoundingTree(cm, maxDepth, dt, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
