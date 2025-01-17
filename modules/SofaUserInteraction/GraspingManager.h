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
#ifndef SOFA_COMPONENT_COLLISION_GRASPINGMANAGER_H
#define SOFA_COMPONENT_COLLISION_GRASPINGMANAGER_H

#include <sofa/SofaGeneral.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/collision/ContactManager.h>
#include <SofaBaseCollision/SphereModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>

#include <sofa/core/behavior/BaseController.h>
#include <set>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_USER_INTERACTION_API GraspingManager : public core::behavior::BaseController
{
public:
    SOFA_CLASS(GraspingManager,sofa::core::behavior::BaseController);

    typedef TriangleModel::DataTypes DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;

    typedef core::CollisionModel ToolModel;
    typedef core::behavior::MechanicalState<defaulttype::Vec1Types> ToolDOFs;

    Data < bool > active;
    Data < char > keyEvent;
    Data < char > keySwitchEvent;
    Data < double > openAngle;
    Data < double > closedAngle;

protected:
    std::set<ToolModel*> modelTools;
    ToolDOFs* mstateTool;
    core::collision::ContactManager* contactManager;
    bool wasActive;


    GraspingManager();

    virtual ~GraspingManager();
public:
    virtual void init();

    virtual void reset();

    virtual void handleEvent(sofa::core::objectmodel::Event* event);

    virtual void doGrasp();

};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
