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
#include "MappingTasks.h"

#include "TaskScheduler.h"
#ifndef ISSOFA_VERSION 
#include <sofa/simulation/VectorOperations.h>
#else
#include <sofa/simulation/common/VectorOperations.h>
#endif // !ISSOFA_VERSION


#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/MultiVec.h>

namespace sofa
{
namespace simulation
{

using namespace sofa::simulation;

MappingPropagateTask::MappingPropagateTask(sofa::core::BaseMapping *pMapping, std::string baseName)
: Inherit(pMapping, baseName + "_" + pMapping->getName() )
{
}

bool
MappingPropagateTask::run(sofa::simulation::WorkerThread *)
{
    const MappingPropagateTaskInfo& taskInfo = getTaskInfo();
    sofa::core::MechanicalParams mParams     = *taskInfo.pExecParams;
    mParams.update();

    sofa::simulation::common::VectorOperations     vop(&mParams, taskInfo.pNode );
    sofa::core::behavior::MultiVecCoord pos(&vop, taskInfo.positionId );
    sofa::core::behavior::MultiVecDeriv vel(&vop, taskInfo.velocityId );

    mParams.setX(pos);
    mParams.setV(vel);

    sofa::core::BaseMapping* mapping = getObject();

    mapping->apply( &mParams, pos, pos );

    if (taskInfo.bPropagateVelocity ) {
        mapping->applyJ( &mParams, vel, vel);
    }

    return true;
}

MappingPropagateVelocityTask::MappingPropagateVelocityTask(sofa::core::BaseMapping *pMapping, std::string baseName)
: Inherit(pMapping, baseName + "_" + pMapping->getName() )
{
}

bool
MappingPropagateVelocityTask::run(sofa::simulation::WorkerThread *)
{
    const MappingPropagateTaskInfo& taskInfo = getTaskInfo();
    sofa::core::MechanicalParams mParams     = *taskInfo.pExecParams;
    mParams.update();

    sofa::simulation::common::VectorOperations     vop(&mParams, taskInfo.pNode );
    sofa::core::behavior::MultiVecCoord pos(&vop, taskInfo.positionId );
    sofa::core::behavior::MultiVecDeriv vel(&vop, taskInfo.velocityId );

    mParams.setX(pos);
    mParams.setV(vel);

    sofa::core::BaseMapping* mapping = getObject();

    mapping->applyJ( &mParams, vel, vel);

    sofa::helper::vector<sofa::core::behavior::BaseMechanicalState*> mechTo = mapping->getMechTo();
    for (auto& mechanicateState: mechTo)
    {
        mechanicateState->vOp(&mParams, pos.id().getId(mechanicateState), sofa::core::VecCoordId::position(), vel.id().getId(mechanicateState), taskInfo.pNode->getDt());
    }

    return true;
}

} // namespace simulation
} // namespace sofa