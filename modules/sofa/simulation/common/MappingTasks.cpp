/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "MappingTasks.h"

#include <MultiThreading/src/TaskSchedulerBoost.h>
#ifndef ISSOFA_VERSION 
#include <sofa/simulation/VectorOperations.h>
#else
#include <sofa/simulation/common/VectorOperations.h>
#endif // !ISSOFA_VERSION


#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/MultiVec.h>

namespace isphysics
{
namespace base
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

} // namespace base
} // namespace isphysics