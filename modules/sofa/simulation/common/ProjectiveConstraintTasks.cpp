/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "ProjectiveConstraintTasks.h"

#include <MultiThreading/src/TaskSchedulerBoost.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/core/behavior/MultiVec.h>
#include <ISSystem/ISAssert.h>
#ifdef ISSOFA_VERSION
#include <sofa/simulation/common/VectorOperations.h>
#else
#include <sofa/simulation/VectorOperations.h>
#endif

namespace isphysics
{
namespace base
{

    ProjectiveConstraintTask::ProjectiveConstraintTask(core::behavior::BaseProjectiveConstraintSet *pProjectiveConstraintSet, std::string baseName)
        : Inherit(pProjectiveConstraintSet, baseName + "_" + pProjectiveConstraintSet->getName())
    {
    }

    
    bool ProjectiveConstraintTask::run(sofa::simulation::WorkerThread * /*pThread*/)
    {
        const ProjectiveConstraintTaskInfo& taskInfo = getTaskInfo();
        sofa::core::MechanicalParams mechanicalParams = *taskInfo.pExecParams;
        mechanicalParams.update();           
        
        sofa::simulation::common::VectorOperations vop(&mechanicalParams, taskInfo.pNode );

        sofa::core::behavior::MultiVecCoord x(&vop, taskInfo.positionId); // [TODO] store x as a member in case the task is reused
        sofa::core::behavior::MultiVecDeriv v(&vop, taskInfo.velocityId ); // [TODO] store v as a member in case the task is reused

        mechanicalParams.setX(x);
        mechanicalParams.setV(v);

        sofa::core::behavior::BaseProjectiveConstraintSet* projectiveConstraintSet = getObject();

        projectiveConstraintSet->projectPosition(&mechanicalParams, x);

        if (taskInfo.projectVelocity)
        {
            projectiveConstraintSet->projectVelocity(&mechanicalParams, v);
        }
        return true;
    }



} // namespace base
} // namespace isphysics
