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
#include "ProjectiveConstraintTasks.h"

#include "TaskScheduler.h"
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/core/behavior/MultiVec.h>
#include <sofa/helper/assert.h>
#ifdef ISSOFA_VERSION
#include <sofa/simulation/common/VectorOperations.h>
#else
#include <sofa/simulation/VectorOperations.h>
#endif

namespace sofa
{
namespace simulation
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



} // namespace simulation
} // namespace sofa
