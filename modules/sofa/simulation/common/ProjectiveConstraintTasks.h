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
#ifndef SOFA_SIMULATION_PROJECTIVECONSTRAINTTASKS_H
#define SOFA_SIMULATION_PROJECTIVECONSTRAINTTASKS_H

#include <sofa/SofaSimulation.h>

#include "TaskTraits.h"

#ifndef ISSOFA_VERSION 
#include <sofa/simulation/Node.h>
#else
#include <sofa/simulation/common/Node.h>
#endif // !ISSOFA_VERSION



#include <sofa/core/ExecParams.h>
#include <sofa/core/VecId.h>
#include "SofaDependencyTask.h"

#include <unordered_map>



namespace sofa
{
namespace simulation
{

struct SOFA_SIMULATION_COMMON_API ProjectiveConstraintTaskInfo
{
    sofa::simulation::Node          *pNode;
    const sofa::core::ExecParams    *pExecParams;
    sofa::core::VecCoordId          positionId;
    sofa::core::VecDerivId          velocityId;
    bool                            projectVelocity;
};

class SOFA_SIMULATION_COMMON_API ProjectiveConstraintTask : public TSofaDependencyTask<sofa::core::behavior::BaseProjectiveConstraintSet, ProjectiveConstraintTaskInfo>
{
public:
    typedef TSofaDependencyTask<sofa::core::behavior::BaseProjectiveConstraintSet, ProjectiveConstraintTaskInfo> Inherit;
    typedef ProjectiveConstraintTaskInfo TaskInfo;

    ProjectiveConstraintTask(sofa::core::behavior::BaseProjectiveConstraintSet *pProjectiveConstraint,
                             std::string baseName="ProjectiveConstraintTask" );
    
    bool run(sofa::simulation::WorkerThread* thread);
};



template<>
struct TaskTraits<ProjectiveConstraintTask>
{
    using Task = ProjectiveConstraintTask;
    using TaskContainer = std::unordered_map<sofa::core::objectmodel::Base::ExecUID, Task*>;

    static Task* create(sofa::core::behavior::BaseProjectiveConstraintSet* projectiveConstraintSet, const std::string& taskBaseName)
    {
        return new Task(projectiveConstraintSet, taskBaseName);
    }
};

} // namespace simulation
} // namespace sofa

#endif

