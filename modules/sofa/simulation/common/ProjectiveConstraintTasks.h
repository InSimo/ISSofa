/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_PROJECTIVECONSTRAINTTASKS_H
#define ISPHYSICS_BASE_PROJECTIVECONSTRAINTTASKS_H

#include "initPlugin.h"

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

ISPHYSICS_INTERNAL

namespace isphysics
{
namespace base
{

struct ProjectiveConstraintTaskInfo
{
    sofa::simulation::Node          *pNode;
    const sofa::core::ExecParams    *pExecParams;
    sofa::core::VecCoordId          positionId;
    sofa::core::VecDerivId          velocityId;
    bool                            projectVelocity;
};

class ProjectiveConstraintTask : public TSofaDependencyTask<sofa::core::behavior::BaseProjectiveConstraintSet, ProjectiveConstraintTaskInfo>
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

} // namespace base
} // namespace isphysics

#endif

