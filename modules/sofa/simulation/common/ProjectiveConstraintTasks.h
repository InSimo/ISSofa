/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

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

