/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_SIMULATION_MAPPINGTASKS_H
#define SOFA_SIMULATION_MAPPINGTASKS_H

#include <sofa/SofaSimulation.h>

#include "TaskTraits.h"

#include "DependencyTask.h"
#include "Tasks.h"
#include <sofa/core/BaseMapping.h>
#include <sofa/helper/system/config.h>
#ifndef ISSOFA_VERSION 
#include <sofa/simulation/Node.h>
#else
#include <sofa/simulation/common/Node.h>
#endif // !ISSOFA_VERSION 

#include "SofaDependencyTask.h"

#include <unordered_map>



namespace sofa
{
namespace simulation
{

struct SOFA_SIMULATION_COMMON_API MappingPropagateTaskInfo
{
    sofa::simulation::Node          *pNode;
    const sofa::core::ExecParams    *pExecParams;
    sofa::core::VecCoordId          positionId;
    sofa::core::VecDerivId          velocityId;
    bool                            bPropagateVelocity;
};

class SOFA_SIMULATION_COMMON_API MappingPropagateTask : public TSofaDependencyTask< sofa::core::BaseMapping, MappingPropagateTaskInfo>
{
public:
    typedef sofa::simulation::Task::Color   Color;
    typedef TSofaDependencyTask< sofa::core::BaseMapping, MappingPropagateTaskInfo> Inherit;

    MappingPropagateTask(sofa::core::BaseMapping* mapping, std::string baseName="MappingPropagateTask" );
    
    bool run(sofa::simulation::WorkerThread* thread);
};

class SOFA_SIMULATION_COMMON_API MappingPropagateVelocityTask : public TSofaDependencyTask< sofa::core::BaseMapping, MappingPropagateTaskInfo>
{
public:
    typedef sofa::simulation::Task::Color   Color;
    typedef TSofaDependencyTask< sofa::core::BaseMapping, MappingPropagateTaskInfo> Inherit;

    MappingPropagateVelocityTask(sofa::core::BaseMapping* mapping, std::string baseName="MappingPropagateVelocityTask" );

    bool run(sofa::simulation::WorkerThread* thread);
};




template<typename TTask>
struct MappingTaskTraitsBase
{
    using Task = TTask;
    using TaskContainer = std::unordered_map<sofa::core::objectmodel::Base::ExecUID, Task*>;

    static Task* create(sofa::core::BaseMapping* mapping, const std::string& taskBaseName)
    {
        return new Task(mapping, taskBaseName);
    }
};

template<>
struct TaskTraits<MappingPropagateTask>
    : public MappingTaskTraitsBase<MappingPropagateTask>
{};

template<>
struct TaskTraits<MappingPropagateVelocityTask>
    : public MappingTaskTraitsBase<MappingPropagateVelocityTask>
{};

} // namespace simulation
} // namespace sofa

#endif // SOFA_SIMULATION_MAPPINGTASKS_H
