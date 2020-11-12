/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_MAPPINGTASKS_H
#define ISPHYSICS_BASE_MAPPINGTASKS_H

#include "TaskTraits.h"

#include <ISPhysicsBase/DependencyTask.h>
#include <MultiThreading/src/Tasks.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/helper/system/config.h>
#ifndef ISSOFA_VERSION 
#include <sofa/simulation/Node.h>
#else
#include <sofa/simulation/common/Node.h>
#endif // !ISSOFA_VERSION 

#include "SofaDependencyTask.h"

#include <unordered_map>

ISPHYSICS_INTERNAL

namespace isphysics
{
namespace base
{

struct MappingPropagateTaskInfo
{
    sofa::simulation::Node          *pNode;
    const sofa::core::ExecParams    *pExecParams;
    sofa::core::VecCoordId          positionId;
    sofa::core::VecDerivId          velocityId;
    bool                            bPropagateVelocity;
};

class MappingPropagateTask : public TSofaDependencyTask< sofa::core::BaseMapping, MappingPropagateTaskInfo>
{
public:
    typedef sofa::simulation::Task::Color   Color;
    typedef TSofaDependencyTask< sofa::core::BaseMapping, MappingPropagateTaskInfo> Inherit;

    MappingPropagateTask(sofa::core::BaseMapping* mapping, std::string baseName="MappingPropagateTask" );
    
    bool run(sofa::simulation::WorkerThread* thread);
};

class MappingPropagateVelocityTask : public TSofaDependencyTask< sofa::core::BaseMapping, MappingPropagateTaskInfo>
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

} // namespace base
} // namespace isphysics

#endif // ISPHYSICS_BASE_MAPPINGTASKS_H
