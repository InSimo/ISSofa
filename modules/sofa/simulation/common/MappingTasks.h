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
