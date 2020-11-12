/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_SIMULATION_MECHANICALTASKDEPENDENCYGRAPH_H
#define SOFA_SIMULATION_MECHANICALTASKDEPENDENCYGRAPH_H

#include "TaskDependencyGraph.h"
#include "TaskDependencyGraph.inl"
#include <sofa/core/BaseState.h>



namespace sofa
{
namespace simulation
{

struct MechanicalTaskDependencyTraits
{
    typedef sofa::core::BaseState TaskInputData;
    typedef sofa::core::BaseState TaskOutputData;
    typedef sofa::core::objectmodel::Base::ExecUID TaskOutputID;
    static TaskOutputID getID(const TaskOutputData* o)
    {
        return o->getExecUID();
    }
};

typedef sofa::simulation::TaskDependencyGraph< MechanicalTaskDependencyTraits > MechanicalTaskDependencyGraph;



} // namespace simulation
} // namespace sofa

#endif // SOFA_SIMULATION_MECHANICALTASKDEPENDENCYGRAPH_H

