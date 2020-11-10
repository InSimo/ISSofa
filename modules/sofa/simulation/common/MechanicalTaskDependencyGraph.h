/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_MECHANICALTASKDEPENDENCYGRAPH_H
#define ISPHYSICS_BASE_MECHANICALTASKDEPENDENCYGRAPH_H

#include <ISPhysicsBase/TaskDependencyGraph.h>
#include <ISPhysicsBase/TaskDependencyGraph.inl>
#include <sofa/core/BaseState.h>

ISPHYSICS_PUBLIC

namespace isphysics
{
namespace base
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



} // namespace base
} // namespace isphysics

#endif // ISPHYSICS_BASE_MECHANICALTASKDEPENDENCYGRAPH_H

