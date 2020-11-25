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

