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
#include "MechanicalAccumulateMatrixDerivMT.h"

#include "AccumulateTasks.h"
#include "TaskDependencyGraph.inl"

namespace sofa
{
namespace simulation
{

namespace
{

AccumulateTask* findOrCreate(sofa::core::BaseMapping* mapping,
                             MechanicalAccumulateMatrixDerivMT::MapAccumulateTasks& accumulateTasks)
{
    AccumulateTask* task = NULL;

    MechanicalAccumulateMatrixDerivMT::MapAccumulateTasks::iterator itFind = accumulateTasks.find(mapping->getExecUID());
    if (itFind != accumulateTasks.end()) {
        task = itFind->second;
    }
    else
    {
        task = new AccumulateTask(mapping);
        accumulateTasks.insert(itFind, std::make_pair(mapping->getExecUID(), task));
    }

    return task;
}

} // namespace

void MechanicalAccumulateMatrixDerivMT::bwdMechanicalMapping(sofa::simulation::Node* /*node*/, sofa::core::BaseMapping* mapping)
{
    if (mapping)
    {
        AccumulateTask* task = findOrCreate(mapping, m_accumulateTasks);
        task->enable(&m_taskStatus, m_accumulateTaskInfo);
        m_taskDependencyGraph.addTask(task, mapping->getTo(), mapping->getFrom());
    }
}



} // namespace simulation
} // namespace sofa
