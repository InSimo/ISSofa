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
#ifndef SOFA_SIMULATION_DEPENDENCYTASK_H
#define SOFA_SIMULATION_DEPENDENCYTASK_H

#include <sofa/SofaSimulation.h>
#include "Tasks.h"

#include <vector>
#include <mutex>

namespace sofa
{

namespace simulation
{

class SOFA_SIMULATION_COMMON_API DependencyTask : public Task
{
public: 

    virtual bool runTask(WorkerThread* thread);

    void addSuccessor(DependencyTask* task);

    bool hasPredecessors() const {
        return !m_predecessors.empty();
    }

    const std::vector<DependencyTask*>& getSuccessors() const {
        return m_successors;
    }

    const std::vector<DependencyTask*>& getPredecessors() const {
        return m_predecessors;
    }

    virtual void disable();

    int  getRefCount() const;

protected:

    DependencyTask();
    
    void clear();
    
    /// thread safe
    void tryAdd(WorkerThread* thread); 

private:
    std::vector<DependencyTask*>   m_successors;
    std::vector<DependencyTask*>   m_predecessors;
    int                            m_refCount;
    std::mutex                     m_refCountMutex;

};

} // namespace simulation

} // namespace sofa


#endif

