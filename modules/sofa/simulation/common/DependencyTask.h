/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#ifndef ISPHYSICS_BASE_DEPENDENCYTASK_H
#define ISPHYSICS_BASE_DEPENDENCYTASK_H

#include "initPlugin.h"
#include <MultiThreading/src/Tasks.h>

#include <vector>
#include <mutex>

ISPHYSICS_PUBLIC

namespace sofa
{

namespace simulation
{

class SOFA_ISPHYSICS_BASE_API DependencyTask : public Task
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

