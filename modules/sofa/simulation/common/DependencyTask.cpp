/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "DependencyTask.h"
#include "TaskScheduler.h"

namespace sofa
{

namespace simulation
{

DependencyTask::DependencyTask()
:Task()
,m_refCount(0)
{
}

void
DependencyTask::disable()
{
    Task::disable();
    clear();
}

void
DependencyTask::clear()
{
    m_refCount =0;
    m_successors.clear();
    m_predecessors.clear();
}

void 
DependencyTask::addSuccessor(DependencyTask* task)
{
    if (task == this)
    {
        return; // ignore dependencies with myself
    }

    typename std::vector<DependencyTask*>::const_iterator itfind = std::find(m_successors.begin(), m_successors.end(), task);

    if (itfind == m_successors.end() ) // this will be true only if task is not already in m_successors
    {
        ++task->m_refCount;
        this->m_successors.push_back(task);
        task->m_predecessors.push_back(this);
    }
}

bool
DependencyTask::runTask(WorkerThread* thread)
{
    execThreadIndex = thread->getThreadIndex();
    execTime.first = sofa::helper::system::thread::CTime::getFastTime();

    bool res = run(thread);

    for(std::vector<DependencyTask*>::iterator it = m_successors.begin(), it_end = m_successors.end(); 
        it != it_end; ++it)
    {
        DependencyTask* successor = *it;
        successor->tryAdd(thread);
    }
    execTime.second = sofa::helper::system::thread::CTime::getFastTime();
    return res;
}

void 
DependencyTask::tryAdd(WorkerThread* thread)
{
    bool last = false;
    {
        std::lock_guard<std::mutex> lock( m_refCountMutex );
        --m_refCount;
        if(m_refCount == 0 )
        {
            last = true;
        }
    }
    if (last)
    {
        thread->addStealableTask(this);
    }
}

int 
DependencyTask::getRefCount() const
{
     return m_refCount;
}


} // namespace simulation

} // namespace sofa

