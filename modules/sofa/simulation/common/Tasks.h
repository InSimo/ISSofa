/*                               nulstein Evoke 2009
*
*
* ____________________________________
* Copyright 2009 Intel Corporation
* All Rights Reserved
*
* Permission is granted to use, copy, distribute and prepare derivative works of this
* software for any purpose and without fee, provided, that the above copyright notice
* and this statement appear in all copies.  Intel makes no representations about the
* suitability of this software for any purpose.  THIS SOFTWARE IS PROVIDED "AS IS."
* INTEL SPECIFICALLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, AND ALL LIABILITY,
* INCLUDING CONSEQUENTIAL AND OTHER INDIRECT DAMAGES, FOR THE USE OF THIS SOFTWARE,
* INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PROPRIETARY RIGHTS, AND INCLUDING THE
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  Intel does not
* assume any responsibility for any errors which may appear in this software nor any
* responsibility to update it.
* ____________________________________
*
*
* A multicore tasking engine in some 500 lines of C
* This is the code corresponding to the seminar on writing a task-scheduler suitable 
* for use in multicore optimisation of small prods by Jerome Muffat-Meridol.
*
* Credits :
* -=-=-=-=-
*  .music taken from M40-Southbound, by Ghaal (c)2009
*  .liposuction advice from Matt Pietrek
*     http://www.microsoft.com/msj/archive/S572.aspx
*  .ordering display list ideas based on Christer Ericson's article 
*     http://realtimecollisiondetection.net/blog/?p=86
*  .Approximate Math Library by Alex Klimovitski, Intel GmbH
*  .kkrunchy packed this exe, kudos to ryg/farbrausch
*     http://www.farbrausch.de/~fg/kkrunchy/
*/

#ifndef SOFA_SIMULATION_TASKS_H
#define SOFA_SIMULATION_TASKS_H

#include <sofa/SofaSimulation.h>
#include <sofa/helper/system/thread/CTime.h>
#include <sofa/defaulttype/Vec.h>
#include "TaskStatus.h"

namespace sofa
{

namespace simulation
{

class WorkerThread;
class TaskScheduler;

class SOFA_SIMULATION_COMMON_API Task
{
public:
    virtual ~Task();

    typedef sofa::helper::system::thread::ctime_t ctime_t;
    typedef std::pair<ctime_t,ctime_t> TimeInterval;
    typedef sofa::defaulttype::Vec4f Color;
    using Status = TaskStatus;

    virtual const char* getName() const;
    virtual Color getColor() const;
    
    virtual bool runTask(WorkerThread* thread);

    const TimeInterval& getExecTime() const { return execTime; }
    int getExecThreadIndex() const { return execThreadIndex; }
    ctime_t getExecDuration() const { return execTime.second - execTime.first; }

    static bool compareExecDuration(Task* a, Task* b)
    {
        return a->getExecDuration() < b->getExecDuration();
    }
    
    static bool compareExecDurationReverse(Task* a, Task* b)
    {
        return a->getExecDuration() > b->getExecDuration();
    }

    void enable(const Task::Status* pStatus);

    virtual void disable();

    bool isEnabled() const;

protected:
    
    virtual bool run(WorkerThread* thread) = 0;

    Task();

    inline Task::Status* getStatus(void) const;

    friend class WorkerThread;

    TimeInterval execTime;

    int execThreadIndex;

private:
    const Task::Status*	m_Status;
};

} // namespace simulation

} // namespace sofa


#include "Tasks.inl"


#endif // SOFA_SIMULATION_TASKS_H
