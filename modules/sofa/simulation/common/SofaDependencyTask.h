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
#ifndef SOFA_SIMULATION_SOFADEPENDENCYTASK_H
#define SOFA_SIMULATION_SOFADEPENDENCYTASK_H

#include "DependencyTask.h"
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/assert.h>



namespace sofa
{
namespace simulation
{

class BaseSofaDependencyTask : public sofa::simulation::DependencyTask
{
public:

    virtual sofa::core::objectmodel::BaseObject* getBaseObject() const = 0;
};


template< typename TSofaObject, typename TTaskInfo >
class TSofaDependencyTask : public BaseSofaDependencyTask
{
public:
    TSofaDependencyTask(TSofaObject* sofaObject, std::string baseName);

    const char* getName() const;

    sofa::core::objectmodel::BaseObject* getBaseObject() const;

    TSofaObject* getObject() const;

    void setTaskInfo(const TTaskInfo& taskInfo);

    const TTaskInfo& getTaskInfo() const;
    
    void enable(const sofa::simulation::Task::Status* pStatus, const TTaskInfo& taskInfo);

    void disable();

protected:

    TTaskInfo& getTaskInfo();

private:
    TSofaObject*    m_sofaObjectPtr;
    std::string     m_name;
    TTaskInfo       m_taskInfo;
};

template< typename TSofaObject, typename TTaskInfo >
TSofaDependencyTask<TSofaObject, TTaskInfo>::TSofaDependencyTask(TSofaObject* sofaObject, std::string baseName)
:m_sofaObjectPtr(sofaObject)
,m_name(baseName)
{
    SOFA_ASSERT_FAST(sofaObject);
}

template< typename TSofaObject, typename TTaskInfo >
const char* TSofaDependencyTask<TSofaObject, TTaskInfo>::getName() const
{
    return m_name.c_str();
}

template< typename TSofaObject, typename TTaskInfo >
sofa::core::objectmodel::BaseObject* TSofaDependencyTask<TSofaObject, TTaskInfo>::getBaseObject() const
{
    return m_sofaObjectPtr;
}

template< typename TSofaObject, typename TTaskInfo >
TSofaObject* TSofaDependencyTask<TSofaObject, TTaskInfo>::getObject() const
{
    return m_sofaObjectPtr;
}

template< typename TSofaObject, typename TTaskInfo >
void TSofaDependencyTask<TSofaObject, TTaskInfo>::setTaskInfo(const TTaskInfo& taskInfo)
{
    m_taskInfo = taskInfo;
}

template< typename TSofaObject, typename TTaskInfo >
const TTaskInfo& TSofaDependencyTask<TSofaObject, TTaskInfo>::getTaskInfo() const
{
    return m_taskInfo;
}

template< typename TSofaObject, typename TTaskInfo >
void TSofaDependencyTask<TSofaObject, TTaskInfo>::enable(const sofa::simulation::Task::Status* pStatus, const TTaskInfo& taskInfo)
{
    m_taskInfo = taskInfo;
    BaseSofaDependencyTask::enable(pStatus);
}

template< typename TSofaObject, typename TTaskInfo >
void TSofaDependencyTask<TSofaObject, TTaskInfo>::disable()
{
    BaseSofaDependencyTask::disable();
}

template< typename TSofaObject, typename TTaskInfo >
TTaskInfo& TSofaDependencyTask<TSofaObject, TTaskInfo>::getTaskInfo()
{
    return m_taskInfo;
}




} // namespace simulation
} // namespace sofa

#endif // SOFA_SIMULATION_SOFADEPENDENCYTASK_H
