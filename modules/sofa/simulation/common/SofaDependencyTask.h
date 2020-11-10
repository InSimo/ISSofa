/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_SOFADEPENDENCYTASK_H
#define ISPHYSICS_BASE_SOFADEPENDENCYTASK_H

#include "DependencyTask.h"
#include <sofa/core/objectmodel/BaseObject.h>
#include <ISSystem/ISAssert.h>

ISPHYSICS_PUBLIC

namespace isphysics
{
namespace base
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
    ISASSERT_FAST(sofaObject);
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




} // namespace base
} // namespace isphysics

#endif // ISPHYSICS_BASE_SOFADEPENDENCYTASK_H
