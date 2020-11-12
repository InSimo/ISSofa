/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "AccumulateTasks.h"

namespace sofa
{

namespace simulation
{

class WorkerThread;

} // namespace simulation
} // sofa

namespace sofa
{
namespace simulation
{

AccumulateTask::AccumulateTask(sofa::core::BaseMapping* mapping, std::string baseName)
    : Inherit(mapping,baseName + '_' + mapping->getClassName() + '_' + mapping->getName())
{
}

bool AccumulateTask::run(sofa::simulation::WorkerThread*)
{
    sofa::core::BaseMapping* mapping   = getObject();
    const AccumulateTaskInfo& taskInfo = getTaskInfo();
    mapping->applyJT(taskInfo.m_cparams, taskInfo.m_res, taskInfo.m_res);

    return true;
}



} // namespace simulation
} // namespace sofa
