/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef HELPERS_TASK_SCHEDULER_H
#define HELPERS_TASK_SCHEDULER_H

#include <sofa/simulation/common/Tasks.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/glut.h>
#include <sofa/helper/system/thread/CTime.h>
#include <sstream>
#include <chrono>
#include <iomanip>
#include "initPlugin.h"

ISPHYSICS_INTERNAL

namespace sofa
{
namespace simulation
{
    class TaskScheduler;
} // namespace simulation
} // namespace sofa

namespace isphysics
{

namespace base
{

struct Timer
{
    using Time = sofa::helper::system::thread::ctime_t;
    using TimeInterval = std::pair<Time, Time>;

    TimeInterval    execTime = { 0, 0 };
    Time            iterTime = 0;
};

void start(Timer& timer);
void stop(Timer& timer);

namespace taskscheduler
{

struct TaskDrawInfo
{
    /// Mouse position, used to detect which task is currently hovered
    sofa::helper::fixed_array<int, 2> mousePosition = { 0, 0 };
    Timer timer;
};

struct TaskExportInfo
{
    Timer timer;
    std::map<std::string, int> taskIdMap;
    std::ostringstream googleChartStream;
    std::ofstream logTasksFileHTML;
};

void setup(sofa::simulation::TaskScheduler& taskScheduler,
           unsigned int& lastThreadsCount,
           std::size_t threadsCount,
           bool taskLogEnabled);

void logTasksHTML(const sofa::simulation::TaskScheduler& scheduler,
                  TaskExportInfo& exportInfo,
                  float timeScale,
                  float timeOffset,
                  const float currentTime,
                  const float dt,
                  const bool displayAllTimeStepHTML,
                  std::string logTasksFileName);

void draw(const sofa::simulation::TaskScheduler& taskScheduler,
          const sofa::core::visual::VisualParams* vparams,
          const TaskDrawInfo& drawInfo,
          float lineSpacing,
          float taskTextSize,
          float timeScale,
          float timeOffset,
          float taskLabelMinDuration,
          float maxStepDurationGoal);

} // namespace taskscheduler

} // namespace base

} // namespace isphysics

#endif

