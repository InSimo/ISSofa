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
#include "TaskSchedulerHelpers.h"

#include <sofa/helper/assert.h>
#include <sofa/simulation/common/Tasks.h>
#include <sofa/simulation/common/TaskScheduler.h>

namespace sofa
{

namespace simulation
{

namespace taskscheduler
{

void start(TaskTimer& timer)
{
    const TaskTimer::Time t0 = sofa::helper::system::thread::CTime::getFastTime();


    if (timer.execTime.first != timer.execTime.second) {
        timer.iterTime = t0 - timer.execTime.first;
    } else {
        timer.iterTime = 0;
    }

    timer.execTime.first = t0;
}

void stop(TaskTimer& timer)
{
    timer.execTime.second = sofa::helper::system::thread::CTime::getFastTime();
}


void setup(sofa::simulation::TaskScheduler& taskScheduler,
           unsigned int& lastThreadsCount,
           std::size_t threadsCount,
           bool taskLogEnabled)
{
    if (threadsCount != lastThreadsCount)
    {
        lastThreadsCount = threadsCount;
        taskScheduler.start(threadsCount);
    }

    for (unsigned int i = 0; i < taskScheduler.getThreadCount(); i++)
    {
        sofa::simulation::WorkerThread* workerThread = taskScheduler.getWorkerThread(i);
        SOFA_ASSERT_FAST_MSG(workerThread != nullptr, "Worker thread is nullptr !");
        workerThread->enableTaskLog(taskLogEnabled);
    }

    if (taskLogEnabled)
    {
        for (unsigned int i = 0; i < taskScheduler.getThreadCount(); i++)
        {
            sofa::simulation::WorkerThread* workerThread = taskScheduler.getWorkerThread(i);
            workerThread->clearTaskLog();
        }
    }
}

void drawFilledSquare(float x0, float y0, float x1, float y1)
{
    glBegin(GL_TRIANGLE_STRIP);
    glVertex2f(x0, y0);
    glVertex2f(x1, y0);
    glVertex2f(x0, y1);
    glVertex2f(x1, y1);
    glEnd();
}

void drawSquare(float x0, float y0, float x1, float y1)
{
    glBegin(GL_LINE_LOOP);
    glVertex2f(x0, y0);
    glVertex2f(x1, y0);
    glVertex2f(x1, y1);
    glVertex2f(x0, y1);
    glEnd();
}

void draw(const sofa::simulation::TaskScheduler& taskScheduler,
          const sofa::core::visual::VisualParams* vparams,
          const TaskDrawInfo& drawInfo,
          float lineSpacing,
          float taskTextSize,
          float timeScale,
          float timeOffset,
          float taskLabelMinDuration,
          float maxStepDurationGoal)
{


    const unsigned int nbThreads = taskScheduler.getThreadCount();

    const sofa::core::visual::VisualParams::Viewport& viewport = vparams->viewport();
    if (viewport[0] != 0 || viewport[1] != 0)
    {
        return; // do not display metrics in viewports other that the main view
    }

    float glutCharacterHeight = (float)glutStrokeWidth(GLUT_STROKE_MONO_ROMAN, 'X');
    const float glutLineHeight = lineSpacing * glutCharacterHeight;

    float width = viewport[2] * glutCharacterHeight / taskTextSize;
    float height = viewport[3] * glutCharacterHeight / taskTextSize;
    float tscale = width / timeScale;

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0,width,0,height,-1,1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glEnable( GL_LINE_SMOOTH );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glEnable( GL_BLEND );
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glLoadIdentity();

    glLineWidth(1.0f);
    const TaskTimer& timer = drawInfo.timer;
    sofa::helper::system::thread::ctime_t ticks = sofa::helper::system::thread::CTime::getTicksPerSec();
    sofa::helper::system::thread::ctime_t t0 = timer.execTime.first + sofa::helper::system::thread::ctime_t(timeOffset * ticks);
    sofa::helper::system::thread::ctime_t tlabel = sofa::helper::system::thread::ctime_t(taskLabelMinDuration * ticks);

    if (nbThreads > 0)
    {
        sofa::helper::system::thread::ctime_t tthread0lasttaskend = 0;
        sofa::helper::system::thread::ctime_t tlasttaskend = 0;
        for (unsigned int tindex = 0; tindex < nbThreads; ++tindex)
        {
            sofa::simulation::WorkerThread* thread = taskScheduler.getWorkerThread(tindex);
            const std::vector<sofa::simulation::Task*>& tasks = thread->getTaskLog();
            if (tasks.empty()) continue;
            sofa::helper::system::thread::ctime_t tlast = tasks[tasks.size()-1]->getExecTime().second - t0;
            if (tindex == 0)
            {
                tthread0lasttaskend = tlast;
            }
            if (tlast > tlasttaskend) tlasttaskend = tlast;
        }
        {
            sofa::helper::system::thread::ctime_t t1 = tthread0lasttaskend;
            sofa::helper::system::thread::ctime_t t2 = tlasttaskend;
            sofa::helper::system::thread::ctime_t t3 = (timer.execTime.second - t0);
            sofa::helper::system::thread::ctime_t t4 = timer.iterTime;
            float w1 = tscale*0.000001f*((t1*1000000)/ticks);
            float w2 = tscale*0.000001f*((t2*1000000)/ticks);
            float w3 = tscale*0.000001f*((t3*1000000)/ticks);
            float w4 = tscale*0.000001f*((t4*1000000)/ticks);
            float h1 = 1*glutLineHeight;
            float h2 = nbThreads*glutLineHeight;
            glColor4f(0.5f,0.5f,0.5f,0.5f);
            if (t1 > 0)
            {
                drawFilledSquare(0.0f, 0.0f, w1, h1);
            }
            if (t3 > t2)
            {
                drawFilledSquare(w2, 0.0f, w3, h1);
            }
            glColor4f(0.5f,0.5f,0.5f,0.25f);
            if (t2 > t1)
            {
                drawFilledSquare(w1, 0.0f, w2, h1);
            }
            if (t4 > t3)
            {
                drawFilledSquare(w3, 0.0f, w4, h1);
            }
            if (nbThreads > 1)
            {
                drawFilledSquare(0.0f, h1, w3, h2);
            }
        }

        // Convert screen pixel coordinates into GL coordinates to test which task is hovered (/!\ the glOrtho() call above changes things a bit)
        const float mouseX = drawInfo.mousePosition[0] * glutCharacterHeight / taskTextSize;
        const float mouseY = (viewport[3] - drawInfo.mousePosition[1]) * glutCharacterHeight / taskTextSize;
        sofa::simulation::Task* currentHoveredTask = nullptr;

        for (unsigned int tindex = 0; tindex < nbThreads; ++tindex)
        {
            sofa::simulation::WorkerThread* thread = taskScheduler.getWorkerThread(tindex);
            const std::vector<sofa::simulation::Task*>& tasks = thread->getTaskLog();
            for (std::size_t taskindex = 0; taskindex < tasks.size(); ++taskindex)
            {
                sofa::simulation::Task* task = tasks[taskindex];

                sofa::helper::system::thread::ctime_t t1 = task->getExecTime().first;
                t1 = (t1 >= t0) ? t1 - t0 : 0;
                sofa::helper::system::thread::ctime_t t2 = task->getExecTime().second;
                if (t2 >= t0)
                    t2 -= t0;
                else
                    continue;
                float x1 = tscale*0.000001f*((t1*1000000)/ticks);
                float x2 = tscale*0.000001f*((t2*1000000)/ticks);
                float y1 = task->getExecThreadIndex() * glutLineHeight;
                float y2 = y1 + glutLineHeight;
                sofa::simulation::Task::Color color = task->getColor();
                glColor4fv(color.ptr());
                drawFilledSquare(x1, y1, x2, y2);

                // Draw black square around the task, but only if the task is wide enough
                const float pixelWidth = (x2 - x1) / (glutCharacterHeight / taskTextSize);
                if (pixelWidth > 2)
                {
                    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
                    drawSquare(x1, y1, x2, y2);
                }

                if (t2-t1 >= tlabel)
                {
                    glTranslatef(x1+0.5f*glutCharacterHeight,y1+0.5f*glutCharacterHeight,0);
                    glLineWidth(1.0f);
                    glColor4f(1.0f,1.0f,1.0f,1.0f);
                    const char* c_str = task->getName();
                    while(*c_str)
                    {
                        glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, *c_str);
                        c_str++;
                    }
                    glLoadIdentity();
                }

                if (mouseX > x1 && mouseX < x2 && mouseY > y1 && mouseY < y2)
                {
                    currentHoveredTask = task;
                }
            }
        }

        // Draw task name and task duration as a tooltip if the mouse if hovering it
        // Draw begin and end times under the task name in the same tooltip
        if (currentHoveredTask)
        {
            // Generate drawn strings
            const float taskDurationMs = 1000.0f * currentHoveredTask->getExecDuration() / ticks;
            std::ostringstream taskTextStream;
            taskTextStream << currentHoveredTask->getName() << " (" << taskDurationMs << "ms)";
            const std::string taskText(taskTextStream.str());

            sofa::helper::system::thread::ctime_t t1 = currentHoveredTask->getExecTime().first - timer.execTime.first;
            sofa::helper::system::thread::ctime_t t2 = currentHoveredTask->getExecTime().second - timer.execTime.first;
            const float taskBeginMs = 1000.0f * t1 / ticks;
            const float taskEndMs = 1000.0f * t2 / ticks;
            std::ostringstream taskBeginEndTimesTextStream;
            taskBeginEndTimesTextStream << taskBeginMs << "ms->" << taskEndMs << "ms";
            const std::string taskBeginEndTimesText(taskBeginEndTimesTextStream.str());

            // offset by a fixed amount from the mouse pointer (above and to the right)
            float x1 = mouseX + 100;
            float x2 = x1 + glutCharacterHeight*(taskText.size()+1);
            float y1 = mouseY + 100;
            float y2 = y1 + 1.5f*glutLineHeight;

            // avoid drawing the tooltip off-screen => switch side of the mouse pointer
            if (x2 > width)
            {
                x2 = mouseX - 100;
                x1 = x2 - glutCharacterHeight*(taskText.size()+1);
            }

            // Draw transparent background rectangle
            const sofa::simulation::Task::Color color = currentHoveredTask->getColor();
            glColor4f(color[0], color[1], color[2], 0.5f);
            drawFilledSquare(x1, y1, x2, y2);

            // Draw task name and duration
            glTranslatef(x1+0.5f*glutCharacterHeight, y1+1.f*glutLineHeight, 0.0f);
            glLineWidth(1.0f);
            glColor4f(1.0f,1.0f,1.0f,1.0f);
            const char* c_str = taskText.c_str();
            while(*c_str)
            {
                glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, *c_str);
                c_str++;
            }
            glLoadIdentity();

            // Draw task begin and end times
            glTranslatef(x1+0.5f*glutCharacterHeight, y1+0.25f*glutLineHeight, 0.0f);
            glLineWidth(1.0f);
            glColor4f(1.0f,1.0f,1.0f,1.0f);
            c_str = taskBeginEndTimesText.c_str();
            while(*c_str)
            {
                glutStrokeCharacter(GLUT_STROKE_ROMAN, *c_str);
                c_str++;
            }
            glLoadIdentity();
        }
    }

    // Green bar represents maximum time that we want to allow per step
    glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
    float x = tscale*(maxStepDurationGoal - timeOffset);
    float y = glutLineHeight / 3.0f;
    glBegin(GL_TRIANGLE_STRIP);
    glVertex2f(0.0f, 0.0f);
    glVertex2f(x, 0.0f);
    glVertex2f(0.0f, y);
    glVertex2f(x, y);
    glEnd();

    glColor4f(1.0f,1.0f,1.0f,1.0f);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDisable( GL_BLEND );
    glDisable( GL_LINE_SMOOTH );
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void logTasksHTML(const sofa::simulation::TaskScheduler& scheduler,
                  TaskExportInfo& exportInfo,
                  float timeScale,
                  float timeOffset,
                  const float currentTime,
                  const float dt,
                  const bool displayAllTimeStepHTML,
                  std::string logTasksFileName)
{
    const unsigned int nbThreads = scheduler.getThreadCount();
    const float lineHeight = 15.f;

    float tscale = 4000.f / timeScale;

    const TaskTimer& timer = exportInfo.timer;
    sofa::helper::system::thread::ctime_t ticks = sofa::helper::system::thread::CTime::getTicksPerSec();
    sofa::helper::system::thread::ctime_t t0 = timer.execTime.first + sofa::helper::system::thread::ctime_t(timeOffset * ticks);

    if (nbThreads > 0)
    {
        sofa::helper::system::thread::ctime_t tlasttaskend = 0;
        for (unsigned int tindex = 0; tindex < nbThreads; ++tindex)
        {
            sofa::simulation::WorkerThread* thread = scheduler.getWorkerThread(tindex);
            const std::vector<sofa::simulation::Task*>& tasks = thread->getTaskLog();
            if (tasks.empty()) continue;
            sofa::helper::system::thread::ctime_t tlast = tasks[tasks.size() - 1]->getExecTime().second - t0;
            if (tlast > tlasttaskend) tlasttaskend = tlast;
        }

        if (!exportInfo.logTasksFileHTML.is_open())
        {
            if (logTasksFileName.empty())
            {
                const time_t timeNow = time(0);
                struct tm * timeInfo = localtime(&timeNow);
                std::ostringstream oss;
                oss << std::setfill('0')
                    << std::setw(4) << (1900 + timeInfo->tm_year) << "_"
                    << std::setw(2) << (timeInfo->tm_mon + 1) << "_"  // Month
                    << std::setw(2) << timeInfo->tm_mday << "__" // Day
                    << std::setw(2) << timeInfo->tm_hour << "_" // Hours
                    << std::setw(2) << timeInfo->tm_min << "_"  // Minutes
                    << std::setw(2) << timeInfo->tm_sec;        // Seconds

                exportInfo.logTasksFileHTML.open("logTasksResults_" + oss.str() + ".html");
            }
            else
            {
                exportInfo.logTasksFileHTML.open(logTasksFileName + ".html");
            }
            exportInfo.logTasksFileHTML.clear();
            exportInfo.logTasksFileHTML
                << "<html overflow=\"scroll\">" << std::endl
                << "<head>" << std::endl;

            // css
            exportInfo.logTasksFileHTML
                << "    <style>"
                << "        rect {"
                << "            stroke-width:0.1;"
                << "            stroke:rgb(0,0,0);"
                << "        }"
                << "    </style>";
            // js init
            exportInfo.logTasksFileHTML
                << "    <script type=\"text/javascript\" src=\"https://www.gstatic.com/charts/loader.js\"></script>" << std::endl
                << "    <script type=\"text/javascript\">" << std::endl
                << "        google.charts.load('current', {'packages':['corechart']});" << std::endl
                << "        function displayId(beginTime, endTime, timeStep) {" << std::endl
                << "            var timeStepClass = document.getElementsByClassName(\"timeStep\");" << std::endl
                << "            var i;" << std::endl
                << "            for (i = 0; i < timeStepClass.length; i++) {" << std::endl
                << "                timeStepClass[i].style.display = \"none\";" << std::endl
                << "            }" << std::endl << std::endl
                << "            if (!beginTime) {" << std::endl
                << "                beginTime = 0;" << std::endl
                << "            }" << std::endl
                << "            if (!endTime) {" << std::endl
                << "                endTime = timeStepClass.length;" << std::endl
                << "            }" << std::endl << std::endl
                << "            var time;" << std::endl
                << "            for (time = beginTime; time <= endTime; time = Math.round((Number(time) + Number(timeStep)) * 100) / 100) {" << std::endl
                << "                var timeSvg = document.getElementById(\"timeStep_\" + time);" << std::endl
                << "                if (timeSvg) {" << std::endl
                << "                    timeSvg.style.display = \"block\";" << std::endl
                << "                }" << std::endl
                << "            }" << std::endl
                << "       }" << std::endl
                << "    </script>" << std::endl
                << "</head>" << std::endl;

            exportInfo.logTasksFileHTML
                << "<body>" << std::endl;

            // description
            exportInfo.logTasksFileHTML
                << "<h1>This file shows time consumption per task for the FreeMotionAnimationLoopMT</h1>"
                << "<p><i>Note that the time displayed on the graphs is cumulated, without consideration of multithreading.</i>"
                << "</p>" << std::endl;

            exportInfo.logTasksFileHTML
                << "<div>Filter begin time</div>" << std::endl
                << "<input type=\"text\" id=\"rangeBegin\" name=\"rangeBegin\" /> <br/>" << std::endl
                << "<div>Filter end time</div>" << std::endl
                << "<input type=\"text\" id=\"rangeEnd\" name=\"rangeEnd\" /> <br/>" << std::endl
                << "<button onclick=\"displayId(document.getElementById('rangeBegin').value, document.getElementById('rangeEnd').value, " << dt << ");\">Display tasks</button>" << std::endl
                << "<button onclick=\"drawChart();\">Draw graphs</button> <br/>" << std::endl
                << "<div id=\"chart_div\" style=\"width: 1500px; height: 1000px; display:none;\"></div>" << std::endl
                << "<div id=\"curve_chart\" style=\"width: 1500px; height: 1000px; display:none;\"></div>" << std::endl
                << "<br/><br/>" << std::endl;

            exportInfo.googleChartStream
                << "<script type=\"text/javascript\">" << std::endl
                << "    function drawChart() {" << std::endl
                << "        var charDivDisplay = document.getElementById('chart_div').style.display;" << std::endl
                << "        if (charDivDisplay != \"block\") {" << std::endl
                << "            document.getElementById('chart_div').style.display = \"block\";" << std::endl
                << "            document.getElementById('curve_chart').style.display = \"block\";" << std::endl
                << "        }" << std::endl
                << "        else {" << std::endl
                << "            document.getElementById('chart_div').style.display = \"none\";" << std::endl
                << "            document.getElementById('curve_chart').style.display = \"none\";" << std::endl
                << "            return;" << std::endl
                << "        }" << std::endl
                << "        var data = new google.visualization.DataTable();" << std::endl
                << "        data.addColumn('string', 'step');" << std::endl
                << "        var colors = [];" << std::endl
                << "        var steppedOptions = {" << std::endl
                << "            title: 'Stepped Area Chart'," << std::endl
                << "            vAxis : {title: 'Time (ms)'}," << std::endl
                << "            isStacked : true," << std::endl
                << "            colors : []" << std::endl
                << "        };" << std::endl
                << "        var lineOptions = {" << std::endl
                << "            title: 'Line Chart'," << std::endl
                << "            vAxis : {title: 'Time (ms)'}," << std::endl
                << "            colors : []" << std::endl
                << "        };" << std::endl
                << "        var steppedChart = new google.visualization.SteppedAreaChart(document.getElementById('chart_div'));" << std::endl
                << "        var lineChart = new google.visualization.LineChart(document.getElementById('curve_chart'));" << std::endl
                << "        var rangeBegin = document.getElementById('rangeBegin').value;" << std::endl
                << "        var rangeEnd = document.getElementById('rangeEnd').value;" << std::endl
                << "        var row = 0;" << std::endl;
        }
        const char* displayTimeStep = displayAllTimeStepHTML ? "block" : "none";
        exportInfo.logTasksFileHTML << "<div class=\"timeStep\" id=\"timeStep_" << currentTime << "\" style=\"display:" << displayTimeStep << ";\" >" << std::endl
            << " <div>Time : " << currentTime << "</div>" << std::endl;
        const float width = tscale*0.000001f*((tlasttaskend * 1000000) / ticks) / 10;
        exportInfo.logTasksFileHTML << " <svg width = \"" << width << "\" height = \"" << nbThreads*lineHeight << "\">" << std::endl;

        std::ostringstream cellStream;
        cellStream
            << "        if ((!rangeBegin || rangeBegin < " << currentTime << ") && (!rangeEnd || " << currentTime << " < rangeEnd))" << std::endl
            << "        {" << std::endl
            << "            data.addRow();" << std::endl
            << "            var time = " << currentTime << ";" << std::endl
            << "            data.setCell(row, 0, time.toString());" << std::endl;

        std::map<std::string, int> taskIdThisStepMap;

        for (unsigned int tindex = 0; tindex < nbThreads; ++tindex)
        {
            sofa::simulation::WorkerThread* thread = scheduler.getWorkerThread(tindex);
            const std::vector<sofa::simulation::Task*>& tasks = thread->getTaskLog();
            for (std::size_t taskindex = 0; taskindex < tasks.size(); ++taskindex)
            {
                sofa::simulation::Task* task = tasks[taskindex];

                sofa::helper::system::thread::ctime_t t1 = task->getExecTime().first;
                t1 = (t1 >= t0) ? t1 - t0 : 0;
                sofa::helper::system::thread::ctime_t t2 = task->getExecTime().second;
                if (t2 >= t0)
                    t2 -= t0;
                else
                    continue;
                const float x1 = tscale*0.000001f*((t1 * 1000000) / ticks);
                const float x2 = tscale*0.000001f*((t2 * 1000000) / ticks);
                sofa::simulation::Task::Color color = task->getColor();

                // Avoid task with a size <= 1
                if (std::abs(x2 - x1) > 1.f)
                {
                    const std::string& taskName = task->getName();
                    const int red = static_cast<int>(std::floor(color[0] * 255.f)),
                              blue = static_cast<int>(std::floor(color[1] * 255.f)),
                              green = static_cast<int>(std::floor(color[2] * 255.f));
                    const float duration = static_cast<float>((t2 - t1) * 1000.0f) / ticks;

                    // google chart
                    {
                        const int instanceThisStep = taskIdThisStepMap[taskName]++;
                        const std::string taskNameInstance = taskName + ((instanceThisStep == 0) ? "" : "_" + std::to_string(instanceThisStep));

                        const bool isNewEntry = exportInfo.taskIdMap.find(taskNameInstance) == exportInfo.taskIdMap.end();

                        // New task detected
                        if (isNewEntry)
                        {
                            const int newId = exportInfo.taskIdMap.size();
                            exportInfo.taskIdMap[taskNameInstance] = newId;
                            exportInfo.googleChartStream << "        data.addColumn('number', '" << taskNameInstance << "');" << std::endl;
                            std::ostringstream redStream, blueStream, greenStream;
                            redStream << std::hex << red;
                            blueStream << std::hex << blue;
                            greenStream << std::hex << green;
                            exportInfo.googleChartStream << "        colors.push('#"
                                << ((redStream.str().size() == 1) ? "0" + redStream.str() : redStream.str())
                                << ((blueStream.str().size() == 1) ? "0" + blueStream.str() : blueStream.str())
                                << ((greenStream.str().size() == 1) ? "0" + greenStream.str() : greenStream.str())
                                << "');" << std::endl;
                        }
                        cellStream << "            data.setCell(row, " << exportInfo.taskIdMap[taskNameInstance] + 1 << ", " << duration << ");" << std::endl;
                    }

                    // SVG draw
                    {
                        exportInfo.logTasksFileHTML
                            << "  <g>" << std::endl
                            << "   <title>" << taskName << " (" << duration << "ms)</title>" << std::endl
                            << "   <rect "
                            << "x=\"" << x1 / 10.f << "\" "
                            << "width=\"" << std::abs(x2 - x1) / 10.f << "\" "
                            << "y=\"" << lineHeight * tindex << "\" "
                            << "height=\"" << lineHeight << "\" "
                            << "style=\"fill:rgb(" << red << "," << blue << "," << green << ")\""
                            << " />" << std::endl
                            << "  </g>" << std::endl;
                    }
                }
            }
        }

        cellStream
            << "            row++;" << std::endl
            << "        }" << std::endl;
        exportInfo.googleChartStream << cellStream.str();

        exportInfo.logTasksFileHTML << " </svg>" << std::endl << "</div>" << std::endl;
    }
}


} // namespace taskscheduler

} // namespace simulation

} // namespace sofa