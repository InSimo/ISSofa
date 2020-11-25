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
#ifndef SOFA_SIMULATION_MECHANICALPROJECTPOSITIONANDVELOCITYVISITORMT_H
#define SOFA_SIMULATION_MECHANICALPROJECTPOSITIONANDVELOCITYVISITORMT_H

#include <sofa/SofaSimulation.h>
#include "TaskDependencyGraph.h"

#include "DependencyTask.h"
#include "Tasks.h"
#include <sofa/core/objectmodel/BaseObject.h>

#ifndef ISSOFA_VERSION 
#include <sofa/simulation/MechanicalVisitor.h>
#else
#include <sofa/simulation/common/MechanicalVisitor.h>
#endif // !ISSOFA_VERSION


#include "MechanicalTaskDependencyGraph.h"
#include "ProjectiveConstraintTasks.h"



namespace sofa
{
namespace simulation
{

typedef ProjectiveConstraintTaskInfo MechanicalProjectPositionAndVelocityVisitorInfo;
/**
    * @class MechanicalProjectPositionAndVelocityVisitorMT
    * @brief [TODO]
    */
class SOFA_SIMULATION_COMMON_API MechanicalProjectPositionAndVelocityVisitorMT : public sofa::simulation::BaseMechanicalVisitor
{
public:
    typedef MechanicalProjectPositionAndVelocityVisitorInfo                                        VisitorInfo;
    using MapProjectTask = typename TaskTraits<ProjectiveConstraintTask>::TaskContainer;
    using TaskDependencyGraph =  MechanicalTaskDependencyGraph;
    using Result = sofa::simulation::Visitor::Result;

    MechanicalProjectPositionAndVelocityVisitorMT(const VisitorInfo& info,
                                                  MapProjectTask& projectTasks,
                                                  TaskDependencyGraph& taskDependencyGraph,
                                                  sofa::simulation::Task::Status& status);

    virtual Result  fwdProjectiveConstraintSet(sofa::simulation::Node* node, sofa::core::behavior::BaseProjectiveConstraintSet* projectiveConstraintSet);
    virtual Result  fwdMechanicalMapping(sofa::simulation::Node* node, sofa::core::BaseMapping* mapping);

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalProjectPositionAndVelocityVisitorMT";}
    virtual std::string getInfos() const
    {
        std::string name="x["+ m_visitorInfo.positionId.getName()+"] v["+ m_visitorInfo.velocityId.getName()+"]";
        return name;
    }

    /// Specify whether this action can be parallelized.
    virtual bool isThreadSafe() const
    {
        return false;
    }

#ifdef SOFA_DUMP_VISITOR_INFO
    void setReadWriteVectors()
    {
        addReadWriteVector(m_taskInfo.positionId);
        if (m_taskInfo.bApplyVelocity) {
            addReadWriteVector(m_taskInfo.velocityId);

#endif

private:
    VisitorInfo                                 m_visitorInfo;
    MapProjectTask&                             m_projectTasks;
    TaskDependencyGraph&                        m_taskDependencyGraph;
    sofa::simulation::Task::Status&             m_taskStatus;
};



} // namespace simulation
} // namespace sofa

#endif
