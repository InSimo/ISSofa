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
#ifndef SOFA_SIMULATION_MECHANICALPROPAGATEPOSITIONANDVELOCITYVISITORMT_H
#define SOFA_SIMULATION_MECHANICALPROPAGATEPOSITIONANDVELOCITYVISITORMT_H


#include "TaskDependencyGraph.h"
#include "TaskTraits.h"

#include "Tasks.h"
#include <sofa/core/objectmodel/BaseObject.h>
#ifndef ISSOFA_VERSION 
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/UpdateMappingVisitor.h>
#else
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#endif // !ISSOFA_VERSION

#include "MechanicalTaskDependencyGraph.h"
#include "MappingTasks.h"



namespace sofa
{
namespace simulation
{

typedef MappingPropagateTaskInfo MechanicalPropagatePositionAndVelocityVisitoInfo;

/**
    * @class TMechanicalPropagatePositionAndVelocityVisitorMT
    * @brief [TODO]
    */
template<typename TTask>
class TMechanicalPropagatePositionAndVelocityVisitorMT : public sofa::simulation::BaseMechanicalVisitor
{
public:
    using Task = TTask;
    using MapPropagationTask = typename TaskTraits<Task>::TaskContainer;
    using TaskDependencyGraph =  MechanicalTaskDependencyGraph;

    using VisitorInfo =  MechanicalPropagatePositionAndVelocityVisitoInfo;
    using Result =  sofa::simulation::Visitor::Result;
        
    TMechanicalPropagatePositionAndVelocityVisitorMT(const VisitorInfo& info,
                                                    MapPropagationTask& mapPropagationTask,
                                                    TaskDependencyGraph& taskDependencyGraph,
                                                    sofa::simulation::Task::Status& status,
                                                    std::string taskBaseName="MappingPropagateTask" );

    virtual Result  fwdMechanicalMapping(sofa::simulation::Node *node, sofa::core::BaseMapping *mapping);

    virtual bool stopAtMechanicalMapping(sofa::simulation::Node* /*node*/, core::BaseMapping* /*map*/)
    {
        return false; // !map->isMechanical();
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "TMechanicalPropagatePositionAndVelocityVisitorMT";}
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
        }
    }
#endif

private:
    VisitorInfo                                 m_visitorInfo;
    MapPropagationTask&                         m_propagationTasks;
    TaskDependencyGraph&                        m_taskDependencyGraph;
    sofa::simulation::Task::Status&             m_taskStatus;
    std::string                                 m_taskBaseName;
};

using MechanicalPropagatePositionAndVelocityVisitorMT = TMechanicalPropagatePositionAndVelocityVisitorMT<MappingPropagateTask>;


typedef MappingPropagateTaskInfo UpdateMappingVisitorInfo;

/**
    * @class UpdateMappingVisitorMT
    * @brief [TODO]
    */
class SOFA_SIMULATION_COMMON_API UpdateMappingVisitorMT : public sofa::simulation::Visitor
{
public:
    typedef UpdateMappingVisitorInfo                                                VisitorInfo;
    typedef MechanicalPropagatePositionAndVelocityVisitorMT::MapPropagationTask     MapPropagationTask;
    typedef MechanicalPropagatePositionAndVelocityVisitorMT::TaskDependencyGraph    TaskDependencyGraph;
    typedef sofa::simulation::Visitor::Result                                       Result;
        
    UpdateMappingVisitorMT(const VisitorInfo& info,
                           MapPropagationTask& mapPropagationTask,
                           TaskDependencyGraph& taskDependencyGraph,
                           sofa::simulation::Task::Status& status);

    void processMapping(sofa::simulation::Node* node, sofa::core::BaseMapping* mapping);
    void processMechanicalMapping(sofa::simulation::Node*, sofa::core::BaseMapping* mapping);

    virtual Result processNodeTopDown(sofa::simulation::Node* node);

    /// Return a category name for this action.
    /// Only used for debugging / profiling purposes
    virtual const char* getCategoryName() const { return "mapping"; }
    virtual const char* getClassName() const { return "UpdateMappingVisitorMT"; }

    /// Specify whether this action can be parallelized.
    virtual bool isThreadSafe() const { return false; }
private:
    VisitorInfo                                 m_visitorInfo;
    MapPropagationTask&                         m_propagationTasks;
    TaskDependencyGraph&                        m_taskDependencyGraph;
    sofa::simulation::Task::Status&             m_taskStatus;
    std::string                                 m_taskBaseName;
};



} // namespace simulation
} // namespace sofa

#endif
