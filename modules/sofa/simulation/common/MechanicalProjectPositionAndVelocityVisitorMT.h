/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

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
