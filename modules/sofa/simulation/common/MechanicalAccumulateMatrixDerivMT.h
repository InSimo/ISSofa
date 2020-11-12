/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_SIMULATION_MECHANICALACCUMULATEMATRIXDERIVMT_H
#define SOFA_SIMULATION_MECHANICALACCUMULATEMATRIXDERIVMT_H

#include "AccumulateTasks.h"
#include "MechanicalTaskDependencyGraph.h"
#include <sofa/SofaSimulation.h>

#include "Tasks.h"


#ifndef ISSOFA_VERSION 
#include <sofa/simulation/MechanicalVisitor.h>
#else
#include <sofa/simulation/common/MechanicalVisitor.h>
#endif // !ISSOFA_VERSION

#include <unordered_map>



namespace sofa
{

namespace simulation
{

template<typename TTask>
class TaskDependencyGraph;

} // namespace simulation
}

namespace sofa
{
namespace simulation
{

class AccumulateTask;


/**
    * @class MechanicalAccumulateMatrixDerivMT
    * @brief Accumulate Jacobian matrices through the mappings up to the independant DOFs
    */
class SOFA_SIMULATION_COMMON_API MechanicalAccumulateMatrixDerivMT : public sofa::simulation::BaseMechanicalVisitor
{
public:
    typedef std::unordered_map<sofa::core::objectmodel::Base::ExecUID, AccumulateTask*>   MapAccumulateTasks;
    typedef MechanicalTaskDependencyGraph                                                 TaskDependencyGraph;

    MechanicalAccumulateMatrixDerivMT(const AccumulateTaskInfo& taskInfo,
                                      MapAccumulateTasks& accumulateTasks,
                                      TaskDependencyGraph& taskDependencyGraph,
                                      sofa::simulation::Task::Status& taskStatus)
        : BaseMechanicalVisitor(taskInfo.m_cparams)
        , m_accumulateTaskInfo(taskInfo)
        , m_accumulateTasks(accumulateTasks)
        , m_taskDependencyGraph(taskDependencyGraph)
        , m_taskStatus(taskStatus)
    {
#ifdef SOFA_DUMP_VISITOR_INFO
        setReadWriteVectors();
#endif
    }

    const core::ConstraintParams* constraintParams() const { return m_accumulateTaskInfo.m_cparams; }

    virtual void bwdMechanicalMapping(sofa::simulation::Node* /*node*/, core::BaseMapping* mapping);

    /// Return true to reverse the order of traversal of child nodes
    virtual bool childOrderReversed(sofa::simulation::Node* /*node*/) { return m_accumulateTaskInfo.m_reverseOrder; }

    /// This visitor must go through all mechanical mappings, even if isMechanical flag is disabled
    virtual bool stopAtMechanicalMapping(sofa::simulation::Node* /*node*/, core::BaseMapping* /*mapping*/)
    {
        return false; // !mapping->isMechanical();
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalAccumulateMatrixDerivMT"; }

    virtual bool isThreadSafe() const
    {
        return false;
    }

#ifdef SOFA_DUMP_VISITOR_INFO
    void setReadWriteVectors()
    {
    }
#endif

protected:
    AccumulateTaskInfo                      m_accumulateTaskInfo;
    MapAccumulateTasks&                     m_accumulateTasks;
    TaskDependencyGraph&                    m_taskDependencyGraph;
    sofa::simulation::Task::Status&         m_taskStatus;
};


} // namespace simulation
} // namespace sofa

#endif
