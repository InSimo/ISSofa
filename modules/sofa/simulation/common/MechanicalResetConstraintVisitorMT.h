/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_MECHANICALRESETCONSTRAINTVISITORMT_H
#define ISPHYSICS_BASE_MECHANICALRESETCONSTRAINTVISITORMT_H

#include "initPlugin.h"

#include <MultiThreading/src/Tasks.h>
#include <MultiThreading/src/TaskSchedulerBoost.h>
#ifndef ISSOFA_VERSION
#include <sofa/simulation/MechanicalVisitor.h>
#else
#include <sofa/simulation/common/MechanicalVisitor.h>
#endif // !ISSOFA_VERSION

#include <ISSystem/ISAssert.h>
#include <unordered_map>

ISPHYSICS_PUBLIC

namespace isphysics
{
namespace base
{

template< class Object >
class ResetConstraintTask : public sofa::simulation::Task
{
public:
    ResetConstraintTask(Object* obj)
        : m_obj(obj), m_cParams(nullptr), m_name("ResetConstraint_" + obj->getName())
    {}

    void disable() override
    {
        m_cParams = nullptr;
    }
   
    bool run(sofa::simulation::WorkerThread *) override
    {
        ISASSERT_FAST_MSG(m_cParams != nullptr, "Must call setConstraintParams with a valid ConstraintParams instance before attempting to run");
        resetConstraint();
        return true;
    }

    const char* getName() const override
    {
        return m_name.c_str();
    }

    void setConstraintParams(const sofa::core::ConstraintParams* cParams)
    {
        m_cParams    = cParams;
    }
    
protected:
    void resetConstraint();
    
    Object* m_obj;
    const sofa::core::ConstraintParams* m_cParams;
    std::string m_name;
};


class SOFA_ISPHYSICS_BASE_API MechanicalResetConstraintVisitorMT : public sofa::simulation::BaseMechanicalVisitor
{
public:
    using BaseMechanicalState = sofa::core::behavior::BaseMechanicalState;
    using BaseConstraintSet   = sofa::core::behavior::BaseConstraintSet;
    
    using ResetConstraintMechanicalStateMap = std::unordered_map<sofa::core::objectmodel::Base::ExecUID, ResetConstraintTask<BaseMechanicalState>*>;
    using ResetConstraintConstraintSetMap   = std::unordered_map<sofa::core::objectmodel::Base::ExecUID, ResetConstraintTask<BaseConstraintSet>*>;
    
    MechanicalResetConstraintVisitorMT(const sofa::core::ConstraintParams* params, 
                                       ResetConstraintMechanicalStateMap& resetConstraintsMechanicalStateTasks,
                                       ResetConstraintConstraintSetMap& resetConstraintsConstraintSetTasks,
                                       sofa::simulation::Task::Status& taskStatus
                                       )
        : BaseMechanicalVisitor(params),
          m_cParams(params),
          m_resetConstraintsMechanicalStateTasks(resetConstraintsMechanicalStateTasks),
          m_resetConstraintsConstraintSetTasks(resetConstraintsConstraintSetTasks),
          m_taskStatus(taskStatus),
          m_currentThread(sofa::simulation::WorkerThread::GetCurrent()) // default to the thread that created the visitor
    {
#ifdef SOFA_DUMP_VISITOR_INFO
        setReadWriteVectors();
#endif
    }

    Result fwdMechanicalState(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseMechanicalState* mm) override;
    Result fwdMappedMechanicalState(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseMechanicalState* mm) override;
    Result fwdConstraintSet(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseConstraintSet* cs) override;

    // This visitor must go through all mechanical mappings, even if isMechanical flag is disabled
    bool stopAtMechanicalMapping(sofa::simulation::Node* /*node*/, sofa::core::BaseMapping* /*map*/) override
    {
        return false; // !map->isMechanical();
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    const char* getClassName() const override { return "MechanicalResetConstraintVisitorMT"; }

    /// Specify whether this action can be parallelized.
    bool isThreadSafe() const override
    {
        return true;
    }
#ifdef SOFA_DUMP_VISITOR_INFO
    void setReadWriteVectors()
    {
    }
#endif
    
    /// Change the thread which will receive created tasks
    void setExecutionThread(sofa::simulation::WorkerThread* thread)
    {
        m_currentThread = thread;
    }
    
protected:
    const sofa::core::ConstraintParams*     m_cParams;
    ResetConstraintMechanicalStateMap&      m_resetConstraintsMechanicalStateTasks;
    ResetConstraintConstraintSetMap&        m_resetConstraintsConstraintSetTasks;
    sofa::simulation::WorkerThread*         m_currentThread;
    sofa::simulation::Task::Status&         m_taskStatus;
};


} // namespace base
} // namespace isphysics

#endif
