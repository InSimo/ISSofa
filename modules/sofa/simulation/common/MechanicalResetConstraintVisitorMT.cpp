/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "MechanicalResetConstraintVisitorMT.h"

namespace isphysics
{
namespace base
{

namespace
{

template< class Object, class TasksMap >
ResetConstraintTask<Object>* findOrCreate(Object* obj, TasksMap& resetConstraintTasks, const sofa::core::ConstraintParams* cParams)
{
    ResetConstraintTask<Object>*& task = resetConstraintTasks[obj->getExecUID()];
    
    if (task == nullptr)
    {
        task = new ResetConstraintTask<Object>(obj);
    }

    task->setConstraintParams(cParams);

    return task;
}

} // namespace


template < class Object >
void ResetConstraintTask<Object>::resetConstraint()
{
    m_obj->resetConstraint(m_cParams);
}

// Special case : BaseConstraintSet::resetConstraint does not take an argument
template <>
void ResetConstraintTask<sofa::core::behavior::BaseConstraintSet>::resetConstraint()
{
    m_obj->resetConstraint();
}


sofa::simulation::Visitor::Result MechanicalResetConstraintVisitorMT::fwdMechanicalState(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseMechanicalState* mm)
{
    ISASSERT_FAST(m_cParams != nullptr);
    ResetConstraintTask<BaseMechanicalState>* task = findOrCreate(mm, m_resetConstraintsMechanicalStateTasks, m_cParams);
    task->enable(&m_taskStatus);
    m_currentThread->addStealableTask(task);
    
    return RESULT_CONTINUE;
}


sofa::simulation::Visitor::Result MechanicalResetConstraintVisitorMT::fwdMappedMechanicalState(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseMechanicalState* mm)
{
    ResetConstraintTask<BaseMechanicalState>* task = findOrCreate(mm, m_resetConstraintsMechanicalStateTasks, m_cParams);
    task->enable(&m_taskStatus);
    m_currentThread->addStealableTask(task);
    
    return RESULT_CONTINUE;
}


sofa::simulation::Visitor::Result MechanicalResetConstraintVisitorMT::fwdConstraintSet(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseConstraintSet* cs)
{
    ResetConstraintTask<BaseConstraintSet>* task = findOrCreate(cs, m_resetConstraintsConstraintSetTasks, m_cParams);
    task->enable(&m_taskStatus);
    m_currentThread->addStealableTask(task);
    
    return RESULT_CONTINUE;
}


} // namespace base
} // namespace isphysics
