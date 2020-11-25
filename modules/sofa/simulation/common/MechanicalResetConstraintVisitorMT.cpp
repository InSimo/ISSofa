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
#include "MechanicalResetConstraintVisitorMT.h"

namespace sofa
{
namespace simulation
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
    SOFA_ASSERT_FAST(m_cParams != nullptr);
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


} // namespace simulation
} // namespace sofa
