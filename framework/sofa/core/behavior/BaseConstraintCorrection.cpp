/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#define SOFA_CORE_BEHAVIOR_BASECONSTRAINTCORRECTION_CPP

#include "BaseConstraintCorrection.h"
#include "ConstraintSolver.h"

namespace sofa
{

namespace core
{

namespace behavior
{

BaseConstraintCorrection::BaseConstraintCorrection() :
    m_constraintSolverPtr(nullptr)
{

}

BaseConstraintCorrection::~BaseConstraintCorrection()
{
    if (m_constraintSolverPtr)
    {
        m_constraintSolverPtr->unregisterConstraintCorrection(this);
    }
}

void BaseConstraintCorrection::init()
{
    Inherit1::init();

    objectmodel::BaseContext* context = this->getContext();
    context->get(m_constraintSolverPtr);
    if (!m_constraintSolverPtr)
    {
        serr << "No Constraint Solver is found in the graph " << sendl;
    }
    else
    {
        m_constraintSolverPtr->registerConstraintCorrection(this);
    }
}

} // namespace behavior

} // namespace core

} // namespace sofa
