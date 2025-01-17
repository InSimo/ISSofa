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



#include <sofa/simulation/common/VelocityThresholdVisitor.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <iostream>

namespace sofa
{

namespace simulation
{

using std::cerr;
using std::endl;

VelocityThresholdVisitor::VelocityThresholdVisitor( const core::ExecParams* params /* PARAMS FIRST */, core::MultiVecId v, double t  )
    : Visitor(params), vid(v), threshold(t)
{
}

Visitor::Result VelocityThresholdVisitor::processNodeTopDown(simulation::Node* node)
{
    sofa::core::behavior::BaseMechanicalState* state = node->mechanicalState;

    if (state != NULL)
    {
        state->vThreshold(vid.getId(state),threshold);
    }
    return Visitor::RESULT_CONTINUE;
}

} // namespace simulation

} // namespace sofa

