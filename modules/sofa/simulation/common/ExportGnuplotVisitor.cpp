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
#include <sofa/simulation/common/ExportGnuplotVisitor.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <iostream>

namespace sofa
{

namespace simulation
{

using std::cerr;
using std::endl;

simulation::Visitor::Result InitGnuplotVisitor::processNodeTopDown(simulation::Node* node)
{
    if (node->interactionForceField.getSize() != 0)
    {
        int size = node->interactionForceField.getSize();
        for(int i = 0; i < size; i++)
        {
            if (node->interactionForceField.getValue()[i] )
            {
                node->interactionForceField.getValue()[i]->initGnuplot(gnuplotDirectory);
            }
        }
    }

    if (node->mechanicalState)
    {
        node->mechanicalState->initGnuplot(gnuplotDirectory);
    }
    if (node->mass)
    {
        node->mass->initGnuplot(gnuplotDirectory);
    }
    return RESULT_CONTINUE;
}

ExportGnuplotVisitor::ExportGnuplotVisitor(const core::ExecParams* params /* PARAMS FIRST */, double time)
    : Visitor(params), m_time(time)
{

}

simulation::Visitor::Result ExportGnuplotVisitor::processNodeTopDown(simulation::Node* node)
{
    if (node->interactionForceField.getSize() != 0)
    {
        int size = node->interactionForceField.getSize();
        for(int i = 0; i < size; i++)
        {
            if (node->interactionForceField.getValue()[i] )
            {
                node->interactionForceField.getValue()[i]->exportGnuplot(m_time);
            }
        }
    }

    if (node->mechanicalState)
    {
        node->mechanicalState->exportGnuplot(m_time);
    }
    if (node->mass)
    {
        node->mass->exportGnuplot(core::MechanicalParams::defaultInstance(), m_time);
    }
    return RESULT_CONTINUE;
}


} // namespace simulation

} // namespace sofa

