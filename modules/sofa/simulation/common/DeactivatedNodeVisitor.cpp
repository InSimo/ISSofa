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
#include <sofa/simulation/common/DeactivatedNodeVisitor.h>

namespace sofa
{

namespace simulation
{

Visitor::Result DeactivationVisitor::processNodeTopDown(simulation::Node* node)
{
    if (active)
    {
        for (simulation::Node::ChildIterator itChild = node->child.begin(); itChild != node->child.end(); ++itChild)
        {
            simulation::Node *child = itChild->get();
            child->setActive(active);
        }

    }
    return RESULT_CONTINUE;
}

void DeactivationVisitor::processNodeBottomUp(simulation::Node* node)
{
    if (!active)
    {
        node->setActive(active);
    }

}


} // namespace simulation

} // namespace sofa

