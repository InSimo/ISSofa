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
#include <sofa/simulation/common/InitVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/Simulation.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/defaulttype/BoundingBox.h>

//#include "MechanicalIntegration.h"

namespace sofa
{

namespace simulation
{

Visitor::Result InitVisitor::processNodeTopDown(simulation::Node* node)
{
    static sofa::core::objectmodel::Tag tagNoBBox("NoBBox");
    if (!rootNode) rootNode=node;

    node->initialize();
#ifdef SOFA_SMP_NUMA
    if(node->getProcessor()!=-1)
    {
        std::cerr<<"set preferred cpu "<<node->getProcessor()/2<<std::endl;
        numa_set_preferred(node->getProcessor()/2);
    }
#endif

    sofa::defaulttype::BoundingBox* nodeBBox = node->f_bbox.beginEdit(params);
    nodeBBox->invalidate();

    for(unsigned int i=0; i<node->object.size(); ++i)
    {
        sofa::core::objectmodel::BaseObject* o = node->object[i].get();
        o->init();
        if (!o->hasTag(tagNoBBox))
        {
            o->computeBBox(params);
            nodeBBox->include(o->f_bbox.getValue(params));
        }
    }
    node->f_bbox.endEdit(params);
    return RESULT_CONTINUE;
}


void InitVisitor::processNodeBottomUp(simulation::Node* node)
{
    // init all the components in reverse order
    static sofa::core::objectmodel::Tag tagNoBBox("NoBBox");
    node->setDefaultVisualContextValue();
    sofa::defaulttype::BoundingBox* nodeBBox = node->f_bbox.beginEdit(params);

    for(unsigned int i=node->object.size(); i>0; --i)
    {
        sofa::core::objectmodel::BaseObject* o = node->object[i-1].get();
        o->bwdInit();
        if (!o->hasTag(tagNoBBox))
        {
            nodeBBox->include(o->f_bbox.getValue(params));
        }
    }

    node->f_bbox.endEdit(params);
    node->bwdInit();
}



} // namespace simulation

} // namespace sofa

