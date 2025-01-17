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
#ifndef SOFA_SIMULATION_VelocityThresholdVisitor_H
#define SOFA_SIMULATION_VelocityThresholdVisitor_H

#include <sofa/simulation/common/Visitor.h>
#include <sofa/core/MultiVecId.h>

namespace sofa
{

namespace simulation
{

class SOFA_SIMULATION_COMMON_API VelocityThresholdVisitor : public Visitor
{
public:
    virtual Visitor::Result processNodeTopDown(simulation::Node* node);

    VelocityThresholdVisitor(const core::ExecParams* params /* PARAMS FIRST */, core::MultiVecId v, double threshold);



    /// Return a category name for this action.
    /// Only used for debugging / profiling purposes
    virtual const char* getCategoryName() const
    {
        return "threshold";
    }
    virtual const char* getClassName() const { return "VelocityThresholdVisitor"; }

protected:
    core::MultiVecId vid; ///< Id of the vector to process
    double threshold; ///< All the entries below this threshold will be set to 0.
};

} // namespace simulation

} // namespace sofa

#endif
