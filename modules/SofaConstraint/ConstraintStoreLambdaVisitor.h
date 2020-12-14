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

#ifndef SOFA_CONSTRAINT_CONSTRAINTSTORELAMBDAVISITOR_H
#define SOFA_CONSTRAINT_CONSTRAINTSTORELAMBDAVISITOR_H

#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/SofaGeneral.h>


namespace sofa
{
namespace simulation
{

class SOFA_CONSTRAINT_API ConstraintStoreLambdaVisitor : public BaseMechanicalVisitor
{
public:
    ConstraintStoreLambdaVisitor(const sofa::core::ConstraintParams* cParams, const sofa::defaulttype::BaseVector* lambda);

    Visitor::Result fwdConstraintSet(simulation::Node* node, core::behavior::BaseConstraintSet* cSet) override;

    void bwdMechanicalMapping(simulation::Node* node, core::BaseMapping* map) override;

    bool childOrderReversed(simulation::Node* /*node*/) override
    {
        return true;
    }

    virtual bool stopAtMechanicalMapping(sofa::simulation::Node* /*node*/, core::BaseMapping* /*mapping*/)
    {
        return false; // !mapping->isMechanical();
    }

private:
    const sofa::core::ConstraintParams* m_cParams;
    const sofa::defaulttype::BaseVector* m_lambda;
    bool  m_accumulateMappings;
};


}

}

#endif 
