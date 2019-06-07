/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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

#include "ConstraintStoreLambdaVisitor.h"

namespace sofa
{
namespace simulation
{

ConstraintStoreLambdaVisitor::ConstraintStoreLambdaVisitor(const sofa::core::ConstraintParams* cParams, const sofa::defaulttype::BaseVector* lambda)
:BaseMechanicalVisitor(cParams)
,m_cParams(cParams)
,m_lambda(lambda)
,m_accumulateMappings(false)
{
}

Visitor::Result ConstraintStoreLambdaVisitor::fwdConstraintSet(simulation::Node* node, core::behavior::BaseConstraintSet* cSet)
{
    if (core::behavior::BaseConstraint *c = core::behavior::BaseConstraint::DynamicCast(cSet))
    {
        const bool applyConstraint = c->d_isConstitutiveConstraint.getValue() == m_cParams->isAssemblingConstitutiveConstraints();
        
        if (applyConstraint)
        {
            ctime_t t0 = begin(node, c);
            c->storeLambda(m_cParams, m_cParams->lambda(), m_lambda);
            end(node, c, t0);
            m_accumulateMappings = true;
        }
    }
    return RESULT_CONTINUE;
}

void ConstraintStoreLambdaVisitor::bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    if (m_accumulateMappings)
    {
        sofa::core::MechanicalParams mparams(*m_cParams);
        mparams.setDx(m_cParams->dx());
        mparams.setF(m_cParams->lambda());
        map->applyJT(&mparams, m_cParams->lambda(), m_cParams->lambda());
    }
}

}

}
