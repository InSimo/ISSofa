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

#include "ConstraintAddCToMatrixVisitor.h"

namespace sofa
{
namespace simulation
{

ConstraintAddCToMatrixVisitor::ConstraintAddCToMatrixVisitor(const sofa::core::ConstraintParams* cParams, sofa::defaulttype::BaseMatrix* C, const sofa::defaulttype::BaseVector* phi)
:BaseMechanicalVisitor(cParams)
,m_cParams(cParams)
,m_C(C)
,m_phi(phi)
{
}


Visitor::Result ConstraintAddCToMatrixVisitor::fwdConstraintSet(simulation::Node* /*node*/, core::behavior::BaseConstraintSet* cSet)
{
    if (core::behavior::BaseConstraint *c = core::behavior::BaseConstraint::DynamicCast(cSet))
    {
        const bool applyConstraint = c->d_isConstitutiveConstraint.getValue() == m_cParams->isAssemblingConstitutiveConstraints();

        if (applyConstraint)
        {
            c->addCToMatrix(m_cParams, m_C, m_phi);
        }
    }

    return RESULT_CONTINUE;
}

}

}

