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
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>

namespace sofa
{

namespace core
{

namespace behavior
{

SOFA_ABSTRACT_CLASS_IMPL((BaseForceField));
SOFA_ABSTRACT_CLASS_IMPL((BaseInteractionForceField));

BaseForceField::BaseForceField()
    : objectmodel::BaseObject()
    , isCompliance( initData(&isCompliance, false, "isCompliance", "Consider the component as a compliance, else as a stiffness"))
    , rayleighStiffness( initData(&rayleighStiffness, SReal(0), "rayleighStiffness", "Rayleigh damping - stiffness matrix coefficient"))
{
}

void BaseForceField::addMBKdx(const MechanicalParams* mparams /* PARAMS FIRST */, MultiVecDerivId dfId)
{
    if (mparams->kFactor() != 0.0 || mparams->bFactor() != 0.0)
    {
        //rewrite kFactor to include Rayleigh damping defined at ForceField level if any
        const SReal kFactor = mparams->kFactor();
        mparams->setKFactor(kFactor + mparams->bFactor()*this->rayleighStiffness.getValue());

        addDForce(mparams /* PARAMS FIRST */, dfId);

        //reset the kFactor so that other ForceField are not impacted
        mparams->setKFactor(kFactor);
    }
}

void BaseForceField::addBToMatrix(const MechanicalParams* /*mparams*/ /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/)
{
}

void BaseForceField::addMBKToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    if (mparams->kFactor() != 0.0)
    {
        //rewrite kFactor to include Rayleigh damping defined at ForceField level if any
        const SReal kFactor = mparams->kFactor();
        mparams->setKFactor(kFactor + mparams->bFactor()*this->rayleighStiffness.getValue());
        
        addKToMatrix(mparams /* PARAMS FIRST */, matrix);
        
        //reset the kFactor so that other ForceField are not impacted
        mparams->setKFactor(kFactor);
    }
    if (mparams->bFactor() != 0.0)
    {
        addBToMatrix(mparams /* PARAMS FIRST */, matrix);
    }
}

void BaseForceField::addSubMBKToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> subMatrixIndex)
{
    if (mparams->kFactor() != 0.0)
    {
        //rewrite kFactor to include Rayleigh damping defined at ForceField level if any
        const SReal kFactor = mparams->kFactor();
        mparams->setKFactor(kFactor + mparams->bFactor()*this->rayleighStiffness.getValue());

        addSubKToMatrix(mparams /* PARAMS FIRST */, matrix, subMatrixIndex);

        //reset the kFactor so that other ForceField are not impacted
        mparams->setKFactor(kFactor);
    }
    if (mparams->bFactor() != 0.0)
    {
        addSubBToMatrix(mparams /* PARAMS FIRST */, matrix, subMatrixIndex);
    }
}

void BaseForceField::addSubKToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> & /*subMatrixIndex*/) 
{
    addKToMatrix(mparams,matrix);
}

void BaseForceField::addSubBToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> & /*subMatrixIndex*/) 
{
    addBToMatrix(mparams,matrix);
}




} // namespace behavior

} // namespace core

} // namespace sofa
