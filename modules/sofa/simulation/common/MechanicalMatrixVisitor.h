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
#ifndef SOFA_SIMULATION_MECHANICALMATRIXVISITOR_H
#define SOFA_SIMULATION_MECHANICALMATRIXVISITOR_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/behavior/Mass.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>
#include <sofa/core/behavior/BaseInteractionConstraint.h>
#include <sofa/core/behavior/BaseInteractionProjectiveConstraintSet.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/core/behavior/BaseConstraintSet.h>
#include <sofa/defaulttype/BaseMatrix.h>
#include <sofa/defaulttype/BaseVector.h>
#include <iostream>

#include <sofa/core/ExecParams.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>

namespace sofa
{

namespace simulation
{


/** Compute the size of a mechanical matrix (mass or stiffness) of the whole scene */
class SOFA_SIMULATION_COMMON_API MechanicalGetMatrixDimensionVisitor : public BaseMechanicalVisitor
{
public:
    unsigned int * const nbRow;
    unsigned int * const nbCol;
    sofa::core::behavior::MultiMatrixAccessor* matrix;

    MechanicalGetMatrixDimensionVisitor(
        const core::ExecParams* params /* PARAMS FIRST  = core::ExecParams::defaultInstance()*/, unsigned int * const _nbRow, unsigned int * const _nbCol,
        sofa::core::behavior::MultiMatrixAccessor* _matrix = NULL )
        : BaseMechanicalVisitor(params) , nbRow(_nbRow), nbCol(_nbCol), matrix(_matrix)
    {}

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* ms)
    {
        //ms->contributeToMatrixDimension(nbRow, nbCol);
        const unsigned int n = ms->getMatrixSize();
        if (nbRow) *nbRow += n;
        if (nbCol) *nbCol += n;
        if (matrix) matrix->addMechanicalState(ms);
        return RESULT_CONTINUE;
    }

    virtual Result fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* mm)
    {
        if (matrix) matrix->addMechanicalMapping(mm);
        return RESULT_CONTINUE;
    }

    virtual Result fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* ms)
    {
        if (matrix) matrix->addMappedMechanicalState(ms);
        return RESULT_CONTINUE;
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalGetMatrixDimensionVisitor"; }

};

/** Compute the size of a mechanical matrix (mass or stiffness) of the whole scene */
class SOFA_SIMULATION_COMMON_API MechanicalGetConstraintJacobianVisitor : public BaseMechanicalVisitor
{
public:
    const core::ConstraintParams* cparams;
    defaulttype::BaseMatrix * J;
    const sofa::core::behavior::MultiMatrixAccessor* matrix;
    int offset;

    MechanicalGetConstraintJacobianVisitor(
        const core::ConstraintParams* cparams, defaulttype::BaseMatrix * _J, const sofa::core::behavior::MultiMatrixAccessor* _matrix = NULL)
        : BaseMechanicalVisitor(cparams) , cparams(cparams), J(_J), matrix(_matrix), offset(0)
    {}

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* ms)
    {
        if (matrix) offset = matrix->getGlobalOffset(ms);

        unsigned int o = (unsigned int)offset;
        ms->getConstraintJacobian(cparams,J,o);
        offset = (int)o;
        return RESULT_CONTINUE;
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalGetConstraintJacobianVisitor"; }
};

/** Apply the motion correction computed from constraint force influence  */
class SOFA_SIMULATION_COMMON_API MechanicalIntegrateConstraintsVisitor : public BaseMechanicalVisitor
{
public:

    const sofa::core::ConstraintParams* cparams;
    const double positionFactor;// use the OdeSolver to get the position integration factor
    const double velocityFactor;// use the OdeSolver to get the position integration factor
    sofa::core::ConstMultiVecDerivId correctionId;
    sofa::core::MultiVecDerivId dxId;
    sofa::core::MultiVecCoordId xId;
    sofa::core::MultiVecDerivId vId;
    const sofa::core::behavior::MultiMatrixAccessor* matrix;
    int offset;

    MechanicalIntegrateConstraintsVisitor(
        const core::ConstraintParams* cparams,
        double pf, double vf, 
        sofa::core::ConstMultiVecDerivId correction,
        sofa::core::MultiVecDerivId dx = sofa::core::MultiVecDerivId(sofa::core::VecDerivId::dx()),
        sofa::core::MultiVecCoordId x  = sofa::core::MultiVecCoordId(sofa::core::VecCoordId::position()),
        sofa::core::MultiVecDerivId v  = sofa::core::MultiVecDerivId(sofa::core::VecDerivId::velocity()),
        const sofa::core::behavior::MultiMatrixAccessor* _matrix = NULL)
        :BaseMechanicalVisitor(cparams) 
        ,cparams(cparams)
        ,positionFactor(pf)
        ,velocityFactor(vf)
        ,correctionId(correction)
        ,dxId(dx)
        ,xId(x)
        ,vId(v)
        ,matrix(_matrix)
        ,offset(0)
    {}

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* ms)
    {
        if (matrix) offset = matrix->getGlobalOffset(ms);
        if (offset >= 0)
        {
            unsigned int o = (unsigned int)offset;
            offset = (int)o;

            if (positionFactor != 0)
            {
                //x = x_free + correction * positionFactor;
                ms->vOp(params, xId.getId(ms), cparams->x().getId(ms), correctionId.getId(ms), positionFactor);
            }

            if (velocityFactor != 0)
            {
                //v = v_free + correction * velocityFactor;
                ms->vOp(params, vId.getId(ms), cparams->v().getId(ms), correctionId.getId(ms), velocityFactor);
            }

            const double correctionFactor = cparams->constOrder() == sofa::core::ConstraintParams::ConstOrder::VEL ? velocityFactor : positionFactor;
            
            //dx *= correctionFactor;
            ms->vOp(params,dxId.getId(ms),core::VecDerivId::null(), correctionId.getId(ms), correctionFactor);
        }

        return RESULT_CONTINUE;
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalIntegrateConstraintsVisitor"; }
};



/** Accumulate the entries of a mechanical matrix (mass or stiffness) of the whole scene */
class SOFA_SIMULATION_COMMON_API MechanicalAddMBK_ToMatrixVisitor : public MechanicalVisitor
{
public:
    const sofa::core::behavior::MultiMatrixAccessor* matrix;

    MechanicalAddMBK_ToMatrixVisitor(const core::MechanicalParams* mparams /* PARAMS FIRST  = core::MechanicalParams::defaultInstance()*/, const sofa::core::behavior::MultiMatrixAccessor* _matrix )
        : MechanicalVisitor(mparams) ,  matrix(_matrix) //,m(_m),b(_b),k(_k)
    {
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalAddMBK_ToMatrixVisitor"; }

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*ms*/)
    {
        //ms->setOffset(offsetOnExit);
        return RESULT_CONTINUE;
    }

    virtual Result fwdForceField(simulation::Node* /*node*/, core::behavior::BaseForceField* ff)
    {
        if (matrix != NULL)
        {
            assert( !ff->isCompliance.getValue() ); // if one day this visitor has to be used with compliance, K from compliance should not be added (by tweaking mparams with kfactor=0)
            ff->addMBKToMatrix(this->mparams /* PARAMS FIRST */, matrix);
        }

        return RESULT_CONTINUE;
    }

    //Masses are now added in the addMBKToMatrix call for all ForceFields

};

/** Accumulate the entries of a mechanical matrix (mass or stiffness) of the whole scene ONLY ON THE subMatrixIndex */
class SOFA_SIMULATION_COMMON_API MechanicalAddSubMBK_ToMatrixVisitor : public MechanicalVisitor
{
public:
    const sofa::core::behavior::MultiMatrixAccessor* matrix;
    const helper::vector<unsigned> & subMatrixIndex; // index of the point where the matrix must be computed

    MechanicalAddSubMBK_ToMatrixVisitor(const core::MechanicalParams* mparams /* PARAMS FIRST  = core::MechanicalParams::defaultInstance()*/, const sofa::core::behavior::MultiMatrixAccessor* _matrix, const helper::vector<unsigned> & Id)
        : MechanicalVisitor(mparams) ,  matrix(_matrix), subMatrixIndex(Id) //,m(_m),b(_b),k(_k)
    {
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalAddSubMBK_ToMatrixVisitor"; }

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*ms*/)
    {
        //ms->setOffset(offsetOnExit);
        return RESULT_CONTINUE;
    }

    virtual Result fwdForceField(simulation::Node* /*node*/, core::behavior::BaseForceField* ff)
    {
        if (matrix != NULL)
        {
            assert( !ff->isCompliance.getValue() ); // if one day this visitor has to be used with compliance, K from compliance should not be added (by tweaking mparams with kfactor=0)
            ff->addSubMBKToMatrix(this->mparams /* PARAMS FIRST */, matrix, subMatrixIndex);
        }

        return RESULT_CONTINUE;
    }

    //Masses are now added in the addMBKToMatrix call for all ForceFields

};

/** Apply projective constaints of the whole scene */
class SOFA_SIMULATION_COMMON_API MechanicalAddProjectiveConstraint_ToMatrixVisitor : public MechanicalVisitor
{
public:
    const sofa::core::behavior::MultiMatrixAccessor* matrix;

    MechanicalAddProjectiveConstraint_ToMatrixVisitor(const core::MechanicalParams* mparams /* PARAMS FIRST  = core::MechanicalParams::defaultInstance()*/, const sofa::core::behavior::MultiMatrixAccessor* _matrix )
        : MechanicalVisitor(mparams) ,  matrix(_matrix) //,m(_m),b(_b),k(_k)
    {
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalAddProjectiveConstraint_ToMatrixVisitor"; }

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*ms*/)
    {
        //ms->setOffset(offsetOnExit);
        return RESULT_CONTINUE;
    }

    virtual Result fwdProjectiveConstraintSet(simulation::Node* /*node*/, core::behavior::BaseProjectiveConstraintSet* c)
    {
        if (matrix != NULL)
        {
            c->applyConstraint(this->mparams /* PARAMS FIRST */, matrix);
        }

        return RESULT_CONTINUE;
    }
};

class SOFA_SIMULATION_COMMON_API MechanicalMultiVectorToBaseVectorVisitor : public BaseMechanicalVisitor
{
public:
    sofa::core::ConstMultiVecId src;
    sofa::defaulttype::BaseVector *vect;
    const sofa::core::behavior::MultiMatrixAccessor* matrix;
    int offset;

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalMultiVector2BaseVectorVisitor"; }

    MechanicalMultiVectorToBaseVectorVisitor(
        const core::ExecParams* params /* PARAMS FIRST  = core::ExecParams::defaultInstance()*/,
        sofa::core::ConstMultiVecId _src, defaulttype::BaseVector * _vect,
        const sofa::core::behavior::MultiMatrixAccessor* _matrix = NULL )
        : BaseMechanicalVisitor(params) , src(_src), vect(_vect), matrix(_matrix), offset(0)
    {
    }

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
    {
        if (matrix) offset = matrix->getGlobalOffset(mm);
        if (vect != NULL && offset >= 0)
        {
            unsigned int o = (unsigned int)offset;
            mm->copyToBaseVector(vect, src.getId(mm), o);
            offset = (int)o;
        }
        //if (!matrix) offset += mm->getMatrixSize();
        return RESULT_CONTINUE;
    }
};

class SOFA_SIMULATION_COMMON_API MechanicalMultiVectorPeqBaseVectorVisitor : public BaseMechanicalVisitor
{
public:
    sofa::defaulttype::BaseVector *src;
    sofa::core::MultiVecDerivId dest;
    const sofa::core::behavior::MultiMatrixAccessor* matrix;
    int offset;

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalMultiVectorPeqBaseVectorVisitor"; }

    MechanicalMultiVectorPeqBaseVectorVisitor(
        const core::ExecParams* params /* PARAMS FIRST  = core::ExecParams::defaultInstance()*/, sofa::core::MultiVecDerivId _dest, defaulttype::BaseVector * _src,
        const sofa::core::behavior::MultiMatrixAccessor* _matrix = NULL )
        : BaseMechanicalVisitor(params) , src(_src), dest(_dest), matrix(_matrix), offset(0)
    {
    }

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
    {
        if (matrix) offset = matrix->getGlobalOffset(mm);
        if (src!= NULL && offset >= 0)
        {
            unsigned int o = (unsigned int)offset;
            mm->addFromBaseVectorSameSize(dest.getId(mm), src, o);
            offset = (int)o;
        }
        //if (!matrix) offset += mm->getMatrixSize();

        return RESULT_CONTINUE;
    }
};

class SOFA_SIMULATION_COMMON_API MechanicalMultiVectorFromBaseVectorVisitor : public BaseMechanicalVisitor
{
public:
    sofa::defaulttype::BaseVector *src;
    sofa::core::MultiVecId dest;
    const sofa::core::behavior::MultiMatrixAccessor* matrix;
    int offset;

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalMultiVectorFromBaseVectorVisitor"; }

    MechanicalMultiVectorFromBaseVectorVisitor(
        const core::ExecParams* params /* PARAMS FIRST  = core::ExecParams::defaultInstance()*/, sofa::core::MultiVecId _dest,
        defaulttype::BaseVector * _src,
        const sofa::core::behavior::MultiMatrixAccessor* _matrix = NULL )
        : BaseMechanicalVisitor(params) , src(_src), dest(_dest), matrix(_matrix), offset(0)
    {
    }

    virtual Result fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
    {
        if (matrix) offset = matrix->getGlobalOffset(mm);
        if (src!= NULL && offset >= 0)
        {
            unsigned int o = (unsigned int)offset;
            mm->copyFromBaseVector(dest.getId(mm), src, o);
            offset = (int)o;
        }
        //if (!matrix) offset += mm->getMatrixSize();

        return RESULT_CONTINUE;
    }
};

} // namespace simulation

} // namespace sofa

#endif
