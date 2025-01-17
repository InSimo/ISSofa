/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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
#ifndef SOFA_CORE_BEHAVIOR_BASECONSTRAINTSET_H
#define SOFA_CORE_BEHAVIOR_BASECONSTRAINTSET_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/SofaFramework.h>

#include <sofa/defaulttype/BaseVector.h>


namespace sofa
{

namespace core
{

namespace behavior
{

class SOFA_CORE_API BaseConstraintSet : public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS_EXTERNAL((BaseConstraintSet), ((objectmodel::BaseObject)));

protected:
    BaseConstraintSet()
        : group(initData(&group, 0, "group", "ID of the group containing this constraint. This ID is used to specify which constraints are solved by which solver, by specifying in each solver which groups of constraints it should handle."))
        , m_constraintIndex(initData(&m_constraintIndex, (unsigned int)0, "constraintIndex", "Constraint index (first index in the right hand term resolution vector)"))
        , d_isConstitutiveConstraint(initData(&d_isConstitutiveConstraint, false, "isConstitutiveConstraint", "Constitutive constraints are factored in the same system as the dynamic matrix ( M,B,K )"))
    {
    }

    virtual ~BaseConstraintSet() { }
	
private:
	BaseConstraintSet(const BaseConstraintSet& n) ;
	BaseConstraintSet& operator=(const BaseConstraintSet& n) ;	

public:
    virtual void resetConstraint() {};

    /// Construct the Jacobian Matrix
    ///
    /// \param cId is the result constraint sparse matrix Id
    /// \param cIndex is the index of the next constraint equation: when building the constraint matrix, you have to use this index, and then update it
    /// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC)
    virtual void buildConstraintMatrix(const ConstraintParams* cParams, MultiMatrixDerivId cId, unsigned int &cIndex) = 0;

    /// Construct the Constraint violations vector
    ///
    /// \param v is the result vector that contains the whole constraints violations
    /// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC)
    virtual void getConstraintViolation(const ConstraintParams* cParams, defaulttype::BaseVector *v) = 0;

protected:

    Data< int > group;
public:
    Data< unsigned int > m_constraintIndex; /// Constraint index (first index in the right hand term resolution vector)
    Data< bool >         d_isConstitutiveConstraint;/// Constitutive constraints are factored in the same system as the dynamic matrix ( M,B,K )
};

} // namespace behavior

} // namespace core

} // namespace sofa

#endif
