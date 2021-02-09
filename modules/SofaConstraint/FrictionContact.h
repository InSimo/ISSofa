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
#ifndef SOFA_COMPONENT_COLLISION_FRICTIONCONTACT_H
#define SOFA_COMPONENT_COLLISION_FRICTIONCONTACT_H

#include <sofa/core/collision/Contact.h>
#include <sofa/core/collision/Intersection.h>
#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaConstraint/UnilateralInteractionConstraint.h>
#include <sofa/helper/Factory.h>
#include <SofaBaseCollision/BaseContactMapper.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <SofaConstraint/ContactIdentifier.h>

namespace sofa
{

namespace component
{

namespace collision
{



template <class TCollisionModel1, class TCollisionModel2>
class FrictionContact : public core::collision::Contact, public ContactIdentifier
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(FrictionContact, TCollisionModel1, TCollisionModel2), core::collision::Contact);
    typedef TCollisionModel1 CollisionModel1;
    typedef TCollisionModel2 CollisionModel2;
    typedef core::collision::Intersection Intersection;
    typedef typename TCollisionModel1::DataTypes::CPos TVec1;
    typedef typename TCollisionModel2::DataTypes::CPos TVec2;
    typedef sofa::defaulttype::StdVectorTypes<TVec1,TVec1, typename TCollisionModel1::DataTypes::Real > DataTypes1; 
    typedef sofa::defaulttype::StdVectorTypes<TVec2,TVec2, typename TCollisionModel1::DataTypes::Real > DataTypes2;

    typedef core::behavior::MechanicalState<DataTypes1> MechanicalState1;
    typedef core::behavior::MechanicalState<DataTypes2> MechanicalState2;
    typedef typename CollisionModel1::Element CollisionElement1;
    typedef typename CollisionModel2::Element CollisionElement2;
    typedef core::collision::DetectionOutputContainer OutputVector;
    typedef core::collision::TDetectionOutputContainer<CollisionModel1,CollisionModel2> TOutputVector;

protected:
    CollisionModel1* model1;
    CollisionModel2* model2;
    Intersection* intersectionMethod;
    bool selfCollision; ///< true if model1==model2 (in this case, only mapper1 is used)
    ContactMapper<CollisionModel1,DataTypes1> mapper1;
    ContactMapper<CollisionModel2,DataTypes2> mapper2;

    constraintset::UnilateralInteractionConstraint<sofa::defaulttype::Vec3Types>::SPtr m_constraint;
    core::objectmodel::BaseContext* parent;

    Data<double> mu, tol;
    std::vector< sofa::core::collision::DetectionOutput* > contacts;
    std::vector< std::pair< std::pair<int, int>, double > > mappedContacts;

    void activateMappers();

    void setInteractionTags(MechanicalState1* mstate1, MechanicalState2* mstate2);

    FrictionContact() {}

    FrictionContact(CollisionModel1* model1, CollisionModel2* model2, Intersection* intersectionMethod);
    virtual ~FrictionContact();
public:
    void cleanup() override;

    std::pair<core::CollisionModel*,core::CollisionModel*> getCollisionModels() override { return std::make_pair(model1,model2); }

    void setDetectionOutputs(OutputVector* outputs) override;

    void createResponse(core::objectmodel::BaseContext* group) override;

    virtual void updateContactsMappers() override;

    void removeResponse() override;
    
    void resetResponse() override;
};

} // collision

} // component

} // sofa

#endif // SOFA_COMPONENT_COLLISION_FRICTIONCONTACT_H
