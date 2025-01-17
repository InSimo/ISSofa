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
#ifndef SOFA_COMPONENT_COLLISION_STICKCONTACTCONSTRAINT_INL
#define SOFA_COMPONENT_COLLISION_STICKCONTACTCONSTRAINT_INL

#include <SofaConstraint/StickContactConstraint.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaBaseCollision/DefaultContactManager.h>
#include <SofaMeshCollision/BarycentricContactMapper.h>
#include <SofaMeshCollision/IdentityContactMapper.h>
#include <SofaMeshCollision/RigidContactMapper.h>
#include <sofa/simulation/common/Node.h>
#include <iostream>

namespace sofa
{

namespace component
{

namespace collision
{

template < class TCollisionModel1, class TCollisionModel2 >
StickContactConstraint<TCollisionModel1,TCollisionModel2>::StickContactConstraint(CollisionModel1* model1, CollisionModel2* model2, Intersection* intersectionMethod)
    : model1(model1)
    , model2(model2)
    , intersectionMethod(intersectionMethod)
    , m_constraint(NULL)
    , parent(NULL)
    , f_keepAlive(initData(&f_keepAlive, true, "keepAlive", "set to true to keep this contact alive even after collisions are no longer detected"))
{
    mapper1.setCollisionModel(model1);
    mapper2.setCollisionModel(model2);
    this->f_printLog.setValue(true);
    //contacts.clear();
    //mappedContacts.clear();

}

template < class TCollisionModel1, class TCollisionModel2 >
StickContactConstraint<TCollisionModel1,TCollisionModel2>::~StickContactConstraint()
{
}

template < class TCollisionModel1, class TCollisionModel2 >
void StickContactConstraint<TCollisionModel1,TCollisionModel2>::cleanup()
{
    sout << "CLEANUP" << sendl;
    if (m_constraint)
    {
        //m_constraint->cleanup();

        if (parent != NULL)
            parent->removeObject(m_constraint);

        parent = NULL;
        //delete m_constraint;
        intrusive_ptr_add_ref(m_constraint.get()); // HACK: keep created constraints to avoid crash
        m_constraint.reset();

        mapper1.cleanup();

        mapper2.cleanup();
    }
    contacts.clear();
    mappedContacts.clear();
}


template < class TCollisionModel1, class TCollisionModel2 >
void StickContactConstraint<TCollisionModel1,TCollisionModel2>::setDetectionOutputs(OutputContainer* o)
{
    using DetectionOutputID = typename TOutputContainer::DetectionOutputID;

    this->f_printLog.setValue(true);
    sout << "setDetectionOutputs(" << (o == NULL ? -1 : (int)static_cast<TOutputContainer*>(o)->size()) << ")" << sendl;
    contacts.clear();
    if (!o) return;
    TOutputContainer& outputs = *static_cast<TOutputContainer*>(o);
    // We need to remove duplicate contacts
    const double minDist2 = 0.00000001f;


    int SIZE = outputs.size();
    std::cout << SIZE << " contacts" << std::endl;

    contacts.reserve(SIZE);

    //m_constraint->clear(SIZE);
    //mapper1.resize(SIZE);
    //mapper2.resize(SIZE);

    int OUTSIZE = 0;
    //const double d0 = intersectionMethod->getContactDistance() + model1->getProximity() + model2->getProximity(); // - 0.001;

    // the following procedure cancels the duplicated detection outputs
    for (const sofa::core::collision::DetectionOutput& o : outputs)
    {
        bool found = false;
        for (int i=0; i<cpt && !found; i++)
        {
            const sofa::core::collision::DetectionOutput* p = &outputs[DetectionOutputID(i)];
            if ((o.point[0]-p->point[0]).norm2()+(o.point[1]-p->point[1]).norm2() < minDist2)
                found = true;
        }

        if (found) continue;
        contacts.push_back(&o);
        ++OUTSIZE;
    }

    if (OUTSIZE<SIZE && this->f_printLog.getValue())
    {
        // DUPLICATED CONTACTS FOUND
        sout << "Removed " << (SIZE-OUTSIZE) <<" / " << SIZE << " collision points." << sendl;
    }

}

template < class TCollisionModel1, class TCollisionModel2 >
void StickContactConstraint<TCollisionModel1,TCollisionModel2>::activateMappers()
{
    if (!m_constraint)
    {
        sout << "Creating StickContactConstraint bilateral constraints"<<sendl;
        MechanicalState1* mstate1 = mapper1.createMapping(GenerateStringID::generate().c_str());
        mapper1.setConstraintMode();
        MechanicalState2* mstate2 = mapper2.createMapping(GenerateStringID::generate().c_str());
        mapper2.setConstraintMode();
        m_constraint = sofa::core::objectmodel::New<constraintset::BilateralInteractionConstraint<defaulttype::Vec3Types> >(mstate1, mstate2);
        m_constraint->setName( getName() );
    }

    sout << "activateMappers(" << contacts.size() << ")" << sendl;

    int size = contacts.size();
    m_constraint->clear(size);
    mapper1.resize(size);
    mapper2.resize(size);

    int i = 0;
    const double d0 = intersectionMethod->getContactDistance() + model1->getProximity() + model2->getProximity(); // - 0.001;

    //std::cout<<" d0 = "<<d0<<std::endl;

    mappedContacts.resize(contacts.size());
    for (const sofa::core::collision::DetectionOutput* o : contacts)
    {
        //std::cout<<" collisionElements :"<<o->elem.first<<" - "<<o->elem.second<<std::endl;
        CollisionElement1 elem1(o->elem.first);
        CollisionElement2 elem2(o->elem.second);
        int index1 = elem1.getIndex();
        int index2 = elem2.getIndex();
        //std::cout<<" indices :"<<index1<<" - "<<index2<<std::endl;

        typename DataTypes1::Real r1 = 0.;
        typename DataTypes2::Real r2 = 0.;
        //double constraintValue = ((o->point[1] - o->point[0]) * o->normal) - intersectionMethod->getContactDistance();

        // Create mapping for first point
        index1 = mapper1.addPointB(o->point[0], index1, r1
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
                , o->baryCoords[0]
#endif
                                  );
        // Create mapping for second point
        index2 = mapper2.addPointB(o->point[1], index2, r2
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
                , o->baryCoords[1]
#endif
                                  );
        double distance = d0 + r1 + r2;

        mappedContacts[i].first.first = index1;
        mappedContacts[i].first.second = index2;
        mappedContacts[i].second = distance;
    }

    // Update mappings
    mapper1.update();
    mapper1.updateXfree();
    mapper2.update();
    mapper2.updateXfree();
    /*

            CollisionElement1 elem1(o->elem.first);
            CollisionElement2 elem2(o->elem.second);
            int index1 = elem1.getIndex();
            int index2 = elem2.getIndex();

            typename DataTypes1::Real r1 = 0.0;
            typename DataTypes2::Real r2 = 0.0;
            // Create mapping for first point
            index1 = mapper1.addPointB(o->point[0], index1, r1
    #ifdef DETECTIONOUTPUT_BARYCENTRICINFO
                                      , o->baryCoords[0]
    #endif
            );
            // Create mapping for second point
            index2 = mapper2.addPointB(o->point[1], index2, r2
    #ifdef DETECTIONOUTPUT_BARYCENTRICINFO
                                      , o->baryCoords[1]
    #endif
            );

            double distance = d0 + r1 + r2;

            m_constraint->addContact(o->normal, o->point[0], o->point[1], distance, index1, index2, o->point[0], o->point[1], OUTSIZE, 0);
            ++OUTSIZE;
    	}
        // Update mappings
        mapper1.update();
        mapper2.update();
    */
    sout << contacts.size() << " StickContactConstraint created"<<sendl;
    sout << "mstate1 size = " << m_constraint->getMState1()->getSize() << " x = " << m_constraint->getMState1()->read(core::ConstVecCoordId::position())->getValue().size() << " xfree = " << m_constraint->getMState1()->read(core::ConstVecCoordId::freePosition())->getValue().size() << sendl;
    sout << "mstate2 size = " << m_constraint->getMState2()->getSize() << " x = " << m_constraint->getMState2()->read(core::ConstVecCoordId::position())->getValue().size() << " xfree = " << m_constraint->getMState2()->read(core::ConstVecCoordId::freePosition())->getValue().size() << sendl;
    //std::cerr<<" end activateMappers call"<<std::endl;

}

template < class TCollisionModel1, class TCollisionModel2 >
void StickContactConstraint<TCollisionModel1,TCollisionModel2>::createResponse(core::objectmodel::BaseContext* group)
{
	sout << "->createResponse(" << group->getName() << ")" << sendl;
    if (!contacts.empty() || !keepAlive())
	{
        activateMappers();
        int i = 0;
        for (const sofa::core::collision::DetectionOutput* o : contacts)
        {
            int index1 = mappedContacts[i].first.first;
            int index2 = mappedContacts[i].first.second;
            double distance = mappedContacts[i].second;

            // Polynome de Cantor de NxN sur N bijectif f(x,y)=((x+y)^2+3x+y)/2
            //long index = cantorPolynomia(o->id /*cantorPolynomia(index1, index2)*/,id);

            // Add contact in unilateral constraint
            m_constraint->addContact(o->normal, o->point[0], o->point[1], distance, index1, index2, o->point[0], o->point[1], i, o->id);

            //m_constraint->addContact(mu_, o->normal, distance, index1, index2, index, o->id);
        }
	}

    if (m_constraint!=NULL)
    {
        if (parent!=NULL)
        {
            parent->removeObject(this);
            parent->removeObject(m_constraint);
        }
        parent = group;
        if (parent!=NULL)
        {
            //sout << "Attaching contact response to "<<parent->getName()<<sendl;
            parent->addObject(this);
            parent->addObject(m_constraint);
        }
    }
}

template < class TCollisionModel1, class TCollisionModel2 >
void StickContactConstraint<TCollisionModel1,TCollisionModel2>::removeResponse()
{
	sout << "->removeResponse()" << sendl;
    if (m_constraint)
    {
        //mapper1.resize(0);
        //mapper2.resize(0);
        if (parent!=NULL)
        {
            //sout << "Removing contact response from "<<parent->getName()<<sendl;
            parent->removeObject(this);
            parent->removeObject(m_constraint);
        }
        parent = NULL;
    }
}

template < class TCollisionModel1, class TCollisionModel2 >
void StickContactConstraint<TCollisionModel1,TCollisionModel2>::resetResponse()
{
	sout << "->resetResponse()" << sendl;
    if (m_constraint)
    {
        mapper1.resize(0);
        mapper2.resize(0);
    }
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
