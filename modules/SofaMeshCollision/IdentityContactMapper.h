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
#ifndef SOFA_COMPONENT_COLLISION_IDENTITYCONTACTMAPPER_H
#define SOFA_COMPONENT_COLLISION_IDENTITYCONTACTMAPPER_H

#include <sofa/helper/system/config.h>
#include <sofa/helper/Factory.h>
#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaBaseMechanics/IdentityMapping.h>
#include <SofaRigid/RigidMapping.h>
#include <SofaBaseMechanics/SubsetMapping.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/Simulation.h>
#include <SofaBaseCollision/BaseContactMapper.h>
#include <SofaBaseCollision/SphereModel.h>
#include <SofaMeshCollision/TriangleModel.h>
//#include <SofaMiscCollision/TetrahedronModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
//#include <SofaVolumetricData/DistanceGridCollisionModel.h>
#include <SofaBaseMechanics/IdentityMapping.h>
#include <iostream>


namespace sofa
{

namespace component
{

namespace collision
{

/// Base class for IdentityMapping based mappers
template<class TCollisionModel, class DataTypes>
class IdentityContactMapper : public BaseContactMapper<DataTypes>
{
public:
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef TCollisionModel MCollisionModel;
    typedef typename MCollisionModel::InDataTypes InDataTypes;
    typedef core::behavior::MechanicalState<InDataTypes> InMechanicalState;
    typedef core::behavior::MechanicalState<typename IdentityContactMapper::DataTypes> MMechanicalState;
    typedef component::container::MechanicalObject<typename IdentityContactMapper::DataTypes> MMechanicalObject;
    typedef mapping::IdentityMapping< InDataTypes, typename IdentityContactMapper::DataTypes > MMapping;
    MCollisionModel* model;
    typename MMapping::SPtr mapping;

    IdentityContactMapper()
        : model(NULL), mapping(NULL)
    {
    }

    void setCollisionModel(MCollisionModel* model)
    {
        this->model = model;
    }

    void cleanup();

    MMechanicalState* createMapping(const char* name="contactPoints");

    void setConstraintMode()
    {
        if (mapping!=NULL)
        {
            mapping->setForcesMapped(false);
            mapping->setMassesMapped(false);
        }
    }

    void resize(int /*size*/)
    {
    }

    int addPoint(const Coord&, int index, Real&)
    {
        return index;
    }

    void update()
    {
        if (mapping!=NULL)
        {
            mapping->apply(core::MechanicalParams::defaultInstance(), core::VecCoordId::position(), core::ConstVecCoordId::position());
            mapping->applyJ(core::MechanicalParams::defaultInstance(), core::VecDerivId::velocity(), core::ConstVecDerivId::velocity());
        }
    }

    void updateXfree()
    {
        if (mapping!=NULL)
        {
            mapping->apply(core::MechanicalParams::defaultInstance(), core::VecCoordId::freePosition(), core::ConstVecCoordId::freePosition());
            mapping->applyJ(core::MechanicalParams::defaultInstance(), core::VecDerivId::freeVelocity(), core::ConstVecDerivId::freeVelocity());

        }
    }
    
    void updateX0()
    {
        if(mapping!=NULL)
        {
             mapping->apply(core::MechanicalParams::defaultInstance(), core::VecCoordId::restPosition(), core::ConstVecCoordId::restPosition());
        }
    }

    virtual sofa::core::BaseMapping* getMapping()
    {
        return mapping.get();
    }
};

/// Specialization of IdentityContactMapper when mapping to the same DataTypes, as no mapping is required in this case
template<class TCollisionModel>
class IdentityContactMapper<TCollisionModel, typename TCollisionModel::InDataTypes> : public BaseContactMapper<typename TCollisionModel::InDataTypes>
{
public:
    typedef TCollisionModel MCollisionModel;
    typedef typename MCollisionModel::InDataTypes InDataTypes;
    typedef typename MCollisionModel::InDataTypes DataTypes;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef core::behavior::MechanicalState<InDataTypes> InMechanicalState;
    typedef core::behavior::MechanicalState<DataTypes> MMechanicalState;
    MCollisionModel* model;

    IdentityContactMapper()
        : model(NULL)
    {
    }

    void setCollisionModel(MCollisionModel* model)
    {
        this->model = model;
    }

    void cleanup()
    {
    }

    MMechanicalState* createMapping(const char* /*name*/="contactPoints")
    {
        if (model==NULL) return NULL;
        return model->getMechanicalState();
    }

    void setConstraintMode()
    {
    }

    void resize(int /*size*/)
    {
    }

    int addPoint(const Coord& /*P*/, int index, Real&)
    {
        return index;
    }

    void update()
    {
    }

    void updateXfree()
    {
    }

    void updateX0()
    {
    }

    virtual unsigned int getFromElem( unsigned int pid) const override
    {
        return pid;
    }

    virtual sofa::defaulttype::Vector3 getFromBary( unsigned int /*pid*/) const override
    {
        return sofa::defaulttype::Vector3(0, 0, 0);
    }

    virtual sofa::core::BaseMapping* getMapping()
    {
        return NULL;
    }

};

/// Mapper for PointModel
template<class DataTypes>
class ContactMapper<PointModel, DataTypes> : public IdentityContactMapper<PointModel, DataTypes>
{
public:
};

/// Mapper for SphereModel
template<class DataTypes>
class ContactMapper<SphereModel, DataTypes> : public IdentityContactMapper<SphereModel, DataTypes>
{
public:
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    int addPoint(const Coord& /*P*/, int index, Real& r)
    {
        Sphere e(this->model, index);
        r = e.r();
        return index;
    }
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
