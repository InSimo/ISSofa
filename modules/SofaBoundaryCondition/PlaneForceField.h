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
#ifndef SOFA_COMPONENT_INTERACTIONFORCEFIELD_PLANEFORCEFIELD_H
#define SOFA_COMPONENT_INTERACTIONFORCEFIELD_PLANEFORCEFIELD_H

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/SofaGeneral.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

/// This class can be overridden if needed for additionnal storage within template specializations.
template<class DataTypes>
class PlaneForceFieldInternalData
{
public:
};

template<class DataTypes>
class PlaneForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PlaneForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));
    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord   VecCoord;
    typedef typename DataTypes::VecDeriv   VecDeriv;
    typedef typename DataTypes::Coord      Coord;
    typedef typename DataTypes::Deriv      Deriv;
    typedef typename DataTypes::VecReal    VecReal;
    typedef typename DataTypes::Real       Real;
	typedef typename DataTypes::CPos       CPos;
	typedef typename DataTypes::DPos       DPos;
    typedef typename Inherit::DataVecCoord DataVecCoord;
    typedef typename Inherit::DataVecDeriv DataVecDeriv;

    typedef sofa::SingleLink< PlaneForceField<DataTypes>, sofa::core::topology::BaseMeshTopology, 
        sofa::BaseLink::FLAG_STRONGLINK | sofa::BaseLink::FLAG_STOREPATH > BaseMeshTopologyLink;

protected:

    struct PlaneContact
    {
        unsigned int index;
        Real         d;

        PlaneContact(unsigned int index, Real d)
        :index(index)
        ,d(d)
        {
        }

        PlaneContact()
        :index(sofa::core::topology::BaseMeshTopology::InvalidID)
        ,d(0)
        {
        }

        inline friend std::ostream& operator<< ( std::ostream& out, const PlaneContact& planeContact )
        {
            out << "PLANECONTACT"
                << " index= " << planeContact.index
                << " d= "     << planeContact.d 
                << " END\n";
            return out;
        }

        inline friend std::istream& operator>> ( std::istream& in, PlaneContact& planeContact )
        {
            std::string str;
            if ((in >> str) && (str == "PLANECONTACT"))
            {
                while (in >> str)
                {
                    if (str == "END")
                    {
                        break;
                    }
                    else if( str=="index=" )
                    {
                        in >> planeContact.index;
                    }
                    else if( str=="d=" )
                    {
                        in >> planeContact.d;
                    }
                }
            }
            return in;
        }
    };



    PlaneForceFieldInternalData<DataTypes> data;

public:

    Data<DPos>                                        planeNormal;
    Data<VecReal>                                     planeD;
    Data<Real>                                        stiffness;
    Data<Real>                                        damping;
	Data<Real>                                        maxForce;
    Data<sofa::helper::vector< unsigned > >           indices; //< BaseMeshTopology should provide a way to get the PointID from the array of points.
    Data<defaulttype::Vec3f>                          color;
    Data<bool>                                        bDrawEnabled;
    Data<Real>                                        drawSize;
    /// option bilateral : if true, the force field is applied on both side of the plane
    Data<bool>                                        bilateral;
    sofa::Data< sofa::helper::vector<PlaneContact > > contacts;

protected:
    PlaneForceField()
        : planeNormal(initData(&planeNormal, "normal", "plane normal"))
        , planeD(initData(&planeD, VecReal(1, Real(0)), "d", "plane d coef, either one value for all"))
        , stiffness(initData(&stiffness, (Real)500, "stiffness", "force stiffness"))
        , damping(initData(&damping, (Real)5, "damping", "force damping"))
        , maxForce(initData(&maxForce, (Real)0, "maxForce", "if non-null, the max force that can be applied to the object"))
        , indices(initData(&indices,"indices","If not empty the list of indices where this forcefield is applied"))
		, color(initData(&color, defaulttype::Vec3f(0.0f,.5f,.2f), "color", "plane color"))
        , bDrawEnabled(initData(&bDrawEnabled, false, "drawEnabled", "enable/disable drawing of plane"))
        , drawSize(initData(&drawSize, (Real)10.0f, "drawSize", "plane display size if draw is enabled"))
        , bilateral( initData(&bilateral, false, "bilateral", "if true the plane force field is applied on both sides"))
        , contacts( initData(&contacts, "contacts","The information related to the points in violation with the plane"))
    {
        this->addAlias(&bDrawEnabled, "draw");

		Deriv n;
		DataTypes::set(n, 0, 1, 0);
        planeNormal.setValue(DataTypes::getDPos(n));
    }

public:

    void setPlane(const Deriv& normal, Real d);

    void setMState(  core::behavior::MechanicalState<DataTypes>* mstate ) 
    { 
        this->mstate = mstate; 
    }

    void setStiffness(Real stiff)
    {
        stiffness.setValue( stiff );
    }

    void setDamping(Real damp)
    {
        damping.setValue( damp );
    }

    void rotate( Deriv axe, Real angle ); // around the origin (0,0,0)


    virtual void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v);
    virtual void addDForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& df, const DataVecDeriv& dx);

    virtual double getPotentialEnergy(const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, const DataVecCoord&  /* x */) const
    {
        serr << "Get potentialEnergy not implemented" << sendl;
        return 0.0;
    }

    virtual void updateStiffness( const VecCoord& x );

    virtual void addKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix );

    void draw(const core::visual::VisualParams* vparams);
    void drawPlane(const core::visual::VisualParams*, float size=0.0f);
    void computeBBox(const core::ExecParams *);

};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_INTERACTIONFORCEFIELD_PLANEFORCEFIELD_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec3dTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec2dTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec1dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec3fTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec2fTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec1fTypes>;
#endif
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
