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
#ifndef SOFA_COMPONENT_FORCEFIELD_BEAMFEMFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_BEAMFEMFORCEFIELD_INL

#include <sofa/core/behavior/ForceField.inl>
#include <SofaBaseTopology/TopologyData.inl>
#include "BeamFEMForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <SofaBaseTopology/GridTopology.h>
#include <sofa/simulation/common/Simulation.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/Axis.h>
#include <sofa/helper/rmath.h>
#include <assert.h>
#include <iostream>
#include <set>
#include <sofa/helper/system/gl.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/simulation/common/Node.h>

#include "StiffnessContainer.h"
#include "PoissonContainer.h"

namespace sofa
{

namespace component
{

namespace forcefield
{


template<class DataTypes>
BeamFEMForceField<DataTypes>::BeamFEMForceField()
    : beamsData(initData(&beamsData, "beamsData", "Internal element data"))
    , _indexedElements(NULL)
    , _poissonRatio(initData(&_poissonRatio,VecReal(1, Real(0.49)),"poissonRatio","Potion Ratio"))
    , _youngModulus(initData(&_youngModulus,VecReal(1, Real(5000)),"youngModulus","Young Modulus"))
    , _radius(initData(&_radius,VecReal(1, Real(0.1)),"radius","radius of the section"))
    , _radiusInner(initData(&_radiusInner,VecReal(1, Real(0.0)),"radiusInner","inner radius of the section for hollow beams"))
    , _list_segment(initData(&_list_segment,"listSegment", "apply the forcefield to a subset list of beam segments. If no segment defined, forcefield applies to the whole topology"))
    , _applyRigidTransFirstBeam(initData(&_applyRigidTransFirstBeam,false,"applyRigidTransFirstBeam","apply rigid transformation (translation + rotation) on First Beam"))
    , _rigidTransFirstBeam(initData(&_rigidTransFirstBeam,Coord(Vec3(0.0,0.0,0.0),defaulttype::Quat(0.0,0.0,0.0,1.0)),"rigidTransFirstBeam","Rigid transformation applied on first beam"))
    , _rigidY(initData(&_rigidY,(Real)1.0,"rigidY","Increase/Decrease stiffness on Y-axis"))
    , _rigidZ(initData(&_rigidZ,(Real)1.0,"rigidZ","Increase/Decrease stiffness on Z-axis"))
    , _partial_list_segment(false)
    , edgeHandler(NULL)
    , m_rigidTsf(Coord())
{
    edgeHandler = new BeamFFEdgeHandler(this, &beamsData);

	_poissonRatio.setRequired(true);
	_youngModulus.setRequired(true);
}

template<class DataTypes>
BeamFEMForceField<DataTypes>::BeamFEMForceField(Real poissonRatio, Real youngModulus, Real radius, Real radiusInner)
    : beamsData(initData(&beamsData, "beamsData", "Internal element data"))
    , _indexedElements(NULL)
    , _poissonRatio(initData(&_poissonRatio,VecReal(1, poissonRatio),"poissonRatio","Potion Ratio"))
    , _youngModulus(initData(&_youngModulus,VecReal(1, youngModulus),"youngModulus","Young Modulus"))
    , _radius(initData(&_radius,VecReal(1, radius),"radius","radius of the section"))
    , _radiusInner(initData(&_radiusInner,VecReal(1, radiusInner),"radiusInner","inner radius of the section for hollow beams"))
    , _list_segment(initData(&_list_segment,"listSegment", "apply the forcefield to a subset list of beam segments. If no segment defined, forcefield applies to the whole topology"))
    , _applyRigidTransFirstBeam(initData(&_applyRigidTransFirstBeam,false,"applyRigidTransFirstBeam","apply rigid transformation (translation + rotation) on First Beam"))
    , _rigidTransFirstBeam(initData(&_rigidTransFirstBeam,Coord(Vec3(0.0,0.0,0.0),defaulttype::Quat(0.0,0.0,0.0,1.0)),"rigidTransFirstBeam","Rigid transformation applied on first beam"))
    , _rigidY(initData(&_rigidY,(Real)1.0,"rigidY","Increase/Decrease stiffness on Y-axis"))
    , _rigidZ(initData(&_rigidZ,(Real)1.0,"rigidZ","Increase/Decrease stiffness on Z-axis"))
    , _partial_list_segment(false)
    , edgeHandler(NULL)
    , m_rigidTsf(Coord())
{
    edgeHandler = new BeamFFEdgeHandler(this, &beamsData);  

	_poissonRatio.setRequired(true);
	_youngModulus.setRequired(true);
}

template<class DataTypes>
BeamFEMForceField<DataTypes>::~BeamFEMForceField()
{
    if(edgeHandler) delete edgeHandler;
}

template <class DataTypes>
void BeamFEMForceField<DataTypes>::bwdInit()
{
#ifdef SOFA_HAVE_EIGEN2
    core::behavior::BaseMechanicalState* state = this->getContext()->getMechanicalState();
    assert(state);
    matS.resize(state->getMatrixSize(),state->getMatrixSize());
    lastUpdatedStep=-1.0;
#endif
}



template <class DataTypes>
void BeamFEMForceField<DataTypes>::init()
{
    this->core::behavior::ForceField<DataTypes>::init();
    sofa::core::objectmodel::BaseContext* context = this->getContext();

    _topology = context->getMeshTopology();

    stiffnessContainer = context->core::objectmodel::BaseContext::get<container::StiffnessContainer>();
    //  lengthContainer = context->core::objectmodel::BaseContext::get<container::LengthContainer>();
    poissonContainer = context->core::objectmodel::BaseContext::get<container::PoissonContainer>();
    //	radiusContainer = context->core::objectmodel::BaseContext::get<container::RadiusContainer>();

    if (_topology==NULL)
    {
        serr << "ERROR(BeamFEMForceField): object must have a BaseMeshTopology (i.e. EdgeSetTopology or MeshTopology)."<<sendl;
        return;
    }
    else
    {
        if(_topology->getNbEdges()==0)
        {
            serr << "ERROR(BeamFEMForceField): topology is empty."<<sendl;
            return;
        }
        _indexedElements = &_topology->getEdges();
        if (_list_segment.getValue().size() == 0)
        {
            sout<<"Forcefield named "<<this->getName()<<" applies to the wholo topo"<<sendl;
            _partial_list_segment = false;
        }
        else
        {
            sout<<"Forcefield named "<<this->getName()<<" applies to a subset of edges"<<sendl;
            _partial_list_segment = true;

            for (unsigned int j=0; j<_list_segment.getValue().size(); j++)
            {
                unsigned int i = _list_segment.getValue()[j];
                if (i>=_indexedElements->size())
                {
                    serr<<"WARNING defined listSegment is not compatible with topology"<<sendl;
                    _partial_list_segment = false;
                }
            }
        }
    }

    beamsData.createTopologicalEngine(_topology,edgeHandler);
    beamsData.registerTopologicalData();

    bool isTransform = _applyRigidTransFirstBeam.getValue();
    if ( isTransform )
    {
        m_rigidTsf = _rigidTransFirstBeam.getValue();
    }

    reinit();
}

template <class DataTypes>
void BeamFEMForceField<DataTypes>::reinit()
{
    unsigned int n = _indexedElements->size();

    initBeams(n);

    const VecReal& vecPoisson = _poissonRatio.getValue();
    const VecReal& vecYoung = _youngModulus.getValue();
    const VecReal& vecRadius = _radius.getValue();
    const VecReal& vecRadiusInner = _radiusInner.getValue();
    bool doNotInit = false;

    if (vecPoisson.empty())
    {
        doNotInit = true;
    }
    else if (vecPoisson.size() != 1 && vecPoisson.size() != n){
        serr << "poissonRatio vector is not the same size as the number of beams in the topology (" << vecPoisson.size() << "!=" << n << ")." << sendl;
        serr << "Using first value for every beam..." << sendl;
    }
    if (!stiffnessContainer)
    {
        if (vecYoung.empty())
        {
            doNotInit = true;
        }
        else if (vecYoung.size() != 1 && vecYoung.size() != n){
            serr << "youngModulus vector is not the same size as the number of beams in the topology (" << vecYoung.size() << "!=" << n << ")." << sendl;
            serr << "Using first value for every beam..." << sendl;
        }
    }
    if (vecRadius.empty())
    {
        doNotInit = true;
    }
    else if (vecRadius.size() != 1 && vecRadius.size() != n){
        serr << "radius vector is not the same size as the number of beams in the topology (" << vecRadius.size() << "!=" << n << ")." << sendl;
        serr << "Using first value for every beam..." << sendl;
    }
    if (vecRadiusInner.empty())
    {
        doNotInit = true;
    }
    else if (vecRadiusInner.size() != 1 && vecRadiusInner.size() != n){
        serr << "radiusInner vector is not the same size as the number of beams in the topology (" << vecRadiusInner.size() << "!=" << n << ")." << sendl;
        serr << "Using first value for every beam..." << sendl;
    }

    if (!doNotInit)
    {
        for (unsigned int i=0; i<n; ++i)
        {
            reinitBeam(i);
        }
        sout << "BeamFEMForceField: init OK, " << n << " elements" << sendl;
    }
    else
    {
        serr << "BeamFEMForceField: bad init, " << n << " elements" << sendl;
    }
}

template <class DataTypes>
void BeamFEMForceField<DataTypes>::reinitBeam(unsigned int i)
{
    double stiffness, length, radius, poisson, radiusInner;
    Index a = (*_indexedElements)[i][0];
    Index b = (*_indexedElements)[i][1];

    const VecCoord& x0 = this->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

    //if (poissonContainer)
    //	poisson = poissonContainer->getPoisson(i) ;
    //else
    if (_poissonRatio.getValue().size() == 1) 
    {
        poisson = _poissonRatio.getValue()[0];
    }
    else
    {
        poisson = _poissonRatio.getValue()[i];
    }
    if (stiffnessContainer)
    {
        stiffness = stiffnessContainer->getStiffness(i);
    }
    else if (_youngModulus.getValue().size() == 1)
    {
        stiffness = _youngModulus.getValue()[0];
    }
    else
    {
        stiffness = _youngModulus.getValue()[i];
    }
    if (_radius.getValue().size() == 1) 
    {
        radius = _radius.getValue()[0];
    }
    else
    {
        radius = _radius.getValue()[i];
    }
    if (_radiusInner.getValue().size() == 1) 
    {
        radiusInner = _radiusInner.getValue()[0];
    }
    else
    {
        radiusInner = _radiusInner.getValue()[i];
    }

    length = (x0[a].getCenter()-x0[b].getCenter()).norm();

    setBeam(i, stiffness, length, poisson, radius, radiusInner);

    computeStiffness(i,a,b);

    initLarge(i,a,b);
}

template< class DataTypes>
void BeamFEMForceField<DataTypes>::BeamFFEdgeHandler::applyCreateFunction(unsigned int edgeIndex, BeamInfo &ei, const topology::Edge &, const sofa::helper::vector<unsigned int> &, const sofa::helper::vector<double> &)
{
    if(ff)
    {
        ff->reinitBeam(edgeIndex);
        ei = ff->beamsData.getValue()[edgeIndex];
    }
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::addForce(const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecDeriv &  dataF, const DataVecCoord &  dataX , const DataVecDeriv & /*dataV*/ )
{
    VecDeriv& f = *(dataF.beginEdit());
    const VecCoord& p=dataX.getValue();

    f.resize(p.size());

    typename VecElement::const_iterator it;

    if (_partial_list_segment)
    {

        for (unsigned int j=0; j<_list_segment.getValue().size(); j++)
        {
            unsigned int i = _list_segment.getValue()[j];
            Element edge= (*_indexedElements)[i];
            Index a = edge[0];
            Index b = edge[1];
            initLarge(i,a,b);
            accumulateForceLarge( f, p, i, a, b );
        }
    }
    else
    {
        unsigned int i;
        for(it=_indexedElements->begin(),i=0; it!=_indexedElements->end(); ++it,++i)
        {

            Index a = (*it)[0];
            Index b = (*it)[1];

            initLarge(i,a,b);
            accumulateForceLarge( f, p, i, a, b );
        }
    }

    dataF.endEdit();
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::addDForce(const sofa::core::MechanicalParams *mparams /* PARAMS FIRST */, DataVecDeriv& datadF , const DataVecDeriv& datadX)
{
    VecDeriv& df = *(datadF.beginEdit());
    const VecDeriv& dx=datadX.getValue();
    Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    df.resize(dx.size());

    if (_partial_list_segment)
    {

        for (unsigned int j=0; j<_list_segment.getValue().size(); j++)
        {
            unsigned int i = _list_segment.getValue()[j];
            Element edge= (*_indexedElements)[i];
            Index a = edge[0];
            Index b = edge[1];

            applyStiffnessLarge(df, dx, i, a, b, kFactor);
        }

    }
    else
    {
        typename VecElement::const_iterator it;
        unsigned int i = 0;
        for(it = _indexedElements->begin() ; it != _indexedElements->end() ; ++it, ++i)
        {
            Index a = (*it)[0];
            Index b = (*it)[1];

            applyStiffnessLarge(df, dx, i, a, b, kFactor);
        }
    }

    datadF.endEdit();
}

template<class DataTypes>
typename BeamFEMForceField<DataTypes>::Real BeamFEMForceField<DataTypes>::peudo_determinant_for_coef ( const defaulttype::Mat<2, 3, Real>&  M )
{
    return  M[0][1]*M[1][2] - M[1][1]*M[0][2] -  M[0][0]*M[1][2] + M[1][0]*M[0][2] + M[0][0]*M[1][1] - M[1][0]*M[0][1];
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::computeStiffness(int i, Index , Index )
{
    Real   phiy, phiz;
    Real _L = (Real)beamsData.getValue()[i]._L;
    Real _A = (Real)beamsData.getValue()[i]._A;
    Real _nu = (Real)beamsData.getValue()[i]._nu;
    Real _E = (Real)beamsData.getValue()[i]._E;
    Real _Iy = (Real)beamsData.getValue()[i]._Iy;
    Real _Iz = (Real)beamsData.getValue()[i]._Iz;
    Real _Asy = (Real)beamsData.getValue()[i]._Asy;
    Real _Asz = (Real)beamsData.getValue()[i]._Asz;
    Real _G = (Real)beamsData.getValue()[i]._G;
    Real _J = (Real)beamsData.getValue()[i]._J;
    Real L2 = (Real) (_L * _L);
    Real L3 = (Real) (L2 * _L);
    Real EIy = (Real)((_E * _Iy) * _rigidY.getValue());
    Real EIz = (Real)((_E * _Iz) * _rigidZ.getValue());

    // Find shear-deformation parameters
    if (_Asy == 0)
        phiy = 0.0;
    else
        phiy = (Real)(24.0*(1.0+_nu)*_Iz/(_Asy*L2));

    if (_Asz == 0)
        phiz = 0.0;
    else
        phiz = (Real)(24.0*(1.0+_nu)*_Iy/(_Asz*L2));
    helper::vector<BeamInfo>& bd = *(beamsData.beginEdit());
    StiffnessMatrix& k_loc = bd[i]._k_loc;

    // Define stiffness matrix 'k' in local coordinates
    k_loc.clear();
    k_loc[6][6]   = k_loc[0][0]   = _E*_A/_L;
    k_loc[7][7]   = k_loc[1][1]   = (Real)(12.0*EIz/(L3*(1.0+phiy)));
    k_loc[8][8]   = k_loc[2][2]   = (Real)(12.0*EIy/(L3*(1.0+phiz)));
    k_loc[9][9]   = k_loc[3][3]   = _G*_J/_L;
    k_loc[10][10] = k_loc[4][4]   = (Real)((4.0+phiz)*EIy/(_L*(1.0+phiz)));
    k_loc[11][11] = k_loc[5][5]   = (Real)((4.0+phiy)*EIz/(_L*(1.0+phiy)));

    k_loc[4][2]   = (Real)(-6.0*EIy/(L2*(1.0+phiz)));
    k_loc[5][1]   = (Real)( 6.0*EIz/(L2*(1.0+phiy)));
    k_loc[6][0]   = -k_loc[0][0];
    k_loc[7][1]   = -k_loc[1][1];
    k_loc[7][5]   = -k_loc[5][1];
    k_loc[8][2]   = -k_loc[2][2];
    k_loc[8][4]   = -k_loc[4][2];
    k_loc[9][3]   = -k_loc[3][3];
    k_loc[10][2]  = k_loc[4][2];
    k_loc[10][4]  = (Real)((2.0-phiz)*EIy/(_L*(1.0+phiz)));
    k_loc[10][8]  = -k_loc[4][2];
    k_loc[11][1]  = k_loc[5][1];
    k_loc[11][5]  = (Real)((2.0-phiy)*EIz/(_L*(1.0+phiy)));
    k_loc[11][7]  = -k_loc[5][1];

    for (int i=0; i<=10; i++)
        for (int j=i+1; j<12; j++)
            k_loc[i][j] = k_loc[j][i];

    beamsData.endEdit();
}

inline defaulttype::Quat qDiff(defaulttype::Quat a, const defaulttype::Quat& b)
{
    if (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]<0)
    {
        a[0] = -a[0];
        a[1] = -a[1];
        a[2] = -a[2];
        a[3] = -a[3];
    }
    defaulttype::Quat q = b.inverse() * a;
    return q;
}

////////////// large displacements method
template<class DataTypes>
void BeamFEMForceField<DataTypes>::initLarge(int i, Index a, Index b)
{
    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();

    defaulttype::Quat quatA, quatB, dQ;
    Vec3 dW;

    bool isTransform = _applyRigidTransFirstBeam.getValue() && a == 0;
    quatA = isTransform ? (x[a].mult(m_rigidTsf)).getOrientation() : x[a].getOrientation();
    quatB = x[b].getOrientation();

    quatA.normalize();
    quatB.normalize();

    dQ = qDiff(quatB, quatA);
    dQ.normalize();

    dW = dQ.getLog();

    SReal Theta = dW.norm();


    if(Theta>(SReal)0.0000001)
    {
        dW.normalize();

        beamQuat(i) = quatA*dQ.axisToQuat(dW, Theta/2);
        beamQuat(i).normalize();
    }
    else
        beamQuat(i)= quatA;

    beamsData.endEdit();
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::accumulateForceLarge( VecDeriv& f, const VecCoord & x, int i, Index a, Index b )
{
    const VecCoord& x0 = this->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

    bool isTransform = _applyRigidTransFirstBeam.getValue() && a == 0;
    defaulttype::Quat x_a_getOrientation = isTransform ? (x[a].mult(m_rigidTsf)).getOrientation() : x[a].getOrientation();
    x_a_getOrientation.normalize();

    beamQuat(i) = x_a_getOrientation;
    beamQuat(i).normalize();

    beamsData.endEdit();

    defaulttype::Vec<3,Real> u, P1P2, P1P2_0;
    // local displacement
    Displacement depl;

    // translations //
    P1P2_0 = x0[b].getCenter() - x0[a].getCenter();
    P1P2_0 = x0[a].getOrientation().inverseRotate(P1P2_0);
    P1P2 = x[b].getCenter() - x[a].getCenter();
    P1P2 = x_a_getOrientation.inverseRotate(P1P2);
    u = P1P2 - P1P2_0;

    depl[0] = 0.0; 	depl[1] = 0.0; 	depl[2] = 0.0;
    depl[6] = u[0]; depl[7] = u[1]; depl[8] = u[2];

    // rotations //
    defaulttype::Quat dQ0, dQ;

    // dQ = QA.i * QB ou dQ = QB * QA.i() ??
    dQ0 = qDiff(x0[b].getOrientation(), x0[a].getOrientation()); // x0[a].getOrientation().inverse() * x0[b].getOrientation();
    dQ =  qDiff(x[b].getOrientation(), x_a_getOrientation); // x[a].getOrientation().inverse() * x[b].getOrientation();
    //u = dQ.getLog() - dQ0.getLog();

    dQ0.normalize();
    dQ.normalize();

    defaulttype::Quat tmpQ = qDiff(dQ,dQ0);
    tmpQ.normalize();

    u = tmpQ.getLog(); //dQ.getLog() - dQ0.getLog();

    depl[3] = 0.0; 	depl[4] = 0.0; 	depl[5] = 0.0;
    depl[9] = u[0]; depl[10]= u[1]; depl[11]= u[2];

    // this computation can be optimised: (we know that half of "depl" is null)
    Displacement force = beamsData.getValue()[i]._k_loc * depl;


    // Apply lambda transpose (we use the rotation value of point a for the beam)

    Vec3 fa1 = x_a_getOrientation.rotate(defaulttype::Vec3d(force[0],force[1],force[2]));
    Vec3 fa2 = x_a_getOrientation.rotate(defaulttype::Vec3d(force[3],force[4],force[5]));

    Vec3 fb1 = x_a_getOrientation.rotate(defaulttype::Vec3d(force[6],force[7],force[8]));
    Vec3 fb2 = x_a_getOrientation.rotate(defaulttype::Vec3d(force[9],force[10],force[11]));


    f[a] += Deriv(-fa1, -fa2);
    f[b] += Deriv(-fb1, -fb2);

}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::applyStiffnessLarge(VecDeriv& df, const VecDeriv& dx, int i, Index a, Index b, double fact)
{
    Displacement local_depl;
    defaulttype::Vec<3,Real> u;
    defaulttype::Quat& q = beamQuat(i); //x[a].getOrientation();
    q.normalize();

    u = q.inverseRotate(getVCenter(dx[a]));
    local_depl[0] = u[0];
    local_depl[1] = u[1];
    local_depl[2] = u[2];

    u = q.inverseRotate(getVOrientation(dx[a]));
    local_depl[3] = u[0];
    local_depl[4] = u[1];
    local_depl[5] = u[2];

    u = q.inverseRotate(getVCenter(dx[b]));
    local_depl[6] = u[0];
    local_depl[7] = u[1];
    local_depl[8] = u[2];

    u = q.inverseRotate(getVOrientation(dx[b]));
    local_depl[9] = u[0];
    local_depl[10] = u[1];
    local_depl[11] = u[2];

    Displacement local_force = beamsData.getValue()[i]._k_loc * local_depl;

    Vec3 fa1 = q.rotate(defaulttype::Vec3d(local_force[0],local_force[1] ,local_force[2] ));
    Vec3 fa2 = q.rotate(defaulttype::Vec3d(local_force[3],local_force[4] ,local_force[5] ));
    Vec3 fb1 = q.rotate(defaulttype::Vec3d(local_force[6],local_force[7] ,local_force[8] ));
    Vec3 fb2 = q.rotate(defaulttype::Vec3d(local_force[9],local_force[10],local_force[11]));


    df[a] += Deriv(-fa1,-fa2) * fact;
    df[b] += Deriv(-fb1,-fb2) * fact;
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::addKToMatrix(const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix )
{
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix(this->mstate);
    Real k = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());
    defaulttype::BaseMatrix* mat = r.matrix;

    if (r)
    {
        unsigned int i=0;

		unsigned int &offset = r.offset;

        if (_partial_list_segment)
        {

            for (unsigned int j=0; j<_list_segment.getValue().size(); j++)
            {

                i = _list_segment.getValue()[j];
                Element edge= (*_indexedElements)[i];
                Index a = edge[0];
                Index b = edge[1];

                Displacement local_depl;
                defaulttype::Vec3d u;
                defaulttype::Quat& q = beamQuat(i); //x[a].getOrientation();
                q.normalize();
                Transformation R,Rt;
                q.toMatrix(R);
                Rt.transpose(R);
                const StiffnessMatrix& K0 = beamsData.getValue()[i]._k_loc;
                StiffnessMatrix K;
                for (int x1=0; x1<12; x1+=3)
                    for (int y1=0; y1<12; y1+=3)
                    {
                        defaulttype::Mat<3,3,Real> m;
                        K0.getsub(x1,y1, m);
                        m = R*m*Rt;
                        K.setsub(x1,y1, m);
                    }
                int index[12];
                for (int x1=0; x1<6; x1++)
                    index[x1] = offset+a*6+x1;
                for (int x1=0; x1<6; x1++)
                    index[6+x1] = offset+b*6+x1;
                for (int x1=0; x1<12; ++x1)
                    for (int y1=0; y1<12; ++y1)
                        mat->add(index[x1], index[y1], - K(x1,y1)*k);

            }

        }
        else
        {
            typename VecElement::const_iterator it;
            for(it = _indexedElements->begin() ; it != _indexedElements->end() ; ++it, ++i)
            {
                Index a = (*it)[0];
                Index b = (*it)[1];


                Displacement local_depl;
                defaulttype::Vec3d u;
                defaulttype::Quat& q = beamQuat(i); //x[a].getOrientation();
                q.normalize();
                Transformation R,Rt;
                q.toMatrix(R);
                Rt.transpose(R);
                const StiffnessMatrix& K0 = beamsData.getValue()[i]._k_loc;
                StiffnessMatrix K;
                for (int x1=0; x1<12; x1+=3)
                    for (int y1=0; y1<12; y1+=3)
                    {
                        defaulttype::Mat<3,3,Real> m;
                        K0.getsub(x1,y1, m);
                        m = R*m*Rt;
                        K.setsub(x1,y1, m);
                    }
                int index[12];
                for (int x1=0; x1<6; x1++)
                    index[x1] = offset+a*6+x1;
                for (int x1=0; x1<6; x1++)
                    index[6+x1] = offset+b*6+x1;
                for (int x1=0; x1<12; ++x1)
                    for (int y1=0; y1<12; ++y1)
                        mat->add(index[x1], index[y1], - K(x1,y1)*k);

            }
        }

    }

}


template<class DataTypes>
void BeamFEMForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowForceFields()) return;
    if (!this->mstate) return;

    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();

    sofa::helper::vector< defaulttype::Vector3 > points[3];

    if (_partial_list_segment)
    {
        for (unsigned int j=0; j<_list_segment.getValue().size(); j++)
            drawElement(_list_segment.getValue()[j], points, x);
    }
    else
    {
        for (unsigned int i=0; i<_indexedElements->size(); ++i)
            drawElement(i, points, x);
    }
    vparams->drawTool()->drawLines(points[0], 1, defaulttype::Vec<4,float>(1,0,0,1));
    vparams->drawTool()->drawLines(points[1], 1, defaulttype::Vec<4,float>(0,1,0,1));
    vparams->drawTool()->drawLines(points[2], 1, defaulttype::Vec<4,float>(0,0,1,1));
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::drawElement(int i, sofa::helper::vector< defaulttype::Vector3 >* points, const VecCoord& x)
{
    Index a = (*_indexedElements)[i][0];
    Index b = (*_indexedElements)[i][1];
    //sout << "edge " << i << " : "<<a<<" "<<b<<" = "<<x[a].getCenter()<<"  -  "<<x[b].getCenter()<<" = "<<beamsData[i]._L<<sendl;
    defaulttype::Vec3d p; p = (x[a].getCenter()+x[b].getCenter())*0.5;
    defaulttype::Vec3d beamVec;
    beamVec[0]=beamsData.getValue()[i]._L*0.5; beamVec[1] = 0.0; beamVec[2] = 0.0;

    const defaulttype::Quat& q = beamQuat(i);
    // axis X
    points[0].push_back(p - q.rotate(beamVec) );
    points[0].push_back(p + q.rotate(beamVec) );
    // axis Y
    beamVec[0]=0.0; beamVec[1] = beamsData.getValue()[i]._r*0.5;
    points[1].push_back(p );
    points[1].push_back(p + q.rotate(beamVec) );
    // axis Z
    beamVec[1]=0.0; beamVec[2] = beamsData.getValue()[i]._r*0.5;
    points[2].push_back(p);
    points[2].push_back(p + q.rotate(beamVec) );
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::initBeams(unsigned int size)
{
    helper::vector<BeamInfo>& bd = *(beamsData.beginEdit());
    bd.resize(size);
    beamsData.endEdit();
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::setBeam(unsigned int i, double E, double L, double nu, double r, double rInner)
{
    helper::vector<BeamInfo>& bd = *(beamsData.beginEdit());
    bd[i].init(E,L,nu,r,rInner);
    beamsData.endEdit();
}

template<class DataTypes>
void BeamFEMForceField<DataTypes>::BeamInfo::init(double E, double L, double nu, double r, double rInner)
{
    _E = E;
    _E0 = E;
    _nu = nu;
    _L = L;
    _r = r;
    _rInner = rInner;

    _G=_E/(2.0*(1.0+_nu));
    _Iz = M_PI*(r*r*r*r - rInner*rInner*rInner*rInner)/4.0;

    _Iy = _Iz ;
    _J = _Iz+_Iy;
    _A = M_PI*(r*r - rInner*rInner);

    _Asy = 0.0;
    _Asz = 0.0;
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_BEAMFEMFORCEFIELD_INL
