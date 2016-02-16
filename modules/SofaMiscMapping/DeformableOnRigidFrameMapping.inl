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
#ifndef SOFA_COMPONENT_MAPPING_DEFORMABLEONRIGIDFRAME_INL
#define SOFA_COMPONENT_MAPPING_DEFORMABLEONRIGIDFRAME_INL

#include <SofaMiscMapping/DeformableOnRigidFrameMapping.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/Multi2Mapping.inl>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/core/VecId.h>

#include <sofa/simulation/common/Simulation.h>

#include <string.h>
#include <iostream>


/*!
*   This mapping is derived from the RigidMapping. The difference is :
*   In the RigidMapping, the rigid is considered like a perfect rigid (non-deformable)
*   In this one, the rigid allow a low deformation of the rigid.
*
*   Principale difference with the RigidMapping is in the fonctions apply, applyJ and applyJT
*/

namespace sofa
{

namespace component
{

namespace mapping
{

template <class TIn, class TInRoot, class TOut>
DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::DeformableOnRigidFrameMapping()
    : d_index ( initData ( &d_index, ( unsigned ) 0,"d_index","input DOF d_index" ) )
    , d_globalToLocalCoords ( initData ( &d_globalToLocalCoords,"d_globalToLocalCoords","are the output DOFs initially expressed in global coordinates" ) )
    , d_rootAngularForceScaleFactor(initData(&d_rootAngularForceScaleFactor, (Real)1.0, "rootAngularForceScaleFactor", "Scale factor applied on the angular force accumulated on the rigid model"))
    , d_rootLinearForceScaleFactor(initData(&d_rootLinearForceScaleFactor, (Real)1.0, "rootLinearForceScaleFactor", "Scale factor applied on the linear force accumulated on the rigid model"))
    , m_fromModel(NULL)
    , m_toModel(NULL)
    , m_fromRootModel(NULL)
{
}

template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::init()
{
    if(this->getFromModels1().empty())
    {
        serr << "Error while initializing ; input Model not found" << sendl;
        return;
    }

    if(this->getToModels().empty())
    {
        serr << "Error while initializing ; output Model not found" << sendl;
        return;
    }

    m_fromModel = this->getFromModels1()[0];
    m_toModel = this->getToModels()[0];
    m_toModel->resize(m_fromModel->getSize());

    //Root
    if(!this->getFromModels2().empty())
    {
        m_fromRootModel = this->getFromModels2()[0];
        sout << "Root Model found : Name = " << m_fromRootModel->getName() << sendl;
    }

    Inherit::init();
}

template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::clear(int /*reserve*/)
{

}

template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::apply( typename Out::VecCoord& out, const typename In::VecCoord& inDeformed, const typename InRoot::VecCoord& inRigid  )
{
    rotatedPoints.resize(inDeformed.size());
    out.resize(inDeformed.size());

    const typename InRoot::Coord& inRigid_i = inRigid[d_index.getValue()];

    Coord translation = inRigid_i.getCenter();
    Mat rotation;
    inRigid_i.writeRotationMatrix(rotation);
    m_rootX = inRigid_i; // save the coordinate that is begin currently used ( ie position() versus freePosition() ), 
    //  so that we are consistent during subsequent calls to applyJ 

    for(unsigned int i=0; i<inDeformed.size(); i++)
    {
        rotatedPoints[i] = rotation*inDeformed[i];
        out[i] = rotatedPoints[i];
        out[i] += translation;
    }


}

template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::applyJ( typename Out::VecDeriv&  out , const typename In::VecDeriv& inDeformed , const typename InRoot::VecDeriv& inRigid)
{

    out.resize(inDeformed.size());

    const typename InRoot::Deriv& inRigid_i = (inRigid)[ d_index.getValue() ];

    const Deriv& v     = getVCenter(inRigid_i);
    const Deriv& omega = getVOrientation(inRigid_i);

    for(unsigned int i=0; i<inDeformed.size(); i++)
    {
        out[i]  = cross(omega,rotatedPoints[i]);
        out[i] += m_rootX.getOrientation().rotate(inDeformed[i]); //velocity on the local system : (Vrigid + Vdeform)
        out[i] += v; //center velocity
    }

}
template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in, typename InRoot::VecDeriv& outRoot)
{

    Deriv v,omega;

    for(unsigned int i=0; i<in.size(); i++)
    {
        const Deriv& f = in[i];
        v += f;
        omega += cross(rotatedPoints[i],f);
    }

    typename InRoot::Deriv& rigidForce = outRoot[ d_index.getValue() ];

    getVCenter(rigidForce)      += v;
    getVOrientation(rigidForce) += omega;
    for(unsigned int i=0; i<in.size(); i++)
    {
        out[i] += m_rootX.getOrientation().inverseRotate(in[i]);
    }

}

template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::applyJT( typename In::MatrixDeriv&  out , const typename Out::MatrixDeriv&  in , typename InRoot::MatrixDeriv&  outroot)
{

    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();

        // Creates a constraints if the input constraint is not empty.
        if (colIt != colItEnd)
        {
            Vector v, omega;

            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());
            typename InRoot::MatrixDeriv::RowIterator oRoot = outroot.writeLine(rowIt.index());

            while (colIt != colItEnd)
            {
                const unsigned int node_index = colIt.index();
                // out = Jt in
                // Jt = [ I     ]
                //      [ -OM^t ]
                // -OM^t = OM^

                const Deriv f = colIt.val();
                v += f;
                omega += cross(rotatedPoints[node_index], f);
                InDeriv f_deform = m_rootX.getOrientation().inverseRotate(f);

                o.addCol(node_index, f_deform);

                ++colIt;
            }

            //std::cout << "omega = " << omega.norm() << std::endl;
            //std::cout << "v = " << v.norm() << std::endl;

            const InRootDeriv result(d_rootLinearForceScaleFactor.getValue() * v, d_rootAngularForceScaleFactor.getValue() * omega);
            oRoot.addCol(d_index.getValue(), result);

        }
    }
}



template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::handleTopologyChange(core::topology::Topology* t)
{
	core::topology::BaseMeshTopology* from = dynamic_cast<core::topology::BaseMeshTopology*>(t);
    if(from == NULL ) {
		this->serr << __FUNCTION__ << ": could not cast topology to BaseMeshTopology" << this->sendl; 
		return;
	}
	std::list<const core::topology::TopologyChange *>::const_iterator itBegin = from->beginChange();
    std::list<const core::topology::TopologyChange *>::const_iterator itEnd = from->endChange();

    for ( std::list<const core::topology::TopologyChange *>::const_iterator changeIt = itBegin;
            changeIt != itEnd; ++changeIt )
    {
        const core::topology::TopologyChangeType changeType = ( *changeIt )->getChangeType();
        switch ( changeType )
        {
			case core::topology::ENDING_EVENT:       ///< To notify the end for the current sequence of topological change events
			{
                core::Multi2Mapping<TIn, TInRoot, TOut>::apply(core::MechanicalParams::defaultInstance() /* PARAMS FIRST */, core::VecCoordId::restPosition(), core::ConstVecCoordId::restPosition());
				if(this->f_applyRestPosition.getValue() )
                    core::Multi2Mapping<TIn, TInRoot, TOut>::apply(core::MechanicalParams::defaultInstance() /* PARAMS FIRST */, core::VecCoordId::position(), core::ConstVecCoordId::position());
				break;
			}
			default:
				break;

		}

	}

}


/// Template specialization for 2D rigids
// template<typename real1, typename real2>
// void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::StdRigidTypes<2, real1> >, core::behavior::MechanicalState< defaulttype::StdVectorTypes<defaulttype::Vec<2, real2>, defaulttype::Vec<2, real2>, real2 > > > >::applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
// {
//     Deriv v;
//     Real omega;
//     v = in[d_index.getValue()].getVCenter();
//     omega = (Real)in[d_index.getValue()].getVOrientation();
//     out.resize(points.size());
//     for(unsigned int i=0;i<points.size();i++)
//     {
//         out[i] =  v + Deriv(-rotatedPoints[i][1],rotatedPoints[i][0])*omega;
//     }
// }
//#ifndef SOFA_FLOAT
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2dTypes >, core::behavior::MechanicalState< defaulttype::Vec2dTypes > > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2dTypes >, core::behavior::MechanicalState< defaulttype::Vec2dTypes > > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2dTypes >, core::behavior::MechanicalState< defaulttype::Vec2dTypes > > >::applyJT( In::MatrixDeriv& out, const Out::MatrixDeriv& in );
//#endif
//#ifndef SOFA_DOUBLE
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2fTypes >, core::behavior::MechanicalState< defaulttype::Vec2fTypes > > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2fTypes >, core::behavior::MechanicalState< defaulttype::Vec2fTypes > > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2fTypes >, core::behavior::MechanicalState< defaulttype::Vec2fTypes > > >::applyJT( In::MatrixDeriv& out, const Out::MatrixDeriv& in );
//#endif
//
//#ifndef SOFA_FLOAT
//#ifndef SOFA_DOUBLE
//template<>
//void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2fTypes >, core::behavior::MechanicalState< defaulttype::Vec2dTypes > > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );
//template<>
//void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2dTypes >, core::behavior::MechanicalState< defaulttype::Vec2fTypes > > >::applyJ( Out::VecDeriv& out, const In::VecDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2fTypes >, core::behavior::MechanicalState< defaulttype::Vec2dTypes > > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2dTypes >, core::behavior::MechanicalState< defaulttype::Vec2fTypes > > >::applyJT( In::VecDeriv& out, const Out::VecDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2fTypes >, core::behavior::MechanicalState< defaulttype::Vec2dTypes > > >::applyJT( In::MatrixDeriv& out, const Out::MatrixDeriv& in );
//template<>
//    void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::Rigid2dTypes >, core::behavior::MechanicalState< defaulttype::Vec2fTypes > > >::applyJT( In::MatrixDeriv& out, const Out::MatrixDeriv& in );
//#endif
//#endif
/// Template specialization for 2D rigids
// template<typename real1, typename real2>
// void DeformableOnRigidFrameMapping< core::behavior::MechanicalMapping< core::behavior::MechanicalState< defaulttype::StdRigidTypes<2, real1> >, core::behavior::MechanicalState< defaulttype::StdVectorTypes<defaulttype::Vec<2, real2>, defaulttype::Vec<2, real2>, real2 > > > >::applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
// {
//     Deriv v;
//     Real omega;
//     for(unsigned int i=0;i<points.size();i++)
//     {
//         Deriv f = in[i];
//         v += f;
//         omega += cross(rotatedPoints[i],f);
//     }
//     out[d_index.getValue()].getVCenter() += v;
//     out[d_index.getValue()].getVOrientation() += (typename In::Real)omega;
// }


//
//template <class TIn, class TInRoot, class TOut>
//void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::propagateX()
//{
//	if (m_fromModel!=NULL && m_toModel->getX()!=NULL && m_fromModel->getX()!=NULL)
//		apply(*m_toModel->getX(),m_fromModel->getX(), (m_fromRootModel==NULL ? NULL : m_fromRootModel->read(core::ConstVecCoordId::position())->getValue()));
//
//
//	if( this->f_printLog.getValue())	{
//		serr<<"DeformableOnRigidFrameMapping::propageX processed :"<<sendl;
//		if (m_fromRootModel!=NULL)
//			serr<<"input root: "<<*m_fromRootModel->getX();
//		serr<<"  - input: "<<*m_fromModel->getX()<<"  output : "<<*m_toModel->getX()<<sendl;
//	}
//
//
//}
//
//template <class TIn, class TInRoot, class TOut>
//void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::propagateXfree()
//{
//	if (m_fromModel!=NULL && m_toModel->getXfree()!=NULL && m_fromModel->getXfree()!=NULL)
//		apply(*m_toModel->getXfree(), *m_fromModel->getXfree(), (m_fromRootModel==NULL ? NULL : m_fromRootModel->getXfree()));
//
//	if( this->f_printLog.getValue()){
//		serr<<"DeformableOnRigidFrameMapping::propageXfree processed"<<sendl;
//		if (m_fromRootModel!=NULL)
//			serr<<"input root: "<<*m_fromRootModel->getXfree();
//		serr<<"  - input: "<<*m_fromModel->getXfree()<<"  output : "<<*m_toModel->getXfree()<<sendl;
//	}
//
//}
//
//
//template <class TIn, class TInRoot, class TOut>
//void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::propagateV()
//{
//	if (m_fromModel!=NULL && m_toModel->getV()!=NULL && m_fromModel->getV()!=NULL)
//		applyJ(m_toModel->read(core::ConstVecDerivId::velocity())->getValue(), m_fromModel->read(core::ConstVecCoordId::velocity())->getValue(), (m_fromRootModel==NULL ? NULL : m_fromRootModel->getV()));
//
//	if( this->f_printLog.getValue()){
//		serr<<"DeformableOnRigidFrameMapping::propagateV processed"<<sendl;
//		if (m_fromRootModel!=NULL)
//			serr<<"V input root: "<<m_fromRootModel->read(core::ConstVecDerivId::velocity())->getValue();
//		serr<<"  - V input: "<<m_fromModel->read(core::ConstVecDerivId::velocity())->getValue()<<"   V output : "<<m_toModel->read(core::ConstVecCoordId::velocity())->getValue()<<sendl;
//	}
//
//}
//
//
//
//template <class TIn, class TInRoot, class TOut>
//void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::propagateDx()
//{
//	if (m_fromModel!=NULL && m_toModel->getDx()!=NULL && m_fromModel->getDx()!=NULL)
//		applyJ(*m_toModel->getDx(), *m_fromModel->getDx(), (m_fromRootModel==NULL ? NULL : m_fromRootModel->getDx()));
//
//
//	if( this->f_printLog.getValue()){
//		serr<<"DeformableOnRigidFrameMapping::propagateDx processed"<<sendl;
//		if (m_fromRootModel!=NULL)
//			serr<<"input root: "<<*m_fromRootModel->getDx();
//		serr<<"  - input: "<<*m_fromModel->getDx()<<"  output : "<<*m_toModel->getDx()<<sendl;
//	}
//
//}
//
//
//
//template <class TIn, class TInRoot, class TOut>
//void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::accumulateForce()
//{
//	if (m_fromModel!=NULL && m_toModel->getF()!=NULL && m_fromModel->getF()!=NULL)
//		applyJT(*m_fromModel->getF(), *m_toModel->getF(), (m_fromRootModel==NULL ? NULL : m_fromRootModel->getF()));
//
//
//	if( this->f_printLog.getValue()){
//		serr<<"DeformableOnRigidFrameMapping::accumulateForce processed"<<sendl;
//		serr<<" input f : "<<*m_toModel->getF();
//		if (m_fromRootModel!=NULL)
//			serr<<"- output root: "<<*m_fromRootModel->getF();
//		serr<<"  - output F: "<<*m_fromModel->getF()<<sendl;
//	}
//
//}
//
//
//
//template <class TIn, class TInRoot, class TOut>
//void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::accumulateDf()
//{
//	//if (m_fromModel!=NULL && m_toModel->getF()!=NULL && m_fromModel->getF()!=NULL)
//	applyJT(*m_fromModel->getF(), *m_toModel->getF(), (m_fromRootModel==NULL ? NULL : m_fromRootModel->getF()));
//
//
//	if( this->f_printLog.getValue()){
//		serr<<"DeformableOnRigidFrameMapping::accumulateDf processed"<<sendl;
//		serr<<" input df : "<<*m_toModel->getF();
//		if (m_fromRootModel!=NULL)
//			serr<<"- output root: "<<*m_fromRootModel->getF();
//		serr<<"  - output: "<<*m_fromModel->getF()<<sendl;
//	}
//
//}
//
//
//
//template <class TIn, class TInRoot, class TOut>
//void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::accumulateConstraint()
//{
//	if (m_fromModel!=NULL && m_toModel->getC()!=NULL && m_fromModel->getC()!=NULL)
//	{
//		//propagateX();
//		applyJT(*m_fromModel->getC(), *m_toModel->getC(), (m_fromRootModel==NULL ? NULL : m_fromRootModel->getC()));
//
//		//// Accumulate contacts indices through the MechanicalMapping
//		//std::vector<unsigned int>::iterator it = m_toModel->getConstraintId().begin();
//		//std::vector<unsigned int>::iterator itEnd = m_toModel->getConstraintId().end();
//
//		//while (it != itEnd)
//		//{
//		//	m_fromModel->setConstraintId(*it);
//		//	// in case of a "multi-mapping" (the articulation system is placed on a  simulated object)
//		//	// the constraints are transmitted to the rootModle (the <rigidtype> object which is the root of the articulated system)
//		//	if (m_fromRootModel!=NULL)
//		//		m_fromRootModel->setConstraintId(*it);
//		//	it++;
//		//}
//	}
//}

/*
template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::recomputeRigidMass()
{

if (m_fromModel==NULL || m_fromRootModel==NULL)
return;
std::cout<<"recmpute Rigid Mass" <<std::endl;


masses = dynamic_cast<BaseMass*> (m_fromModel->getContext()->getMass());
if(!masses)
return;

totalMass = 0.0;
//compute the total mass of the object
for (unsigned int i=0 ; i<m_fromModel->getX()->size() ; i++)
totalMass += masses->getElementMass(i);







sofa::core::objectmodel::
m_fromModel->getContext()->get(m_fromRootModel, core::objectmodel::BaseContext::SearchUp);


}
*/



template <class TIn, class TInRoot, class TOut>
void DeformableOnRigidFrameMapping<TIn, TInRoot, TOut>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowMappings()) return;
    std::vector< sofa::defaulttype::Vector3 > points;
    sofa::defaulttype::Vector3 point;

    const typename Out::VecCoord& x = m_toModel->read(core::ConstVecCoordId::position())->getValue();
    for (unsigned int i=0; i<x.size(); i++)
    {
        point = Out::getCPos(x[i]);
        points.push_back(point);
    }
    vparams->drawTool()->drawPoints(points, 7, sofa::defaulttype::Vec<4,float>(1,1,0,1));
}

} // namespace mapping

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_MAPPING_DEFORMABLEONRIGIDFRAME_INL
