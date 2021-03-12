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
#ifndef SOFA_COMPONENT_MAPPING_RIGIDRIGIDMAPPING_INL
#define SOFA_COMPONENT_MAPPING_RIGIDRIGIDMAPPING_INL

#include <SofaRigid/RigidRigidMapping.h>
#include <sofa/core/Mapping.inl>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/Axis.h>
#include <sofa/helper/gl/template.h>
#include <sofa/core/BlocMatrixWriter.h>
#include <sofa/helper/decompose.h>

namespace sofa
{

namespace component
{

namespace mapping
{

template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::init()
{
    if (this->points.getValue().empty() && this->toModel!=NULL)
    {
        const OutVecCoord& x =this->toModel->read(core::ConstVecCoordId::position())->getValue();
        OutVecCoord& pts = *points.beginEdit();

        pts.resize(x.size());
        unsigned int i=0;

        if(globalToLocalCoords.getValue() == true)
        {
            const typename In::VecCoord& xfrom =this->fromModel->read(core::ConstVecCoordId::position())->getValue();

            for (i = 0; i < x.size(); ++i)
            {
                pts[i].getCenter() = xfrom[index.getValue()].unprojectPoint(x[i].getCenter());
                pts[i].getOrientation() = xfrom[index.getValue()].getOrientation().inverse() * x[i].getOrientation();
            }
        }
        else
        {
            for (i=0; i<x.size(); ++i)
                pts[i] = x[i];
        }
        points.endEdit();
    }

    this->Inherit::init();
}

template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::clear()
{
    (*this->points.beginEdit()).clear();
    this->points.endEdit();
}

template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::apply(const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data<OutVecCoord>& dOut, const Data<InVecCoord>& dIn)
{
    helper::WriteAccessor< Data<OutVecCoord> > out = dOut;
    helper::ReadAccessor< Data<InVecCoord> > in = dIn;

    out.resize(points.getValue().size());
    pointsR0.resize(points.getValue().size());

    for (unsigned int i=0; i<points.getValue().size(); ++i)
    {
        pointsR0[i] = in[index.getValue()].projectVector(points.getValue()[i].getCenter());
        out[i] = in[index.getValue()].mult(points.getValue()[i]);
    }
  
}

template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::applyJ(const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data<OutVecDeriv>& dOut, const Data<InVecDeriv>& dIn)
{
    helper::WriteAccessor< Data<OutVecDeriv> > childVelocities = dOut;
    helper::ReadAccessor< Data<InVecDeriv> > parentVelocities = dIn;

    childVelocities.resize(points.getValue().size());

    const Vector& v = getVCenter(parentVelocities[index.getValue()]);
    const Vector& omega = getVOrientation(parentVelocities[index.getValue()]);
    for (unsigned int i=0; i<points.getValue().size(); ++i)
    {
        // out = J in
        // J   = [ I  -OM^ ]
        //       [ 0   I   ]
        getVCenter(childVelocities[i]) =  v - cross(pointsR0[i], omega);
        getVOrientation(childVelocities[i]) = omega;
    }

}


template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::applyJT(const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data<InVecDeriv>& dOut, const Data<OutVecDeriv>& dIn)
{
    helper::WriteAccessor< Data<InVecDeriv> > parentForces = dOut;
    helper::ReadAccessor< Data<OutVecDeriv> > childForces = dIn;

    Vector v,omega;

    for (unsigned childIndex=0; childIndex<points.getValue().size(); ++childIndex)
    {
        // out = Jt in
        // Jt = [ I    0 ]
        //      [ OM^ I ]
        const Vector& f = getVCenter(childForces[childIndex]);
        v += f;
        // the sign also change in the cross product since we transpose a skew symmetric matrix
        omega += getVOrientation(childForces[childIndex]) + cross(pointsR0[childIndex], f );
    }

    unsigned parentIndex = index.getValue();
    getVCenter(parentForces[parentIndex]) += v;
    getVOrientation(parentForces[parentIndex]) += omega;
}


template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::applyJT(const core::ConstraintParams * /*cparams*/ /* PARAMS FIRST */, Data<InMatrixDeriv>& dOut, const Data<OutMatrixDeriv>& dIn)
{
    InMatrixDeriv& out = *dOut.beginEdit();
    const OutMatrixDeriv& in = dIn.getValue();


    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        Vector v, omega;

        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();

        for (typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
        {
            const OutDeriv& data = colIt.val();
            // out = Jt in
            // Jt = [ I    0 ]
            //      [ OM^ I ]
            Vector f = getVCenter(data);
            v += f;
            // the sign also change in the cross product since we transpose a skew symmetric matrix
            omega += getVOrientation(data) + cross(pointsR0[colIt.index()], f);
        }

        const InDeriv result(v, omega);
        typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());
        o.addCol(index.getValue(), result);
    }

    dOut.endEdit();
}


template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::computeAccFromMapping(const core::MechanicalParams *mparams /* PARAMS FIRST */, Data<OutVecDeriv>& dAcc_out, const Data<InVecDeriv>& dV_in, const Data<InVecDeriv>& dAcc_in)
{
    const InVecDeriv& v_in = dV_in.getValue();
    //	const InVecDeriv& acc_in = dAcc_in.getValue();

    {
        OutVecDeriv& acc_out = *dAcc_out.beginEdit();
        acc_out.clear();
        acc_out.resize(points.getValue().size());
        dAcc_out.endEdit();
    }

    // current acceleration on acc_in is applied on the child (when more than one mapping)
    applyJ(mparams /* PARAMS FIRST */, dAcc_out, dAcc_in);

    OutVecDeriv& acc_out = *dAcc_out.beginEdit();

    // computation of the acceleration due to the current velocity
    // a+= w^(w^OM)

    Vector omega = getVOrientation(v_in[index.getValue()]);

    for (unsigned int i=0; i<points.getValue().size(); i++)
    {
        getVCenter(acc_out[i]) +=   cross(omega, cross(omega, pointsR0[i]) );
    }

    dAcc_out.endEdit();
}


template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::applyDJT(const core::MechanicalParams* mparams /* PARAMS FIRST */, core::MultiVecDerivId parentForceChangeId, core::ConstMultiVecDerivId )
{
    if( !d_useGeometricStiffness.getValue() ) return;

    helper::WriteAccessor<Data<InVecDeriv> > parentForces (*parentForceChangeId[this->fromModel.get(mparams)].write());
    helper::ReadAccessor<Data<InVecDeriv> > parentDisplacements (*mparams->readDx(this->fromModel));
    Real kfactor = (Real)mparams->kFactor();

    unsigned parentIndex = index.getValue();
    if (vecK.size() != points.getValue().size() || previousChildForces.size() !=  points.getValue().size())
    {
        serr<<"previous force msize mismatch with current size"<<sendl;
        return;
    }
    for (unsigned childIndex=0; childIndex<points.getValue().size(); ++childIndex)
    {
        typename TIn::AngularVector& parentTorque = getVOrientation(parentForces[parentIndex]);

        const typename TIn::AngularVector& parentRotation = getVOrientation(parentDisplacements[parentIndex]);
        parentTorque +=  vecK[childIndex]*parentRotation* kfactor;
//        parentTorque -= TIn::crosscross(previousChildForces[childIndex], parentRotation, pointsR0[childIndex] ) * kfactor;
//        std::cout<<"sym : "<<vecK[childIndex]*parentRotation* kfactor<<"   true :  "<<-TIn::crosscross(previousChildForces[childIndex], parentRotation, pointsR0[childIndex] ) * kfactor<<std::endl;


    }
}

template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::updateK(const sofa::core::MechanicalParams *mparams, sofa::core::ConstMultiVecDerivId childForceId )
{
    if( !d_useGeometricStiffness.getValue() ) return;
    previousChildForces.clear();
    sofa::helper::ReadAccessor< sofa::Data<OutVecDeriv > > childForce( *childForceId[this->toModel.get(mparams)].read() );
    //Real kfactor = (Real)mparams->kFactor();
    vecK.clear();
    defaulttype::Mat<3,3,Real> K;
    for (unsigned childIndex=0; childIndex<points.getValue().size(); ++childIndex)
    {
        const typename Out::Deriv& lambda = childForce[childIndex];
        const typename Out::Deriv::Vec3& f = lambda.getLinear();
        previousChildForces.push_back(f);
        K = defaulttype::crossProductMatrix<Real>( f ) * defaulttype::crossProductMatrix<Real>( pointsR0[childIndex]);
        K.symmetrize();
        sofa::helper::Decompose<Real>::NSDProjection( K ); // negative, semi-definite projection
        vecK.push_back(K);

    }

}



template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::addGeometricStiffnessToMatrix(const sofa::core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    if( !d_useGeometricStiffness.getValue() ) return;

    sofa::core::BlocMatrixWriter< defaulttype::Mat<3, 3, Real> > writer;
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix(this->getMechFrom()[0]);
    writer.addGeometricStiffnessToMatrix(this, mparams, r);
}


template <class TIn, class TOut>
template<class MatrixWriter>
void RigidRigidMapping<TIn, TOut>::addGeometricStiffnessToMatrixT(const sofa::core::MechanicalParams* mparams, MatrixWriter mwriter)
{
    const auto kFact = mparams->kFactor();
    unsigned parentIndex = index.getValue();
    defaulttype::Mat<3,3,Real> Kg;
    for (auto& K : vecK)
    {
        Kg += K * kFact;

    }
    mwriter.addDiag(2*parentIndex + 1, Kg);

}


template <class TIn, class TOut>
void RigidRigidMapping<TIn, TOut>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
	if (!getShow(this,vparams)) return;
    const typename Out::VecCoord& x =this->toModel->read(core::ConstVecCoordId::position())->getValue();
    for (unsigned int i=0; i<x.size(); ++i)
    {
        helper::gl::Axis::draw(x[i].getCenter(), x[i].getOrientation(), axisLength.getValue());
    }
#endif /* SOFA_NO_OPENGL */
}

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
