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
#ifndef SOFA_COMPONENT_MAPPING_RIGIDRIGIDMAPPING_INL
#define SOFA_COMPONENT_MAPPING_RIGIDRIGIDMAPPING_INL

#include <SofaRigid/RigidRigidMapping.h>
#include <sofa/core/Mapping.inl>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/Axis.h>
#include <sofa/helper/gl/template.h>


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
        unsigned int i=0, cpt=0;

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
        //      [ -OM^ I ]
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
void RigidRigidMapping<TIn, TOut>::applyDJT(const core::MechanicalParams* mparams /* PARAMS FIRST */, core::MultiVecDerivId parentForceChangeId, core::ConstMultiVecDerivId )
{
    helper::ReadAccessor<Data<OutVecDeriv> > childForces (*mparams->readF(this->toModel));
    helper::WriteAccessor<Data<InVecDeriv> > parentForces (*parentForceChangeId[this->fromModel.get(mparams)].write());
    helper::ReadAccessor<Data<InVecDeriv> > parentDisplacements (*mparams->readDx(this->fromModel));
    Real kfactor = (Real)mparams->kFactor();

    unsigned parentIndex = index.getValue();
    for (unsigned childIndex=0; childIndex<points.getValue().size(); ++childIndex)
    {
        typename TIn::AngularVector& parentTorque = getVOrientation(parentForces[parentIndex]);
        const typename TIn::AngularVector& parentRotation = getVOrientation(parentDisplacements[parentIndex]);
        parentTorque -=  TIn::crosscross(getLinear(childForces[childIndex]), parentRotation, pointsR0[childIndex] ) * kfactor;
    }
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
            //      [ -OM^ I ]
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
void RigidRigidMapping<TIn, TOut>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
	if (!getShow(this,vparams)) return;
    const typename Out::VecCoord& x =this->toModel->read(core::ConstVecCoordId::position())->getValue();
    for (unsigned int i=0; i<x.size(); ++i)
    {
        helper::gl::Axis::draw(x[i].getCenter(), x[i].getOrientation(), axisLength.getValue());
    }
    glEnd();
#endif /* SOFA_NO_OPENGL */
}

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
