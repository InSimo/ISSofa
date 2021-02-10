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
#define SOFA_COMPONENT_FORCEFIELD_RESTSHAPESPRINGFORCEFIELD_CPP

#include <SofaDeformable/RestShapeSpringsForceField.inl>

#include <sofa/core/visual/DrawTool.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

#ifndef SOFA_FLOAT

template<>
void RestShapeSpringsForceField<Rigid3dTypes>::addForce(const core::MechanicalParams* /* mparams */ /* PARAMS FIRST */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& /* v */)
{
    sofa::helper::WriteAccessor< DataVecDeriv > f1 = f;
    sofa::helper::ReadAccessor< DataVecCoord > p1 = x;

    sofa::helper::ReadAccessor< DataVecCoord > p0 = *getExtPosition();

    f1.resize(p1.size());

    if (d_springLengthThreshold.isSet())
    {
        breakElongatedSprings(x);
    }

    if (recompute_indices.getValue())
    {
        recomputeIndices();
    }

    const VecReal& k = stiffness.getValue();
    const VecReal& k_a = angularStiffness.getValue();

    for (unsigned int i = 0; i < m_indices.size(); i++)
    {
        const unsigned int index = m_indices[i];
        const unsigned int ext_index = m_ext_indices[i];

        // translation
        if (i >= m_pivots.size())
        {
            Vec3d dx = p1[index].getCenter() - p0[ext_index].getCenter();
            getVCenter(f1[index]) -=  dx * (i < k.size() ? k[i] : k[0]) ;
        }
        else
        {
            CPos localPivot = p0[ext_index].getOrientation().inverseRotate(m_pivots[i] - p0[ext_index].getCenter());
            CPos rotatedPivot = p1[index].getOrientation().rotate(localPivot);
            CPos pivot2 = p1[index].getCenter() + rotatedPivot;
            CPos dx = pivot2 - m_pivots[i];
            getVCenter(f1[index]) -= dx * (i < k.size() ? k[i] : k[0]) ;
        }

        // rotation
        Quatd dq = p1[index].getOrientation() * p0[ext_index].getOrientation().inverse();
        Vec3d dir;
        double angle=0;
        dq.normalize();

        if (dq[3] < 0)
        {
            // WARNING inversion quaternion
            dq = dq * -1.0;
        }

        if (dq[3] < 0.999999999999999)
            dq.quatToAxis(dir, angle, false);

        getVOrientation(f1[index]) -= dir * angle * (i < k_a.size() ? k_a[i] : k_a[0]);
    }
}


template<>
void RestShapeSpringsForceField<Rigid3dTypes>::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx)
{
    sofa::helper::WriteAccessor< DataVecDeriv > df1 = df;
    sofa::helper::ReadAccessor< DataVecDeriv > dx1 = dx;

    const VecReal& k = stiffness.getValue();
    const VecReal& k_a = angularStiffness.getValue();
    Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    unsigned int curIndex = 0;

    for (unsigned int i=0; i<m_indices.size(); i++)
    {
        curIndex = m_indices[i];
        getVCenter(df1[curIndex])	 -=  getVCenter(dx1[curIndex]) * ( (i < k.size()) ? k[i] : k[0] ) * kFactor ;
        getVOrientation(df1[curIndex]) -=  getVOrientation(dx1[curIndex]) * (i < k_a.size() ? k_a[i] : k_a[0]) * kFactor ;
    }
}


template<>
void RestShapeSpringsForceField<Rigid3dTypes>::addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix )
{
    const VecReal& k = stiffness.getValue();
    const VecReal& k_a = angularStiffness.getValue();
    const int N = 6;
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
    sofa::defaulttype::BaseMatrix* mat = mref.matrix;
    unsigned int offset = mref.offset;
    Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    unsigned int curIndex = 0;

    for (unsigned int index = 0; index < m_indices.size(); index++)
    {
        curIndex = m_indices[index];

        // translation
        for(int i = 0; i < 3; i++)
        {
            mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * (index < k.size() ? k[index] : k[0]));
        }

        // rotation
        for(int i = 3; i < 6; i++)
        {
            mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * (index < k_a.size() ? k_a[index] : k_a[0]));
        }
    }
}


#endif // SOFA_FLOAT

#ifndef SOFA_DOUBLE

template<>
void RestShapeSpringsForceField<Rigid3fTypes>::addForce(const core::MechanicalParams* /*mparams*/, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& /*v*/)
{
    sofa::helper::WriteAccessor< DataVecDeriv > f1 = f;
    sofa::helper::ReadAccessor< DataVecCoord > p1 = x;

    sofa::helper::ReadAccessor< DataVecCoord > p0 = *getExtPosition();

    f1.resize(p1.size());

    if (d_springLengthThreshold.isSet())
    {
        breakElongatedSprings(x);
    }

    if (recompute_indices.getValue())
    {
        recomputeIndices();
    }

    const VecReal& k = stiffness.getValue();
    const VecReal& k_a = angularStiffness.getValue();

    for (unsigned int i=0; i<m_indices.size(); i++)
    {
        const unsigned int index = m_indices[i];
        const unsigned int ext_index = m_ext_indices[i];

        // translation
        Vec3f dx = p1[index].getCenter() - p0[ext_index].getCenter();
        getVCenter(f1[index]) -=  dx * (i < k.size() ? k[i] : k[0]) ;

        // rotation
        Quatf dq = p1[index].getOrientation() * p0[ext_index].getOrientation().inverse();
        Vec3f dir;
        Real angle=0;
        dq.normalize();
        if (dq[3] < 0.999999999999999)
            dq.quatToAxis(dir, angle);
        dq.quatToAxis(dir, angle);

        getVOrientation(f1[index]) -= dir * angle * (i < k_a.size() ? k_a[i] : k_a[0]) ;
    }
}


template<>
void RestShapeSpringsForceField<Rigid3fTypes>::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx)
{
    sofa::helper::WriteAccessor< DataVecDeriv > df1 = df;
    sofa::helper::ReadAccessor< DataVecDeriv > dx1 = dx;
    Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    const VecReal& k = stiffness.getValue();
    const VecReal& k_a = angularStiffness.getValue();

    unsigned int curIndex = 0;

    for (unsigned int i=0; i<m_indices.size(); i++)
    {
        curIndex = m_indices[i];
        getVCenter(df1[curIndex])      -=  getVCenter(dx1[curIndex])      * (i < k.size() ? k[i] : k[0])   * kFactor ;
        getVOrientation(df1[curIndex]) -=  getVOrientation(dx1[curIndex]) * (i < k_a.size() ? k_a[i] : k_a[0]) * kFactor ;
    }
}


template<>
void RestShapeSpringsForceField<Rigid3fTypes>::addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix )
{
    const VecReal& k = stiffness.getValue();
    const VecReal& k_a = angularStiffness.getValue();
    const int N = 6;

    sofa::core::behavior::MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
    sofa::defaulttype::BaseMatrix* mat = mref.matrix;
    unsigned int offset = mref.offset;
    Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    unsigned int curIndex = 0;


    for (unsigned int index = 0; index < m_indices.size(); index++)
    {
        curIndex = m_indices[index];

        // translation
        for(int i = 0; i < 3; i++)
        {
            mat->add(offset + N * curIndex + i, offset + N * curIndex + i, kFact * (index < k.size() ? k[index] : k[0]));
        }

        // rotation
        for(int i = 3; i < 6; i++)
        {
            mat->add(offset + N * curIndex + i, offset + N * curIndex + i, kFact * (index < k_a.size() ? k_a[index] : k_a[0]));
        }
    }
}

#endif // SOFA_DOUBLE

SOFA_DECL_CLASS(RestShapeSpringsForceField)

int RestShapeSpringsForceFieldClass = core::RegisterObject("Simple elastic springs applied to given degrees of freedom between their current and rest shape position")
#ifndef SOFA_FLOAT
    .add< RestShapeSpringsForceField<Vec3dTypes> >()
    .add< RestShapeSpringsForceField<Vec1dTypes> >()
    .add< RestShapeSpringsForceField<Rigid3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    .add< RestShapeSpringsForceField<Vec3fTypes> >()
    .add< RestShapeSpringsForceField<Vec1fTypes> >()
    .add< RestShapeSpringsForceField<Rigid3fTypes> >()
#endif
;

#ifndef SOFA_FLOAT
template class SOFA_DEFORMABLE_API RestShapeSpringsForceField<Vec3dTypes>;
template class SOFA_DEFORMABLE_API RestShapeSpringsForceField<Vec1dTypes>;
template class SOFA_DEFORMABLE_API RestShapeSpringsForceField<Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_DEFORMABLE_API RestShapeSpringsForceField<Vec3fTypes>;
template class SOFA_DEFORMABLE_API RestShapeSpringsForceField<Vec1fTypes>;
template class SOFA_DEFORMABLE_API RestShapeSpringsForceField<Rigid3fTypes>;
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa
