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
#ifndef SOFA_COMPONENT_FORCEFIELD_ELLIPSOIDFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_ELLIPSOIDFORCEFIELD_INL

#include <SofaBoundaryCondition/EllipsoidForceField.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/ForceField.inl>
#include <sofa/helper/system/config.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>
#include <sofa/helper/gl/template.h>
#include <assert.h>
#include <iostream>

namespace sofa
{

namespace component
{

namespace forcefield
{

// v = sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1
// dv/dxj = xj/rj^2 * 1/sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)

// f  = -stiffness * v * (dv/dp) / norm(dv/dp)

// fi = -stiffness * (sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1) * (xi/ri^2) / sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4)

// dfi/dxj = -stiffness * [ d(sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1)/dxj *   (xi/ri^2) / sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4)
//                          +  (sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1)     * d(xi/ri^2)/dxj / sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4)
//                          +  (sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1)     *  (xi/ri^2) * d(1/sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4))/dxj ]
// dfi/dxj = -stiffness * [ xj/rj^2 * 1/sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2) * (xi/ri^2) / sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4)
//                          +  (sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1)       * (i==j)/ri^2 / sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4)
//                          +  (sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1)       * (xi/ri^2) * (-1/2*2xj/rj^4*1/(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4) ]
// dfi/dxj = -stiffness * [ xj/rj^2 * 1/sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2) * (xi/ri^2) / sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4)
//                          +  (sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1)       * (i==j)/ri^2 / sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4)
//                          +  (sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2)-1)       * (xi/ri^2) * (-xj/rj^4*1/(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4) ]

// dfi/dxj = -stiffness * [ (xj/rj^2) * (xi/ri^2) * 1/(sqrt(x0^2/r0^2+x1^2/r1^2+x2^2/r2^2) * sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4))
//                          +  v       * (i==j) / (ri^2*sqrt(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4))
//                          +  v       * (xi/ri^2) * (xj/rj^2) * 1/(rj^2*(x0^2/r0^4+x1^2/r1^4+x2^2/r2^4) ]


template<class DataTypes>
void EllipsoidForceField<DataTypes>::addForce(const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecDeriv &  dataF, const DataVecCoord &  dataX , const DataVecDeriv & dataV )
{

    VecDeriv& f1        = *(dataF.beginEdit());
    const VecCoord& p1  =   dataX.getValue()  ;
    const VecDeriv& v1  =   dataV.getValue()  ;

    sofa::helper::ReadAccessor< sofa::Data< sofa::helper::vector<unsigned > > > indices = d_indices;
    const sofa::helper::vector<CPos> vcenter = this->center.getValue();
    const sofa::helper::vector<Quat> vquat = this->orientations.getValue();
    const sofa::helper::vector<CPos> vr = this->vradius.getValue();
    const Real stiffness = this->stiffness.getValue();
    const Real stiffabs = helper::rabs(stiffness);
    unsigned int nelems = (vr.size() > vcenter.size()) ? vr.size() : vcenter.size();

    //const Real s2 = (stiff < 0 ? - stiff*stiff : stiff*stiff );
    sofa::helper::vector<CPos> inv_r2;
    inv_r2.resize(nelems);

    sofa::helper::vector<Contact>* contacts = this->contacts.beginEdit();
    contacts->clear();
    f1.resize(p1.size());

    if(indices.empty() )
    {
        for (unsigned int i = 0; i < p1.size(); i++)
        {
            CPos bdp;
            Real bnorm2 = -1;
            int be = -1;

            for (unsigned int e = 0; e < nelems; ++e)
            {
                Quat quat = vquat.size() > e ? vquat[e] : Quat();
                CPos dp = quat.inverseRotate(DataTypes::getCPos(p1[i]) - vcenter[e]);
                for (int j = 0; j < N; j++)
                {
                    inv_r2[e][j] = 1 / (vr[e][j] * vr[e][j]);
                }
                Real norm2 = 0;
                for (int j = 0; j < N; j++)
                {
                    norm2 += (dp[j] * dp[j]) * inv_r2[e][j];
                }
                if (be == -1 || norm2 < bnorm2)
                {
                    bnorm2 = norm2;
                    be = e;
                    bdp = dp;
                }
            }

            if ((bnorm2-1)*stiffness<0)
            {
                int e = be;
                Real norm = helper::rsqrt(bnorm2);
                Real v = norm-1;
                DPos grad;
                for (int j = 0; j < N; j++)
                {
                    grad[j] = bdp[j] * inv_r2[e][j];
                }
                Real gnorm2 = grad.norm2();
                Real gnorm = helper::rsqrt(gnorm2);
                //grad /= gnorm; //.normalize();
                Real forceIntensity = -stiffabs*v/gnorm;
                Real dampingIntensity = this->damping.getValue()*helper::rabs(v);
                DPos force = grad*forceIntensity - DataTypes::getDPos(v1[i])*dampingIntensity;
                //f1[i]+=force;
                DataTypes::setDPos(f1[i],DataTypes::getDPos(f1[i])+force);
                Contact c;
                c.index = i;
                Real fact1 = -stiffabs / (norm * gnorm);
                Real fact2 = -stiffabs*v / gnorm;
                Real fact3 = -stiffabs*v / gnorm2;
                for (int ci = 0; ci < N; ++ci)
                {
                    for (int cj = 0; cj < N; ++cj)
                    {
                        c.m[ci][cj] = grad[ci] * grad[cj] * (fact1 + fact3 * inv_r2[e][cj]);
                    }
                    c.m[ci][ci] += fact2 * inv_r2[e][ci];
                }
                contacts->push_back(c);
            }
        }
    }
    else
    {
        for(std::size_t ind=0;ind<indices.size();++ind)
        {
            CPos bdp;
            Real bnorm2 = -1;
            int be = -1;

            for (unsigned int e = 0; e < nelems; ++e)
            {
                Quat quat = vquat.size() > e ? vquat[e] : Quat();
                CPos dp = quat.inverseRotate(DataTypes::getCPos(p1[indices[ind]]) - vcenter[e]);
                for (int j = 0; j < N; j++)
                {
                    inv_r2[e][j] = 1 / (vr[e][j] * vr[e][j]);
                }
                Real norm2 = 0;
                for (int j = 0; j < N; j++)
                {
                    norm2 += (dp[j] * dp[j]) * inv_r2[e][j];
                }
                if (be == -1 || norm2 < bnorm2)
                {
                    bnorm2 = norm2;
                    be = e;
                    bdp = dp;
                }
            }

            if ((bnorm2-1)*stiffness<0)
            {
                int e = be;
                Real norm = helper::rsqrt(bnorm2);
                Real v = norm-1;
                DPos grad;
                for (int j = 0; j < N; j++)
                {
                    grad[j] = bdp[j] * inv_r2[e][j];
                }
                Real gnorm2 = grad.norm2();
                Real gnorm = helper::rsqrt(gnorm2);
                //grad /= gnorm; //.normalize();
                Real forceIntensity = -stiffabs*v/gnorm;
                Real dampingIntensity = this->damping.getValue()*helper::rabs(v);
                DPos force = grad*forceIntensity - DataTypes::getDPos(v1[indices[ind]])*dampingIntensity;
                //f1[i]+=force;
                DataTypes::setDPos(f1[indices[ind]],DataTypes::getDPos(f1[indices[ind]])+force);
                Contact c;
                c.index = indices[ind];
                Real fact1 = -stiffabs / (norm * gnorm);
                Real fact2 = -stiffabs*v / gnorm;
                Real fact3 = -stiffabs*v / gnorm2;
                for (int ci = 0; ci < N; ++ci)
                {
                    for (int cj = 0; cj < N; ++cj)
                    {
                        c.m[ci][cj] = grad[ci] * grad[cj] * (fact1 + fact3 * inv_r2[e][cj]);
                    }
                    c.m[ci][ci] += fact2 * inv_r2[e][ci];
                }
                contacts->push_back(c);
            }
        }
    }
    nbContact = contacts->size();

    this->contacts.endEdit();

    dataF.endEdit();
}

template<class DataTypes>
void EllipsoidForceField<DataTypes>::addDForce(const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv&   datadF , const DataVecDeriv&   datadX )
{
    Real kFactor        = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());
    VecDeriv& df1       = *(datadF.beginEdit());
    const VecDeriv& dx1 =   datadX.getValue()  ;


    df1.resize(dx1.size());
    const sofa::helper::vector<Contact>& contacts = this->contacts.getValue();
    for (unsigned int i=0; i<contacts.size(); i++)
    {
        const Contact& c = contacts[i];
        assert((unsigned)c.index<dx1.size());
        DPos du = DataTypes::getDPos(dx1[c.index]);
        DPos dforce = c.m * du;
        dforce *= kFactor;
        //df1[c.index] += dforce;
        DataTypes::setDPos(df1[c.index], DataTypes::getDPos(df1[c.index]) + dforce);
    }


    datadF.endEdit();
}

template<class DataTypes>
void EllipsoidForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix * mat, SReal kFactor, unsigned int &offset)
{
    const Real fact = (Real)(kFactor);
    for (unsigned int i=0; i<this->contacts.getValue().size(); i++)
    {
        const Contact& c = (this->contacts.getValue())[i];
        unsigned int p = c.index;
        for (int l=0; l<N; ++l)
            for (int k=0; k<N; ++k)
            {
                SReal coef = c.m[l][k] * fact;
                mat->add(offset + p*Deriv::total_size + l, offset + p*Deriv::total_size + k, coef);
            }
    }
}

template<class DataTypes>
void EllipsoidForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
    if (!vparams->displayFlags().getShowForceFields()) return;
    if (!bDrawEnabled.getValue()) return;

    const sofa::helper::vector<CPos> vcenter = this->center.getValue();
    const sofa::helper::vector<Quat> vquat = this->orientations.getValue();
    const sofa::helper::vector<CPos> vr = this->vradius.getValue();

    unsigned int nelems = (vr.size() > vcenter.size()) ? vr.size() : vcenter.size();

    for (unsigned int e = 0; e < nelems; ++e)
    {
        CPos c = vcenter[e];
        Quat quat = vquat.size() > e ? vquat[e] : Quat();
        Real cx = c.size()>0 ? c[0] : 0;
        Real cy = c.size()>1 ? c[1] : 0;
        Real cz = c.size()>2 ? c[2] : 0;
        CPos r = vr[e];
        Real rx = r.size()>0 ? r[0] : 0;
        Real ry = r.size()>1 ? r[1] : 0;
        Real rz = r.size()>2 ? r[2] : 0;

        glEnable(GL_CULL_FACE);
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glColor3f(color.getValue()[0],color.getValue()[1],color.getValue()[2]);

        glPushMatrix();
        glTranslated(cx, cy, cz);

        Real R[4][4];

        quat.inverse().buildRotationMatrix(R);
        helper::gl::glMultMatrix( &(R[0][0]));
        glScaled(rx, ry, (stiffness.getValue() > 0 ? rz : -rz));
        glutSolidSphere(1,32,16);
        glPopMatrix();

        glDisable(GL_CULL_FACE);
        glDisable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
    }
#endif /* SOFA_NO_OPENGL */
}


} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_ELLIPSOIDFORCEFIELD_INL
