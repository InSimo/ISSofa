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
#ifndef SOFA_COMPONENT_INTERACTIONFORCEFIELD_MESHSPRINGFORCEFIELD_INL
#define SOFA_COMPONENT_INTERACTIONFORCEFIELD_MESHSPRINGFORCEFIELD_INL

#include <SofaDeformable/MeshSpringForceField.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaDeformable/StiffSpringForceField.inl>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <iostream>



namespace sofa
{

namespace component
{

namespace interactionforcefield
{

template <class DataTypes>
MeshSpringForceField<DataTypes>::~MeshSpringForceField()
{
}

template <class DataTypes>
double MeshSpringForceField<DataTypes>::getPotentialEnergy() const
{
    serr<<"MeshSpringForceField::getPotentialEnergy-not-implemented !!!"<<sendl;
    return 0;
}

template<class DataTypes>
void MeshSpringForceField<DataTypes>::addSpring(std::set<std::pair<int,int> >& sset, int m1, int m2, Real stiffness, Real damping)
{
    if (localRange.getValue()[0] >= 0)
    {
        if (m1 < localRange.getValue()[0] || m2 < localRange.getValue()[0]) return;
    }
    if (localRange.getValue()[1] >= 0)
    {
        if (m1 > localRange.getValue()[1] && m2 > localRange.getValue()[1]) return;
    }

    if (m1<m2)
    {
        if (sset.count(std::make_pair(m1,m2))>0) return;
        sset.insert(std::make_pair(m1,m2));
    }
    else
    {
        if (sset.count(std::make_pair(m2,m1))>0) return;
        sset.insert(std::make_pair(m2,m1));
    }
    
    helper::ReadAccessor< sofa::Data<VecCoord> > X01 = this->mstate1->readRestPositions();
    helper::ReadAccessor< sofa::Data<VecCoord> > X02 = this->mstate2->readRestPositions();
    const Coord& scale3d = d_scale3d.getValue();

    Deriv v = X02[m2] - X01[m1];

    for (unsigned int i=0; i<scale3d.size(); ++i)
    {
        v[i] *= scale3d[i];
    }

    Real l = v.norm() * defaultTension.getValue();
    
    this->springs.beginEdit()->push_back(typename SpringForceField<DataTypes>::Spring(m1,m2,stiffness/l, damping/l, l, noCompression.getValue()));
    this->springs.endEdit();

    initialStiffnessVec.push_back(stiffness);
    initialDampingVec.push_back(damping);
}

template<class DataTypes>
void MeshSpringForceField<DataTypes>::reinit()
{
    initialStiffnessVec.clear();
    initialDampingVec.clear();

    this->StiffSpringForceField<DataTypes>::clear();
    if(!(this->mstate1) || !(this->mstate2))
        this->mstate2 = this->mstate1 = sofa::core::behavior::MechanicalState<DataTypes>::DynamicCast(this->getContext()->getMechanicalState());

    if (this->mstate1==this->mstate2)
    {
        sofa::core::topology::BaseMeshTopology* topology = this->getContext()->getMeshTopology();

        if (topology != NULL)
        {
            std::set< std::pair<int,int> > sset;
            int n;
            Real s, d;
            if (this->linesStiffness.getValue() != 0.0 || this->linesDamping.getValue() != 0.0)
            {
                s = this->linesStiffness.getValue();
                d = this->linesDamping.getValue();
                n = topology->getNbLines();
                for (int i=0; i<n; ++i)
                {
                    sofa::core::topology::BaseMeshTopology::Line e = topology->getLine(i);
                    this->addSpring(sset, e[0], e[1], s, d);
                }
            }
            if (this->trianglesStiffness.getValue() != 0.0 || this->trianglesDamping.getValue() != 0.0)
            {
                s = this->trianglesStiffness.getValue();
                d = this->trianglesDamping.getValue();
                n = topology->getNbTriangles();
                for (int i=0; i<n; ++i)
                {
                    sofa::core::topology::BaseMeshTopology::Triangle e = topology->getTriangle(i);
                    this->addSpring(sset, e[0], e[1], s, d);
                    this->addSpring(sset, e[0], e[2], s, d);
                    this->addSpring(sset, e[1], e[2], s, d);
                }
            }
            if (this->quadsStiffness.getValue() != 0.0 || this->quadsDamping.getValue() != 0.0)
            {
                s = this->quadsStiffness.getValue();
                d = this->quadsDamping.getValue();
                n = topology->getNbQuads();
                for (int i=0; i<n; ++i)
                {
                    sofa::core::topology::BaseMeshTopology::Quad e = topology->getQuad(i);
                    this->addSpring(sset, e[0], e[1], s, d);
                    this->addSpring(sset, e[0], e[2], s, d);
                    this->addSpring(sset, e[0], e[3], s, d);
                    this->addSpring(sset, e[1], e[2], s, d);
                    this->addSpring(sset, e[1], e[3], s, d);
                    this->addSpring(sset, e[2], e[3], s, d);
                }
            }
            if (this->tetrahedraStiffness.getValue() != 0.0 || this->tetrahedraDamping.getValue() != 0.0)
            {
                s = this->tetrahedraStiffness.getValue();
                d = this->tetrahedraDamping.getValue();
                n = topology->getNbTetrahedra();
                for (int i=0; i<n; ++i)
                {
                    sofa::core::topology::BaseMeshTopology::Tetra e = topology->getTetrahedron(i);
                    this->addSpring(sset, e[0], e[1], s, d);
                    this->addSpring(sset, e[0], e[2], s, d);
                    this->addSpring(sset, e[0], e[3], s, d);
                    this->addSpring(sset, e[1], e[2], s, d);
                    this->addSpring(sset, e[1], e[3], s, d);
                    this->addSpring(sset, e[2], e[3], s, d);
                }
            }

            if (this->cubesStiffness.getValue() != 0.0 || this->cubesDamping.getValue() != 0.0)
            {
                s = this->cubesStiffness.getValue();
                d = this->cubesDamping.getValue();
#ifdef SOFA_NEW_HEXA
                n = topology->getNbHexahedra();
                for (int i=0; i<n; ++i)
                {
                    sofa::core::topology::BaseMeshTopology::Hexa e = topology->getHexahedron(i);
#else
                n = topology->getNbCubes();
                for (int i=0; i<n; ++i)
                {
                    sofa::core::topology::BaseMeshTopology::Cube e = topology->getCube(i);
#endif
                    for (int i=0; i<8; i++)
                        for (int j=i+1; j<8; j++)
                        {
                            this->addSpring(sset, e[i], e[j], s, d);
                        }
                }
            }
        }
    }
    this->StiffSpringForceField<DataTypes>::init();
}

template<class DataTypes>
void MeshSpringForceField<DataTypes>::init()
{
    prevDefaultTension = this->defaultTension.getValue();

    reinit();
}

template <class DataTypes>
void MeshSpringForceField<DataTypes>::reset()
{
	reinit();
}

template<class DataTypes>
void MeshSpringForceField<DataTypes>::handleEvent(sofa::core::objectmodel::Event* e)
{
    if (simulation::AnimateBeginEvent::DynamicCast(e) != 0)
    {
      if (prevDefaultTension != this->defaultTension.getValue())
      {
        //std::cout << "A new defaultTension value has been set : "<< this->defaultTension.getValue() << " -> reinint() is called in MeshSpringForceField" <<  std::endl;
        this->reinit();
        prevDefaultTension = this->defaultTension.getValue();
      }
      const Coord& scale3d = d_scale3d.getValue();
      if (prevScale3d != scale3d)
      {
          helper::ReadAccessor< sofa::Data<VecCoord> > X01 = this->mstate1->readRestPositions();
          helper::ReadAccessor< sofa::Data<VecCoord> > X02 = this->mstate2->readRestPositions();

          typedef typename Inherit1::Spring  Spring;
          sofa::helper::vector<Spring >& ss = *this->springs.beginEdit();

          for (unsigned int i=0; i<ss.size(); ++i)
          {
              Spring& s = ss[i];

              Deriv v = X02[s.m2] - X01[s.m1];

              for (unsigned int j=0; j<scale3d.size(); ++j)
              {
                  v[j] *= scale3d[j];
              }
              Real l = v.norm() * defaultTension.getValue();
              s.initpos = l;
              s.ks = initialStiffnessVec[i]/l;
              s.kd = initialDampingVec[i]/l;
          }
          this->springs.endEdit();
      }
    }
}

template<class DataTypes>
void MeshSpringForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if(d_drawEnabled.getValue() )
    {
        typedef typename Inherit1::Spring  Spring;
        sofa::helper::vector<Spring >& ss = *this->springs.beginEdit();
        
        const VecCoord& p1 = this->mstate1->read(core::ConstVecCoordId::position())->getValue();
        const VecCoord& p2 = this->mstate2->read(core::ConstVecCoordId::position())->getValue();
        
        Real minElongation = std::numeric_limits<Real>::max();
        Real maxElongation = 0.;
        for (unsigned int i=0; i<ss.size(); ++i)
        {
            Spring& s = ss[i];
            Deriv v = p1[s.m1] - p2[s.m2];
            Real elongation = (s.initpos - v.norm()) / s.initpos;
            maxElongation = std::max(maxElongation, elongation);
            minElongation = std::min(minElongation, elongation);
        }
        
        const Real minElongationRange = d_drawMinElongationRange.getValue();
        const Real maxElongationRange = d_drawMaxElongationRange.getValue();
        Real range = std::min(std::max(maxElongation, std::abs(minElongation)), maxElongationRange) - minElongationRange;
        range = (range < 0.f) ? 1.f : range;
        const Real drawSpringSize = d_drawSpringSize.getValue();

        for (unsigned int i=0; i<ss.size(); ++i)
        {
            Spring& s = ss[i];
            const Coord pa[2] = {p1[s.m1], p2[s.m2]};
            sofa::helper::vector<sofa::defaulttype::Vector3> points;
            points.assign(pa, pa + 2);
            Deriv v = pa[0] - pa[1];
            Real elongation = (s.initpos - v.norm()) / s.initpos;
            float R = 0.;
            float G = 0.;
            float B = 1.;
            if(elongation < 0.)
            {
                elongation = std::abs(elongation);
                B = (float)((range-std::min(elongation - minElongationRange, range))/range);
                B = (B < 0.f) ? 0.f : B;
                R = 1.f - B;
            }
            else
            {
                B = (float)((range-std::min(elongation - minElongationRange, range))/range);
                B = (B < 0.f) ? 0.f : B;
                G = 1.f - B;
            }

            vparams->drawTool()->drawLines(points, (float)drawSpringSize, sofa::defaulttype::Vec4f(R, G, B, 1.f));
        }
        this->springs.endEdit();
    }
}

} // namespace interactionforcefield

} // namespace component

} // namespace sofa

#endif  /* SOFA_COMPONENT_INTERACTIONFORCEFIELD_MESHSPRINGFORCEFIELD_INL */
