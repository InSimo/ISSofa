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
#ifndef SOFA_COMPONENT_FORCEFIELD_CONSTANTFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_CONSTANTFORCEFIELD_INL

#include <SofaBoundaryCondition/ConstantForceField.h>

#include <SofaBaseTopology/TopologySubsetData.inl>

#include <sofa/core/visual/VisualParams.h>


namespace sofa
{

namespace component
{

namespace forcefield
{


template<class DataTypes>
ConstantForceField<DataTypes>::ConstantForceField()
    : m_points(initData(&m_points, "points", "points where the forces are applied"))
    , d_forces(initData(&d_forces, "forces", "applied forces at each point"))
    , d_force(initData(&d_force, "force", "applied force to all points if forces attribute is not specified"))
    , d_totalForce(initData(&d_totalForce, "totalForce", "total force for all points, will be distributed uniformly over points"))
    , d_arrowSizeCoef(initData(&d_arrowSizeCoef,0.0, "arrowSizeCoef", "Size of the drawn arrows (0->no arrows, sign->direction of drawing"))
    , d_indexFromEnd(initData(&d_indexFromEnd,(bool)false,"indexFromEnd", "Concerned DOFs indices are numbered from the end of the MState DOFs vector"))
    , d_handleTopologyChange(initData(&d_handleTopologyChange, true, "handleTopologyChange", "Enable support of topological changes for point indices (disable if another component takes care of this)"))
    , d_startTime(initData(&d_startTime, 0.0, "startTime", "Start of time which the force is activated"))
    , d_endTime(initData(&d_endTime, std::numeric_limits<double>::max(), "endTime", "End of time which the force is activated"))
    , d_loopTime(initData(&d_loopTime, 0.0, "loopTime", "Time at which the time-based activation loops"))
    , m_topology(NULL)
{
}


template<class DataTypes>
void ConstantForceField<DataTypes>::init()
{
    m_topology = this->getContext()->getMeshTopology();

    if (d_handleTopologyChange.getValue())
    {
        // Initialize functions and parameters for topology data and handler
        m_points.createTopologicalEngine(m_topology);
        m_points.registerTopologicalData();
    }

    Inherit::init();
}


template<class DataTypes>
bool ConstantForceField<DataTypes>::isActive() const
{
    const double startTime = this->d_startTime.getValue();
    const double endTime = this->d_endTime.getValue();
    const double loopTime = this->d_loopTime.getValue();
    double time = this->getContext()->getTime();
    if (loopTime != 0)
    {
        time = fmod(time, loopTime);
    }
    return (time >= startTime && time < endTime);
}


template<class DataTypes>
void ConstantForceField<DataTypes>::addForce(const core::MechanicalParams* /*mparams*/, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& /*d_v*/)
{
    if (!isActive())
    {
        return;
    }

    sofa::helper::WriteAccessor< core::objectmodel::Data< VecDeriv > > f = d_f;
    f.resize(d_x.getValue().size());

    Deriv singleForce;
    const Deriv& totalForce = d_totalForce.getValue();
    const Deriv& force = d_force.getValue();

    if (totalForce * totalForce > 0.0)
    {
        if (!m_points.getValue().empty())
        {
            singleForce = totalForce / (Real)m_points.getValue().size();
        }
    }
    else if (force * force > 0.0)
    {
        singleForce = force;
    }

    const VecIndex& indices = m_points.getValue();
    const VecDeriv& forces = d_forces.getValue();
    const Deriv f_end = (forces.empty()? singleForce : forces[forces.size() - 1]);
    unsigned int i = 0;

    if (!d_indexFromEnd.getValue())
    {
        for (; i < forces.size(); i++)
        {
            // if indices are not set, use the force indices
            const unsigned int index = (i >= indices.size() ? i : indices[i]);

            if (index >= f.size())
            {
                serr << "WARNING in " << __FUNCTION__ << " forces dimension exceeds DOFs dimension" << sendl;
            }
            else
            {
                f[index] += forces[i];
            }
        }

        for (; i < indices.size(); i++)
        {
            const unsigned int index = indices[i];
            if (index >= f.size())
            {
                serr << "WARNING in " << __FUNCTION__ << " stored indices exceeds DOFs dimension" << sendl;
            }
            else
            {
                f[index] += f_end;
            }
        }
    }
    else
    {
        for (; i < forces.size(); i++)
        {
            // if indices are not set, use the force indices
            const int index = f.size() - indices[i] - 1;
            if (index < 0 || index >= (int)f.size())
            {
                serr << "WARNING in " << __FUNCTION__ << " forces dimension exceeds DOFs dimension" << sendl;
            }
            else
            {
                f[index] += forces[i];
            }
        }

        for (; i < indices.size(); i++)
        {
            const int index = f.size() - indices[i] - 1;
            if (index < 0 || index >= (int)f.size())
            {
                serr << "WARNING in " << __FUNCTION__ << " index exceeds DOFs dimension" << sendl;
            }
            else
            {
                f[index] += f_end;
            }
        }
    }
}


template<class DataTypes>
void ConstantForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix * /* mat */, SReal /* k */, unsigned int & /* offset */)
{
}


template <class DataTypes>
double ConstantForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams* , const DataVecCoord& d_x) const
{
    if (!isActive())
    {
        return 0.0;
    }

    double e = 0;
   
    const VecIndex& indices = m_points.getValue();
    const VecDeriv& forces = d_forces.getValue();
    const VecCoord& x = d_x.getValue();
    const Deriv f_end = (forces.empty()? d_force.getValue() : forces[forces.size()-1]);
    unsigned int i = 0;

    if (!d_indexFromEnd.getValue())
    {
        for (; i<forces.size(); i++)
        {
            e -= forces[i] * x[indices[i]];
        }
        for (; i<indices.size(); i++)
        {
            e -= f_end * x[indices[i]];
        }
    }
    else
    {
        for (; i < forces.size(); i++)
        {
            e -= forces[i] * x[x.size() - indices[i] -1];
        }
        for (; i < indices.size(); i++)
        {
            e -= f_end * x[x.size() - indices[i] -1];
        }
    }
    
    return e;
}


template <class DataTypes>
void ConstantForceField<DataTypes>::setForce(unsigned i, const Deriv& force)
{
    VecIndex& indices = *m_points.beginEdit();
    VecDeriv& f = *d_forces.beginEdit();
    indices.push_back(i);
    f.push_back(force);
    m_points.endEdit();
    d_forces.endEdit();
}


template<class DataTypes>
void ConstantForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!isActive())
    {
        return;
    }

    double aSC = d_arrowSizeCoef.getValue();

    if ((!vparams->displayFlags().getShowForceFields() && (aSC == 0)) || (aSC < 0.0))
    {
        return;
    }

    const Deriv &totalForce = d_totalForce.getValue();
    const Deriv &force = d_force.getValue();
    Deriv singleForce;

    if (totalForce * totalForce > 0.0)
    {
        for (unsigned comp = 0; comp < totalForce.size(); comp++)
        {
            singleForce[comp] = (totalForce[comp]) / (Real(m_points.getValue().size()));
        }
    }
    else if (force * force > 0.0)
    {
        singleForce = force;
    }

    const VecIndex& indices = m_points.getValue();
    const VecDeriv& f = d_forces.getValue();
    const Deriv f_end = (f.empty()? singleForce : f[f.size()-1]);
    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();

    {
        if( fabs(aSC)<1.0e-10 )
        {
            std::vector<defaulttype::Vector3> points;
            for (unsigned int i=0; i<indices.size(); i++)
            {
                Real xx,xy,xz,fx,fy,fz;

                if (!d_indexFromEnd.getValue())
                {
                    if (indices[i] < x.size())
                    {
                        DataTypes::get(xx, xy, xz, x[indices[i]]);
                    }
                }
                else
                {
                    if ((x.size() - indices[i] - 1) < x.size() && (x.size() - indices[i] - 1) >= 0)
                    {
                        DataTypes::get(xx, xy, xz, x[x.size() - indices[i] - 1]);
                    }
                }

                DataTypes::get(fx,fy,fz,(i<f.size())? f[i] : f_end);

                points.push_back(defaulttype::Vector3(xx, xy, xz ));
                points.push_back(defaulttype::Vector3(xx+fx, xy+fy, xz+fz ));
            }
            vparams->drawTool()->drawLines(points, 2, defaulttype::Vec<4,float>(0,1,0,1));
        }
        else
        {
            for (unsigned int i=0; i<indices.size(); i++)
            {
                Real xx,xy,xz,fx,fy,fz;

                if (!d_indexFromEnd.getValue())
                {
                    if (indices[i] < x.size())
                    {
                        DataTypes::get(xx, xy, xz, x[indices[i]]);
                    }
                }
                else
                {
                    if ((x.size() - indices[i] - 1) < x.size() && (x.size() - indices[i] - 1) >= 0)
                    {
                        DataTypes::get(xx, xy, xz, x[x.size() - indices[i] - 1]);
                    }
                }

                DataTypes::get(fx,fy,fz,(i<f.size())? f[i] : f_end);

                defaulttype::Vector3 p1( xx, xy, xz);
                defaulttype::Vector3 p2( aSC*fx+xx, aSC*fy+xy, aSC*fz+xz );

                float norm = (float)(p2-p1).norm();

                if( aSC > 0)
                {
                    //helper::gl::drawArrow(p1,p2, norm/20.0);
                    vparams->drawTool()->drawArrow(p1,p2, norm/20.0f, defaulttype::Vec<4,float>(1.0f,0.4f,0.4f,1.0f));
                }
                else
                {
                    //helper::gl::drawArrow(p2,p1, norm/20.0);
                    vparams->drawTool()->drawArrow(p2,p1, norm/20.0f, defaulttype::Vec<4,float>(1.0f,0.4f,0.4f,1.0f));
                }
            }
        }
    }
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_CONSTANTFORCEFIELD_INL
