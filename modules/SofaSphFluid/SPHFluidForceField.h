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
#ifndef SOFA_COMPONENT_FORCEFIELD_SPHFLUIDFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_SPHFLUIDFORCEFIELD_H

#include <sofa/helper/system/config.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <SofaSphFluid/SpatialGridContainer.h>
#include <SofaSphFluid/SPHKernel.h>
#include <sofa/helper/rmath.h>
#include <vector>
#include <math.h>

#include <sofa/SofaAdvanced.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

/// This class can be overridden if needed for additionnal storage within template specializations.
template<class DataTypes>
class SPHFluidForceFieldInternalData
{
public:
};

template<class DataTypes>
class SPHFluidForceField : public sofa::core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SPHFluidForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef sofa::core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename Coord::value_type Real;

    typedef core::objectmodel::Data<VecDeriv>    DataVecDeriv;
    typedef core::objectmodel::Data<VecCoord>    DataVecCoord;

public:
    Data< Real > particleRadius;
    Data< Real > particleMass;
    Data< Real > pressureStiffness; ///< 100 - 1000 m2/s2
    Data< Real > density0; ///< 1000 kg/m3 for water
    Data< Real > viscosity;
    Data< Real > surfaceTension;
    //Data< int  > pressureExponent;
    Data< int > kernelType;
    Data< int > pressureType;
    Data< int > viscosityType;
    Data< int > surfaceTensionType;

protected:
    struct Particle
    {
        Real density;
        Real pressure;
        Deriv normal;
        Real curvature;
        sofa::helper::vector< std::pair<int,Real> > neighbors; ///< indice + r/h
#ifdef SOFA_DEBUG_SPATIALGRIDCONTAINER
        sofa::helper::vector< std::pair<int,Real> > neighbors2; ///< indice + r/h
#endif
    };

    Real lastTime;
    sofa::helper::vector<Particle> particles;

    typedef sofa::component::container::SpatialGridContainer<DataTypes> Grid;

    Grid* grid;

    SPHFluidForceFieldInternalData<DataTypes> data;
    friend class SPHFluidForceFieldInternalData<DataTypes>;

public:
    /// this method is called by the SpatialGrid when w connection between two particles is detected
    void addNeighbor(int i1, int i2, Real r2, Real h2)
    {
        Real r_h = (Real)sqrt(r2/h2);
        if (i1<i2)
            particles[i1].neighbors.push_back(std::make_pair(i2,r_h));
        else
            particles[i2].neighbors.push_back(std::make_pair(i1,r_h));
    }

protected:

    /// Color Smoothing Kernel: same as Density
    Real  constWc(Real h) const
    {
        return (Real)(315 / (64*R_PI*h*h*h));
    }
    Real  Wc(Real r_h, Real C)
    {
        Real a = (1-r_h*r_h);
        return  C*a*a*a;
    }
    Real  constGradWc(Real h) const
    {
        return -6*constWc(h)/h;
    }
    Deriv gradWc(const Deriv& d, Real r_h, Real C)
    {
        Real a = (1-r_h*r_h);
        return d*(C*a*a)*r_h;
    }
    Real  constLaplacianWc(Real h) const
    {
        return -6*constWc(h)/(h*h);
    }
    Real  laplacianWc(Real r_h, Real C)
    {
        Real r2_h2 = r_h*r_h;
        return C*((1-r2_h2)*(1-5*r2_h2));
    }


    struct DForce
    {
        unsigned int a,b;
        Real df;
    };

    sofa::helper::vector<DForce> dforces;


    SPHFluidForceField();
public:
    Real getParticleRadius() const { return particleRadius.getValue(); }
    void setParticleRadius(Real v) { particleRadius.setValue(v);    }
    Real getParticleMass() const { return particleMass.getValue(); }
    void setParticleMass(Real v) { particleMass.setValue(v);    }
    Real getPressureStiffness() const { return pressureStiffness.getValue(); }
    void setPressureStiffness(Real v) { pressureStiffness.setValue(v);    }
    Real getDensity0() const { return density0.getValue(); }
    void setDensity0(Real v) { density0.setValue(v);    }
    Real getViscosity() const { return viscosity.getValue(); }
    void setViscosity(Real v) { viscosity.setValue(v);    }
    Real getSurfaceTension() const { return surfaceTension.getValue(); }
    void setSurfaceTension(Real v) { surfaceTension.setValue(v);    }

    Real getParticleField(int i, Real r2_h2)
    {
        Real a = 1-r2_h2;
        return (a*a*a)/particles[i].density;
    }

    Real getParticleFieldConstant(Real h)
    {
        return constWc(h)*particleMass.getValue();
    }

    virtual void init();

    virtual void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v);
    virtual void addDForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& d_df, const DataVecDeriv& d_dx);
    double getPotentialEnergy(const core::MechanicalParams* /* mparams */ /* PARAMS FIRST */, const DataVecCoord& /* d_x */) const ;


    void draw(const core::visual::VisualParams* vparams);

protected:
    void computeNeighbors(const core::MechanicalParams* mparams /* PARAMS FIRST */, const DataVecCoord& d_x, const DataVecDeriv& d_v);
    template<class Kd, class Kp, class Kv, class Kc>
    void computeForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v);
};

#ifndef SOFA_FLOAT
using sofa::defaulttype::Vec3dTypes;
using sofa::defaulttype::Vec2dTypes;
#endif

#ifndef SOFA_DOUBLE
using sofa::defaulttype::Vec2fTypes;
using sofa::defaulttype::Vec3fTypes;
#endif

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_SPHFLUIDFORCEFIELD_CPP)

#ifndef SOFA_FLOAT
extern template class SOFA_SPH_FLUID_API SPHFluidForceField<Vec3dTypes>;
extern template class SOFA_SPH_FLUID_API SPHFluidForceField<Vec2dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_SPH_FLUID_API SPHFluidForceField<Vec3fTypes>;
extern template class SOFA_SPH_FLUID_API SPHFluidForceField<Vec2fTypes>;
#endif

#endif // defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_SPHFLUIDFORCEFIELD_CPP)

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_SPHFLUIDFORCEFIELD_H
