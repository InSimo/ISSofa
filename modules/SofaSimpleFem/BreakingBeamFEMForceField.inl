#ifndef SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_INL

#include "BreakingBeamFEMForceField.h"

namespace sofa
{

namespace component
{

namespace forcefield
{


template<class DataTypes>
BreakingBeamFEMForceField<DataTypes>::BreakingBeamFEMForceField()
    : Inherit1()
    , _breakableBeamThreshold(initData(&_breakableBeamThreshold,(Real)0.0,"breakableBeamThreshold","Percentage of beams extension over which one of the beam (given by breakableBeamIndex) will break."))
    , _breakableBeamIndex(initData(&_breakableBeamIndex,0,"breakableBeamIndex","Index of the beam that can be broken when breakableBeamThreshold is reached."))

{

}

template<class DataTypes>
BreakingBeamFEMForceField<DataTypes>::BreakingBeamFEMForceField(Real poissonRatio, Real youngModulus, Real radius, Real radiusInner)
    : Inherit1(poissonRatio, youngModulus, radius, radiusInner)
    , _breakableBeamThreshold(initData(&_breakableBeamThreshold,(Real)0.0,"breakableBeamThreshold","Percentage of beams extension over which one of the beam (given by breakableBeamIndex) will break."))
    , _breakableBeamIndex(initData(&_breakableBeamIndex,0,"breakableBeamIndex","Index of the beam that can be broken when breakableBeamThreshold is reached."))
{

}

template<class DataTypes>
BreakingBeamFEMForceField<DataTypes>::~BreakingBeamFEMForceField()
{

}

template<class DataTypes>
void BreakingBeamFEMForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    Inherit1::draw(vparams);

    const Real percentageThreshold = _breakableBeamThreshold.getValue();
    if ( percentageThreshold > Real(0.0) )
    {
        const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();
        const VecCoord& x0 = this->mstate->read(core::ConstVecCoordId::restPosition())->getValue();

        unsigned int lastInd = x.size() - 1;
        Real extension0 = ( x0[lastInd].getCenter() - x0[0].getCenter() ).norm();
        Real extension = ( x[lastInd].getCenter() - x[0].getCenter() ).norm();

        Real percentageExtension = ( extension0 != Real(0.0) ) ? extension / extension0 * 100 : Real (0.0);
        if ( percentageExtension > percentageThreshold )
        {
            const unsigned int index = _breakableBeamIndex.getValue();
            std::vector< defaulttype::Vector3 > points[1];
            points[0].push_back( x[index].getCenter() );
            vparams->drawTool()->drawSpheres(points[0], 1.0f, defaulttype::Vec<4,float>(1,0,0,1));
        }
    }
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_INL
