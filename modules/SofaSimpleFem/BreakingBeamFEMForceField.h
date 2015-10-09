#ifndef SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_H

#include "BeamFEMForceField.inl"

namespace sofa
{

namespace component
{

namespace forcefield
{

template<class DataTypes>
class BreakingBeamFEMForceField : public BeamFEMForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(BreakingBeamFEMForceField,DataTypes), SOFA_TEMPLATE(BeamFEMForceField,DataTypes));

    void draw(const core::visual::VisualParams* vparams);

protected:

    BreakingBeamFEMForceField();
    BreakingBeamFEMForceField(Real poissonRatio, Real youngModulus, Real radius, Real radiusInner);
    virtual ~BreakingBeamFEMForceField();

    Data<Real>          _breakableBeamThreshold;
    Data<unsigned int>  _breakableBeamIndex;

};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_SIMPLE_FEM_API BreakingBeamFEMForceField<defaulttype::Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_SIMPLE_FEM_API BreakingBeamFEMForceField<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_H
