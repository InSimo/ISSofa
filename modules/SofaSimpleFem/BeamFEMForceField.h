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
#ifndef SOFA_COMPONENT_FORCEFIELD_BEAMFEMFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_BEAMFEMFORCEFIELD_H

#include <sofa/SofaCommon.h>
#include <sofa/core/behavior/ForceField.h>
#include <SofaBaseTopology/TopologyData.h>

#ifdef SOFA_HAVE_EIGEN2
#include <SofaEigen2Solver/EigenSparseMatrix.h>
#endif


namespace sofa
{

namespace component
{

namespace container
{
class StiffnessContainer;
class PoissonContainer;
} // namespace container

namespace forcefield
{

/** Compute Finite Element forces based on 6D beam elements.
*/
template<class DataTypes>
class BeamFEMForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(BeamFEMForceField,DataTypes), SOFA_TEMPLATE(core::behavior::ForceField,DataTypes));

    typedef typename DataTypes::Real        Real        ;
    typedef typename DataTypes::Coord       Coord       ;
    typedef typename DataTypes::Deriv       Deriv       ;
    typedef typename DataTypes::VecCoord    VecCoord    ;
    typedef typename DataTypes::VecDeriv    VecDeriv    ;
    typedef typename DataTypes::VecReal     VecReal     ;
    typedef Data<VecCoord>                  DataVecCoord;
    typedef Data<VecDeriv>                  DataVecDeriv;

    typedef unsigned int Index;
    typedef topology::Edge Element;
    typedef sofa::helper::vector<topology::Edge> VecElement;
    typedef helper::vector<unsigned int> VecIndex;
    typedef defaulttype::Vec<3, Real> Vec3;

protected:

    typedef defaulttype::Vec<12, Real> Displacement;        ///< the displacement vector
    typedef defaulttype::Mat<3, 3, Real> Transformation; ///< matrix for rigid transformations like rotations
    typedef defaulttype::Mat<12, 12, Real> StiffnessMatrix;

    struct BeamInfo
    {
        double _E0,_E; //Young
        double _nu;//Poisson
        double _L; //length
        double _r; //radius of the section
        double _rInner; //inner radius of the section if beam is hollow
        double _G; //shear modulus
        double _Iy;
        double _Iz; //Iz is the cross-section moment of inertia (assuming mass ratio = 1) about the z axis;
        double _J;  //Polar moment of inertia (J = Iy + Iz)
        double _A; // A is the cross-sectional area;
        double _Asy; //_Asy is the y-direction effective shear area =  10/9 (for solid circular section) or 0 for a non-Timoshenko beam
        double _Asz; //_Asz is the z-direction effective shear area;
        StiffnessMatrix _k_loc; //k_loc is the stiffness in the local frame

        defaulttype::Quat quat;

        void init(double E, double L, double nu, double r, double rInner);
        /// Output stream
        inline friend std::ostream& operator<< ( std::ostream& os, const BeamInfo& bi )
        {
            os	<< bi._E0 << " "
                << bi._E << " "
                << bi._nu << " "
                << bi._L << " "
                << bi._r << " "
                << bi._rInner << " "
                << bi._G << " "
                << bi._Iy << " "
                << bi._Iz << " "
                << bi._J << " "
                << bi._A << " "
                << bi._Asy << " "
                << bi._Asz << " "
                << bi._k_loc;
            return os;
        }

        /// Input stream
        inline friend std::istream& operator>> ( std::istream& in, BeamInfo& bi )
        {
            in	>> bi._E0
                >> bi._E
                >> bi._nu
                >> bi._L
                >> bi._r
                >> bi._rInner
                >> bi._G
                >> bi._Iy
                >> bi._Iz
                >> bi._J
                >> bi._A
                >> bi._Asy
                >> bi._Asz
                >> bi._k_loc;
            return in;
        }
    };

    topology::EdgeData< sofa::helper::vector<BeamInfo> > beamsData;
#ifdef SOFA_HAVE_EIGEN2
    linearsolver::EigenBaseSparseMatrix<typename DataTypes::Real> matS;
#endif

    class BeamFFEdgeHandler : public topology::TopologyDataHandler<topology::Edge,sofa::helper::vector<BeamInfo> >
    {
    public:
        typedef typename BeamFEMForceField<DataTypes>::BeamInfo BeamInfo;
        BeamFFEdgeHandler(BeamFEMForceField<DataTypes>* ff, topology::EdgeData<sofa::helper::vector<BeamInfo> >* data)
            :topology::TopologyDataHandler<topology::Edge,sofa::helper::vector<BeamInfo> >(data),ff(ff) {}

        void applyCreateFunction(unsigned int edgeIndex, BeamInfo&,
                                 const topology::Edge& e,
                                 const sofa::helper::vector<unsigned int> &,
                                 const sofa::helper::vector< double > &);

    protected:
        BeamFEMForceField<DataTypes>* ff;

    };

    const VecElement *_indexedElements;
    Data<VecReal> _poissonRatio;
    Data<VecReal> _youngModulus;
    Data<VecReal> _radius;
    Data<VecReal> _radiusInner;
    Data< VecIndex > _list_segment;
    bool _partial_list_segment;

    Data<bool> _applyRigidTransFirstBeam;
    Data<Coord> _rigidTransFirstBeam;

    Data<Real> _rigidY;
    Data<Real> _rigidZ;

    Coord m_rigidTsf;

#ifdef SOFA_HAVE_EIGEN2
    double lastUpdatedStep;
#endif

    container::StiffnessContainer* stiffnessContainer;
//	container::LengthContainer* lengthContainer;
    container::PoissonContainer* poissonContainer;
//	container::RadiusContainer* radiusContainer;

    defaulttype::Quat& beamQuat(int i)
    {
        helper::vector<BeamInfo>& bd = *(beamsData.beginEdit());
        return bd[i].quat;
    }
    sofa::core::topology::BaseMeshTopology* _topology;
    BeamFFEdgeHandler* edgeHandler;


    BeamFEMForceField();
    BeamFEMForceField(Real poissonRatio, Real youngModulus, Real radius, Real radiusInner);
    virtual ~BeamFEMForceField();
public:
    virtual void init();
    virtual void bwdInit();
    virtual void reinit();
    virtual void reinitBeam(unsigned int i);

    virtual void addForce(const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecDeriv &  dataF, const DataVecCoord &  dataX , const DataVecDeriv & dataV );
    virtual void addDForce(const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecDeriv&   datadF , const DataVecDeriv&   datadX );
    virtual void addKToMatrix(const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix );

    virtual double getPotentialEnergy(const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, const DataVecCoord&  /* x */) const
    {
        serr << "Get potentialEnergy not implemented" << sendl;
        return 0.0;
    }

    void draw(const core::visual::VisualParams* vparams);

    void setBeam(unsigned int i, double E, double L, double nu, double r, double rInner);
    void initBeams(unsigned int size);

protected:

    void drawElement(int i, std::vector< defaulttype::Vector3 >* points, const VecCoord& x);

    Real peudo_determinant_for_coef ( const defaulttype::Mat<2, 3, Real>&  M );

    void computeStiffness(int i, Index a, Index b);

    ////////////// large displacements method
    vector<Transformation> _nodeRotations;
    void initLarge(int i, Index a, Index b);
    void accumulateForceLarge( VecDeriv& f, const VecCoord& x, int i, Index a, Index b);
    void applyStiffnessLarge( VecDeriv& f, const VecDeriv& x, int i, Index a, Index b, double fact=1.0);
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_BEAMFEMFORCEFIELD_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_SIMPLE_FEM_API BeamFEMForceField<defaulttype::Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_SIMPLE_FEM_API BeamFEMForceField<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_BEAMFEMFORCEFIELD_H
