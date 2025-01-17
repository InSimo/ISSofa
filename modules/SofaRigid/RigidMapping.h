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
#ifndef SOFA_COMPONENT_MAPPING_RIGIDMAPPING_H
#define SOFA_COMPONENT_MAPPING_RIGIDMAPPING_H

#include <sofa/core/Mapping.h>
#include <sofa/core/objectmodel/DataFileName.h>

#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.h>
#ifdef SOFA_HAVE_EIGEN2
#include <SofaEigen2Solver/EigenSparseMatrix.h>
#endif
#include <sofa/SofaCommon.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <vector>
#include <memory>

namespace sofa
{

namespace component
{

namespace mapping
{

/// This class can be overridden if needed for additionnal storage within template specializations.
template<class InDataTypes, class OutDataTypes>
class RigidMappingInternalData
{
public:
};

template <class TIn, class TOut>
class RigidMapping : public core::Mapping<TIn, TOut>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(RigidMapping,TIn,TOut), SOFA_TEMPLATE2(core::Mapping,TIn,TOut));

    typedef core::Mapping<TIn, TOut> Inherit;
    typedef TIn In;
    typedef TOut Out;
    typedef Out OutDataTypes;
    typedef typename Out::VecCoord VecCoord;
    typedef typename Out::VecDeriv VecDeriv;
    typedef typename Out::Coord Coord;
    typedef typename Out::Deriv Deriv;
    typedef typename Out::MatrixDeriv OutMatrixDeriv;
    typedef typename In::Real InReal;
    typedef typename In::Deriv InDeriv;
    typedef typename InDeriv::Pos DPos;
    typedef typename InDeriv::Rot DRot;
    typedef typename In::VecCoord InVecCoord;
    typedef typename In::VecDeriv InVecDeriv;
    typedef typename In::MatrixDeriv InMatrixDeriv;
    typedef typename Coord::value_type Real;
    enum
    {
        N = OutDataTypes::spatial_dimensions
    };
    enum
    {
        NIn = sofa::defaulttype::DataTypeInfo<InDeriv>::FinalSize
    };
    enum
    {
        NOut = sofa::defaulttype::DataTypeInfo<Deriv>::FinalSize
    };
    typedef defaulttype::Mat<N, N, Real> Mat;
    typedef defaulttype::Vec<N, Real> Vector;
    typedef defaulttype::Mat<NOut, NIn, Real> MBloc;
    typedef sofa::defaulttype::CompressedRowSparseMatrixMechanical<MBloc> MatrixType;
#ifdef SOFA_HAVE_EIGEN2
    typedef linearsolver::EigenSparseMatrix<In,Out>    SparseMatrixEigen;
#endif


    Data<VecCoord> points;    ///< mapped points in local coordinates
    VecCoord rotatedPoints;   ///< vectors from frame origin to mapped points, projected to world coordinates
    RigidMappingInternalData<In, Out> data;
    Data<unsigned int> index;
    sofa::core::objectmodel::DataFileName fileRigidMapping;
    Data<bool> useX0;
    Data<bool> indexFromEnd;
    Data< bool > d_useGeometricStiffness;

    /**
     * pointsPerRigid:
     *  - no value specified : simple rigid mapping, all points attached to the same frame (index=0)
     *  - one value specified : same number of points for each frame
     *  - n values are specified : heterogeneous distribution of points per frame
     */
    Data<sofa::helper::vector<unsigned int> > pointsPerFrame;
    Data<bool> globalToLocalCoords;

protected:
    RigidMapping();
    virtual ~RigidMapping() {}
public:
    int addPoint(const Coord& c);
    int addPoint(const Coord& c, int indexFrom);

    virtual void init();

    /// Compute the local coordinates based on the current output coordinates.
    virtual void reinit();

    virtual void apply(const core::MechanicalParams *mparams /* PARAMS FIRST */, Data<VecCoord>& out, const Data<InVecCoord>& in);

    virtual void applyJ(const core::MechanicalParams *mparams /* PARAMS FIRST */, Data<VecDeriv>& out, const Data<InVecDeriv>& in);

    virtual void applyJT(const core::MechanicalParams *mparams /* PARAMS FIRST */, Data<InVecDeriv>& out, const Data<VecDeriv>& in);

    virtual void applyJT(const core::ConstraintParams *cparams /* PARAMS FIRST */, Data<InMatrixDeriv>& out, const Data<OutMatrixDeriv>& in);

    virtual void applyDJT(const core::MechanicalParams* mparams /* PARAMS FIRST  = core::MechanicalParams::defaultInstance()*/, core::MultiVecDerivId parentForce, core::ConstMultiVecDerivId  childForce );

    virtual void updateK( const sofa::core::MechanicalParams* mparams, sofa::core::ConstMultiVecDerivId childForce ) override;

    void addGeometricStiffnessToMatrix(const sofa::core::MechanicalParams* /*mparams*/, const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/ ) override;
    template<class MatrixWriter>
    void addGeometricStiffnessToMatrixT(const sofa::core::MechanicalParams* mparams, MatrixWriter m);

    virtual const sofa::defaulttype::BaseMatrix* getJ();

#ifdef SOFA_HAVE_EIGEN2
    virtual const helper::vector<sofa::defaulttype::BaseMatrix*>* getJs();
#endif

    virtual void draw(const core::visual::VisualParams* vparams);

    void clear(int reserve = 0);

    void setRepartition(unsigned int value);
    void setRepartition(sofa::helper::vector<unsigned int> values);

    /// get the index of the rigid frame mapping the given output point
    unsigned int getRigidIndexFromOutIndex(unsigned int pout) const;

protected:
    class Loader;

    void load(const char* filename);
    const VecCoord& getPoints();

    bool updateJ;
    sofa::helper::vector<Mat> vecK;

#ifdef SOFA_HAVE_EIGEN2
    SparseMatrixEigen eigenJacobian;                      ///< Jacobian of the mapping used by getJs
    helper::vector<sofa::defaulttype::BaseMatrix*> eigenJacobians; /// used by getJs
#endif
};

template <int N, class Real>
struct RigidMappingMatrixHelper;

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_MAPPING_RIGIDMAPPING_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid2dTypes, sofa::defaulttype::Vec2dTypes >;
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid3dTypes, sofa::defaulttype::ExtVec3fTypes >;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid2fTypes, sofa::defaulttype::Vec2fTypes >;
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid3fTypes, sofa::defaulttype::ExtVec3fTypes >;
#endif

#ifndef SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid2dTypes, sofa::defaulttype::Vec2fTypes >;
extern template class SOFA_RIGID_API RigidMapping< sofa::defaulttype::Rigid2fTypes, sofa::defaulttype::Vec2dTypes >;
#endif
#endif
#endif

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
