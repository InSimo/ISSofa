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
#ifndef SOFA_COMPONENT_MAPPING_SUBSETMULTIMAPPING_H
#define SOFA_COMPONENT_MAPPING_SUBSETMULTIMAPPING_H

#include <sofa/core/MultiMapping.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/SofaMisc.h>
#include <sofa/defaulttype/Vec3Types.h>

namespace sofa
{

namespace component
{

namespace mapping
{

/**
 * @class SubsetMapping
 * @brief Compute a subset of input points
 */
template <class TIn, class TOut>
class SubsetMultiMapping : public core::MultiMapping<TIn, TOut>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(SubsetMultiMapping, TIn, TOut), SOFA_TEMPLATE2(core::MultiMapping, TIn, TOut));

    typedef core::MultiMapping<TIn, TOut> Inherit;
    typedef TIn In;
    typedef TOut Out;

    typedef typename Out::VecCoord OutVecCoord;
    typedef typename Out::VecDeriv OutVecDeriv;
    typedef typename Out::Coord OutCoord;
    typedef typename Out::Deriv OutDeriv;
    typedef typename Out::MatrixDeriv OutMatrixDeriv;
    typedef typename In::VecCoord InVecCoord;
    typedef typename In::VecDeriv InVecDeriv;
    typedef typename In::Coord InCoord;
    typedef typename In::Deriv InDeriv;
    typedef typename In::MatrixDeriv InMatrixDeriv;
    typedef typename InCoord::value_type Real;
    typedef typename helper::vector <const InVecCoord*> vecConstInVecCoord;
    typedef typename helper::vector<OutVecCoord*> vecOutVecCoord;

    typedef Data<InVecCoord> InDataVecCoord;
    typedef Data<InVecDeriv> InDataVecDeriv;
    typedef Data<InMatrixDeriv> InDataMatrixDeriv;

    typedef Data<OutVecCoord> OutDataVecCoord;
    typedef Data<OutVecDeriv> OutDataVecDeriv;
    typedef Data<OutMatrixDeriv> OutDataMatrixDeriv;

    /// Correspondance array
    typedef core::topology::BaseMeshTopology::SetIndex IndexArray;

    virtual void init();

    void handleTopologyChange(sofa::core::topology::Topology* t) override;

    // Append particle of given index within the given model to the subset.
    void addPoint(const core::BaseState* fromModel, int index);
    // Append particle of given index within the given model to the subset.
    void addPoint(int fromModel, int index);

    // usual Mapping API
    virtual void apply(const core::MechanicalParams* mparams /* PARAMS FIRST */, const helper::vector<OutDataVecCoord*>& dataVecOutPos, const helper::vector<const InDataVecCoord*>& dataVecInPos);
    virtual void applyJ(const core::MechanicalParams* mparams /* PARAMS FIRST */, const helper::vector<OutDataVecDeriv*>& dataVecOutVel, const helper::vector<const InDataVecDeriv*>& dataVecInVel);
    virtual void applyJT(const core::MechanicalParams* mparams /* PARAMS FIRST */, const helper::vector<InDataVecDeriv*>& dataVecOutForce, const helper::vector<const OutDataVecDeriv*>& dataVecInForce);
    virtual void applyDJT(const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, core::MultiVecDerivId /*inForce*/, core::ConstMultiVecDerivId /*outForce*/) {}

    virtual void applyJT( const core::ConstraintParams* cparams /* PARAMS FIRST */, const helper::vector< InDataMatrixDeriv* >& dataMatOutConst, const helper::vector< const OutDataMatrixDeriv* >& dataMatInConst );
    
    Data< vector<unsigned> > indexPairs;                     ///< Two indices per child: the parent, and the index within the parent
    Data< vector<unsigned> > d_identityIndices;              ///< One index per child to select the parent : the index in the parent is the same as the child index

protected :

    SubsetMultiMapping()
        : Inherit()
        , indexPairs( initData( &indexPairs, vector<unsigned>(), "indexPairs", "list of couples (parent index + index in the parent)"))
        , d_identityIndices( initData( &d_identityIndices, vector<unsigned>(), "identityIndices", "One index per child to select the parent : the index in the parent is the same as the child index"))
    {
    }

    virtual ~SubsetMultiMapping();


    /// Pointer to the current topology
    sofa::core::topology::BaseMeshTopology* topology;

    vector<defaulttype::BaseMatrix*> baseMatrices;      ///< Jacobian of the mapping, in a vector

    void fillIndexPairs();
};



#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MISC_MAPPING)

#ifndef SOFA_FLOAT
extern template class SOFA_MISC_MAPPING_API SubsetMultiMapping< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_MISC_MAPPING_API SubsetMultiMapping< sofa::defaulttype::Rigid3dTypes, sofa::defaulttype::Rigid3dTypes >;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_MISC_MAPPING_API SubsetMultiMapping< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_MISC_MAPPING_API SubsetMultiMapping< sofa::defaulttype::Rigid3fTypes, sofa::defaulttype::Rigid3fTypes >;
#endif
#endif

} // namespace mapping

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_MAPPING_SUBSETMULTIMAPPING_H
