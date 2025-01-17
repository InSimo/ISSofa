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
#ifndef SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_FIXEDPLANECONSTRAINT_INL
#define SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_FIXEDPLANECONSTRAINT_INL

#include <sofa/core/behavior/ProjectiveConstraintSet.inl>
#include <SofaBoundaryCondition/FixedPlaneConstraint.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/gl/template.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

#include <SofaBaseTopology/TopologySubsetData.inl>

namespace sofa
{

namespace component
{

namespace projectiveconstraintset
{

template <class DataTypes>
FixedPlaneConstraint<DataTypes>::FixedPlaneConstraint()
    : direction( initData(&direction,"direction","normal direction of the plane"))
    , dmin( initData(&dmin,(Real)0,"dmin","Minimum plane distance from the origin"))
    , dmax( initData(&dmax,(Real)0,"dmax","Maximum plane distance from the origin") )
    , indices( initData(&indices,"indices","Indices of the fixed points"))
{
    selectVerticesFromPlanes=false;
    pointHandler = new FCPointHandler(this, &indices);
}

template <class DataTypes>
FixedPlaneConstraint<DataTypes>::~FixedPlaneConstraint()
{
    if (pointHandler)
        delete pointHandler;
}

// Define TestNewPointFunction
template< class DataTypes>
bool FixedPlaneConstraint<DataTypes>::FCPointHandler::applyTestCreateFunction(unsigned int, const sofa::helper::vector<unsigned int> &, const sofa::helper::vector<double> &)
{
    if (fc)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Define RemovalFunction
template< class DataTypes>
void FixedPlaneConstraint<DataTypes>::FCPointHandler::applyDestroyFunction(unsigned int pointIndex, value_type &)
{
    if (fc)
    {
        fc->removeConstraint((unsigned int) pointIndex);
    }
}
template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::addConstraint(int index)
{
    indices.beginEdit()->push_back(index);
    indices.endEdit();
}

template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::removeConstraint(int index)
{
    removeValue(*indices.beginEdit(),(unsigned int)index);
    indices.endEdit();
}

// -- Constraint interface


template <class DataTypes> template <class DataDeriv>
void FixedPlaneConstraint<DataTypes>::projectResponseT(const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataDeriv& data,
                                                       std::function<void(DataDeriv&, const SetIndexArray&)> project)
{
    project(data, indices.getValue());
}

template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::projectResponse(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& resData)
{
    helper::WriteAccessor<DataVecDeriv> res = resData;
    projectResponseT<VecDeriv>(mparams /* PARAMS FIRST */, res.wref(),
            [&](auto& dx, const SetIndexArray& indices)
            {
                const DPos dir = direction.getValue();
                for (const auto& i : indices)
                {
                    /// only constraint one projection of the displacement to be zero
                    DPos val = DataTypes::getDPos(dx[i]);
                    val -= dir*dot(val,dir);
                    DataTypes::setDPos(dx[i], val);
                }
            });
}

/// project dx to constrained space (dx models a velocity)
template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::projectVelocity(const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecDeriv& /*vData*/)
{

}

/// project x to constrained space (x models a position)
template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::projectPosition(const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecCoord& /*xData*/)
{

}

template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::projectMatrix( sofa::defaulttype::BaseMatrix* M, unsigned /*offset*/ )
{
    // clears the rows and columns associated with constrained particles
    unsigned blockSize = DataTypes::deriv_total_size;

    for(SetIndexArray::const_iterator it= indices.getValue().begin(), iend=indices.getValue().end(); it!=iend; it++ )
    {
        M->clearRowsCols((*it) * blockSize,(*it+1) * (blockSize) );
    }

}
template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::projectJacobianMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataMatrixDeriv& cData)
{
    helper::WriteAccessor<DataMatrixDeriv> c = cData;
    projectResponseT<MatrixDeriv>(mparams /* PARAMS FIRST */, c.wref(),
        [&](auto& dx, const SetIndexArray& indices)
        {
            const DPos dir = direction.getValue();
            auto itRow = dx.begin();
            auto itRowEnd = dx.end();
            while (itRow != itRowEnd)
            {
                for (auto colIt = itRow.begin(); colIt != itRow.end(); colIt++)
                {
                    if (std::find(indices.begin(), indices.end(), colIt.index()) != indices.end())
                    {
                        DPos val = DataTypes::getDPos(colIt.val());
                        Deriv r;
                        DataTypes::setDPos(r, -(dir*dot(val,dir)));
                        dx.writeLine(itRow.index()).addCol(colIt.index(), r);
                    }
                }
            }
        });
}

template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::setDirection(DPos dir)
{
    if (dir.norm2()>0)
    {
        direction.setValue(dir);
    }
}

template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::selectVerticesAlongPlane()
{
    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();
    unsigned int i;
    for(i=0; i<x.size(); ++i)
    {
        if (isPointInPlane(x[i]))
            addConstraint(i);
    }

}
template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::init()
{
    this->core::behavior::ProjectiveConstraintSet<DataTypes>::init();

    topology = this->getContext()->getMeshTopology();

    /// test that dmin or dmax are different from zero
    if (dmin.getValue()!=dmax.getValue())
        selectVerticesFromPlanes=true;

    if (selectVerticesFromPlanes)
        selectVerticesAlongPlane();

    // Initialize functions and parameters
    indices.createTopologicalEngine(topology, pointHandler);
    indices.registerTopologicalData();

}


template <class DataTypes>
void FixedPlaneConstraint<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
    if (!vparams->displayFlags().getShowBehaviorModels()) return;
    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();
    glDisable (GL_LIGHTING);
    glPointSize(10);
    glColor4f (1,1.0,0.5,1);
    glBegin (GL_POINTS);
    for (helper::vector< unsigned int >::const_iterator it = this->indices.getValue().begin(); it != this->indices.getValue().end(); ++it)
    {
        helper::gl::glVertexT(x[*it]);
    }
    glEnd();
#endif /* SOFA_NO_OPENGL */
}

} // namespace constraint

} // namespace component

} // namespace sofa

#endif
