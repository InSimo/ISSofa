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
#ifndef SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectToPlaneConstraint_INL
#define SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_ProjectToPlaneConstraint_INL

#include "ProjectToPlaneConstraint.h"

#include <iostream>

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/ProjectiveConstraintSet.inl>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/MultiMatrixAccessor.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/BasicShapes.h>

#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseTopology/TopologySubsetData.inl>

#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.h>


namespace sofa
{

namespace component
{

namespace projectiveconstraintset
{

template <class DataTypes>
ProjectToPlaneConstraint<DataTypes>::ProjectToPlaneConstraint()
    : core::behavior::ProjectiveConstraintSet<DataTypes>(NULL)
    , f_indices(initData(&f_indices, "indices", "Indices of the fixed points"))
    , f_origin(initData(&f_origin, CPos(), "origin", "A point in the plane"))
    , f_normal(initData(&f_normal, CPos(), "normal", "Normal vector to the plane"))
    , f_stiffness(initData(&f_stiffness, Real(1.0), "stiffness","Stiffness of the plane, added to the diagonal of the mechanical matrix to keep it definite" ))
    , f_drawSize( initData(&f_drawSize,0.0,"drawSize","0 -> point based rendering, >0 -> radius of spheres") )
{
}


template <class DataTypes>
ProjectToPlaneConstraint<DataTypes>::~ProjectToPlaneConstraint()
{
 
}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::computeIndices(const VecCoord& x)
{

    Indices& indices = *f_indices.beginEdit();
    indices.clear();

    const auto& o = f_origin.getValue();
    const auto& n = f_normal.getValue();

    for (std::size_t i = 0; i < x.size(); ++i)
    {

        const CPos& p = DataTypes::getCPos(x[i]);

        const double d = sofa::defaulttype::dot(p - o, n);

        if (d < 0)
        {
            indices.push_back(i);
        }
    }

    f_indices.endEdit();
}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::init()
{
    this->core::behavior::ProjectiveConstraintSet<DataTypes>::init();
    reinit();
}

template <class DataTypes>
void  ProjectToPlaneConstraint<DataTypes>::reinit()
{
    // normalize the normal vector
    CPos n = f_normal.getValue();
    n.normalize();
    f_normal.setValue(n);

    // plane normal direction projection matrix : nn^t  
    // plane normal direction rejection matrix ( ie projection of a vector to the plane) : I-nn^t

    for (unsigned i = 0; i < bsize; i++)
    {
        for (unsigned j = 0; j < bsize; j++)
        {

            m_projection(i, j) = n[i] * n[j];

            if (i == j)
            {
                m_rejection(i,j) = 1 - m_projection(i,j);
            }
            else
            {
                m_rejection(i,j) = -m_projection(i,j);
            }
        }
    }

    // the rejection matrix is symmetric, so equal to its transpose. 
    // Kept a distinct member variable for clarity
    m_rejectionT = m_rejection.transposed();
}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::projectMatrix( sofa::defaulttype::BaseMatrix* /*M*/, unsigned /*offset*/ )
{
    serr << "projectMatrix( sofa::defaulttype::BaseMatrix* M, unsigned offset ) is not implemented " << sendl;

}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::project(Deriv& v) const
{
    v = m_rejection*v;
}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::projectResponse(const core::MechanicalParams* mparams, DataVecDeriv& resData)
{

    const auto* d_x = mparams->readX(this->mstate);

    if (d_x)
    {
        computeIndices(d_x->getValue(mparams));
    }

    auto res = sofa::helper::write( resData, mparams );
    const Indices& indices  = f_indices.getValue(mparams);

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        unsigned index     = indices[i];
        Deriv& f     = res[index];
        project(f);
    }
}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::projectJacobianMatrix(const core::MechanicalParams* mparams , DataMatrixDeriv& cData)
{

    auto cWrite = sofa::helper::write(cData, mparams);
    const Indices& indices = f_indices.getValue(mparams);

    MatrixDeriv& c = cWrite.wref();
    c.compress();

    if (c.rowBegin.size() == 0)
    {
        return;
    }


    for (unsigned iRow = 0; iRow < c.rowBegin.size() - 1; ++iRow)
    {
        unsigned colBegin = c.rowBegin[iRow];
        unsigned colEnd = c.rowBegin[iRow + 1];

        for (unsigned p = colBegin; p < colEnd; ++p)
        {
            unsigned colIndex = c.colsIndex[p];

            auto itFind = std::find(indices.begin(), indices.end(), colIndex);
            const bool found = itFind != indices.end();

            if (found)
            {
                project(c.colsValue[p]);
            }
        }
    }

}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::projectVelocity(const core::MechanicalParams* mparams, DataVecDeriv& vdata)
{
    auto res = sofa::helper::write(vdata, mparams);
    const Indices& indices = f_indices.getValue(mparams);

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        unsigned index = indices[i];
        Deriv& f = res[index];
        project(f);
    }
}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::projectPosition(const core::MechanicalParams* /*mparams*/ , DataVecCoord& /*xData*/)
{
}

// Matrix Integration interface
template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::applyConstraint(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    // compute PAP^t with P being the projection matrix
    // Highly unoptimized since we consider A like a dense matrix

    using BlockMatrix = sofa::defaulttype::Mat<bsize, bsize, Real>;

    core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix(this->mstate.get(mparams));
    const Real kFactor          = mparams->kFactor();
    const Real stiffness        = f_stiffness.getValue(mparams)*kFactor;
    BlockMatrix stiffnessDBlock = m_projection*stiffness;

    if (r)
    {
        const Indices& indices = f_indices.getValue(mparams);

        for (unsigned i = 0; i < indices.size(); ++i)
        {
            unsigned index = indices[i];

            {
                // project all the blocks on each column
                const unsigned bColIndex = r.offset + index*bsize;

                for (int j = 0; j < r.matrix->rowSize(); j += bsize)
                {
                    const int bRowIndex = r.offset + j;

                    BlockMatrix block;

                    for (int bi = 0; bi < bsize; ++bi)
                    {
                        for (int bj = 0; bj < bsize; ++bj)
                        {
                            block(bi, bj) = r.matrix->element(bRowIndex + bi, bColIndex + bj);
                        }
                    }

                    if (sofa::defaulttype::matrix_bloc_traits<BlockMatrix>::empty(block))
                    {
                        continue;
                    }


                    block = block*m_rejection;

                    for (int bi = 0; bi < bsize; ++bi)
                    {
                        for (int bj = 0; bj < bsize; ++bj)
                        {
                            r.matrix->set(bRowIndex + bi, bColIndex + bj, block(bi, bj));
                        }
                    }
                }
            }

            {
                // project all the blocks on each row
                const int bRowIndex = r.offset + index*bsize;
                
                for (int j = 0; j < r.matrix->colSize(); j += bsize)
                {
                    const int bColIndex = r.offset + j;

                    BlockMatrix block;

                    for (int bi = 0; bi < bsize; ++bi)
                    {
                        for (int bj = 0; bj < bsize; ++bj)
                        {
                            block(bi, bj) = r.matrix->element(bRowIndex + bi, bColIndex + bj);
                        }
                    }

                    if (sofa::defaulttype::matrix_bloc_traits<BlockMatrix>::empty(block))
                    {
                        continue;
                    }

                    block = m_rejectionT*block;

                    if (bRowIndex == bColIndex)
                    {
                        // add stiffness on the diagonal to keep the matrix (positive) definite
                        block += stiffnessDBlock;
                    }

                    for (int bi = 0; bi < bsize; ++bi)
                    {
                        for (int bj = 0; bj < bsize; ++bj)
                        {
                            r.matrix->set(bRowIndex + bi, bColIndex + bj, block(bi, bj));
                        }
                    }

                }



            }
        }
    }
}

template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::applyConstraint(defaulttype::BaseVector * /*vect*/, unsigned int /*offset*/)
{
    serr<<"applyConstraint(defaulttype::BaseVector *vect, unsigned int offset) is not implemented "<< sendl;
}




template <class DataTypes>
void ProjectToPlaneConstraint<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
    if (!vparams->displayFlags().getShowBehaviorModels()) return;
    if (!this->isActive()) return;
    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();

    const Indices & indices = f_indices.getValue();

    if( f_drawSize.getValue() == 0) // old classical drawing by points
    {
        sofa::helper::vector< sofa::defaulttype::Vector3 > points;
        sofa::defaulttype::Vector3 point;
        //serr<<"ProjectToPlaneConstraint<DataTypes>::draw(), indices = "<<indices<<sendl;
        for (Indices::const_iterator it = indices.begin();
                it != indices.end();
                ++it)
        {
            point = DataTypes::getCPos(x[*it]);
            points.push_back(point);
        }
        vparams->drawTool()->drawPoints(points, 10, sofa::defaulttype::Vec<4,float>(1,0.5,0.5,1));
    }
    else // new drawing by spheres
    {
        sofa::helper::vector< sofa::defaulttype::Vector3 > points;
        sofa::defaulttype::Vector3 point;
        glColor4f (1.0f,0.35f,0.35f,1.0f);
        for (Indices::const_iterator it = indices.begin();
                it != indices.end();
                ++it)
        {
            point = DataTypes::getCPos(x[*it]);
            points.push_back(point);
        }
        vparams->drawTool()->drawSpheres(points, (float)f_drawSize.getValue(), sofa::defaulttype::Vec<4,float>(1.0f,0.35f,0.35f,1.0f));
    }
#endif /* SOFA_NO_OPENGL */
}

//// Specialization for rigids
//#ifndef SOFA_FLOAT
//template <>
//    void ProjectToPlaneConstraint<Rigid3dTypes >::draw(const core::visual::VisualParams* vparams);
//template <>
//    void ProjectToPlaneConstraint<Rigid2dTypes >::draw(const core::visual::VisualParams* vparams);
//#endif
//#ifndef SOFA_DOUBLE
//template <>
//    void ProjectToPlaneConstraint<Rigid3fTypes >::draw(const core::visual::VisualParams* vparams);
//template <>
//    void ProjectToPlaneConstraint<Rigid2fTypes >::draw(const core::visual::VisualParams* vparams);
//#endif



} // namespace constraint

} // namespace component

} // namespace sofa

#endif


