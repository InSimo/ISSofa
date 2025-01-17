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
#ifndef SOFA_COMPONENT_MECHANICALOBJECT_INL
#define SOFA_COMPONENT_MECHANICALOBJECT_INL

#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/State.inl>
#include <sofa/core/visual/VisualParams.h>
#ifdef SOFA_SMP
#include <SofaBaseMechanics/MechanicalObjectTasks.inl>
#endif
#include <SofaBaseLinearSolver/SparseMatrix.h>
#include <sofa/core/topology/TopologyChange.h>
#include <SofaBaseTopology/RegularGridTopology.h>

#include <sofa/defaulttype/DataTypeInfo.h>

#include <sofa/helper/accessor.h>
#include <sofa/helper/system/glut.h>

#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/Simulation.h>
#ifdef SOFA_DUMP_VISITOR_INFO
#include <sofa/simulation/common/Visitor.h>
#endif

#include <assert.h>
#include <iostream>

#include <SofaBaseTopology/TopologyData.inl>

namespace
{

template<class V>
void renumber(V* v, V* tmp, const sofa::helper::vector< unsigned int > &index )
{
    if (v == NULL)
        return;

    if (v->empty())
        return;

    *tmp = *v;
    for (unsigned int i = 0; i < v->size(); ++i)
        (*v)[i] = (*tmp)[index[i]];
}

} // anonymous namespace


namespace sofa
{

namespace component
{

namespace container
{

namespace internal
{

// internal reinit method specialized version for rigid types ensures that each quaternion is normalized
template< class DataTypes >
void reinit( MechanicalObject<DataTypes>* /*obj*/ ) 
{
}

}

template <class DataTypes>
MechanicalObject<DataTypes>::MechanicalObject()
    : x(initData(&x, "position", "position coordinates of the degrees of freedom"))
    , v(initData(&v, "velocity", "velocity coordinates of the degrees of freedom"))
    , f(initData(&f, "force", "force vector of the degrees of freedom"))
    , externalForces(initData(&externalForces, "externalForce", "externalForces vector of the degrees of freedom"))
    , dx(initData(&dx, "derivX", "dx vector of the degrees of freedom"))
    , dforce(initData(&dforce, "dforce", "dforce vector of the degrees of freedom"))
    , xfree(initData(&xfree, "free_position", "free position coordinates of the degrees of freedom"))
    , vfree(initData(&vfree, "free_velocity", "free velocity coordinates of the degrees of freedom"))
    , x0(initData(&x0, "rest_position", "rest position coordinates of the degrees of freedom"))
    , c(initData(&c, "constraint", "constraints applied to the degrees of freedom"))
    , reset_position(initData(&reset_position, "reset_position", "reset position coordinates of the degrees of freedom"))
    , reset_velocity(initData(&reset_velocity, "reset_velocity", "reset velocity coordinates of the degrees of freedom"))
	, previous_x(initData(&previous_x, "previous_position", "last position"))
    , restScale(initData(&restScale, (SReal)1.0, "restScale", "optional scaling of rest position coordinates (to simulated pre-existing internal tension)"))
    , d_useTopology(initData(&d_useTopology, true, "useTopology", "Shall this object rely on any active topology to initialize its size and positions"))
    , showObject(initData(&showObject, (bool) false, "showObject", "Show objects"))
    , showFreePositions(initData(&showFreePositions, (bool) false, "showFreePositions", "Display free positions instead of positions"))
    , showObjectScale(initData(&showObjectScale, (float) 0.1, "showObjectScale", "Scale for object display"))
    , showIndices(initData(&showIndices, (bool) false, "showIndices", "Show indices"))
    , showIndicesScale(initData(&showIndicesScale, (float) 0.0001, "showIndicesScale", "Scale for indices display"))
    , showVectors(initData(&showVectors, (bool) false, "showVectors", "Show velocity"))
    , showVectorsScale(initData(&showVectorsScale, (float) 0.0001, "showVectorsScale", "Scale for vectors display"))
    , drawMode(initData(&drawMode,0,"drawMode","The way vectors will be drawn:\n- 0: Line\n- 1:Cylinder\n- 2: Arrow.\n\nThe DOFS will be drawn:\n- 0: point\n- >1: sphere"))
    , d_color(initData(&d_color, defaulttype::Vec4f(1,1,1,1), "showColor", "Color for object display"))
    , isToPrint( initData(&isToPrint, false, "isToPrint", "suppress somes data before using save as function"))
    , translation(initData(&translation, Vector3(), "translation", "Translation of the DOFs"))
    , rotation(initData(&rotation, Vector3(), "rotation", "Rotation of the DOFs"))
    , scale(initData(&scale, Vector3(1.0,1.0,1.0), "scale3d", "Scale of the DOFs in 3 dimensions"))
    , translation2(initData(&translation2, Vector3(), "translation2", "Translation of the DOFs, applied after the rest position has been computed"))
    , rotation2(initData(&rotation2, Vector3(), "rotation2", "Rotation of the DOFs, applied the after the rest position has been computed"))
    , d_size(initData(&d_size, 0, "size", "Size of the vectors"))
    , l_topology(initLink("topology","Link to the topology relevant for this object"))
    , f_reserve(initData(&f_reserve, 0, "reserve", "Size to reserve when creating vectors"))
    , m_gnuplotFileX(NULL)
    , m_gnuplotFileV(NULL)
{
    m_initialized = false;

    data = MechanicalObjectInternalData<DataTypes>(this);

    isToPrint.setDisplayed(false);

    x               .setGroup("Vector");
    v               .setGroup("Vector");
    f               .setGroup("Vector");
    externalForces  .setGroup("Vector");
    dx              .setGroup("Vector");
    dforce          .setGroup("Vector");
    xfree           .setGroup("Vector");
    vfree           .setGroup("Vector");
    x0              .setGroup("Vector");
    c               .setGroup("Vector");
    reset_position  .setGroup("Vector");
    reset_velocity  .setGroup("Vector");
	previous_x      .setGroup("Vector");

    translation     .setGroup("Transformation");
    translation2    .setGroup("Transformation");
    rotation        .setGroup("Transformation");
    rotation2       .setGroup("Transformation");
    scale           .setGroup("Transformation");

    setVecCoord(core::VecCoordId::position().index, &x);
    setVecCoord(core::VecCoordId::freePosition().index, &xfree);
    setVecCoord(core::VecCoordId::restPosition().index, &x0);
    setVecCoord(core::VecCoordId::resetPosition().index, &reset_position);
	setVecCoord(core::VecCoordId::lastPosition().index, &previous_x);
    setVecDeriv(core::VecDerivId::velocity().index, &v);
    setVecDeriv(core::VecDerivId::force().index, &f);
    setVecDeriv(core::VecDerivId::externalForce().index, &externalForces);
    setVecDeriv(core::VecDerivId::dx().index, &dx);
    setVecDeriv(core::VecDerivId::dforce().index, &dforce);
    setVecDeriv(core::VecDerivId::freeVelocity().index, &vfree);
    setVecDeriv(core::VecDerivId::resetVelocity().index, &reset_velocity);
    setVecMatrixDeriv(core::MatrixDerivId::constraintJacobian().index, &c);

    // These vectors are set as modified as they are mandatory in the MechanicalObject.
    x               .forceSet();
    //  x0              .forceSet();
    v               .forceSet();
    dx              .forceSet();
    dforce          .forceSet();
    f               .forceSet();
    externalForces  .forceSet();

    // there is no need for a common user to watch at these vectors
//    dx.setDisplayed( false );
//    freePosition.setDisplayed( false );
//    freeVelocity.setDisplayed( false );

    // do not forget to delete these in the destructor
    // null() vectors must be allocated for now -> TODO how to define an implicit null vector?
    // not to allocate plenty of 0 everywhere...
    write(core::VecCoordId::null())->forceSet();
    write(core::VecDerivId::null())->forceSet();
    write(core::MatrixDerivId::null())->forceSet();


    // default size is 1
    resize(1);
}


template <class DataTypes>
MechanicalObject<DataTypes>::~MechanicalObject()
{
    if (m_gnuplotFileV != NULL)
        delete m_gnuplotFileV;

    if (m_gnuplotFileX != NULL)
        delete m_gnuplotFileX;

    for(unsigned i=core::VecCoordId::V_FIRST_DYNAMIC_INDEX; i<vectorsCoord.size(); i++)
        if( vectorsCoord[i] != NULL ) { delete vectorsCoord[i]; vectorsCoord[i]=NULL; }
    if( vectorsCoord[core::VecCoordId::null().getIndex()] != NULL )
        { delete vectorsCoord[core::VecCoordId::null().getIndex()]; vectorsCoord[core::VecCoordId::null().getIndex()] = NULL; }

    for(unsigned i=core::VecDerivId::V_FIRST_DYNAMIC_INDEX; i<vectorsDeriv.size(); i++)
        if( vectorsDeriv[i] != NULL )  { delete vectorsDeriv[i]; vectorsDeriv[i]=NULL; }
    if( vectorsDeriv[core::VecDerivId::null().getIndex()] != NULL )
        { delete vectorsDeriv[core::VecDerivId::null().getIndex()]; vectorsDeriv[core::VecDerivId::null().getIndex()] = NULL; }

    for(unsigned i=core::MatrixDerivId::V_FIRST_DYNAMIC_INDEX; i<vectorsMatrixDeriv.size(); i++)
        if( vectorsMatrixDeriv[i] != NULL )  { delete vectorsMatrixDeriv[i]; vectorsMatrixDeriv[i]=NULL; }

    if (vectorsMatrixDeriv[core::MatrixDerivId::null().getIndex()] != nullptr) {
        delete vectorsMatrixDeriv[core::MatrixDerivId::null().getIndex()]; 
        vectorsMatrixDeriv[core::MatrixDerivId::null().getIndex()] = nullptr;
    }
}

#ifdef SOFA_HAVE_NEW_TOPOLOGYCHANGES
template <class DataTypes>
void MechanicalObject<DataTypes>::MOPointHandler::applyCreateFunction(unsigned int /*pointIndex*/, Coord& /*dest*/,
                                                                      const sofa::helper::vector< unsigned int > &ancestors,
                                                                      const sofa::helper::vector< double > &coefs)
{
    if (!obj)
        return;

    if (!ancestors.empty() )
    {
        const unsigned int prevSizeMechObj = obj->getSize();
        obj->d_size.setValue( prevSizeMechObj + 1 );

        obj->computeWeightedValue( prevSizeMechObj + 1, ancestors, coefs );
    }
    else
    {
        // No ancestors specified, resize DOFs vectors and set new values to the reset default value.
        obj->d_size.setValue( obj->getSize() + 1 );
    }
}


template <class DataTypes>
void MechanicalObject<DataTypes>::MOPointHandler::applyDestroyFunction(unsigned int, Coord &)
{
    if (!obj)
        return;

    unsigned int prevSizeMechObj   = obj->getSize();
    //unsigned int lastIndexMech = prevSizeMechObj - 1;

    obj->d_size.setValue( prevSizeMechObj - 1 );
    //obj->replaceValue(lastIndexMech, index );
    //obj->resize( prevSizeMechObj - 1 );
}

#endif


template <class DataTypes>
void MechanicalObject<DataTypes>::initGnuplot(const std::string path)
{
    if( !this->getName().empty() )
    {
        if (m_gnuplotFileX != NULL)
            delete m_gnuplotFileX;

        if (m_gnuplotFileV != NULL)
            delete m_gnuplotFileV;

        m_gnuplotFileX = new std::ofstream( (path + this->getName()+"_x.txt").c_str() );
        m_gnuplotFileV = new std::ofstream( (path + this->getName()+"_v.txt").c_str() );
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::exportGnuplot(double time)
{
    if( m_gnuplotFileX!=NULL )
    {
        (*m_gnuplotFileX) << time <<"\t"<< read(core::ConstVecCoordId::position())->getValue() << std::endl;
    }

    if( m_gnuplotFileV!=NULL )
    {
        (*m_gnuplotFileV) << time <<"\t"<< read(core::ConstVecDerivId::velocity())->getValue() << std::endl;
    }
}

template <class DataTypes>
MechanicalObject<DataTypes> &MechanicalObject<DataTypes>::operator = (const MechanicalObject& obj)
{
    resize(obj.getSize());

    return *this;
}


template <class DataTypes>
void MechanicalObject<DataTypes>::parse ( sofa::core::objectmodel::BaseObjectDescription* arg )
{
    Inherited::parse(arg);
    
    if (arg->getAttribute("size") != NULL)
    {
        resize(atoi(arg->getAttribute("size", "0")));
    }

    // DEPRECATED: Warning, you should not use these parameters, but a TransformEngine instead
    if (arg->getAttribute("scale") != NULL)
    {
        SReal s = (SReal)atof(arg->getAttribute("scale", "1.0"));
        scale.setValue(Vector3(s, s, s));
    }

    if (arg->getAttribute("sx") != NULL || arg->getAttribute("sy") != NULL || arg->getAttribute("sz") != NULL)
    {
        scale.setValue(Vector3((SReal)(atof(arg->getAttribute("sx","1.0"))),(SReal)(atof(arg->getAttribute("sy","1.0"))),(SReal)(atof(arg->getAttribute("sz","1.0")))));
    }

    if (arg->getAttribute("rx") != NULL || arg->getAttribute("ry") != NULL || arg->getAttribute("rz") != NULL)
    {
        rotation.setValue(Vector3((SReal)(atof(arg->getAttribute("rx","0.0"))),(SReal)(atof(arg->getAttribute("ry","0.0"))),(SReal)(atof(arg->getAttribute("rz","0.0")))));
    }

    if (arg->getAttribute("dx") != NULL || arg->getAttribute("dy") != NULL || arg->getAttribute("dz") != NULL)
    {
        translation.setValue(Vector3((Real)atof(arg->getAttribute("dx","0.0")), (Real)atof(arg->getAttribute("dy","0.0")), (Real)atof(arg->getAttribute("dz","0.0"))));
    }

    if (arg->getAttribute("rx2") != NULL || arg->getAttribute("ry2") != NULL || arg->getAttribute("rz2") != NULL)
    {
        rotation2.setValue(Vector3((SReal)(atof(arg->getAttribute("rx2","0.0"))),(SReal)(atof(arg->getAttribute("ry2","0.0"))),(SReal)(atof(arg->getAttribute("rz2","0.0")))));
    }

    if (arg->getAttribute("dx2") != NULL || arg->getAttribute("dy2") != NULL || arg->getAttribute("dz2") != NULL)
    {
        translation2.setValue(Vector3((Real)atof(arg->getAttribute("dx2","0.0")), (Real)atof(arg->getAttribute("dy2","0.0")), (Real)atof(arg->getAttribute("dz2","0.0"))));
    }
}


#if 0 //SOFA_HAVE_NEW_TOPOLOGYCHANGES
template <class DataTypes>
void MechanicalObject<DataTypes>::PointCreationFunction(int , void * param, Coord & , const sofa::helper::vector<unsigned int> & ancestors, const sofa::helper::vector<double> & coefs)
{
    MechanicalObject<DataTypes> *meca = (MechanicalObject<DataTypes>*) param;

    if (!meca)
        return;

    if (!ancestors.empty() )
    {
        const unsigned int prevSizeMechObj = meca->getSize();
        meca->d_size.setValue( prevSizeMechObj + 1 );

        meca->computeWeightedValue( prevSizeMechObj + 1, ancestors, coefs );
    }
    else
    {
        // No ancestors specified, resize DOFs vectors and set new values to the reset default value.
        meca->d_size.setValue( meca->getSize() + 1 );
    }
}


template <class DataTypes>
void MechanicalObject<DataTypes>::PointDestroyFunction(int , void * param, Coord &)
{
    MechanicalObject<DataTypes> *meca = (MechanicalObject<DataTypes>*) param;
    if (!meca)
        return;

    unsigned int prevSizeMechObj   = meca->getSize();
    //unsigned int lastIndexMech = prevSizeMechObj - 1;

    meca->d_size.setValue( prevSizeMechObj - 1 );
    //meca->replaceValue(lastIndexMech, index );
    //meca->resize( prevSizeMechObj - 1 );
}

#endif

template <class DataTypes>
void MechanicalObject<DataTypes>::handleStateChange()
{
    if (!l_topology) return;
    //#ifdef SOFA_HAVE_NEW_TOPOLOGYCHANGES
    //    std::cout << "WARNING MechanicalObject<DataTypes>::handleStateChange()" << std::endl;
    //#else
    using sofa::core::topology::TopologyChange;
    using sofa::core::topology::TopologyChangeType;
    using sofa::core::topology::PointsAdded;
    using sofa::core::topology::PointsMoved;
    using sofa::core::topology::PointsRemoved;
    using sofa::core::topology::PointsRenumbering;
    sofa::core::topology::GeometryAlgorithms *geoAlgo = NULL;
    this->getContext()->get(geoAlgo, sofa::core::objectmodel::BaseContext::Local);

    std::list< const TopologyChange * >::const_iterator itBegin = l_topology->beginStateChange();
    std::list< const TopologyChange * >::const_iterator itEnd = l_topology->endStateChange();

    while( itBegin != itEnd )
    {
        TopologyChangeType changeType = (*itBegin)->getChangeType();

        switch( changeType )
        {
        case core::topology::POINTSADDED:
        {
            using sofa::helper::vector;
            const PointsAdded &pointsAdded = *static_cast< const PointsAdded * >( *itBegin );

            unsigned int prevSizeMechObj = getSize();
            unsigned int nbPoints = pointsAdded.getNbAddedVertices();

            if (pointsAdded.pointIndexArray.size() != nbPoints)
            {
                serr << "TOPO STATE EVENT POINTSADDED SIZE MISMATCH: "
                     << nbPoints << " != " << pointsAdded.pointIndexArray.size() << sendl;
            }
            for (unsigned int i=0; i<pointsAdded.pointIndexArray.size(); ++i)
            {
                unsigned int p1 = prevSizeMechObj + i;
                unsigned int p2 = pointsAdded.pointIndexArray[i];
                if (p1 != p2)
                {
                    serr << "TOPO STATE EVENT POINTSADDED INDEX " << i << " MISMATCH: "
                         << p1 << " != " << p2 << sendl;
                }
            }

            const vector<vector<unsigned int>>& ancestors = pointsAdded.ancestorsList;
            const vector<vector<double      >>& coefs     = pointsAdded.coefs;

            resize(prevSizeMechObj + nbPoints);

            if (!ancestors.empty() )
            {
                vector< vector< double > > coefs2;
                coefs2.resize(ancestors.size());

                for (unsigned int i = 0; i < ancestors.size(); ++i)
                {
                    coefs2[i].resize(ancestors[i].size());

                    for (unsigned int j = 0; j < ancestors[i].size(); ++j)
                    {
                        // constructng default coefs if none were defined
                        if (coefs.empty() || coefs[i].empty())
                            coefs2[i][j] = 1.0f / ancestors[i].size();
                        else
                            coefs2[i][j] = coefs[i][j];
                    }
                }

                for (unsigned int i = 0; i < ancestors.size(); ++i)
                {
                    computeWeightedValue( prevSizeMechObj + i, ancestors[i], coefs2[i] );
                }
            }

            if (!pointsAdded.ancestorElems.empty() && (geoAlgo != NULL))
            {
                helper::vector< core::VecCoordId > coordVecs;
                helper::vector< core::VecDerivId > derivVecs;

                for (unsigned int k = 0; k < vectorsCoord.size(); k++)
                {
                    if (vectorsCoord[k] != NULL && vectorsCoord[k]->isSet())
                    {
                        const VecCoord &vecCoord = vectorsCoord[k]->getValue();

                        if (vecCoord.size() != 0)
                        {
                            coordVecs.push_back(k);
                        }
                    }
                }

                for (unsigned int k = 0; k < vectorsDeriv.size(); k++)
                {
                    if (vectorsDeriv[k] != NULL && vectorsDeriv[k]->isSet())
                    {
                        const VecDeriv &vecDeriv = vectorsDeriv[k]->getValue();

                        if (vecDeriv.size() != 0)
                        {
                            derivVecs.push_back(k);
                        }
                    }
                }

                geoAlgo->initPointsAdded(pointsAdded.pointIndexArray, pointsAdded.ancestorElems, coordVecs, derivVecs);
            }
            
            break;
        }
        case core::topology::POINTSREMOVED:
        {
            const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRemoved * >( *itBegin ) )->getArray();

            unsigned int prevSizeMechObj   = getSize();
            unsigned int lastIndexMech = prevSizeMechObj - 1;
            for (unsigned int i = 0; i < tab.size(); ++i)
            {
                replaceValue(lastIndexMech, tab[i] );

                --lastIndexMech;
            }
            resize( prevSizeMechObj - tab.size() );
            break;
        }
        case core::topology::POINTSMOVED:
        {
            using sofa::helper::vector;

            const vector< unsigned int > indicesList = ( static_cast <const PointsMoved *> (*itBegin))->indicesList;
            const vector< vector< unsigned int > > ancestors = ( static_cast< const PointsMoved * >( *itBegin ) )->ancestorsList;
            const vector< vector< double > > coefs = ( static_cast< const PointsMoved * >( *itBegin ) )->baryCoefsList;

            if (ancestors.size() != indicesList.size() || ancestors.empty())
            {
                this->serr << "Error ! MechanicalObject::POINTSMOVED topological event, bad inputs (inputs don't share the same size or are empty)."<<this->sendl;
                break;
            }

            vector< vector < double > > coefs2;
            coefs2.resize (coefs.size());

            for (unsigned int i = 0; i<ancestors.size(); ++i)
            {
                coefs2[i].resize(ancestors[i].size());

                for (unsigned int j = 0; j < ancestors[i].size(); ++j)
                {
                    // constructng default coefs if none were defined
                    if (coefs == (const vector< vector< double > >)0 || coefs[i].size() == 0)
                        coefs2[i][j] = 1.0f / ancestors[i].size();
                    else
                        coefs2[i][j] = coefs[i][j];
                }
            }

            for (unsigned int i = 0; i < indicesList.size(); ++i)
            {
                computeWeightedValue( indicesList[i], ancestors[i], coefs2[i] );
            }

            break;
        }
        case core::topology::POINTSRENUMBERING:
        {
            const sofa::helper::vector<unsigned int> &tab = ( static_cast< const PointsRenumbering * >( *itBegin ) )->getIndexArray();

            renumberValues( tab );
            break;
        }
        default:
            // Ignore events that are not Point-related.
            break;
        };

        ++itBegin;
    }
    //#endif
}

template <class DataTypes>
void MechanicalObject<DataTypes>::replaceValue (const int inputIndex, const int outputIndex)
{
    //const unsigned int maxIndex = std::max(inputIndex, outputIndex);
    const unsigned int maxIndex = inputIndex<outputIndex ? outputIndex : inputIndex;
    const unsigned int vecCoordSize = vectorsCoord.size();
    for (unsigned int i = 0; i < vecCoordSize; i++)
    {
        if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
        {
            VecCoord& vector = *(vectorsCoord[i]->beginEdit());

            if (vector.size() > maxIndex)
                vector[outputIndex] = vector[inputIndex];

            vectorsCoord[i]->endEdit();
        }
    }

    const unsigned int vecDerivSize = vectorsDeriv.size();
    for (unsigned int i = 0; i < vecDerivSize; i++)
    {
        if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
        {
            VecDeriv& vector = *(vectorsDeriv[i]->beginEdit());

            if (vector.size() > maxIndex)
                vector[outputIndex] = vector[inputIndex];

            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::swapValues (const int idx1, const int idx2)
{
    //const unsigned int maxIndex = std::max(idx1, idx2);
    const unsigned int maxIndex = idx1<idx2 ? idx2 : idx1;

    Coord tmp;
    Deriv tmp2;
    unsigned int i;
    for (i=0; i<vectorsCoord.size(); i++)
    {
        if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
        {
            VecCoord& vector = *vectorsCoord[i]->beginEdit();
            if(vector.size() > maxIndex)
            {
                tmp = vector[idx1];
                vector[idx1] = vector[idx2];
                vector[idx2] = tmp;
            }
            vectorsCoord[i]->endEdit();
        }
    }
    for (i=0; i<vectorsDeriv.size(); i++)
    {
        if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
        {
            VecDeriv& vector = *vectorsDeriv[i]->beginEdit();
            if(vector.size() > maxIndex)
            {
                tmp2 = vector[idx1];
                vector[idx1] = vector[idx2];
                vector[idx2] = tmp2;
            }
            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::renumberValues( const sofa::helper::vector< unsigned int > &index )
{
    VecDeriv dtmp;
    VecCoord ctmp;

    for (unsigned int i = 0; i < vectorsCoord.size(); ++i)
    {
        if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
        {
            renumber(vectorsCoord[i]->beginEdit(), &ctmp, index);
            vectorsCoord[i]->endEdit();
        }
    }

    for (unsigned int i = 0; i < vectorsDeriv.size(); ++i)
    {
        if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
        {
            renumber(vectorsDeriv[i]->beginEdit(), &dtmp, index);
            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::resize(const int size)
{
#ifdef SOFA_SMP_NUMA
    if(this->getContext()->getProcessor()!=-1)
        numa_set_preferred(this->getContext()->getProcessor()/2);
#endif

    //if (size!=d_size.getValue())
    {
        if (d_size.getValue() != size)
            d_size.setValue( size );
        for (unsigned int i = 0; i < vectorsCoord.size(); i++)
        {
            if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
            {
                vectorsCoord[i]->beginEdit()->resize(size);
                vectorsCoord[i]->endEdit();
            }
        }

        for (unsigned int i = 0; i < vectorsDeriv.size(); i++)
        {
            if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
            {
                vectorsDeriv[i]->beginEdit()->resize(size);
                vectorsDeriv[i]->endEdit();
            }
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::reserve(const int size)
{
    if (size == 0) return;

    for (unsigned int i = 0; i < vectorsCoord.size(); i++)
    {
        if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
        {
            vectorsCoord[i]->beginEdit()->reserve(size);
            vectorsCoord[i]->endEdit();
        }
    }

    for (unsigned int i = 0; i < vectorsDeriv.size(); i++)
    {
        if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
        {
            vectorsDeriv[i]->beginEdit()->reserve(size);
            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::applyTranslation (const double dx, const double dy, const double dz)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = *this->write(core::VecCoordId::position());

    for (unsigned int i = 0; i < x_wA.size(); i++)
    {
        DataTypes::add(x_wA[i], dx, dy, dz);
    }
}

//Apply Rotation from Euler angles (in degree!)
template <class DataTypes>
void MechanicalObject<DataTypes>::applyRotation (const double rx, const double ry, const double rz)
{
    sofa::defaulttype::Quaternion q =
            helper::Quater< SReal >::createQuaterFromEuler(sofa::defaulttype::Vec< 3, SReal >(rx, ry, rz) * M_PI / 180.0);
    applyRotation(q);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::applyRotation (const defaulttype::Quat q)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = *this->write(core::VecCoordId::position());

    for (unsigned int i = 0; i < x_wA.size(); i++)
    {
        sofa::defaulttype::Vec<3,Real> pos;
        DataTypes::get(pos[0], pos[1], pos[2], x_wA[i]);
        sofa::defaulttype::Vec<3,Real> newposition = q.rotate(pos);
        DataTypes::set(x_wA[i], newposition[0], newposition[1], newposition[2]);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::applyScale(const double sx,const double sy,const double sz)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = this->writePositions();

    const sofa::defaulttype::Vec<3,Real> s((Real)sx, (Real)sy, (Real)sz);
    for (unsigned int i=0; i<x_wA.size(); i++)
    {
        x_wA[i][0] = x_wA[i][0] * s[0];
        x_wA[i][1] = x_wA[i][1] * s[1];
        x_wA[i][2] = x_wA[i][2] * s[2];
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::getIndicesInSpace(sofa::helper::vector<unsigned>& indices, Real xmin, Real xmax, Real ymin, Real ymax, Real zmin, Real zmax) const
{
    helper::ReadAccessor< Data<VecCoord> > x_rA = this->readPositions();

    for( unsigned i=0; i<x_rA.size(); ++i )
    {
        Real x=0.0,y=0.0,z=0.0;
        DataTypes::get(x,y,z,x_rA[i]);
        if( x >= xmin && x <= xmax && y >= ymin && y <= ymax && z >= zmin && z <= zmax )
        {
            indices.push_back(i);
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::computeWeightedValue( const unsigned int i, const sofa::helper::vector< unsigned int >& ancestors, const sofa::helper::vector< double >& coefs)
{
    // HD interpolate position, speed,force,...
    // assume all coef sum to 1.0
    unsigned int j;

    const unsigned int ancestorsSize = ancestors.size();

    helper::vector< Coord > ancestorsCoord(ancestorsSize);
    helper::vector< Deriv > ancestorsDeriv(ancestorsSize);
    helper::vector< Real > ancestorsCoefs(ancestorsSize);

    for (unsigned int k = 0; k < vectorsCoord.size(); k++)
    {
        if (vectorsCoord[k] != NULL && vectorsCoord[k]->isSet())
        {
            VecCoord &vecCoord = *(vectorsCoord[k]->beginEdit());

            if (vecCoord.size() != 0)
            {
                for (j = 0; j < ancestorsSize; ++j)
                {
                    ancestorsCoord[j] = vecCoord[ancestors[j]];
                    ancestorsCoefs[j] = (Real)coefs[j];
                }

                vecCoord[i] = DataTypes::interpolate(ancestorsCoord, ancestorsCoefs);
            }

            vectorsCoord[k]->endEdit();
        }
    }

    for (unsigned int k = 0; k < vectorsDeriv.size(); k++)
    {
        if (vectorsDeriv[k] != NULL && vectorsDeriv[k]->isSet())
        {
            VecDeriv &vecDeriv = *(vectorsDeriv[k]->beginEdit());

            if (vecDeriv.size() != 0)
            {
                for (j = 0; j < ancestorsSize; ++j)
                {
                    ancestorsDeriv[j] = vecDeriv[ancestors[j]];
                    ancestorsCoefs[j] = (Real)coefs[j];
                }

                vecDeriv[i] = DataTypes::interpolate(ancestorsDeriv, ancestorsCoefs);
            }

            vectorsDeriv[k]->endEdit();
        }
    }
}

// Force the position of a point (and force its velocity to zero value)
template <class DataTypes>
void MechanicalObject<DataTypes>::forcePointPosition(const unsigned int i, const sofa::helper::vector< double >& m_x)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = this->writePositions();
    helper::WriteAccessor< Data<VecDeriv> > v_wA = this->writeVelocities();

    DataTypes::set(x_wA[i], m_x[0], m_x[1], m_x[2]);
    DataTypes::set(v_wA[i], (Real) 0.0, (Real) 0.0, (Real) 0.0);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::copyToBaseVector(defaulttype::BaseVector * dest, core::ConstVecId src, unsigned int &offset)
{
    if (src.type == sofa::core::V_COORD)
    {
        helper::ReadAccessor< Data<VecCoord> > vSrc = *this->read(sofa::core::ConstVecCoordId(src));
        constexpr unsigned int coordDim = sofa::defaulttype::DataTypeInfo<Coord>::FinalSize;

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp = (Real)0.0;
                sofa::defaulttype::DataTypeInfo<Coord>::getFinalValue(vSrc[i], j, tmp);
                dest->set(offset + i * coordDim + j, tmp);
            }
        }

        offset += vSrc.size() * coordDim;
    }
    else
    {
        helper::ReadAccessor< Data<VecDeriv> > vSrc = *this->read(sofa::core::ConstVecDerivId(src));
        constexpr unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::FinalSize;

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp;
                sofa::defaulttype::DataTypeInfo<Deriv>::getFinalValue(vSrc[i], j, tmp);
                dest->set(offset + i * derivDim + j, tmp);
            }
        }

        offset += vSrc.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::copyFromBaseVector(sofa::core::VecId dest, const defaulttype::BaseVector *src, unsigned int &offset)
{
    if (dest.type == sofa::core::V_COORD)
    {
        helper::WriteAccessor< Data<VecCoord> > vDest = *this->write(sofa::core::VecCoordId(dest));
        constexpr unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::FinalSize;

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp;
                tmp = (Real)src->element(offset + i * coordDim + j);
                sofa::defaulttype::DataTypeInfo<Coord>::setFinalValue(vDest[i], j, tmp);
            }
        }

        offset += vDest.size() * coordDim;
    }
    else
    {
        helper::WriteAccessor< Data<VecDeriv> > vDest = *this->write(sofa::core::VecDerivId(dest));
        constexpr unsigned int derivDim = sofa::defaulttype::DataTypeInfo<Deriv>::FinalSize;

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp;
                tmp = (Real)src->element(offset + i * derivDim + j);
                defaulttype::DataTypeInfo<Deriv>::setFinalValue(vDest[i], j, tmp);
            }
        }

        offset += vDest.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::addToBaseVector(defaulttype::BaseVector* dest, sofa::core::ConstVecId src, unsigned int &offset)
{
    if (src.type == sofa::core::V_COORD)
    {
        helper::ReadAccessor< Data<VecCoord> > vSrc = *this->read(core::ConstVecCoordId(src));
        constexpr unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::FinalSize;

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Coord>::getFinalValue(vSrc[i], j, tmp);
                dest->add(offset + i * coordDim + j, tmp);
            }
        }

        offset += vSrc.size() * coordDim;
    }
    else
    {
        helper::ReadAccessor< Data<VecDeriv> > vSrc = *this->read(core::ConstVecDerivId(src));
        constexpr unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::FinalSize;

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp;
                defaulttype::DataTypeInfo<Deriv>::getFinalValue(vSrc[i], j, tmp);
                dest->add(offset + i * derivDim + j, tmp);
            }
        }

        offset += vSrc.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::addFromBaseVectorSameSize(sofa::core::VecId dest, const defaulttype::BaseVector *src, unsigned int &offset)
{
    if (dest.type == sofa::core::V_COORD)
    {
        helper::WriteAccessor< Data<VecCoord> > vDest = *this->write(core::VecCoordId(dest));
        constexpr unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::FinalSize;

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Coord>::getFinalValue(vDest[i], j, tmp);
                defaulttype::DataTypeInfo<Coord>::setFinalValue(vDest[i], j, tmp + src->element(offset + i * coordDim + j));
            }
        }

        offset += vDest.size() * coordDim;
    }
    else
    {
        helper::WriteAccessor< Data<VecDeriv> > vDest = *this->write(core::VecDerivId(dest));
        constexpr unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::FinalSize;

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Deriv>::getFinalValue(vDest[i], j, tmp);
                defaulttype::DataTypeInfo<Deriv>::setFinalValue(vDest[i], j, tmp + src->element(offset + i * derivDim + j));
            }
        }

        offset += vDest.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::addFromBaseVectorDifferentSize(sofa::core::VecId dest, const defaulttype::BaseVector* src, unsigned int &offset )
{
    if (dest.type == sofa::core::V_COORD)
    {
        helper::WriteAccessor< Data<VecCoord> > vDest = *this->write(core::VecCoordId(dest));
        constexpr unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::FinalSize;
        const unsigned int nbEntries = src->size()/coordDim;
        for (unsigned int i=0; i<nbEntries; i++)
        {
            for (unsigned int j=0; j<coordDim; ++j)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Coord>::getFinalValue(vDest[i+offset],j,tmp);
                defaulttype::DataTypeInfo<Coord>::setFinalValue(vDest[i+offset],j, tmp + src->element(i*coordDim+j));
            }
        }
        offset += nbEntries;
    }
    else
    {
        helper::WriteAccessor< Data<VecDeriv> > vDest = *this->write(core::VecDerivId(dest));

        constexpr unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::FinalSize;
        const unsigned int nbEntries = src->size()/derivDim;
        for (unsigned int i=0; i<nbEntries; i++)
        {
            for (unsigned int j=0; j<derivDim; ++j)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Deriv>::getFinalValue(vDest[i+offset],j,tmp);
                defaulttype::DataTypeInfo<Deriv>::setFinalValue(vDest[i+offset],j, tmp + src->element(i*derivDim+j));
            }
        }
        offset += nbEntries;
    }


}


template <class DataTypes>
void MechanicalObject<DataTypes>::init()
{
#ifdef SOFA_SMP_NUMA1
    if(this->getContext()->getProcessor()!=-1)
        numa_set_preferred(this->getContext()->getProcessor()/2);
#endif

    if (!l_topology && d_useTopology.getValue())
    {
        l_topology.set( this->getContext()->getActiveMeshTopology() );
    }

    if (l_topology)
    {
        sout << "Initialization with topology " << l_topology->getTypeName() << " " << l_topology->getName() 
             << " ( " << l_topology->getNbPoints() << " points " << (l_topology->hasPos() ? "WITH" : "WITHOUT") << " positions)" << sendl;
    }
    
    // Make sure the sizes of the vectors and the arguments of the scene matches
    const std::vector<std::pair<const std::string, const size_t>> vector_sizes = 
    {
            {x.getName(),                x.getValue().size()},
            {v.getName(),                v.getValue().size()},
            {f.getName(),                f.getValue().size()},
            {externalForces.getName(),   externalForces.getValue().size()},
            {dx.getName(),               dx.getValue().size()},
            {dforce.getName(),           dforce.getValue().size()},
            {xfree.getName(),            xfree.getValue().size()},
            {vfree.getName(),            vfree.getValue().size()},
            {x0.getName(),               x0.getValue().size()},
            {reset_position.getName(),   reset_position.getValue().size()},
            {reset_velocity.getName(),   reset_velocity.getValue().size()}
    };
    // Get the maximum size of all argument's vectors
    size_t minSize = 0, maxSize = 0;
    for (const auto& item : vector_sizes)
    {
        if (item.second == 0)
            continue; // ignore empty vectors
        else if (!minSize)
            minSize = maxSize = item.second; // first non-empty vector
        else if (minSize > item.second)
            minSize = item.second; // new minSize
        else if (maxSize < item.second)
            maxSize = item.second; // new maxSize
    }

    bool copyX0ToX = false;
    //copyXToX0 is not really used for copy, as this is done implicitely afterward.
    //This var shows the symmetry of behaviour between position and rest_position
    bool copyXToX0 = false;

    size_t sizePosition = vectorsCoord[core::ConstVecCoordId::position().index]->getValue().size();
    size_t sizeRestPosition = vectorsCoord[core::ConstVecCoordId::restPosition().index]->getValue().size();

    if (minSize != maxSize)
    {
        // Resize the mechanical object size to match the maximum size of argument's vectors
        if ((size_t)getSize() < maxSize)
        {
            copyX0ToX = sizePosition <  sizeRestPosition;
            copyXToX0 = sizePosition >  sizeRestPosition;
            resize(maxSize);
        }

        // Print a warning if one or more vector don't match the maximum size
        bool allSizeAreEqual = true;
        for (const std::pair<const std::string, const size_t>& vector_size : vector_sizes) 
        {
            const std::string& name = vector_size.first;
            const size_t & size = vector_size.second;
            if (name == x.getName() || name == x0.getName())
                continue;
            //check size>1 due to forceSet in constructor
            //vector can be maxSize if it matches position and rest_position 
            //position and rest_position leads decision on correct vector size
            if (size > 1 &&
                 (   size != maxSize 
                 || (size != sizePosition && !copyX0ToX)
                 || (size != sizeRestPosition && !copyXToX0)))
            {
                allSizeAreEqual = false;
                break;
            }
        }

        //one vector is not the good size
        //special check if position and rest_position have different sizes
        if (!allSizeAreEqual || (copyXToX0 && sizeRestPosition>1) || (copyX0ToX && sizePosition>1))
        {
            std::string message_warning = "One or more of the state vectors passed as argument don't match the size of the others : ";
            for (const std::pair<const std::string, const size_t>& vector_size : vector_sizes) 
            {
                const std::string & name = vector_size.first;
                const size_t & size = vector_size.second;
                //skip size<=1 due to forceSet in constructor
                //if (size <= 1) 
                //    continue;
                message_warning += name + "(size " + std::to_string(size) + ") ";
            }
            message_warning += "max size " + std::to_string(maxSize);
            serr << message_warning << sendl;
        }
    }

    Data<VecCoord>* x_wAData = this->write(sofa::core::VecCoordId::position());
    Data<VecDeriv>* v_wAData = this->write(sofa::core::VecDerivId::velocity());
    VecCoord& x_wA = *x_wAData->beginEdit();
    VecDeriv& v_wA = *v_wAData->beginEdit();

    //case if X0 has been set but not X
    if (copyX0ToX)
    {
        vOp(core::ExecParams::defaultInstance(), core::VecId::position(), core::VecId::restPosition());
    }

    if (x_wA.size() != (std::size_t)d_size.getValue() || v_wA.size() != (std::size_t)d_size.getValue())
    {
        // X and/or V were user-specified
        // copy the last specified velocity to all points

        const unsigned int xSize = x_wA.size();

        helper::WriteAccessor< Data<VecDeriv> > v_wA = *this->write(sofa::core::VecDerivId::velocity());

        if (v_wA.size() >= 1 && v_wA.size() < xSize)
        {
            unsigned int i = v_wA.size();
            Deriv v1 = v_wA[i-1];
            v_wA.resize(xSize);
            while (i < v_wA.size())
                v_wA[i++] = v1;
        }

        resize(xSize > v_wA.size() ? xSize : v_wA.size());
    }
    else if ( x_wA.size() <= 1)
    {
        if (l_topology && l_topology->hasPos() /*&& l_topology->getContext() == this->getContext()*/ )
        {
            int nbp = l_topology->getNbPoints();
            //std::cout<<"Setting "<<nbp<<" points from topology. " << this->getName() << " topo : " << l_topology->getName() <<std::endl;
            // copy the last specified velocity to all points
            if (v_wA.size() >= 1 && v_wA.size() < (unsigned)nbp)
            {
                unsigned int i = v_wA.size();
                Deriv v1 = v_wA[i-1];
                v_wA.resize(nbp);
                while (i < v_wA.size())
                    v_wA[i++] = v1;
            }
            this->resize(nbp);
            for (int i=0; i<nbp; i++)
            {
                x_wA[i] = Coord();
                DataTypes::set(x_wA[i], l_topology->getPX(i), l_topology->getPY(i), l_topology->getPZ(i));
            }
        }
    }

    x_wAData->endEdit();
    v_wAData->endEdit();

    reinit();

    // storing X0 must be done after reinit() that possibly applies transformations
    if( read(core::ConstVecCoordId::restPosition())->getValue().size()!=x_wA.size() )
    {
        // storing X0 from X
        if( restScale.getValue()!=1 )
            vOp(core::ExecParams::defaultInstance(), core::VecId::restPosition(), core::ConstVecId::null(), core::VecId::position(), restScale.getValue());
        else
            vOp(core::ExecParams::defaultInstance(), core::VecId::restPosition(), core::VecId::position());
    }


#if 0// SOFA_HAVE_NEW_TOPOLOGYCHANGES
    x0.createTopologicalEngine(l_topology);
    //x0.setCreateFunction(PointCreationFunction);
    //x0.setDestroyFunction(PointDestroyFunction);
    //x0.setCreateParameter( (void *) this );
    //x0.setDestroyParameter( (void *) this );
    x0.registerTopologicalData();

    x.createTopologicalEngine(l_topology);
    x.setCreateFunction(PointCreationFunction);
    x.setDestroyFunction(PointDestroyFunction);
    x.setCreateParameter( (void *) this );
    x.setDestroyParameter( (void *) this );
    x.registerTopologicalData();

    v.createTopologicalEngine(l_topology);
    v.registerTopologicalData();

    f.createTopologicalEngine(l_topology);
    f.registerTopologicalData();
#endif

    if (rotation2.getValue()[0]!=0.0 || rotation2.getValue()[1]!=0.0 || rotation2.getValue()[2]!=0.0)
    {
        this->applyRotation(rotation2.getValue()[0],rotation2.getValue()[1],rotation2.getValue()[2]);
    }

    if (translation2.getValue()[0]!=0.0 || translation2.getValue()[1]!=0.0 || translation2.getValue()[2]!=0.0)
    {
        this->applyTranslation( translation2.getValue()[0],translation2.getValue()[1],translation2.getValue()[2]);
    }

    m_initialized = true;

    if (f_reserve.getValue() > 0)
        reserve(f_reserve.getValue());

    if (l_topology && getSize() < l_topology->getNbPoints())
    {
        serr << "Resizing from " << getSize() << " to " << l_topology->getNbPoints() << " to match topology points" << sendl;
        resize(l_topology->getNbPoints());
    }

    if(isToPrint.getValue()==true) 
    {
        x.setPersistent(false);
        v.setPersistent(false);
        f.setPersistent(false);
        externalForces.setPersistent(false);
        dx.setPersistent(false);
        dforce.setPersistent(false);
        xfree.setPersistent(false);
        vfree.setPersistent(false);
        x0.setPersistent(false);
        reset_position.setPersistent(false);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::reinit()
{
    Vector3 p0;
    sofa::component::topology::RegularGridTopology *grid;
    this->getContext()->get(grid, sofa::core::objectmodel::BaseContext::Local);
    if (grid) p0 = grid->getP0();

    if (scale.getValue() != Vector3(1.0,1.0,1.0))
    {
        this->applyScale(scale.getValue()[0],scale.getValue()[1],scale.getValue()[2]);
        p0 = p0.linearProduct(scale.getValue());
    }

    if (rotation.getValue()[0]!=0.0 || rotation.getValue()[1]!=0.0 || rotation.getValue()[2]!=0.0)
    {
        this->applyRotation(rotation.getValue()[0],rotation.getValue()[1],rotation.getValue()[2]);

        if (grid)
        {
            this->serr << "Warning ! MechanicalObject initial rotation is not applied to its grid topology"<<this->sendl;
            this->serr << "Regular grid topologies rotations are unsupported."<<this->sendl;
            //  p0 = q.rotate(p0);
        }
    }

    if (translation.getValue()[0]!=0.0 || translation.getValue()[1]!=0.0 || translation.getValue()[2]!=0.0)
    {
        this->applyTranslation( translation.getValue()[0],translation.getValue()[1],translation.getValue()[2]);
        p0 += translation.getValue();
    }


    if (grid)
        grid->setP0(p0);

    internal::reinit(this);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::storeResetState()
{
    // store a reset state only for independent dofs (mapped dofs are deduced from independent dofs)
    if( !isIndependent() ) return;

    // Save initial state for reset button
    vOp(core::ExecParams::defaultInstance(), core::VecId::resetPosition(), core::VecId::position());

    //vOp(VecId::resetVelocity(), VecId::velocity());
    // we only store a resetVelocity if the velocity is not zero
    helper::ReadAccessor< Data<VecDeriv> > v = *this->read(core::VecDerivId::velocity());
    bool zero = true;
    for (unsigned int i=0; i<v.size(); ++i)
    {
        const Deriv& vi = v[i];
        for (unsigned int j=0; j<vi.size(); ++j)
            if (vi[j] != 0) zero = false;
        if (!zero) break;
    }
    if (!zero)
        vOp(core::ExecParams::defaultInstance(), core::VecId::resetVelocity(), core::VecId::velocity());
}

//
// Integration related methods
//
template <class DataTypes>
void MechanicalObject<DataTypes>::reset()
{
    if (!reset_position.isSet())
        return;

    vOp(core::ExecParams::defaultInstance(), core::VecId::position(), core::VecId::resetPosition());

    if (!reset_velocity.isSet())
    {
        vOp(core::ExecParams::defaultInstance(), core::VecId::velocity());
    }
    else
    {
        vOp(core::ExecParams::defaultInstance(), core::VecId::velocity(), core::VecId::resetVelocity());
    }

    if( xfree.isSet() ) vOp(core::ExecParams::defaultInstance(), core::VecId::freePosition(), core::VecId::position());
    if( vfree.isSet() ) vOp(core::ExecParams::defaultInstance(), core::VecId::freeVelocity(), core::VecId::velocity());
}


template <class DataTypes>
void MechanicalObject<DataTypes>::writeVec(core::ConstVecId v, std::ostream &out)
{
    switch (v.type)
    {
    case sofa::core::V_COORD:
        out << this->read(core::ConstVecCoordId(v))->getValue();
        break;
    case sofa::core::V_DERIV:
        out << this->read(core::ConstVecDerivId(v))->getValue();
        break;
    case sofa::core::V_MATDERIV:
        out << this->read(core::ConstMatrixDerivId(v))->getValue();
        break;
    default:
        break;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::readVec(core::VecId v, std::istream &in)
{
    int i = 0;

    switch (v.type)
    {
    case sofa::core::V_COORD:
    {
        Coord coord;
        helper::WriteAccessor< Data< VecCoord > > vec = *this->write(core::VecCoordId(v));

        while (in >> coord)
        {
            if (i >= getSize())
                resize(i+1);
            vec[i++] = coord;
        }

        break;
    }
    case sofa::core::V_DERIV:
    {
        Deriv deriv;
        helper::WriteAccessor< Data< VecDeriv > > vec = *this->write(core::VecDerivId(v));

        while (in >> deriv)
        {
            if (i >= getSize())
                resize(i+1);
            vec[i++] = deriv;
        }

        break;
    }
    case sofa::core::V_MATDERIV:
        //TODO
        break;
    default:
        break;
    }

    if (i < getSize())
        resize(i);
}

template <class DataTypes>
double MechanicalObject<DataTypes>::compareVec(core::ConstVecId v, std::istream &in)
{
    std::string ref,cur;
    getline(in, ref);

    std::ostringstream out;

    switch (v.type)
    {
    case sofa::core::V_COORD:
        out << this->read(core::ConstVecCoordId(v))->getValue();
        break;
    case sofa::core::V_DERIV:
        out << this->read(core::ConstVecDerivId(v))->getValue();
        break;
    case sofa::core::V_MATDERIV:
        out << this->read(core::ConstMatrixDerivId(v))->getValue();
        break;
    default:
        break;
    }

    cur = out.str();

    double error=0;
    std::istringstream compare_ref(ref);
    std::istringstream compare_cur(cur);

    Real value_ref, value_cur;
    unsigned int count=0;
    while (compare_ref >> value_ref && compare_cur >> value_cur )
    {
        error += fabs(value_ref-value_cur);
        count ++;
    }
    if( count == 0 ) return 0; //both vector are empy, so we return 0;

    return error/count;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::writeState(std::ostream& out)
{
    writeVec(core::VecId::position(),out);
    out << " ";
    writeVec(core::VecId::velocity(),out);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::beginIntegration(double /*dt*/)
{
}

template <class DataTypes>
void MechanicalObject<DataTypes>::endIntegration(const core::ExecParams*
                                                 #ifdef SOFA_SMP
                                                 params
                                                 #endif
                                                 /* PARAMS FIRST */, double /*dt*/    )
{
#ifdef SOFA_SMP
    if (params->execMode() == core::ExecParams::EXEC_KAAPI)
    {
        BaseObject::Task<vClear<VecDeriv,Deriv> >
                (this,**defaulttype::getShared(*this->write(VecDerivId::externalForce())),0);
    }
    else
#endif /* SOFA_SMP */
    {
        this->externalForces.beginEdit()->clear();
        this->externalForces.endEdit();
    }
}

template <class DataTypes>
Data<typename MechanicalObject<DataTypes>::VecCoord>* MechanicalObject<DataTypes>::write(core::VecCoordId v)
{
#ifdef SOFA_SMP_NUMA
    //  if(this->getContext()->getProcessor()!=-1)
    //      numa_set_preferred(this->getContext()->getProcessor()/2);
#endif

    if (v.index >= vectorsCoord.size())
    {
        vectorsCoord.resize(v.index + 1, 0);
    }

    if (vectorsCoord[v.index] == NULL)
    {
        vectorsCoord[v.index] = new Data< VecCoord >("VecCoord temporary state vector");
        vectorsCoord[v.index]->setName(v.getName());
        vectorsCoord[v.index]->setGroup("Vector");
        this->addData(vectorsCoord[v.index]);
        if (f_reserve.getValue() > 0)
        {
            vectorsCoord[v.index]->beginWriteOnly()->reserve(f_reserve.getValue());
            vectorsCoord[v.index]->endEdit();
        }
        if( vectorsCoord[v.index]->getValue().size() != (unsigned int)getSize() )
        {
            vectorsCoord[v.index]->beginWriteOnly()->resize( getSize() );
            vectorsCoord[v.index]->endEdit();
        }
    }
    Data<typename MechanicalObject<DataTypes>::VecCoord>* d = vectorsCoord[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
    const typename MechanicalObject<DataTypes>::VecCoord& val = d->getValue();
    if (!val.empty() && val.size() != (unsigned int)this->getSize())
    {
        serr << "Writing to State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
    }
#endif
    return d;
}



template <class DataTypes>
const Data<typename MechanicalObject<DataTypes>::VecCoord>* MechanicalObject<DataTypes>::read(core::ConstVecCoordId v) const
{
    if (v.isNull())
    {
        serr << "Accessing null VecCoord" << sendl;
    }
    if (v.index < vectorsCoord.size() && vectorsCoord[v.index] != NULL)
    {
        const Data<typename MechanicalObject<DataTypes>::VecCoord>* d = vectorsCoord[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
        const typename MechanicalObject<DataTypes>::VecCoord& val = d->getValue();
        if (!val.empty() && val.size() != (unsigned int)this->getSize())
        {
            serr << "Accessing State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
        }
#endif
        return d;
    }
    else
    {
        serr << "Vector " << v << " does not exist" << sendl;
        return NULL;
    }
}


template <class DataTypes>
core::ConstVecCoordId MechanicalObject<DataTypes>::getCoordId(const Data<VecCoord>* d) const
{
    if (d)
    {
        for (unsigned int index = 0; index < (unsigned int)vectorsCoord.size(); ++index)
        {
            if (vectorsCoord[index] == d)
            {
                return core::ConstVecCoordId(index);
            }
        }
    }
    return core::ConstVecCoordId();
}


template <class DataTypes>
Data<typename MechanicalObject<DataTypes>::VecDeriv>* MechanicalObject<DataTypes>::write(core::VecDerivId v)
{
#ifdef SOFA_SMP_NUMA
    //  if(this->getContext()->getProcessor()!=-1)
    //      numa_set_preferred(this->getContext()->getProcessor()/2);
#endif

    if (v.index >= vectorsDeriv.size())
    {
        vectorsDeriv.resize(v.index + 1, 0);
    }

    if (vectorsDeriv[v.index] == NULL)
    {
        vectorsDeriv[v.index] = new Data< VecDeriv >("VecDeriv temporary state vector");
        vectorsDeriv[v.index]->setName(v.getName());
        vectorsDeriv[v.index]->setGroup("Vector");
        this->addData(vectorsDeriv[v.index]);
        if (f_reserve.getValue() > 0)
        {
            vectorsDeriv[v.index]->beginEdit()->reserve(f_reserve.getValue());
            vectorsDeriv[v.index]->endEdit();
        }
        if( vectorsDeriv[v.index]->getValue().size() != (unsigned int)getSize() )
        {
            vectorsDeriv[v.index]->beginEdit()->resize( getSize() );
            vectorsDeriv[v.index]->endEdit();
        }
    }
    Data<typename MechanicalObject<DataTypes>::VecDeriv>* d = vectorsDeriv[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
    const typename MechanicalObject<DataTypes>::VecDeriv& val = d->getValue();
    if (!val.empty() && val.size() != (unsigned int)this->getSize())
    {
        serr << "Writing to State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
    }
#endif
    return d;
}

template <class DataTypes>
const Data<typename MechanicalObject<DataTypes>::VecDeriv>* MechanicalObject<DataTypes>::read(core::ConstVecDerivId v) const
{
    if (v.index < vectorsDeriv.size())
    {
        const Data<typename MechanicalObject<DataTypes>::VecDeriv>* d = vectorsDeriv[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
        if(d!=NULL)
        {
            const typename MechanicalObject<DataTypes>::VecDeriv& val = d->getValue();
            if (!val.empty() && val.size() != (unsigned int)this->getSize())
            {
                serr << "Accessing State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
            }
        }
#endif
        return d;
    }
    else
    {
        serr << "Vector " << v << "does not exist" << sendl;
        return NULL;
    }
}


template <class DataTypes>
core::ConstVecDerivId MechanicalObject<DataTypes>::getDerivId(const Data<VecDeriv>* d) const
{
    if (d)
    {
        for (unsigned int index = 0; index < (unsigned int)vectorsDeriv.size(); ++index)
        {
            if (vectorsDeriv[index] == d)
            {
                return core::ConstVecDerivId(index);
            }
        }
    }
    return core::ConstVecDerivId();
}


template <class DataTypes>
Data<typename MechanicalObject<DataTypes>::MatrixDeriv>* MechanicalObject<DataTypes>::write(core::MatrixDerivId v)
{
#ifdef SOFA_SMP_NUMA
    //  if(this->getContext()->getProcessor()!=-1)
    //      numa_set_preferred(this->getContext()->getProcessor()/2);
#endif

    if (v.index >= vectorsMatrixDeriv.size())
    {
        vectorsMatrixDeriv.resize(v.index + 1, 0);
    }

    if (vectorsMatrixDeriv[v.index] == NULL)
    {
        vectorsMatrixDeriv[v.index] = new Data< MatrixDeriv >("MatrixDeriv temporary state vector");
        vectorsMatrixDeriv[v.index]->setName(v.getName());
        vectorsMatrixDeriv[v.index]->setGroup("Vector");
        this->addData(vectorsMatrixDeriv[v.index]);
    }

    return vectorsMatrixDeriv[v.index];
}

template <class DataTypes>
const Data<typename MechanicalObject<DataTypes>::MatrixDeriv>* MechanicalObject<DataTypes>::read(core::ConstMatrixDerivId v) const
{
    if (v.index < vectorsMatrixDeriv.size())
        return vectorsMatrixDeriv[v.index];
    else
    {
        serr << "Vector " << v << "does not exist" << sendl;
        return NULL;
    }
}

template <class DataTypes>
core::ConstMatrixDerivId MechanicalObject<DataTypes>::getMatrixDerivId(const Data<MatrixDeriv>* d) const
{
    if (d)
    {
        for (unsigned int index = 0; index < (unsigned int)vectorsMatrixDeriv.size(); ++index)
        {
            if (vectorsMatrixDeriv[index] == d)
            {
                return core::ConstMatrixDerivId(index);
            }
        }
    }
    return core::ConstMatrixDerivId();
}

template <class DataTypes>
void MechanicalObject<DataTypes>::setVecCoord(unsigned int index, Data< VecCoord > *v)
{
    if (index >= vectorsCoord.size())
    {
        vectorsCoord.resize(index + 1, 0);
    }

    vectorsCoord[index] = v;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::setVecDeriv(unsigned int index, Data< VecDeriv > *v)
{
    if (index >= vectorsDeriv.size())
    {
        vectorsDeriv.resize(index + 1, 0);
    }

    vectorsDeriv[index] = v;
}


template <class DataTypes>
void MechanicalObject<DataTypes>::setVecMatrixDeriv(unsigned int index, Data < MatrixDeriv > *m)
{
    if (index >= vectorsMatrixDeriv.size())
    {
        vectorsMatrixDeriv.resize(index + 1, 0);
    }

    vectorsMatrixDeriv[index] = m;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAvail(const core::ExecParams* /* params */ /* PARAMS FIRST */, core::VecCoordId& v)
{
    for (unsigned int i = v.index; i < vectorsCoord.size(); ++i)
    {
        if (vectorsCoord[i] && vectorsCoord[i]->isSet())
            v.index = i+1;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAvail(const core::ExecParams* /* params */ /* PARAMS FIRST */, core::VecDerivId& v)
{
    for (unsigned int i = v.index; i < vectorsDeriv.size(); ++i)
    {
        if (vectorsDeriv[i] && vectorsDeriv[i]->isSet())
            v.index = i+1;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAvail(const core::ExecParams*, core::MatrixDerivId& v)
{
    for (unsigned int i = v.index; i < vectorsMatrixDeriv.size(); ++i)
    {
        if (vectorsMatrixDeriv[i] && vectorsMatrixDeriv[i]->isSet())
            v.index = i+1;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAlloc(const core::ExecParams* params /* PARAMS FIRST */, core::VecCoordId v)
{
#ifdef SOFA_SMP_NUMA
    if(this->getContext()->getProcessor()!=-1)
        numa_set_preferred(this->getContext()->getProcessor()/2);
#endif
    if (v.index >= sofa::core::VecCoordId::V_FIRST_DYNAMIC_INDEX)
    {
        Data<VecCoord>* vec_d = this->write(v);
        vec_d->beginEdit(params)->resize(d_size.getValue());
        vec_d->endEdit(params);
#ifdef SOFA_SMP
        if (params->execMode() == core::ExecParams::EXEC_KAAPI)
        {
            BaseObject::Task< VecInitResize < VecCoord > >(this,**defaulttype::getShared(*this->write(VecCoordId(v))), this->d_size.getValue());
        }
#endif /* SOFA_SMP */
    }

    //vOp(v); // clear vector
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAlloc(const core::ExecParams* params /* PARAMS FIRST */, core::VecDerivId v)
{
#ifdef SOFA_SMP_NUMA
    if(this->getContext()->getProcessor()!=-1)
        numa_set_preferred(this->getContext()->getProcessor()/2);
#endif

    if (v.index >= sofa::core::VecDerivId::V_FIRST_DYNAMIC_INDEX)
    {
        Data<VecDeriv>* vec_d = this->write(v);
        vec_d->beginEdit(params)->resize(d_size.getValue());
        vec_d->endEdit(params);
#ifdef SOFA_SMP
        if (params->execMode() == core::ExecParams::EXEC_KAAPI)
        {
            BaseObject::Task < VecInitResize < VecDeriv > >(this,**defaulttype::getShared(*this->write(VecDerivId(v))), this->d_size.getValue());
        }
#endif /* SOFA_SMP */
    }

    //vOp(v); // clear vector
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAlloc(const core::ExecParams* /*params*/, core::MatrixDerivId v)
{
    if (v.index >= sofa::core::MatrixDerivId::V_FIRST_DYNAMIC_INDEX)
    {
        this->write(v);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vRealloc(const core::ExecParams* params /* PARAMS FIRST */, core::VecCoordId v)
{
    Data<VecCoord>* vec_d = this->write(v);

    if ( !vec_d->isSet(params) )
    {
        vec_d->beginEdit(params)->resize(d_size.getValue());
        vec_d->endEdit(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vRealloc(const core::ExecParams* params /* PARAMS FIRST */, core::VecDerivId v)
{
    Data<VecDeriv>* vec_d = this->write(v);

    if ( !vec_d->isSet(params) /*&& v.index >= sofa::core::VecDerivId::V_FIRST_DYNAMIC_INDEX*/ )
    {
        vec_d->beginEdit(params)->resize(d_size.getValue());
        vec_d->endEdit(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vRealloc(const core::ExecParams* params, core::MatrixDerivId v)
{
    Data<MatrixDeriv>* vec_d = this->write(v);

    if (!vec_d->isSet(params) )
    {
        vec_d->beginEdit(params);
        vec_d->endEdit(params);
    }
}


template <class DataTypes>
void MechanicalObject<DataTypes>::vFree(const core::ExecParams* params /* PARAMS FIRST */, core::VecCoordId vId)
{
    if (vId.index >= sofa::core::VecCoordId::V_FIRST_DYNAMIC_INDEX)
    {
        Data< VecCoord >* vec_d = this->write(vId);

        VecCoord *vec = vec_d->beginEdit(params);
        vec->resize(0);
        vec_d->endEdit(params);

        vec_d->unset(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vFree(const core::ExecParams* params /* PARAMS FIRST */, core::VecDerivId vId)
{
    if (vId.index >= sofa::core::VecDerivId::V_FIRST_DYNAMIC_INDEX)
    {
        Data< VecDeriv >* vec_d = this->write(vId);

        VecDeriv *vec = vec_d->beginEdit(params);
        vec->resize(0);
        vec_d->endEdit(params);

        vec_d->unset(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vFree(const core::ExecParams* params, core::MatrixDerivId vId)
{
    if (vId.index >= sofa::core::VecDerivId::V_FIRST_DYNAMIC_INDEX)
    {
        Data< MatrixDeriv >* vec_d = this->write(vId);

        MatrixDeriv *vec = vec_d->beginEdit(params);
        vec->clear();
        vec_d->endEdit(params);
        vec_d->unset(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vInit(const core::ExecParams* params /* PARAMS FIRST */
                                        , core::VecCoordId vId
                                        , core::ConstVecCoordId vSrcId)
{
    Data< VecCoord >* vec_d = this->write(vId);
    const Data<VecCoord>* vec_s = this->read(vSrcId);

    if (vec_d != nullptr && vec_s != nullptr)
    {
        auto dest = sofa::helper::write(*vec_d, params);
        auto src = sofa::helper::read(*vec_s, params);

        const bool needInit = !vec_d->isSet(params) || dest.size() != src.size();
        
        if (needInit)
        {
            vec_d->forceSet(params);

            if (vSrcId != core::ConstVecCoordId::null())
                vOp(params, vId, vSrcId);
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vInit(const core::ExecParams* params /* PARAMS FIRST */,
                                        core::VecDerivId vId,
                                        core::ConstVecDerivId vSrcId)
{
    Data< VecDeriv >* vec_d = this->write(vId);
    const Data<VecDeriv>* vec_s = this->read(vSrcId);

    if (vec_d != nullptr && vec_s != nullptr)
    {
        auto dest = sofa::helper::write(*vec_d, params);
        auto src  = sofa::helper::read(*vec_s, params);

        const bool needInit = !vec_d->isSet(params) || dest.size() != src.size();

        if (needInit)
        {
            vec_d->forceSet(params);

            if (vSrcId != core::ConstVecDerivId::null())
                vOp(params, vId, vSrcId);
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vInit(const core::ExecParams* params, core::MatrixDerivId vId, core::ConstMatrixDerivId vSrcId)
{
    Data< MatrixDeriv >* vec_d     = this->write(vId);

    if (!vec_d->isSet(params))
    {
        vec_d->forceSet(params);

        if (vSrcId != core::ConstMatrixDerivId::null() )
        {
            const Data<MatrixDeriv>* vec_s = this->read(vSrcId);
            if (vec_s != nullptr && vec_s->isSet(params) )
            {
                MatrixDeriv* dest      = vec_d->beginEdit(params);
                const MatrixDeriv& src = vec_s->getValue(params);
                *dest = src;
                vec_d->endEdit(params);
            }
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vOp(const core::ExecParams* params /* PARAMS FIRST */, core::VecId v,
                                      core::ConstVecId a,
                                      core::ConstVecId b, double f)
{
#ifdef SOFA_SMP
    if (params->execMode() == core::ExecParams::EXEC_KAAPI)
    {
        vOp(params /* PARAMS FIRST */, v, a, b, f, NULL);
    }
    else
#endif
    {

        if(v.isNull())
        {
            // ERROR
            serr << this->getName() << " Invalid vOp operation 1 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
            return;
        }
        if (a.isNull())
        {
            if (b.isNull())
            {
                // v = 0
                if (v.type == sofa::core::V_COORD)
                {
                    helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                    vv.resize(this->d_size.getValue());
                    for (unsigned int i=0; i<vv.size(); i++)
                        vv[i] = Coord();
                }
                else
                {
                    helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                    vv.resize(this->d_size.getValue());
                    for (unsigned int i=0; i<vv.size(); i++)
                        vv[i] = Deriv();
                }
            }
            else
            {
                if (b.type != v.type)
                {
                    // ERROR
                    serr << "Invalid vOp operation 2 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
                    return;
                }
                if (v == b)
                {
                    // v *= f
                    if (v.type == sofa::core::V_COORD)
                    {
                        helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                        for (unsigned int i=0; i<vv.size(); i++)
                            vv[i] *= (Real)f;
                    }
                    else
                    {
                        helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                        for (unsigned int i=0; i<vv.size(); i++)
                            vv[i] *= (Real)f;
                    }
                }
                else
                {
                    // v = b*f
                    if (v.type == sofa::core::V_COORD)
                    {
                        helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                        helper::ReadAccessor< Data<VecCoord> > vb( params, *this->read(core::ConstVecCoordId(b)) );
                        vv.resize(vb.size());
                        for (unsigned int i=0; i<vv.size(); i++)
                            vv[i] = vb[i] * (Real)f;
                    }
                    else
                    {
                        helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                        helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );
                        vv.resize(vb.size());
                        for (unsigned int i=0; i<vv.size(); i++)
                            vv[i] = vb[i] * (Real)f;
                    }
                }
            }
        }
        else
        {
            if (a.type != v.type)
            {
                // ERROR
                serr << "Invalid vOp operation 3 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
                return;
            }
            if (b.isNull())
            {
                // v = a
                if (v.type == sofa::core::V_COORD)
                {
                    helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                    helper::ReadAccessor< Data<VecCoord> > va( params, *this->read(core::ConstVecCoordId(a)) );
                    vv.resize(va.size());
                    for (unsigned int i=0; i<vv.size(); i++)
                        vv[i] = va[i];
                }
                else
                {
                    helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                    helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(a)) );
                    vv.resize(va.size());
                    for (unsigned int i=0; i<vv.size(); i++)
                        vv[i] = va[i];
                }
            }
            else
            {
                if (v == a)
                {
                    if (f==1.0)
                    {
                        // v += b
                        if (v.type == sofa::core::V_COORD)
                        {
                            helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                            if (b.type == sofa::core::V_COORD)
                            {
                                helper::ReadAccessor< Data<VecCoord> > vb( params, *this->read(core::ConstVecCoordId(b)) );

                                if (vb.size() > vv.size())
                                    vv.resize(vb.size());

                                for (unsigned int i=0; i<vb.size(); i++)
                                    vv[i] += vb[i];
                            }
                            else
                            {
                                helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );

                                if (vb.size() > vv.size())
                                    vv.resize(vb.size());

                                for (unsigned int i=0; i<vb.size(); i++)
                                    vv[i] += vb[i];
                            }
                        }
                        else if (b.type == sofa::core::V_DERIV)
                        {
                            helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                            helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );

                            if (vb.size() > vv.size())
                                vv.resize(vb.size());

                            for (unsigned int i=0; i<vb.size(); i++)
                                vv[i] += vb[i];
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation 4 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
                            return;
                        }
                    }
                    else
                    {
                        // v += b*f
                        if (v.type == sofa::core::V_COORD)
                        {
                            helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                            if (b.type == sofa::core::V_COORD)
                            {
                                helper::ReadAccessor< Data<VecCoord> > vb( params, *this->read(core::ConstVecCoordId(b)) );

                                if (vb.size() > vv.size())
                                    vv.resize(vb.size());

                                for (unsigned int i=0; i<vb.size(); i++)
                                    vv[i] += vb[i]*(Real)f;
                            }
                            else
                            {
                                helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );

                                if (vb.size() > vv.size())
                                    vv.resize(vb.size());

                                for (unsigned int i=0; i<vb.size(); i++)
                                    vv[i] += vb[i]*(Real)f;
                            }
                        }
                        else if (b.type == sofa::core::V_DERIV)
                        {
                            helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                            helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );

                            if (vb.size() > vv.size())
                                vv.resize(vb.size());

                            for (unsigned int i=0; i<vb.size(); i++)
                                vv[i] += vb[i]*(Real)f;
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation 5 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
                            return;
                        }
                    }
                }
                else if (v == b)
                {
                    if (f==1.0)
                    {
                        // v += a
                        if (v.type == sofa::core::V_COORD)
                        {
                            helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                            if (a.type == sofa::core::V_COORD)
                            {
                                helper::ReadAccessor< Data<VecCoord> > va( params, *this->read(core::ConstVecCoordId(a)) );

                                if (va.size() > vv.size())
                                    vv.resize(va.size());

                                for (unsigned int i=0; i<va.size(); i++)
                                    vv[i] += va[i];
                            }
                            else
                            {
                                helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(a)) );

                                if (va.size() > vv.size())
                                    vv.resize(va.size());

                                for (unsigned int i=0; i<va.size(); i++)
                                    vv[i] += va[i];
                            }
                        }
                        else if (a.type == sofa::core::V_DERIV)
                        {
                            helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                            helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(a)) );

                            if (va.size() > vv.size())
                                vv.resize(va.size());

                            for (unsigned int i=0; i<va.size(); i++)
                                vv[i] += va[i];
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation 6 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
                            return;
                        }
                    }
                    else
                    {
                        // v = a+v*f
                        if (v.type == sofa::core::V_COORD)
                        {
                            helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                            helper::ReadAccessor< Data<VecCoord> > va( params, *this->read(core::ConstVecCoordId(a)) );
                            vv.resize(va.size());
                            for (unsigned int i=0; i<vv.size(); i++)
                            {
                                vv[i] *= (Real)f;
                                vv[i] += va[i];
                            }
                        }
                        else
                        {
                            helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                            helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(a)) );
                            vv.resize(va.size());
                            for (unsigned int i=0; i<vv.size(); i++)
                            {
                                vv[i] *= (Real)f;
                                vv[i] += va[i];
                            }
                        }
                    }
                }
                else
                {
                    if (f==1.0)
                    {
                        // v = a+b
                        if (v.type == sofa::core::V_COORD)
                        {
                            helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                            helper::ReadAccessor< Data<VecCoord> > va( params, *this->read(core::ConstVecCoordId(a)) );
                            vv.resize(va.size());
                            if (b.type == sofa::core::V_COORD)
                            {
                                helper::ReadAccessor< Data<VecCoord> > vb( params, *this->read(core::ConstVecCoordId(b)) );
                                for (unsigned int i=0; i<vv.size(); i++)
                                {
                                    vv[i] = va[i];
                                    vv[i] += vb[i];
                                }
                            }
                            else
                            {
                                helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );
                                for (unsigned int i=0; i<vv.size(); i++)
                                {
                                    vv[i] = va[i];
                                    vv[i] += vb[i];
                                }
                            }
                        }
                        else if (b.type == sofa::core::V_DERIV)
                        {
                            helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                            helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(a)) );
                            helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );
                            vv.resize(va.size());
                            for (unsigned int i=0; i<vv.size(); i++)
                            {
                                vv[i] = va[i];
                                vv[i] += vb[i];
                            }
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation 7 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
                            return;
                        }
                    }
                    else
                    {
                        // v = a+b*f
                        if (v.type == sofa::core::V_COORD)
                        {
                            helper::WriteAccessor< Data<VecCoord> > vv( params, *this->write(core::VecCoordId(v)) );
                            helper::ReadAccessor< Data<VecCoord> > va( params, *this->read(core::ConstVecCoordId(a)) );
                            vv.resize(va.size());
                            if (b.type == sofa::core::V_COORD)
                            {
                                helper::ReadAccessor< Data<VecCoord> > vb( params, *this->read(core::ConstVecCoordId(b)) );
                                for (unsigned int i=0; i<vv.size(); i++)
                                {
                                    vv[i] = va[i];
                                    vv[i] += vb[i]*(Real)f;
                                }
                            }
                            else
                            {
                                helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );
                                for (unsigned int i=0; i<vv.size(); i++)
                                {
                                    vv[i] = va[i];
                                    vv[i] += vb[i]*(Real)f;
                                }
                            }
                        }
                        else if (b.type == sofa::core::V_DERIV)
                        {
                            helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(v)) );
                            helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(a)) );
                            helper::ReadAccessor< Data<VecDeriv> > vb( params, *this->read(core::ConstVecDerivId(b)) );
                            vv.resize(va.size());
                            for (unsigned int i=0; i<vv.size(); i++)
                            {
                                vv[i] = va[i];
                                vv[i] += vb[i]*(Real)f;
                            }
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation 8 ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
                            return;
                        }
                    }
                }
            }
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vMultiOp(const core::ExecParams* params /* PARAMS FIRST */, const VMultiOp& ops)
{
    // optimize common integration case: v += a*dt, x += v*dt
    if (ops.size() == 2
            && ops[0].second.size() == 2
            && ops[0].first.getId(this) == ops[0].second[0].first.getId(this)
            && ops[0].first.getId(this).type == sofa::core::V_DERIV
            && ops[0].second[1].first.getId(this).type == sofa::core::V_DERIV
            && ops[1].second.size() == 2
            && ops[1].first.getId(this) == ops[1].second[0].first.getId(this)
            && ops[0].first.getId(this) == ops[1].second[1].first.getId(this)
            && ops[1].first.getId(this).type == sofa::core::V_COORD)
    {
        helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(ops[0].second[1].first.getId(this))) );
        helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(ops[0].first.getId(this))) );
        helper::WriteAccessor< Data<VecCoord> > vx( params, *this->write(core::VecCoordId(ops[1].first.getId(this))) );

        const unsigned int n = vx.size();
        const Real f_v_v = (Real)(ops[0].second[0].second);
        const Real f_v_a = (Real)(ops[0].second[1].second);
        const Real f_x_x = (Real)(ops[1].second[0].second);
        const Real f_x_v = (Real)(ops[1].second[1].second);

        if (f_v_v == 1.0 && f_x_x == 1.0) // very common case
        {
            if (f_v_a == 1.0) // used by euler implicit and other integrators that directly computes a*dt
            {
                for (unsigned int i=0; i<n; ++i)
                {
                    vv[i] += va[i];
                    vx[i] += vv[i]*f_x_v;
                }
            }
            else
            {
                for (unsigned int i=0; i<n; ++i)
                {
                    vv[i] += va[i]*f_v_a;
                    vx[i] += vv[i]*f_x_v;
                }
            }
        }
        else if (f_x_x == 1.0) // some damping is applied to v
        {
            for (unsigned int i=0; i<n; ++i)
            {
                vv[i] *= f_v_v;
                vv[i] += va[i];
                vx[i] += vv[i]*f_x_v;
            }
        }
        else // general case
        {
            for (unsigned int i=0; i<n; ++i)
            {
                vv[i] *= f_v_v;
                vv[i] += va[i]*f_v_a;
                vx[i] *= f_x_x;
                vx[i] += vv[i]*f_x_v;
            }
        }
    }
    else if(ops.size()==2 //used in the ExplicitBDF solver only (Electrophysiology)
            && ops[0].second.size()==1
            && ops[0].second[0].second == 1.0
            && ops[1].second.size()==3
            )
    {
        helper::ReadAccessor< Data<VecCoord> > v11( params, *this->read(core::ConstVecCoordId(ops[0].second[0].first.getId(this))) );
        helper::ReadAccessor< Data<VecCoord> > v21( params, *this->read(core::ConstVecCoordId(ops[1].second[0].first.getId(this))) );
        helper::ReadAccessor< Data<VecCoord> > v22( params, *this->read(core::ConstVecCoordId(ops[1].second[1].first.getId(this))) );
        helper::ReadAccessor< Data<VecDeriv> > v23( params, *this->read(core::ConstVecDerivId(ops[1].second[2].first.getId(this))) );

        helper::WriteAccessor< Data<VecCoord> > previousPos( params, *this->write(core::VecCoordId(ops[0].first.getId(this))) );
        helper::WriteAccessor< Data<VecCoord> > newPos( params, *this->write(core::VecCoordId(ops[1].first.getId(this))) );

        const unsigned int n = v11.size();
        const Real f_1 = (Real)(ops[1].second[0].second);
        const Real f_2 = (Real)(ops[1].second[1].second);
        const Real f_3 = (Real)(ops[1].second[2].second);

        for (unsigned int i=0; i<n; ++i)
        {
            previousPos[i] = v11[i];
            newPos[i]  = v21[i]*f_1;
            newPos[i] += v22[i]*f_2;
            newPos[i] += v23[i]*f_3;
        }
    }
    else // no optimization for now for other cases
        Inherited::vMultiOp(params, ops);
}

template <class T> inline void clear( T& t )
{
    t.clear();
}

template<> inline void clear( float& t )
{
    t=0;
}

template<> inline void clear( double& t )
{
    t=0;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vThreshold(core::VecId v, SReal t)
{
    if( v.type==sofa::core::V_DERIV)
    {
        helper::WriteAccessor< Data<VecDeriv> > vv = *this->write(core::VecDerivId(v));
        Real t2 = (Real)(t*t);
        for (unsigned int i=0; i<vv.size(); i++)
        {
            if( vv[i]*vv[i] < t2 )
                clear(vv[i]);
        }
    }
    else
    {
        serr<<"vThreshold does not apply to coordinate vectors"<<sendl;
    }
}

template <class DataTypes>
double MechanicalObject<DataTypes>::vDot(const core::ExecParams* params /* PARAMS FIRST */, core::ConstVecId a, core::ConstVecId b)
{
    Real r = 0.0;

    if (a.type == sofa::core::V_COORD && b.type == sofa::core::V_COORD)
    {
        const VecCoord &va = this->read(core::ConstVecCoordId(a))->getValue(params);
        const VecCoord &vb = this->read(core::ConstVecCoordId(b))->getValue(params);

        for (unsigned int i=0; i<va.size(); i++)
        {
            r += va[i] * vb[i];
        }
    }
    else if (a.type == sofa::core::V_DERIV && b.type == sofa::core::V_DERIV)
    {
        const VecDeriv &va = this->read(core::ConstVecDerivId(a))->getValue(params);
        const VecDeriv &vb = this->read(core::ConstVecDerivId(b))->getValue(params);

        for (unsigned int i=0; i<va.size(); i++)
        {
            r += va[i] * vb[i];
        }
    }
    else
    {
        serr << "Invalid dot operation ("<<a<<','<<b<<")" << sendl;
    }

    return r;
}

typedef std::size_t nat;

template <class DataTypes>
double MechanicalObject<DataTypes>::vSum(const core::ExecParams* params, core::ConstVecId a, unsigned l)
{
    Real r = 0.0;

    if (a.type == sofa::core::V_COORD )
    {
        serr << "Invalid vSum operation: can not compute the sum of V_Coord terms in vector "<< a << sendl;
    }
    else if (a.type == sofa::core::V_DERIV)
    {
        const VecDeriv &va = this->read(core::ConstVecDerivId(a))->getValue(params);

        if( l==0 ) for (nat i=0; i<va.size(); i++)
        {
            for(unsigned j=0; j<DataTypes::deriv_total_size; j++)
                if ( fabs(va[i][j])>r) r=fabs(va[i][j]);
        }
        else for (unsigned int i=0; i<va.size(); i++)
        {
            for(unsigned j=0; j<DataTypes::deriv_total_size; j++)
                r += (Real) exp(va[i][j]/l);
        }
    }
    else
    {
        serr << "Invalid vSum operation ("<<a<<")" << sendl;
    }

    return r;
}

template <class DataTypes>
double MechanicalObject<DataTypes>::vMax(const core::ExecParams* params, core::ConstVecId a )
{
    Real r = 0.0;

    if (a.type == sofa::core::V_COORD )
    {
        serr << "Invalid vMax operation: can not compute the max of V_Coord terms in vector "<< a << sendl;
    }
    else if (a.type == sofa::core::V_DERIV)
    {
        const VecDeriv &va = this->read(core::ConstVecDerivId(a))->getValue(params);

        for (nat i=0; i<va.size(); i++)
        {
            for(unsigned j=0; j<DataTypes::deriv_total_size; j++)
                if (fabs(va[i][j])>r) r=fabs(va[i][j]);
        }
    }
    else
    {
        serr << "Invalid vMax operation ("<<a<<")" << sendl;
    }

    return r;
}

template <class DataTypes>
size_t MechanicalObject<DataTypes>::vSize(const core::ExecParams* params /* PARAMS FIRST */, core::ConstVecId v)
{
    if (v.type == sofa::core::V_COORD)
    {
        const VecCoord &vv = this->read(core::ConstVecCoordId(v))->getValue(params);
        return vv.size() * Coord::total_size;
    }
    else if (v.type == sofa::core::V_DERIV)
    {
        const VecDeriv &vv = this->read(core::ConstVecDerivId(v))->getValue(params);
        return vv.size() * Deriv::total_size;
    }
    else
    {
        serr << "Invalid size operation ("<<v<<")" << sendl;
        return 0;
    }
}




#ifndef SOFA_SMP
template <class DataTypes>
void MechanicalObject<DataTypes>::printDOF( core::ConstVecId v, std::ostream& out, int firstIndex, int range) const
{
    const unsigned int size=this->getSize();
    if ((unsigned int) (abs(firstIndex)) >= size) return;
    const unsigned int first=((firstIndex>=0)?firstIndex:size+firstIndex);
    const unsigned int max=( ( (range >= 0) && ( (range+first)<size) ) ? (range+first):size);

    if( v.type==sofa::core::V_COORD)
    {
        const Data<VecCoord>* d_x = this->read(core::ConstVecCoordId(v));
        if (d_x == NULL) return;
        helper::ReadAccessor< Data<VecCoord> > x = *d_x;

        if (x.empty())
            return;

        for (unsigned i=first; i<max; ++i)
        {
            out << x[i];
            if (i != max-1)
                out <<" ";
        }
    }
    else if( v.type==sofa::core::V_DERIV)
    {
        const Data<VecDeriv>* d_x = this->read(core::ConstVecDerivId(v));
        if (d_x == NULL) return;
        helper::ReadAccessor< Data<VecDeriv> > x = *d_x;

        if (x.empty())
            return;

        for (unsigned i=first; i<max; ++i)
        {
            out << x[i];
            if (i != max-1)
                out <<" ";
        }
    }
    else
        out<<"MechanicalObject<DataTypes>::printDOF, unknown v.type = "<<v.type<<std::endl;
}
#endif

template <class DataTypes>
unsigned MechanicalObject<DataTypes>::printDOFWithElapsedTime(core::ConstVecId v, unsigned count, unsigned time, std::ostream& out) const
{
    if (v.type == sofa::core::V_COORD)
    {
        const Data<VecCoord>* d_x = this->read(core::ConstVecCoordId(v));
        if (d_x == NULL) return 0;
        helper::ReadAccessor< Data<VecCoord> > x = *d_x;

        for (unsigned i = 0; i < x.size(); ++i)
        {
            out << count + i << "\t" << time << "\t" << x[i] << std::endl;
        }
        out << std::endl << std::endl;

        return x.size();
    }
    else if (v.type == sofa::core::V_DERIV)
    {
        const Data<VecDeriv>* d_x = this->read(core::ConstVecDerivId(v));
        if (d_x == NULL) return 0;
        helper::ReadAccessor< Data<VecDeriv> > x = *d_x;

        for (unsigned i = 0; i < x.size(); ++i)
        {
            out << count + i << "\t" << time << "\t" << x[i] << std::endl;
        }
        out << std::endl << std::endl;

        return x.size();
    }
    else
        out << "MechanicalObject<DataTypes>::printDOFWithElapsedTime, unknown v.type = " << v.type << std::endl;

    return 0;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::resetConstraint(const core::ConstraintParams* cParams)
{
    Data<MatrixDeriv>& c_data = *this->write(cParams->j().getId(this));
    MatrixDeriv *c = c_data.beginEdit(cParams);
    c->clear();
    c_data.endEdit(cParams);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::getConstraintJacobian(const core::ConstraintParams* cParams, sofa::defaulttype::BaseMatrix* J,unsigned int & off)
{
    // Compute J
    const size_t N = Deriv::size();
    const MatrixDeriv& c = cParams->readJ(this)->getValue(cParams);

    MatrixDerivRowConstIterator rowItEnd = c.end();

    for (MatrixDerivRowConstIterator rowIt = c.begin(); rowIt != rowItEnd; ++rowIt)
    {
        const int cid = rowIt.index();

        MatrixDerivColConstIterator colItEnd = rowIt.end();

        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt) {
            const unsigned int dof = colIt.index();
            const Deriv n = colIt.val();

            for (unsigned int r = 0; r < N; ++r) {
                J->add(cid, off + dof * N + r, n[r]);
            }
        }
    }

    off += this->getSize() * N;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::renumberConstraintId(const sofa::helper::vector<unsigned>& /*renumbering*/)
{
    this->serr << "MechanicalObject<DataTypes>::renumberConstraintId not implemented in the MatrixDeriv constraint API" << this->sendl;
}

template <class DataTypes>
std::list< core::behavior::BaseMechanicalState::ConstraintBlock > MechanicalObject<DataTypes>::constraintBlocks( const std::list<unsigned int> &indices) const
{
    constexpr unsigned int dimensionDeriv = defaulttype::DataTypeInfo< Deriv >::FinalSize;
    assert( indices.size() > 0 );
    assert( dimensionDeriv > 0 );

    // simple column/block map

    typedef sofa::component::linearsolver::SparseMatrix<SReal> matrix_t;
    // typedef sofa::defaulttype::FullMatrix<SReal> matrix_t;

    typedef std::map<unsigned int, matrix_t* > blocks_t;
    blocks_t blocks;

    // for all row indices
    typedef std::list<unsigned int> indices_t;

    const MatrixDeriv& constraints = c.getValue();

    unsigned int block_row = 0;
    for (indices_t::const_iterator rowIt = indices.begin(); rowIt != indices.end(); ++rowIt, ++block_row)
    {
        MatrixDerivRowConstIterator rowIterator = constraints.readLine(*rowIt);

        if (rowIterator != constraints.end())
        {
            MatrixDerivColConstIterator chunk = rowIterator.begin();
            MatrixDerivColConstIterator chunkEnd = rowIterator.end();

            for( ; chunk != chunkEnd ; chunk++)
            {
                const unsigned int column = chunk.index();
                if( blocks.find( column ) == blocks.end() )
                {
                    // nope: let's create it
                    matrix_t* mat = new matrix_t(indices.size(), dimensionDeriv);
                    blocks[column] = mat;
                }

                // now it's created no matter what \o/
                matrix_t& block = *blocks[column];

                // fill the right line of the block
                const Deriv curValue = chunk.val();

                for (unsigned int i = 0; i < dimensionDeriv; ++i)
                {
                    SReal value;
                    defaulttype::DataTypeInfo< Deriv >::getFinalValue(curValue, i, value);
                    block.set(block_row, i, value);
                }
            }
        }
    }

    // put all blocks in a list and we're done
    std::list<ConstraintBlock> res;
    for(blocks_t::const_iterator b = blocks.begin(); b != blocks.end(); ++b)
    {
        res.push_back( ConstraintBlock( b->first, b->second ) );
    }

    return res;
}

template <class DataTypes>
SReal MechanicalObject<DataTypes>::getConstraintJacobianTimesVecDeriv(unsigned int line, core::ConstVecId id)
{
    SReal result = 0;

    const MatrixDeriv& constraints = c.getValue();

    MatrixDerivRowConstIterator rowIterator = constraints.readLine(line);

    if (rowIterator == constraints.end())
        return 0;

    const VecDeriv *data = 0;

    // Maybe we should extend this to restvelocity
    if (id == core::ConstVecId::velocity())
    {
        data = &v.getValue();
    }
    else if (id == core::ConstVecId::dx())
    {
        data = &dx.getValue();
    }
    else
    {
        this->serr << "getConstraintJacobianTimesVecDeriv " << "NOT IMPLEMENTED for " << id.getName() << this->sendl;
        return 0;
    }

    MatrixDerivColConstIterator it = rowIterator.begin();
    MatrixDerivColConstIterator itEnd = rowIterator.end();

    while (it != itEnd)
    {
        result += it.val() * (*data)[it.index()];
        ++it;
    }

    return result;
}

template <class DataTypes>
inline void MechanicalObject<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    if (showIndices.getValue())
    {
        glColor3f(1.0,1.0,1.0);
        float scale = (float)( ( vparams->sceneBBox().maxBBox() - vparams->sceneBBox().minBBox() ).norm() * showIndicesScale.getValue() );

        defaulttype::Mat<4,4, GLfloat> modelviewM;
        const int vsize = d_size.getValue();
        for (int i=0 ; i< vsize ; i++)
        {
            std::ostringstream oss;
            oss << i;
            std::string tmp = oss.str();
            const char* s = tmp.c_str();
            //glVertex3f(getPX(i),getPY(i),getPZ(i) );
            glPushMatrix();

            glTranslatef((float)getPX(i), (float)getPY(i), (float)getPZ(i));
            glScalef(scale,scale,scale);

            // Makes text always face the viewer by removing the scene rotation
            // get the current modelview matrix
            glGetFloatv(GL_MODELVIEW_MATRIX , modelviewM.ptr() );
            modelviewM.transpose();

            defaulttype::Vec3d temp(getPX(i), getPY(i), getPZ(i));
            temp = modelviewM.transform(temp);

            //glLoadMatrixf(modelview);
            glLoadIdentity();

            glTranslatef((float)temp[0], (float)temp[1], (float)temp[2]);
            glScalef(scale,scale,scale);

            while(*s)
            {
                glutStrokeCharacter(GLUT_STROKE_ROMAN, *s);
                s++;
            }

            glPopMatrix();
        }
    }
    if (showVectors.getValue())
    {
        defaulttype::Vec<3, SReal> sceneMinBBox, sceneMaxBBox;
        sofa::simulation::Node* context = sofa::simulation::Node::DynamicCast(this->getContext());
        glColor3f(1.0,1.0,1.0);
        sofa::simulation::getSimulation()->computeBBox((sofa::simulation::Node*)context, sceneMinBBox.ptr(), sceneMaxBBox.ptr());
        //float scale = (sceneMaxBBox - sceneMinBBox).norm() * showVectorsScale.getValue();
        float scale = showVectorsScale.getValue();
        sofa::helper::ReadAccessor< Data<VecDeriv> > v_rA = *this->read(core::ConstVecDerivId::velocity());
        //std::cout << "number of velocity values: " << v_rA.size() << std::endl;
        sofa::helper::vector<Vector3> points;
        points.resize(2);
        for( unsigned i=0; i<v_rA.size(); ++i )
        {
            Real vx=0.0,vy=0.0,vz=0.0;
            DataTypes::get(vx,vy,vz,v_rA[i]);
            //v = DataTypes::getDPos(v_rA[i]);
            //Real vx = v[0]; Real vy = v[1]; Real vz = v[2];
            //std::cout << "v=" << vx << ", " << vy << ", " << vz << std::endl;
            Vector3 p1 = Vector3(getPX(i), getPY(i), getPZ(i));
            Vector3 p2 = Vector3(getPX(i)+scale*vx, getPY(i)+scale*vy, getPZ(i)+scale*vz);

            float rad = (float)( (p1-p2).norm()/20.0 );
            switch (drawMode.getValue())
            {
            case 0:
                points[0] = p1;
                points[1] = p2;
                vparams->drawTool()->drawLines(points, 1, defaulttype::Vec<4,float>(1.0,1.0,1.0,1.0));
                break;
            case 1:
                vparams->drawTool()->drawCylinder(p1, p2, rad, defaulttype::Vec<4,float>(1.0,1.0,1.0,1.0));
                break;
            case 2:
                vparams->drawTool()->drawArrow(p1, p2, rad, defaulttype::Vec<4,float>(1.0,1.0,1.0,1.0));
                break;
            default:
                serr << "No proper drawing mode found!" << sendl;
                break;
            }
        }
    }
    if (showObject.getValue())
    {
        const float scale = showObjectScale.getValue();
        const VecCoord& inputPositions = showFreePositions.getValue() ? read(core::ConstVecCoordId::freePosition())->getValue()
                                                                      : read(core::ConstVecCoordId::position())->getValue();
        sofa::helper::vector<Vector3> positions(inputPositions.size());
        for (unsigned i = 0; i < positions.size(); ++i)
        {
            positions[i] = DataTypes::getCPos(inputPositions[i]);
        }

        switch (drawMode.getValue())
        {
        case 0:
            vparams->drawTool()->drawPoints(positions,scale,defaulttype::Vec<4,float>(d_color.getValue()));
            break;
        case 1:
            glEnable(GL_LIGHTING);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(d_color.getValue()));
            break;
        case 2:
            glEnable(GL_LIGHTING);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(1.0,0.0,0.0,1.0));
            break;
        case 3:
            glEnable(GL_LIGHTING);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(0.0,1.0,0.0,1.0));
            break;
        case 4:
            glEnable(GL_LIGHTING);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(0.0,0.0,1.0,1.0));
            break;
        default:
            serr << "No proper drawing mode found!" << sendl;
            break;
        }
    }
    glPopAttrib();
#endif /* SOFA_NO_OPENGL */
}

#ifdef SOFA_SMP
template < class DataTypes >
void MechanicalObject < DataTypes >::vOpMEq (const core::ExecParams* /* params */ /* PARAMS FIRST */, VecId v, ConstVecId b, a1::Shared < double >*f)
{
    if (v.type == sofa::core::V_COORD)
    {
        if (b.type == sofa::core::V_COORD)
        {
            BaseObject::Task < vOpMinusEqualMult < DataTypes, VecCoord, VecCoord > >
                    (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                     **defaulttype::getShared(*this->read(ConstVecCoordId(b))), *f);
        }
        else
        {
            BaseObject::Task < vOpMinusEqualMult < DataTypes, VecCoord, VecDeriv > >
                    (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                     **defaulttype::getShared(*this->read(ConstVecDerivId(b))), *f);
        }
    }
    else if (b.type == sofa::core::V_DERIV)
    {
        BaseObject::Task < vOpMinusEqualMult < DataTypes, VecDeriv, VecDeriv > >
                (this, **defaulttype::getShared(*this->write(VecDerivId(v))),
                 **defaulttype::getShared(*this->read(ConstVecDerivId(b))), *f);
        // helper::ReadAccessor< Data<VecCoord> > vSrc = *this->read(ConstVecCoordId(src));
        // helper::ReadAccessor< Data<VecCoord> > x_rA = *this->read(ConstVecCoordId::position());
    }
    else
    {
        // ERROR
        //  serr << "Invalid vOp operation ("<<v<<','<<a<<','<<b<<','<<f<<")" << sendl;
        return;
    }
}

// template < class DataTypes >
// void MechanicalObject < DataTypes >::vOp(const core::ExecParams* params /* PARAMS FIRST */, core::VecId v, core::ConstVecId a, core::ConstVecId b , double f){ vOp(params /* PARAMS FIRST */, v,a,b,f,NULL); }

template < class DataTypes >
void MechanicalObject < DataTypes >::vOp (const core::ExecParams* params /* PARAMS FIRST */, core::VecId v, core::ConstVecId a, core::ConstVecId b, double f, a1::Shared  < double >*fSh )
{
    if (params->execMode() != core::ExecParams::EXEC_KAAPI)
    {
        vOp(params /* PARAMS FIRST */, v, a, b, f);
    }
    else
    {
#ifdef SOFA_SMP_NUMA
        if(this->getContext()->getProcessor()!=-1)
            numa_set_preferred(this->getContext()->getProcessor()/2);
#endif
        if (v.isNull ())
        {
            // ERROR
            serr << "Invalid vOp operation (" << v << ',' << a << ','
                 << b << ',' << f << ")" << sendl;
            return;
        }
        if (a.isNull ())
        {
            if (b.isNull ())
            {
                // v = 0
                if (v.type == sofa::core::V_COORD)
                {
                    BaseObject::Task < vClear < VecCoord,Coord > >
                            (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                             (unsigned) (this->vsize));

                    /*unsigned int vt=ExecutionGraph::add_operation("v=0");
                     ExecutionGraph::read_var(vt,vv);
                     ExecutionGraph::write_var(vt,vv); */
                }
                else
                {
                    BaseObject::Task < vClear < VecDeriv,Deriv > >
                            (this,**defaulttype::getShared(*this->write(VecDerivId(v))),
                             (unsigned) (this->vsize));
                    /*unsigned int vt=ExecutionGraph::add_operation("v=0");
                     ExecutionGraph::read_var(vt,vv);
                     ExecutionGraph::write_var(vt,vv);
                     vv->resize(this->this->vsize);
                     */
                }
            }
            else
            {
                if (b.type != v.type)
                {
                    // ERROR
                    serr << "Invalid vOp operation (" << v << ',' << a << ',' << b << ',' << f << ")" << sendl;
                    return;
                }
                if (v == b)
                {
                    // v *= f
                    if (v.type == sofa::core::V_COORD)
                    {

                        /*VecCoord* vv = getVecCoord(v.index);
                         unsigned int vt=ExecutionGraph::add_operation("v*=f");
                         ExecutionGraph::read_var(vt,vv);
                         ExecutionGraph::write_var(vt,vv); */
                        BaseObject::Task < vTEq < VecCoord, Real > >
                                (this,**defaulttype::getShared(*this->write(VecCoordId(v))), f);
                    }
                    else
                    {
                        /*
                         unsigned int vt=ExecutionGraph::add_operation("v*=f");
                         ExecutionGraph::read_var(vt,vv);
                         ExecutionGraph::write_var(vt,vv);
                         */
                        BaseObject::Task < vTEq < VecDeriv, Real > >
                                (this,**defaulttype::getShared(*this->write(VecDerivId(v))), f);
                    }
                }
                else
                {
                    // v = b*f
                    if (v.type == sofa::core::V_COORD)
                    {
                        BaseObject::Task < vEqBF < VecCoord,Real > >
                                (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                 **defaulttype::getShared(*this->read(ConstVecCoordId(b))), f);
                        // >(this,**getVecCoord (b.index), TODO : check order
                        // **getVecCoord (v.index), f);
                    }
                    else
                    {
                        BaseObject::Task < vEqBF < VecDeriv,Real > >
                                (this,**defaulttype::getShared(*this->write(VecDerivId(v))),
                                 **defaulttype::getShared(*this->read(ConstVecDerivId(b))), f);
                        // *getVecDeriv (b.index), TODO : check order
                        // **getVecDeriv (v.index), f);
                    }
                }
            }
        }
        else
        {
            if (a.type != v.type)
            {
                // ERROR
                serr << "Invalid vOp operation (" << v << ',' << a <<
                        ',' << b << ',' << f << ")" << sendl;
                return;
            }
            if (b.isNull ())
            {
                // v = a
                if (v.type == sofa::core::V_COORD)
                {
                    BaseObject::Task < vAssign < VecCoord > >
                            (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                             **defaulttype::getShared(*this->read(ConstVecCoordId(a))));
                }
                else
                {
                    BaseObject::Task < vAssign < VecDeriv > >
                            (this,**defaulttype::getShared(*this->write(VecDerivId(v))),
                             **defaulttype::getShared(*this->read(ConstVecDerivId(a))));
                }
            }
            else
            {
                if (v == a)
                {
                    if (f == 1.0 && !fSh)
                    {
                        // v += b
                        if (v.type == sofa::core::V_COORD)
                        {

                            if (b.type == sofa::core::V_COORD)
                            {
                                BaseObject::Task < vPEq < VecCoord, VecCoord > >
                                        (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(v))));
                            }
                            else
                            {
                                BaseObject::Task < vPEq < VecCoord, VecDeriv > >
                                        (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(v))));
                            }
                        }
                        else if (b.type == sofa::core::V_DERIV)
                        {

                            BaseObject::Task < vPEq <  VecDeriv, VecDeriv > >
                                    (this,**defaulttype::getShared(*this->write(VecDerivId(v))),
                                     **defaulttype::getShared(*this->read(ConstVecDerivId(v))));
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation (" << v <<
                                    ',' << a << ',' << b << ',' << f << ")" << sendl;
                            return;
                        }
                    }
                    else
                    {
                        // v += b*f
                        if (v.type == sofa::core::V_COORD)
                        {
                            if (b.type == sofa::core::V_COORD)
                            {
                                if (fSh)
                                {
                                    BaseObject::Task < vPEqBF < DataTypes, VecCoord, VecCoord > >
                                            (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                             **defaulttype::getShared(*this->read(ConstVecCoordId(b))), *fSh, f);
                                }
                                else
                                {
                                    BaseObject::Task < vPEqBF < DataTypes, VecCoord, VecCoord > >
                                            (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                             **defaulttype::getShared(*this->read(ConstVecCoordId(b))), f);
                                }
                            }
                            else
                            {
                                if (fSh)
                                {
                                    BaseObject::Task < vPEqBF < DataTypes, VecCoord, VecDeriv > >
                                            (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                             **defaulttype::getShared(*this->read(ConstVecDerivId(b))),*fSh, f);
                                }
                                else
                                {
                                    BaseObject::Task < vPEqBF < DataTypes, VecCoord, VecDeriv > >
                                            (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                             **defaulttype::getShared(*this->read(ConstVecDerivId(b))), f);
                                }
                            }
                        }// v.type == sofa::core::V_COORD
                        else if (b.type == sofa::core::V_DERIV)
                        {
                            if (fSh)
                            {
                                BaseObject::Task < vPEqBF < DataTypes, VecDeriv, VecDeriv > >
                                        (this,**defaulttype::getShared(*this->write(VecDerivId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(b))),*fSh, f);
                            }
                            else
                            {
                                BaseObject::Task < vPEqBF < DataTypes, VecDeriv, VecDeriv > >
                                        (this,**defaulttype::getShared(*this->write(VecDerivId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(b))), f);
                            }
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation (" << v <<
                                    ',' << a << ',' << b << ',' << f << ")" << sendl;
                            return;
                        }
                    }
                }
                else if (v == b)
                {
                    if (f == 1.0 && !fSh)
                    {
                        // v += a
                        if (v.type == sofa::core::V_COORD)
                        {

                            if (a.type == sofa::core::V_COORD)
                            {
                                BaseObject::Task < vPEq < VecCoord, VecCoord > >
                                        (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(a))));
                            }
                            else
                            {
                                BaseObject::Task < vPEq < VecCoord, VecDeriv > >
                                        (this,**defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(a))));
                            }
                        }
                        else if (a.type == sofa::core::V_DERIV)
                        {
                            BaseObject::Task< vPEq <VecDeriv, VecDeriv > >
                                    (this, **defaulttype::getShared(*this->write(VecDerivId(v))),
                                     **defaulttype::getShared(*this->read(ConstVecDerivId(a))));
                            // BaseObject::Task<vPEq <VecCoord,VecDeriv> >  TODO CHECK
                            //      (this,**getVecDeriv(v.index),**getVecDeriv(a.index));
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation (" << v << ',' << a << ',' << b << ',' << f << ")" << sendl;
                            return;
                        }
                    }
                    else
                    {
                        // v = a+v*f
                        if (v.type == sofa::core::V_COORD)
                        {
                            if (fSh)
                            {
                                BaseObject::Task < vOpSumMult < DataTypes, VecCoord, VecCoord > >
                                        (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(a))), *fSh, (Real) f);
                            }
                            else
                            {
                                BaseObject::Task < vOpSumMult < DataTypes, VecCoord, VecCoord > >
                                        (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(a))), (Real) f);
                            }
                        }
                        else
                        {
                            if (fSh)
                            {
                                BaseObject::Task < vOpSumMult < DataTypes, VecDeriv, VecDeriv > >
                                        (this, **defaulttype::getShared(*this->write(VecDerivId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(a))), *fSh, (Real) f);
                            }
                            else
                            {
                                BaseObject::Task < vOpSumMult < DataTypes, VecDeriv, VecDeriv > >
                                        (this, **defaulttype::getShared(*this->write(VecDerivId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(a))), (Real) f);
                            }
                        }
                    }
                }
                else
                {
                    if (f == 1.0 && !fSh)
                    {
                        // v = a+b
                        if (v.type == sofa::core::V_COORD)
                        {
                            if (b.type == sofa::core::V_COORD)
                            {
                                BaseObject::Task < vAdd < VecCoord, VecCoord, VecCoord > >
                                        (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(a))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(b))));
                            }
                            else
                            {
                                BaseObject::Task < vAdd < VecCoord, VecCoord, VecDeriv > >
                                        (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(a))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(b))));
                            }
                        }
                        else if (b.type == sofa::core::V_DERIV)
                        {
                            BaseObject::Task < vAdd < VecDeriv, VecDeriv, VecDeriv > >
                                    (this, **defaulttype::getShared(*this->write(VecDerivId(v))),
                                     **defaulttype::getShared(*this->read(ConstVecDerivId(a))),
                                     **defaulttype::getShared(*this->read(ConstVecDerivId(b))));
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation (" << v << ',' << a << ',' << b << ',' << f << ")" << sendl;
                            return;
                        }
                    }
                    else
                    {
                        // v = a+b*f
                        if (v.type == sofa::core::V_COORD)
                        {
                            if (b.type == sofa::core::V_COORD)
                            {
                                BaseObject::Task < vOpVeqAplusBmultF < DataTypes, VecCoord, VecCoord > >
                                        (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(a))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(b))), (Real) f);
                            }
                            else
                            {
                                BaseObject::Task < vOpVeqAplusBmultF < DataTypes, VecCoord, VecDeriv > >
                                        (this, **defaulttype::getShared(*this->write(VecCoordId(v))),
                                         **defaulttype::getShared(*this->read(ConstVecCoordId(a))),
                                         **defaulttype::getShared(*this->read(ConstVecDerivId(b))), (Real) f);
                            }
                        }
                        else if (b.type == sofa::core::V_DERIV)
                        {
                            BaseObject::Task < vOpVeqAplusBmultF < DataTypes, VecDeriv, VecDeriv > >
                                    (this, **defaulttype::getShared(*this->write(VecDerivId(v))),
                                     **defaulttype::getShared(*this->read(ConstVecDerivId(a))),
                                     **defaulttype::getShared(*this->read(ConstVecDerivId(b))), (Real) f);
                        }
                        else
                        {
                            // ERROR
                            serr << "Invalid vOp operation (" << v << ',' << a << ',' << b << ',' << f << ")" << sendl;
                            return;
                        }
                    }
                }
            }
        }
    }
}

template < class DataTypes >
void MechanicalObject < DataTypes >::vDot ( const core::ExecParams* /* params */ /* PARAMS FIRST */, a1::Shared  < double >*res, core::ConstVecId a,  core::ConstVecId b)
{
    //      double r = 0.0;
    if (a.type == sofa::core::V_COORD && b.type == sofa::core::V_COORD)
    {
        if (a.index == b.index)
        {
            BaseObject::Task < vDotOp < DataTypes, VecCoord > >
                    (this, **defaulttype::getShared(*this->read(ConstVecCoordId(a))), *res);
        }
        else
        {
            BaseObject::Task < vDotOp < DataTypes, VecCoord > >
                    (this, **defaulttype::getShared(*this->read(ConstVecCoordId(a))),
                     **defaulttype::getShared(*this->read(ConstVecCoordId(b))), *res);
        }
    }
    else if (a.type == sofa::core::V_DERIV && b.type == sofa::core::V_DERIV)
    {
        if (a.index == b.index)
        {
            BaseObject::Task < vDotOp < DataTypes, VecDeriv > >
                    (this, **defaulttype::getShared(*this->read(ConstVecDerivId(a))), *res);
        }
        else
        {
            BaseObject::Task < vDotOp < DataTypes, VecDeriv > >
                    (this, **defaulttype::getShared(*this->read(ConstVecDerivId(a))),
                     **defaulttype::getShared(*this->read(ConstVecDerivId(b))), *res);
        }
    }
    else
    {
        serr << "Invalid dot operation (" << a << ',' << b << ")" << sendl;
    }
    // return r;
}

//     template < class DataTypes >
//       void MechanicalObject < DataTypes >::setX (VecId v)
//     {
//       if (v.type == sofa::core::V_COORD)
//  {
//
//    this->xSh = *getVecCoord (v.index);
//
//    this->x = getVecCoord (v.index);
//
//  }
//       else
//  {
//    std::cerr << "Invalid setX operation (" << v << ")\n";
//  }
//     }
//
//     template < class DataTypes >
//       void ParallelMechanicalObject < DataTypes >::setXfree (VecId v)
//     {
//       if (v.type == sofa::core::V_COORD)
//  {
//    this->xfreeSh = *getVecCoord (v.index);
//
//    this->xfree = getVecCoord (v.index);
//
//  }
//       else
//  {
//    std::cerr << "Invalid setXfree operation (" << v << ")\n";
//  }
//     }
//
//     template < class DataTypes >
//       void ParallelMechanicalObject < DataTypes >::setVfree (VecId v)
//     {
//       if (v.type == sofa::core::V_DERIV)
//  {
//
//    this->vfreeSh = *getVecDeriv (v.index);
//
//    this->vfree = getVecDeriv (v.index);
//
//  }
//       else
//  {
//    std::cerr << "Invalid setVfree operation (" << v << ")\n";
//  }
//     }
//
//     template < class DataTypes >
//       void ParallelMechanicalObject < DataTypes >::setV (VecId v)
//     {
//       if (v.type == sofa::core::V_DERIV)
//  {
//    this->vSh = *getVecDeriv (v.index);
//
//    this->v = getVecDeriv (v.index);
//
//  }
//       else
//  {
//    std::cerr << "Invalid setV operation (" << v << ")\n";
//  }
//     }
//
//     template < class DataTypes >
//       void ParallelMechanicalObject < DataTypes >::setF (VecId v)
//     {
//       if (v.type == sofa::core::V_DERIV)
//  {
//
//    this->fSh = *getVecDeriv (v.index);
//
//    this->f = getVecDeriv (v.index);
//
//  }
//       else
//  {
//    std::cerr << "Invalid setF operation (" << v << ")\n";
//  }
//     }
//
//     template < class DataTypes >
//       void ParallelMechanicalObject < DataTypes >::setDx (VecId v)
//     {
//       if (v.type == sofa::core::V_DERIV)
//  {
//    this->dxSh = *getVecDeriv (v.index);
//
//
//    this->dx = getVecDeriv (v.index);
//
//  }
//       else
//  {
//    std::cerr << "Invalid setDx operation (" << v << ")\n";
//  }
//     }
//
//
template < class DataTypes >
void MechanicalObject < DataTypes >::printDOF (core::ConstVecId /*v*/, std::ostream & /*out*/, int /* firstIndex */, int /* range */) const
{
    //  if (v.type == sofa::core::V_COORD)
    //  {
    //      VecCoord & x = *getVecCoord (v.index);
    //      Task < printDOFSh < VecCoord > >(this,*x);
    //  }
    //  else if (v.type == sofa::core::V_DERIV)
    //  {
    //      VecDeriv & x = *getVecDeriv (v.index);
    //      Task < printDOFSh < VecDeriv > >(this,*x);
    //  }
    //  else
    //      out << "ParallelMechanicalObject<DataTypes>::printDOF, unknown v.type = " <<
    //          v.type << std::endl;

}
#endif /* SOFA_SMP */

/// Find mechanical particles hit by the given ray.
/// A mechanical particle is defined as a 2D or 3D, position or rigid DOF
/// Returns false if this object does not support picking
template <class DataTypes>
bool MechanicalObject<DataTypes>::pickParticles(const core::ExecParams* /* params */ /* PARAMS FIRST */, double rayOx, double rayOy, double rayOz, double rayDx, double rayDy, double rayDz, double radius0, double dRadius,
                                                std::multimap< double, std::pair<sofa::core::behavior::BaseMechanicalState*, int> >& particles)
{
    if (defaulttype::DataTypeInfo<Coord>::FinalSize == 2 || defaulttype::DataTypeInfo<Coord>::FinalSize == 3
            || (defaulttype::DataTypeInfo<Coord>::FinalSize == 7 && defaulttype::DataTypeInfo<Deriv>::FinalSize == 6))
    {
        // seems to be valid DOFs
        const VecCoord& x =this->read(core::ConstVecCoordId::position())->getValue();

        defaulttype::Vec<3,Real> origin((Real)rayOx, (Real)rayOy, (Real)rayOz);
        defaulttype::Vec<3,Real> direction((Real)rayDx, (Real)rayDy, (Real)rayDz);
//                            cerr<<"MechanicalObject<DataTypes>::pickParticles, ray point = " << rayOx << ", " << rayOy << ", " << rayOz << endl;
//                            cerr<<"MechanicalObject<DataTypes>::pickParticles, ray dir = " << rayDx << ", " << rayDy << ", " << rayDz << endl;
//                            cerr<<"MechanicalObject<DataTypes>::pickParticles, radius0 = " << radius0 << endl;
//                            cerr<<"MechanicalObject<DataTypes>::pickParticles, dRadius = " << dRadius << endl;
        const int vsize = d_size.getValue();
        for (int i=0; i< vsize; ++i)
        {
            defaulttype::Vec<3,Real> pos;
            DataTypes::get(pos[0],pos[1],pos[2],x[i]);

//                                    cerr<<"MechanicalObject<DataTypes>::pickParticles, point " << i << " = " << pos << endl;
            if (pos == origin) continue;
            double dist = (pos-origin)*direction;
            if (dist < 0) continue; // discard particles behind the camera, such as mouse position

            defaulttype::Vec<3,Real> vecPoint = (pos-origin) - direction*dist;
            double distToRay = vecPoint.norm2();
            double maxr = radius0 + dRadius*dist;
//                                    cerr<<"MechanicalObject<DataTypes>::pickParticles, point " << i << ", maxR = " << maxr << endl;
            double r2 = (pos-origin-direction*dist).norm2();
            if (r2 <= maxr*maxr)
            {
                particles.insert(std::make_pair(distToRay,std::make_pair(this,i)));
//                                            cerr<<"MechanicalObject<DataTypes>::pickParticles, point " << i << ", distance = " << r2 << " inserted " << endl;
            }
//                                    cerr<<"MechanicalObject<DataTypes>::pickParticles, point " << i << ", distance = " << r2 << " not inserted " << endl;
        }
        return true;
    }
    else
        return false;
}


template <class DataTypes>
bool MechanicalObject<DataTypes>::addBBox(double* minBBox, double* maxBBox)
{
    // participating to bbox only if it is drawn
    if( !showObject.getValue() ) return false;

    const VecCoord& x = read(core::ConstVecCoordId::position())->getValue();
    for( std::size_t i=0; i<x.size(); i++ )
    {
        defaulttype::Vec<3,Real> p;
        DataTypes::get( p[0], p[1], p[2], x[i] );

        for( unsigned int j=0 ; j<DataTypes::spatial_dimensions && j<3; ++j )
        {
            if(p[j]<minBBox[j]) minBBox[j]=p[j];
            if(p[j]>maxBBox[j]) maxBBox[j]=p[j];
        }
    }
    return true;
}

template <class DataTypes>
bool MechanicalObject<DataTypes>::isIndependent() const
{
    return static_cast<const simulation::Node*>(this->getContext())->mechanicalMapping.empty();
}


} // namespace container

} // namespace component

} // namespace sofa

#endif
