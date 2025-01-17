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
#ifndef SOFA_COMPONENT_ENGINE_SPHEREROI_H
#define SOFA_COMPONENT_ENGINE_SPHEREROI_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/loader/MeshLoader.h>
#include <sofa/helper/gl/BasicShapes.h>
#include <sofa/SofaGeneral.h>

namespace sofa
{

namespace component
{

namespace engine
{

#ifndef SOFA_FLOAT
using sofa::defaulttype::Rigid3dTypes;
#endif //#ifndef SOFA_FLOAT

#ifndef SOFA_DOUBLE
using sofa::defaulttype::Rigid3fTypes;
#endif //#ifndef SOFA_


/**
 * This class find all the points/edges/triangles/tetrahedra located inside a given sphere.
 */
template <class DataTypes>
class SphereROI : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SphereROI,DataTypes),core::DataEngine);
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Real Real;
    typedef defaulttype::Vec<3,Real> Vec3;
    typedef defaulttype::Vec<6,Real> Vec6;
    typedef helper::vector<sofa::core::topology::BaseMeshTopology::EdgeID> SetEdge;
    typedef helper::vector<sofa::core::topology::BaseMeshTopology::TriangleID> SetTriangle;
    typedef helper::vector<sofa::core::topology::BaseMeshTopology::QuadID> SetQuad;
    typedef sofa::core::topology::BaseMeshTopology::SetIndex SetIndex;

    typedef typename DataTypes::CPos CPos;

    typedef unsigned int PointID;
    typedef core::topology::BaseMeshTopology::Edge Edge;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;
    typedef core::topology::BaseMeshTopology::Tetra Tetra;
    typedef core::topology::BaseMeshTopology::Quad Quad;

protected:

    SphereROI();

    ~SphereROI() {}
public:
    void init();

    void reinit();

    void update();

    void draw(const core::visual::VisualParams* vparams);

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (!arg->getAttribute("template"))
        {
            // only check if this template is correct if no template was given
            if (context->getMechanicalState() && sofa::core::behavior::MechanicalState<DataTypes>::DynamicCast(context->getMechanicalState()) == NULL)
                return false; // this template is not the same as the existing MechanicalState
        }

        return BaseObject::canCreate(obj, context, arg);
    }

    /// Construction method called by ObjectFactory.
    template<class T>
    static typename T::SPtr create(T* tObj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        return core::objectmodel::BaseObject::create(tObj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const SphereROI<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


protected:

	bool isPointInSphere(const Vec3& c, const Real& r, const Coord& p);
    bool isPointInSphere(const PointID& pid, const Real& r, const Coord& p);
    bool isEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge);
    bool isTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle);
    bool isQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad);
    bool isTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron);

    bool isPartOfEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge);
    bool isPartOfTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle);
    bool isPartOfQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad);
    bool isPartOfTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron);

public:
    //Input
    Data< helper::vector<Vec3> > centers;
    Data< helper::vector<Real> > radii;

    Data< Vec3 > direction;
    Data< Vec3 > normal;
    Data< Real > edgeAngle;
    Data< Real > triAngle;

    Data<VecCoord> f_X0;
    Data<helper::vector<Edge> > f_edges;
    Data<helper::vector<Triangle> > f_triangles;
    Data<helper::vector<Quad> > f_quads;
    Data<helper::vector<Tetra> > f_tetrahedra;
    Data<bool> f_computeEdges;
    Data<bool> f_computeTriangles;
    Data<bool> f_computeQuads;
    Data<bool> f_computeTetrahedra;
    Data<bool> f_strictlyInROI;

    //Output
    Data<SetIndex> f_indices;
    Data<SetIndex> f_edgeIndices;
    Data<SetIndex> f_triangleIndices;
    Data<SetIndex> f_quadIndices;
    Data<SetIndex> f_tetrahedronIndices;

    Data<VecCoord > f_pointsInROI;
    Data<helper::vector<Edge> > f_edgesInROI;
    Data<helper::vector<Triangle> > f_trianglesInROI;
    Data<helper::vector<Quad> > f_quadsInROI;
    Data<helper::vector<Tetra> > f_tetrahedraInROI;

    Data<SetIndex> f_indicesOut;
    Data<SetIndex> f_edgeIndicesOut;
    Data<SetIndex> f_triangleIndicesOut;
    Data<SetIndex> f_quadIndicesOut;
    Data<SetIndex> f_tetrahedronIndicesOut;

    Data<VecCoord > f_pointsOutROI;
    Data<helper::vector<Edge> > f_edgesOutROI;
    Data<helper::vector<Triangle> > f_trianglesOutROI;
    Data<helper::vector<Quad> > f_quadsOutROI;
    Data<helper::vector<Tetra> > f_tetrahedraOutROI;

    //Parameter
    Data<bool> p_drawSphere;
    Data<bool> p_drawPoints;
    Data<bool> p_drawEdges;
    Data<bool> p_drawTriangles;
    Data<bool> p_drawQuads;
    Data<bool> p_drawTetrahedra;
    Data<double> _drawSize;

};

#ifndef SOFA_FLOAT
template<> bool SphereROI<Rigid3dTypes>::isPointInSphere(const Vec3& c, const Real& r, const Coord& p);
template<> bool SphereROI<Rigid3dTypes>::isPointInSphere(const PointID& pid, const Real& r, const Coord& p);
template<> bool SphereROI<Rigid3dTypes>::isEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge);
template<> bool SphereROI<Rigid3dTypes>::isTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle);
template<> bool SphereROI<Rigid3dTypes>::isQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad);
template<> bool SphereROI<Rigid3dTypes>::isTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron);
template<> bool SphereROI<Rigid3dTypes>::isPartOfEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge);
template<> bool SphereROI<Rigid3dTypes>::isPartOfTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle);
template<> bool SphereROI<Rigid3dTypes>::isPartOfQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad);
template<> bool SphereROI<Rigid3dTypes>::isPartOfTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron);
template<> void SphereROI<Rigid3dTypes>::update();
#endif

#ifndef SOFA_DOUBLE
template<> bool SphereROI<Rigid3fTypes>::isPointInSphere(const Vec3& c, const Real& r, const Coord& p);
template<> bool SphereROI<Rigid3fTypes>::isPointInSphere(const PointID& pid, const Real& r, const Coord& p);
template<> bool SphereROI<Rigid3fTypes>::isEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge);
template<> bool SphereROI<Rigid3fTypes>::isTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle);
template<> bool SphereROI<Rigid3fTypes>::isQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad);
template<> bool SphereROI<Rigid3fTypes>::isTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron);
template<> bool SphereROI<Rigid3fTypes>::isPartOfEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge);
template<> bool SphereROI<Rigid3fTypes>::isPartOfTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle);
template<> bool SphereROI<Rigid3fTypes>::isPartOfQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad);
template<> bool SphereROI<Rigid3fTypes>::isPartOfTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron);
template<> void SphereROI<Rigid3fTypes>::update();
#endif

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_SPHEREROI_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_ENGINE_API SphereROI<defaulttype::Vec3dTypes>;
//extern template class SOFA_ENGINE_API SphereROI<defaulttype::Rigid3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_ENGINE_API SphereROI<defaulttype::Vec3fTypes>;
//extern template class SOFA_ENGINE_API SphereROI<defaulttype::Rigid3fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_ENGINE_SphereROI_H
