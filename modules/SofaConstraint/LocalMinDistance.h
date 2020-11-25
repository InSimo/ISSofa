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
#ifndef SOFA_COMPONENT_COLLISION_LOCALMINDISTANCE_H
#define SOFA_COMPONENT_COLLISION_LOCALMINDISTANCE_H

#include <SofaBaseCollision/BaseProximityIntersection.h>
#include <sofa/helper/FnDispatcher.h>
#include <SofaBaseCollision/SphereModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaBaseCollision/CubeModel.h>
#include <SofaUserInteraction/RayModel.h>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_CONSTRAINT_API LocalMinDistance : public BaseProximityIntersection
{
public:
    SOFA_CLASS(LocalMinDistance,BaseProximityIntersection);

    typedef core::collision::IntersectorFactory<LocalMinDistance> IntersectorFactory;

    // Data<bool> useSphereTriangle;
    // Data<bool> usePointPoint;
    Data<bool> filterIntersection;
    Data<double> angleCone;
    Data<double> coneFactor;
    Data<bool> useLMDFilters;


protected:
    LocalMinDistance();
public:
    virtual void init();

    bool testIntersection(Cube& ,Cube&);

    bool testIntersection(Point&, Point&);
    bool testIntersection(Sphere&, Point&);
    bool testIntersection(Sphere&, Sphere&);
    bool testIntersection(Line&, Point&);
    bool testIntersection(Line&, Sphere&);
    bool testIntersection(Line&, Line&);
    bool testIntersection(Triangle&, Point&);
    bool testIntersection(Triangle&, Sphere&);
    bool testIntersection(Ray&, Sphere&);
    bool testIntersection(Ray&, Triangle&);

    int computeIntersection(Cube&, Cube&, OutputContainer<Cube, Cube>*);
    int computeIntersection(Point&, Point&, OutputContainer<Point, Point>*);
    int computeIntersection(Sphere&, Point&, OutputContainer<Sphere, Point>*);
    int computeIntersection(Sphere&, Sphere&, OutputContainer<Sphere, Sphere>*);
    int computeIntersection(Line&, Point&, OutputContainer<Line, Point>*);
    int computeIntersection(Line&, Sphere&, OutputContainer<Line, Sphere>*);
    int computeIntersection(Line&, Line&, OutputContainer<Line, Line>*);
    int computeIntersection(Triangle&, Point&, OutputContainer<Triangle, Point>*);
    int computeIntersection(Triangle&, Sphere&, OutputContainer<Triangle, Sphere>*);
    int computeIntersection(Ray&, Sphere&, OutputContainer<Ray, Sphere>*);
    int computeIntersection(Ray&, Triangle&, OutputContainer<Ray, Triangle>*);

    /// These methods check the validity of a found intersection.
    /// According to the local configuration around the found intersected primitive,
    /// we build a "Region Of Interest" geometric cone.
    /// Pertinent intersections have to belong to this cone, others are not taking into account anymore.
    bool testValidity(Sphere&, const defaulttype::Vector3&) { return true; }
    bool testValidity(Point&, const defaulttype::Vector3&);
    bool testValidity(Line&, const defaulttype::Vector3&);
    bool testValidity(Triangle&, const defaulttype::Vector3&);

    void draw(const core::visual::VisualParams* vparams);

    /// Actions to accomplish when the broadPhase is started. By default do nothing.
    virtual void beginBroadPhase() {}

    int beginIntersection(sofa::core::CollisionModel* /*model1*/, sofa::core::CollisionModel* /*model2*/, sofa::core::collision::DetectionOutputContainer* /*contacts*/)
    {
        //std::cout << "beginIntersection\n";
        return 0;
    }

private:
    double mainAlarmDistance;
    double mainContactDistance;
};

} // namespace collision

} // namespace component

namespace core
{
namespace collision
{
#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_CONSTRAINT)
extern template class SOFA_CONSTRAINT_API IntersectorFactory<component::collision::LocalMinDistance>;
#endif
}
}

} // namespace sofa

#endif /* SOFA_COMPONENT_COLLISION_LOCALMINDISTANCE_H */
