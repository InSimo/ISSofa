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
#ifndef SOFA_COMPONENT_COLLISION_LMDNEWPROXIMITYINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_LMDNEWPROXIMITYINTERSECTION_H

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

class SOFA_CONSTRAINT_API LMDNewProximityIntersection : public BaseProximityIntersection
{
public:
    SOFA_CLASS(LMDNewProximityIntersection,BaseProximityIntersection);

    template <class Elem1, class Elem2>
    using OutputContainer = sofa::core::collision::TDetectionOutputContainer<typename Elem1::Model, typename Elem2::Model>;

    Data<bool> useLineLine;
protected:
    LMDNewProximityIntersection();
public:
    virtual void init();

    /// Returns true if algorithm uses proximity
    virtual bool useProximity() const { return true; }

    bool testIntersection(Cube& ,Cube&);
    bool testIntersection(Point&, Point&);
    template<class T> bool testIntersection(TSphere<T>&, Point&);
    template<class T1, class T2> bool testIntersection(TSphere<T1>&, TSphere<T2>&);
    bool testIntersection(Line&, Point&);
    template<class T> bool testIntersection(Line&, TSphere<T>&);
    bool testIntersection(Line&, Line&);
    bool testIntersection(Triangle&, Point&);
    template<class T> bool testIntersection(Triangle&, TSphere<T>&);
    bool testIntersection(Triangle&, Line&);
    bool testIntersection(Triangle&, Triangle&);
    bool testIntersection(Ray&, Triangle&);

    int computeIntersection(Cube&, Cube&, OutputContainer<Cube, Cube>*);
    int computeIntersection(Point&, Point&, OutputContainer<Point, Point>*);
    template<class T> int computeIntersection(TSphere<T>&, Point&, OutputContainer<TSphere<T>, Point>*);
    template<class T1, class T2> int computeIntersection(TSphere<T1>&, TSphere<T2>&, OutputContainer<TSphere<T1>, TSphere<T2>>*);
    int computeIntersection(Line&, Point&, OutputContainer<Line, Point>*);
    template<class T> int computeIntersection(Line&, TSphere<T>&, OutputContainer<Line, TSphere<T>>*);
    int computeIntersection(Line&, Line&, OutputContainer<Line, Line>*);
    int computeIntersection(Triangle&, Point&, OutputContainer<Triangle, Point>*);
    template<class T> int computeIntersection(Triangle&, TSphere<T>&, OutputContainer<Triangle, TSphere<T>>*);
    int computeIntersection(Triangle&, Line&, OutputContainer<Triangle, Line>*);
    int computeIntersection(Triangle&, Triangle&, OutputContainer<Triangle, Triangle>*);
    int computeIntersection(Ray&, Triangle&, OutputContainer<Ray, Triangle>*);

    template< class TFilter1, class TFilter2, class TOutputContainer >
    static inline int doIntersectionLineLine(double dist2, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& q1, const defaulttype::Vector3& q2, TOutputContainer* contacts, int id, int indexLine1, int indexLine2, TFilter1 &f1, TFilter2 &f2);

    template< class TFilter1, class TFilter2, class TOutputContainer >
    static inline int doIntersectionLinePoint(double dist2, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& q, TOutputContainer* contacts, int id, int indexLine1, int indexPoint2, TFilter1 &f1, TFilter2 &f2, bool swapElems = false);

    template< class TFilter1, class TFilter2, class TOutputContainer >
    static inline int doIntersectionPointPoint(double dist2, const defaulttype::Vector3& p, const defaulttype::Vector3& q, TOutputContainer* contacts, int id, int indexPoint1, int indexPoint2, TFilter1 &f1, TFilter2 &f2);

    template< class TFilter1, class TFilter2, class TOutputContainer >
    static inline int doIntersectionTrianglePoint(double dist2, int flags, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& p3, const defaulttype::Vector3& n, const defaulttype::Vector3& q, TOutputContainer* contacts, int id, Triangle &e1, unsigned int *edgesIndices, int indexPoint2, TFilter1 &f1, TFilter2 &f2, bool swapElems = false);


    /**
     * @brief Method called at the beginning of the collision detection between model1 and model2.
     * Checks if LMDFilters are associated to the CollisionModels.
     * @TODO Optimization.
     */
//	int beginIntersection(TriangleModel* /*model1*/, TriangleModel* /*model2*/, DiscreteIntersection::OutputVector* /*contacts*/);

protected:
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_COLLISION_LMDNEWPROXIMITYINTERSECTION_H
