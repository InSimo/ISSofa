/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_H

#include "initMeshCollision.h"
#include <sofa/core/CollisionModel.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace collision
{

template<class TDataTypes>
class GenericTriangleModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericTriangleModel<TDataTypes>), ((sofa::core::CollisionModel)));

protected:
    GenericTriangleModel() { enum_type = TRIANGLE_TYPE; };

    virtual ~GenericTriangleModel() {};
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
#ifndef SOFA_FLOAT
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Vec3dTypes>;
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Rigid3dTypes>;
#endif

#ifndef SOFA_DOUBLE
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Vec3fTypes>;
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace collision

} // namespace component

} // namespace sofa 

#endif
