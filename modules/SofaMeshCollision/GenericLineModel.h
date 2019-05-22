/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H

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
class GenericLineModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericLineModel<TDataTypes>), ((sofa::core::CollisionModel)));

protected:
    GenericLineModel() { enum_type = LINE_TYPE; };

    virtual ~GenericLineModel() {};
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
#ifndef SOFA_FLOAT
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Vec3dTypes>;
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Rigid3dTypes>;
#endif

#ifndef SOFA_DOUBLE
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Vec3fTypes>;
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace collision

} // namespace component

} // namespace sofa

#endif
