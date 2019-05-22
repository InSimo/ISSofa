/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/


#include "GenericLineModel.h"

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
#ifndef SOFA_FLOAT
template class SOFA_MESH_COLLISION_API GenericLineModel<Vec3dTypes>;
template class SOFA_MESH_COLLISION_API GenericLineModel<Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_MESH_COLLISION_API GenericLineModel<Vec3fTypes>;
template class SOFA_MESH_COLLISION_API GenericLineModel<Rigid3fTypes>;
#endif

} // namespace collision

} // namespace component

} // namespace sofa
