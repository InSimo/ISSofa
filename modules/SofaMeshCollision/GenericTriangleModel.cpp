/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/


#include "GenericTriangleModel.inl"

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
#ifndef SOFA_FLOAT
template class SOFA_MESH_COLLISION_API GenericTriangleModel<Vec3dTypes>;
template class SOFA_MESH_COLLISION_API GenericTriangleModel<Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_MESH_COLLISION_API GenericTriangleModel<Vec3fTypes>;
template class SOFA_MESH_COLLISION_API GenericTriangleModel<Rigid3fTypes>;
#endif

} // namespace collision

} // namespace component

} // namespace sofa 

