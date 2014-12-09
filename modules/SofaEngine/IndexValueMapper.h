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
#ifndef INDEXVALUEMAPPER_H_
#define INDEXVALUEMAPPER_H_

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/SofaGeneral.h>
#include <sofa/core/DataEngine.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace engine
{

    
template <class VecT>
class IndexValueMapper : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(IndexValueMapper,VecT),sofa::core::DataEngine);
    typedef VecT VecValue;
    typedef typename VecValue::value_type Value;
    typedef unsigned int Index;

protected:

    IndexValueMapper();
    ~IndexValueMapper() {}
public:
    void init();
    void reinit();
    void update();

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const IndexValueMapper<VecT>* = NULL)
    {
        return Data<Value>::templateName();
    }

    //Input
    Data< VecValue > f_inputValues;
    Data<sofa::helper::vector<Index> > f_indices;
    Data<Value> f_value;

    //Output
    Data<VecValue > f_outputValues;

    //Parameter
    Data<Value> p_defaultValue;

};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(INDEXVALUEMAPPER_CPP_)

extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<int> >;
extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<bool> >;
//extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<std::string> >;
#ifndef SOFA_FLOAT
extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<double> >;
extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec2d> >;
extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec3d> >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2dTypes::VecCoord >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2dTypes::VecDeriv >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3dTypes::VecCoord >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3dTypes::VecDeriv >;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<float> >;
extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec2f> >;
extern template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec3f> >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2fTypes::VecCoord >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2fTypes::VecDeriv >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3fTypes::VecCoord >;
extern template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3fTypes::VecDeriv >;
#endif //SOFA_DOUBLE
#endif


} // namespace engine

} // namespace component

} // namespace sofa

#endif /* INDEXVALUEMAPPER_H_ */
