/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_OBJECTMODEL_BASECLASS_H
#define SOFA_CORE_OBJECTMODEL_BASECLASS_H

#include <sofa/defaulttype/BaseClass.h>

namespace sofa
{
namespace core
{
namespace objectmodel
{

class Base;


typedef sofa::defaulttype::BaseRootClass<Base> BaseClass;

// Do not use this macro directly, use SOFA_*_CLASS instead
#define SOFA_BASE_CLASS_DECL                                                  \
    typedef boost::intrusive_ptr<MyType> SPtr;                                \
    template<class SOFA_T> ::sofa::core::objectmodel::BaseData::BaseInitData  \
    initData(::sofa::core::objectmodel::Data<SOFA_T>* field,                  \
             const char* name, const char* help,                              \
             ::sofa::core::objectmodel::BaseData::DataFlags dataflags)        \
    {                                                                         \
        ::sofa::core::objectmodel::BaseData::BaseInitData res;                \
        this->initData0(field, res, name, help, dataflags);                   \
        res.ownerClass = GetClass()->className.c_str();                       \
        return res;                                                           \
    }                                                                         \
    template<class SOFA_T> ::sofa::core::objectmodel::BaseData::BaseInitData  \
    initData(::sofa::core::objectmodel::Data<SOFA_T>* field,                  \
             const char* name, const char* help,                              \
             bool isDisplayed=true, bool isReadOnly=false)                    \
    {                                                                         \
        ::sofa::core::objectmodel::BaseData::BaseInitData res;                \
        this->initData0(field, res, name, help,                               \
                        isDisplayed, isReadOnly);                             \
        res.ownerClass = GetClass()->className.c_str();                       \
        return res;                                                           \
    }                                                                         \
    ::sofa::core::objectmodel::BaseData::BaseInitData                         \
    initData(const char* name, const char* help)                              \
    {                                                                         \
        ::sofa::core::objectmodel::BaseData::BaseInitData res;                \
        this->initData0(res, name, help);                                     \
        res.ownerClass = GetClass()->className.c_str();                       \
        return res;                                                           \
    }                                                                         \
    template<class SOFA_T>                                                    \
    typename ::sofa::core::objectmodel::Data<SOFA_T>::InitData initData(      \
        ::sofa::core::objectmodel::Data<SOFA_T>* field, const SOFA_T& value,  \
        const char* name, const char* help,                                   \
        bool isDisplayed=true, bool isReadOnly=false)                         \
    {                                                                         \
        typename ::sofa::core::objectmodel::Data<SOFA_T>::InitData res;       \
        this->initData0(field, res, value, name, help,                        \
                        isDisplayed, isReadOnly);                             \
        res.ownerClass = GetClass()->className.c_str();                       \
        return res;                                                           \
    }                                                                         \
    ::sofa::core::objectmodel::BaseLink::InitLink<MyType>                     \
    initLink(const char* name, const char* help)                              \
    {                                                                         \
        return ::sofa::core::objectmodel::BaseLink::InitLink<MyType>          \
            (this, name, help);                                               \
    }                                                                         \
    using Inherit1::sout;                                                     \
    using Inherit1::serr;                                                     \
    using Inherit1::sendl


#define SOFA_ABSTRACT_CLASS_FLAGS(T,Parents,Flags) \
    SOFA_SIMPLE_CLASS_FLAGS((::sofa::core::objectmodel::Base),T,Parents,Flags); \
    SOFA_BASE_CLASS_DECL; \
    SOFA_ABSTRACT_CLASS_DECL

#define SOFA_CLASS_FLAGS(T,Parents,Flags) \
    SOFA_SIMPLE_CLASS_FLAGS((::sofa::core::objectmodel::Base),T,Parents,Flags); \
    SOFA_BASE_CLASS_DECL; \
    SOFA_CLASS_DECL

#define SOFA_ABSTRACT_CLASS_DEFAULT(T,Parents) \
    SOFA_ABSTRACT_CLASS_FLAGS(T,Parents,(void)); \
    SOFA_CLASS_DEFAULT_DECL
#define SOFA_CLASS_DEFAULT(T,Parents) \
    SOFA_CLASS_FLAGS(T,Parents,(void)); \
    SOFA_CLASS_DEFAULT_DECL

#define SOFA_ABSTRACT_CLASS_EXTERNAL(T,Parents) \
    SOFA_ABSTRACT_CLASS_FLAGS(T,Parents,(::sofa::defaulttype::ClassFlag_External)); \
    SOFA_CLASS_EXTERNAL_DECL
#define SOFA_CLASS_EXTERNAL(T,Parents) \
    SOFA_CLASS_FLAGS(T,Parents,(::sofa::defaulttype::ClassFlag_External)); \
    SOFA_CLASS_EXTERNAL_DECL

#define SOFA_ABSTRACT_CLASS_UNIQUE(T,Parents) \
    SOFA_ABSTRACT_CLASS_FLAGS(T,Parents,(::sofa::defaulttype::ClassFlag_Unique)); \
    SOFA_CLASS_UNIQUE_DECL
#define SOFA_CLASS_UNIQUE(T,Parents) \
    SOFA_CLASS_FLAGS(T,Parents,(::sofa::defaulttype::ClassFlag_Unique)); \
    SOFA_CLASS_UNIQUE_DECL

}

}

}

#endif

