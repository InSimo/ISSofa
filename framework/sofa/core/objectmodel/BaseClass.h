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

#include <sofa/helper/system/config.h>
#include <sofa/helper/vector.h>
#include <sofa/helper/DecodeTypeName.h>
#include <sofa/SofaFramework.h>
#include <sofa/core/objectmodel/SPtr.h>
#include <string>
#include <map>
#include <typeinfo>
#include <tuple>
#include <memory>
#include <type_traits>

namespace sofa
{

namespace core
{

namespace objectmodel
{

class Base;
class Event;
class BaseObjectDescription;

#if !defined(NDEBUG) || defined(SOFA_CHECK_CONTAINER_ACCESS)
//#define SOFA_CLASS_DCAST_COUNT
#define SOFA_CLASS_DCAST_CHECK
#endif

class SOFA_CORE_API BaseClassInfo
{
public:
    const std::string fullTypeName;
    const std::string namespaceName;
    const std::string className;
    const std::string templateName;
    const std::string shortName;
    std::vector<std::string> targetNames;

    template<class T>
    BaseClassInfo(T* ptr = nullptr)
        : fullTypeName(T::fullTypeName(ptr))
        , namespaceName(T::namespaceName(ptr))
        , className(T::className(ptr))
        , templateName(T::templateName(ptr))
        , shortName(T::shortName(ptr))
    {
#ifdef SOFA_TARGET
        targetNames.push_back(sofa_tostring(SOFA_TARGET));
#endif
    }

private:
    BaseClassInfo()
    {
    }
public:
    //BaseClassInfo() = delete;
    BaseClassInfo(BaseClassInfo&&) = default;
    BaseClassInfo(const BaseClassInfo&) = default;
    void operator=(BaseClassInfo&&) = delete;
    void operator=(const BaseClassInfo&) = delete;

    bool operator==(const BaseClassInfo& c) const
    {
        return (this->fullTypeName == c.fullTypeName);
    }

    bool operator!=(const BaseClassInfo& c) const
    {
        return !((*this) == c);
    }

    /// Helper method to decode the type name
    static std::string decodeFullName(const std::type_info& t)
    {
        return sofa::helper::DecodeTypeName::decodeFullName(t);
    }

    /// Helper method to decode the type name to a more readable form if possible
    static std::string decodeTypeName(const std::type_info& t)
    {
        return sofa::helper::DecodeTypeName::decodeTypeName(t);
    }

    /// Helper method to extract the class name (removing namespaces and templates)
    static std::string decodeClassName(const std::type_info& t)
    {
        return sofa::helper::DecodeTypeName::decodeClassName(t);
    }

    /// Helper method to extract the namespace (removing class name and templates)
    static std::string decodeNamespaceName(const std::type_info& t)
    {
        return sofa::helper::DecodeTypeName::decodeNamespaceName(t);
    }

    /// Helper method to extract the template name (removing namespaces and class name)
    static std::string decodeTemplateName(const std::type_info& t)
    {
        return sofa::helper::DecodeTypeName::decodeTemplateName(t);
    }

    /// Helper method to get the full type name
    template<class T>
    static std::string defaultFullTypeName(const T* = nullptr)
    {
        return decodeFullName(typeid(T));
    }

    /// Helper method to get the type name
    template<class T>
    static std::string defaultTypeName(const T* = nullptr)
    {
        return decodeTypeName(typeid(T));
    }

    /// Helper method to get the class name
    template<class T>
    static std::string defaultClassName(const T* = nullptr)
    {
        return decodeClassName(typeid(T));
    }

    /// Helper method to get the namespace name
    template<class T>
    static std::string defaultNamespaceName(const T* = nullptr)
    {
        return decodeNamespaceName(typeid(T));
    }

    /// Helper method to get the template name
    template<class T>
    static std::string defaultTemplateName(const T* = nullptr)
    {
        return decodeTemplateName(typeid(T));
    }
};


/**
 *  \brief Class hierarchy reflection base class
 *
 *  This class provides information on the class and parent classes of components.
 *  It is created by using the SOFA_CLASS macro on each new class declaration.
 *  All classes deriving from Base, Event, BaseMatrix, BaseVector should use the SOFA_*_CLASS macros within their declaration.
 *
 */
template<class TRootType>
class SOFA_CORE_API BaseRootClass : public BaseClassInfo
{
public:
    typedef TRootType RootType;
    typedef BaseRootClass<RootType> RootClass;

protected:
    explicit BaseRootClass(BaseClassInfo&& info);
    virtual ~BaseRootClass();

    static std::size_t NewClassId();

    void linkNewClass();

    // Thread safety structure, private to implementation
    struct DerivedLock;
    DerivedLock* derivedLock;
    helper::vector<const RootClass*> derived;
    virtual void addDerived(const RootClass* c);
    virtual void removeDerived(const RootClass* c);
    virtual void removeParent(const RootClass* c);

public:
    helper::vector<const RootClass*> parents;

    virtual const RootClass* findDerived(const BaseClassInfo& info) const;
    virtual bool findDerived(const BaseClassInfo& info, std::vector<const RootClass*>& result) const;

    void dumpInfo(std::ostream& out, int indent = 0) const;
    void dumpHierarchy(std::ostream& out, int indent = 0) const;

    bool hasParent(const RootClass* c) const
    {
        if (*this == *c) return true;
        for (unsigned int i=0; i<parents.size(); ++i)
            if (parents[i]->hasParent(c)) return true;
        return false;
    }

    bool operator==(const RootClass& c) const
    {
        if (this == &c) return true;
        const BaseClassInfo& thisinfo = *this;
        const BaseClassInfo& cinfo = c;
        return thisinfo == cinfo;
    }

    bool operator!=(const RootClass& c) const
    {
        return !((*this)==c);
    }

    virtual void* dynamicCast(RootType* obj) const = 0;

    virtual bool isInstance(RootType* obj) const = 0;

protected:
    typedef void*(StaticCastFunction)(void*);
    typedef const void*(StaticCastConstFunction)(const void*);
    std::vector<StaticCastFunction*> staticCastFunctions;
    std::vector<StaticCastConstFunction*> staticCastConstFunctions;
    mutable std::size_t dynamicCastCount = 0;

public:
    bool isExternalClass;
    std::size_t classId;
    StaticCastFunction* getStaticCast(std::size_t id) const
    {
        return (id < staticCastFunctions.size()) ? staticCastFunctions[id] : nullptr;
    }
    StaticCastConstFunction* getStaticCastConst(std::size_t id) const
    {
        return (id < staticCastConstFunctions.size()) ? staticCastConstFunctions[id] : nullptr;
    }
};

template<class TTo, class TFrom>
class StaticCastFunctions
{
public:
    static TTo* StaticCast(TFrom* p)
    {
        return p;
    }
    static void* StaticCastVoid(void* p)
    {
        TFrom* pfrom = static_cast<TFrom*>(p);
        TTo* pto = pfrom;
        return pto;
    }
    static const TTo* StaticCastConst(const TFrom* p)
    {
        return p;
    }
    static const void* StaticCastConstVoid(const void* p)
    {
        const TFrom* pfrom = static_cast<const TFrom*>(p);
        const TTo* pto = pfrom;
        return pto;
    }
};

template<class T>
class StaticCastFunctions<T,T>
{
public:
    static T* StaticCast(T* p)
    {
        return p;
    }
    static void* StaticCastVoid(void* p)
    {
        return p;
    }
    static const T* StaticCastConst(const T* p)
    {
        return p;
    }
    static const void* StaticCastConstVoid(const void* p)
    {
        return p;
    }
};

typedef BaseRootClass<Base> BaseClass;

class ClassFlag_External
{
};

class ClassFlag_Unique
{
};

// To specify template classes in C macro parameters, we can't write any commas, hence templates with more than 2 parameters have to use the following macros
#define SOFA_TEMPLATE(Class,P1) Class<P1>
#define SOFA_TEMPLATE2(Class,P1,P2) Class<P1,P2>
#define SOFA_TEMPLATE3(Class,P1,P2,P3) Class<P1,P2,P3>
#define SOFA_TEMPLATE4(Class,P1,P2,P3,P4) Class<P1,P2,P3,P4>
#define SOFA_PARENTS(...) (__VA_ARGS__)

#define SOFA_SINGLE_ARG(...) __VA_ARGS__

#define SOFA_BUNDLE(...) (__VA_ARGS__)
#define SOFA_UNBUNDLE(...) __VA_ARGS__

// this is sometimes required to get around issues with visual c++ preprocessor...
#define SOFA_MACRO_CALL(x) x

// SOFA_NUM_ARGS(...) evaluates to the literal number of the passed-in arguments.
#define SOFA_NUM_ARGS2(X,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define SOFA_NUM_ARGS(...) SOFA_MACRO_CALL(SOFA_NUM_ARGS2(0, __VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0))

#define SOFA_MACRO_CALL_ARGS4(M,N,A) M##N A
#define SOFA_MACRO_CALL_ARGS3(M,N,A) SOFA_MACRO_CALL_ARGS4(M,N,A)
#define SOFA_MACRO_CALL_ARGS2(M,N,A) SOFA_MACRO_CALL_ARGS3(M,N,A)
#define SOFA_MACRO_CALL_ARGS(M,A)    SOFA_MACRO_CALL_ARGS2(M,SOFA_NUM_ARGS A,A)

#define SOFA_TYPEDEF_PARENTS_1(P1) \
    typedef SOFA_UNBUNDLE P1 Inherit1; \
    static_assert(std::is_same<Inherit1, typename Inherit1::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P1)); \
    /* This method is added to insure derived classes use the CLASS macro */  \
    void isClassMacroUsed(Inherit1*) {} \
    typedef std::tuple<SOFA_UNBUNDLE P1*> MyParents

#define SOFA_TYPEDEF_PARENTS_2(P1,P2) \
    typedef SOFA_UNBUNDLE P1 Inherit1; \
    typedef SOFA_UNBUNDLE P2 Inherit2; \
    static_assert(std::is_same<Inherit1, typename Inherit1::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P1)); \
    static_assert(std::is_same<Inherit2, typename Inherit2::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P2)); \
    /* This method is added to insure derived classes use the CLASS macro */  \
    void isClassMacroUsed(Inherit1*) {} \
    void isClassMacroUsed(Inherit2*) {} \
    typedef std::tuple<SOFA_UNBUNDLE P1*, SOFA_UNBUNDLE P2*> MyParents

#define SOFA_TYPEDEF_PARENTS_3(P1,P2,P3) \
    typedef SOFA_UNBUNDLE P1 Inherit1; \
    typedef SOFA_UNBUNDLE P2 Inherit2; \
    typedef SOFA_UNBUNDLE P3 Inherit3; \
    static_assert(std::is_same<Inherit1, typename Inherit1::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P1)); \
    static_assert(std::is_same<Inherit2, typename Inherit2::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P2)); \
    static_assert(std::is_same<Inherit3, typename Inherit3::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P3)); \
    /* This method is added to insure derived classes use the CLASS macro */  \
    void isClassMacroUsed(Inherit1*) {} \
    void isClassMacroUsed(Inherit2*) {} \
    void isClassMacroUsed(Inherit3*) {} \
    typedef std::tuple<SOFA_UNBUNDLE P1*, SOFA_UNBUNDLE P2*, SOFA_UNBUNDLE P3*> MyParents

#define SOFA_TYPEDEF_PARENTS_4(P1,P2,P3,P4) \
    typedef SOFA_UNBUNDLE P1 Inherit1; \
    typedef SOFA_UNBUNDLE P2 Inherit2; \
    typedef SOFA_UNBUNDLE P3 Inherit3; \
    typedef SOFA_UNBUNDLE P4 Inherit4; \
    static_assert(std::is_same<Inherit1, typename Inherit1::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P1)); \
    static_assert(std::is_same<Inherit2, typename Inherit2::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P2)); \
    static_assert(std::is_same<Inherit3, typename Inherit3::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P3)); \
    static_assert(std::is_same<Inherit4, typename Inherit4::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P4)); \
    /* This method is added to insure derived classes use the CLASS macro */  \
    void isClassMacroUsed(Inherit1*) {} \
    void isClassMacroUsed(Inherit2*) {} \
    void isClassMacroUsed(Inherit3*) {} \
    void isClassMacroUsed(Inherit4*) {} \
    typedef std::tuple<SOFA_UNBUNDLE P1*, SOFA_UNBUNDLE P2*, SOFA_UNBUNDLE P3*, SOFA_UNBUNDLE P4*> MyParents

#define SOFA_TYPEDEF_PARENTS_5(P1,P2,P3,P4,P5) \
    typedef SOFA_UNBUNDLE P1 Inherit1; \
    typedef SOFA_UNBUNDLE P2 Inherit2; \
    typedef SOFA_UNBUNDLE P3 Inherit3; \
    typedef SOFA_UNBUNDLE P4 Inherit4; \
    typedef SOFA_UNBUNDLE P5 Inherit5; \
    static_assert(std::is_same<Inherit1, typename Inherit1::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P1)); \
    static_assert(std::is_same<Inherit2, typename Inherit2::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P2)); \
    static_assert(std::is_same<Inherit3, typename Inherit3::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P3)); \
    static_assert(std::is_same<Inherit4, typename Inherit4::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P4)); \
    static_assert(std::is_same<Inherit5, typename Inherit5::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P5)); \
    /* This method is added to insure derived classes use the CLASS macro */  \
    void isClassMacroUsed(Inherit1*) {} \
    void isClassMacroUsed(Inherit2*) {} \
    void isClassMacroUsed(Inherit3*) {} \
    void isClassMacroUsed(Inherit4*) {} \
    void isClassMacroUsed(Inherit5*) {} \
    typedef std::tuple<SOFA_UNBUNDLE P1*, SOFA_UNBUNDLE P2*, SOFA_UNBUNDLE P3*, SOFA_UNBUNDLE P4*, SOFA_UNBUNDLE P5*> MyParents

#define SOFA_TYPEDEF_PARENTS_6(P1,P2,P3,P4,P5,P6) \
    typedef SOFA_UNBUNDLE P1 Inherit1; \
    typedef SOFA_UNBUNDLE P2 Inherit2; \
    typedef SOFA_UNBUNDLE P3 Inherit3; \
    typedef SOFA_UNBUNDLE P4 Inherit4; \
    typedef SOFA_UNBUNDLE P5 Inherit5; \
    typedef SOFA_UNBUNDLE P6 Inherit6; \
    static_assert(std::is_same<Inherit1, typename Inherit1::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P1)); \
    static_assert(std::is_same<Inherit2, typename Inherit2::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P2)); \
    static_assert(std::is_same<Inherit3, typename Inherit3::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P3)); \
    static_assert(std::is_same<Inherit4, typename Inherit4::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P4)); \
    static_assert(std::is_same<Inherit5, typename Inherit5::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P5)); \
    static_assert(std::is_same<Inherit6, typename Inherit6::MyType>::value, "Incorrect CLASS macro in " sofa_tostring(P6)); \
    /* This method is added to insure derived classes use the CLASS macro */  \
    void isClassMacroUsed(Inherit1*) {} \
    void isClassMacroUsed(Inherit2*) {} \
    void isClassMacroUsed(Inherit3*) {} \
    void isClassMacroUsed(Inherit4*) {} \
    void isClassMacroUsed(Inherit5*) {} \
    void isClassMacroUsed(Inherit6*) {} \
    typedef std::tuple<SOFA_UNBUNDLE P1*, SOFA_UNBUNDLE P2*, SOFA_UNBUNDLE P3*, SOFA_UNBUNDLE P4*, SOFA_UNBUNDLE P5*, SOFA_UNBUNDLE P6*> MyParents

#define SOFA_TYPEDEF_PARENTS(A)    SOFA_MACRO_CALL_ARGS(SOFA_TYPEDEF_PARENTS_,A)

#define SOFA_VIRTUAL_PARENTS_1(P1) \
    void isClassMacroUsed(SOFA_UNBUNDLE P1 *) {} \
    friend MyClass

#define SOFA_VIRTUAL_PARENTS_2(P1,P2) \
    void isClassMacroUsed(SOFA_UNBUNDLE P1 *) {} \
    void isClassMacroUsed(SOFA_UNBUNDLE P2 *) {} \
    friend MyClass

#define SOFA_VIRTUAL_PARENTS_3(P1,P2,P3) \
    void isClassMacroUsed(SOFA_UNBUNDLE P1 *) {} \
    void isClassMacroUsed(SOFA_UNBUNDLE P2 *) {} \
    void isClassMacroUsed(SOFA_UNBUNDLE P3 *) {} \
    friend MyClass

#define SOFA_VIRTUAL_PARENTS_4(P1,P2,P3,P4) \
    void isClassMacroUsed(SOFA_UNBUNDLE P1 *) {} \
    void isClassMacroUsed(SOFA_UNBUNDLE P2 *) {} \
    void isClassMacroUsed(SOFA_UNBUNDLE P3 *) {} \
    void isClassMacroUsed(SOFA_UNBUNDLE P4 *) {} \
    friend MyClass

#define SOFA_VIRTUAL_PARENTS(Parents)    SOFA_MACRO_CALL_ARGS(SOFA_VIRTUAL_PARENTS_,Parents)
/*
#define SOFA_TYPEDEF_PARENTS(...) \
    typedef std::tuple<__VA_ARGS__> MyParents; \
    typedef typename std::tuple_element<0,MyParents>::type Inherit1;
*/

#define SOFA_ROOT_CLASS_EXTERNAL(Root) \
    typedef SOFA_UNBUNDLE Root RootType; \
    typedef ::sofa::core::objectmodel::BaseRootClass<RootType> RootClass; \
    typedef RootType MyType; \
    typedef std::tuple<> MyParents; \
    typedef ::sofa::core::objectmodel::ClassFlag_External MyClassFlags; \
    typedef ::sofa::core::objectmodel::TClass< MyType, MyParents, RootType, MyClassFlags > MyClass; \
    SOFA_ROOT_CLASS_DECL; \
    SOFA_CLASS_EXTERNAL_DECL

#define SOFA_SIMPLE_CLASS_FLAGS(Root,T,Parents,Flags) \
    typedef SOFA_UNBUNDLE T MyType; \
    SOFA_TYPEDEF_PARENTS(Parents); \
    typedef SOFA_UNBUNDLE Root RootType; \
    typedef SOFA_UNBUNDLE Flags MyClassFlags; \
    typedef ::sofa::core::objectmodel::TClass< MyType, MyParents, RootType, MyClassFlags > MyClass; \
    SOFA_SIMPLE_CLASS_DECL

#define SOFA_SIMPLE_CLASS_EXTERNAL(Root,T,Parents) \
    SOFA_SIMPLE_CLASS_FLAGS(Root,T,Parents,(::sofa::core::objectmodel::ClassFlag_External)); \
    SOFA_CLASS_EXTERNAL_DECL

#define SOFA_SIMPLE_CLASS_UNIQUE(Root,T,Parents) \
    SOFA_SIMPLE_CLASS_FLAGS(Root,T,Parents,(::sofa::core::objectmodel::ClassFlag_Unique)); \
    SOFA_CLASS_UNIQUE_DECL

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
    SOFA_ABSTRACT_CLASS_FLAGS(T,Parents,(::sofa::core::objectmodel::ClassFlag_External)); \
    SOFA_CLASS_EXTERNAL_DECL
#define SOFA_CLASS_EXTERNAL(T,Parents) \
    SOFA_CLASS_FLAGS(T,Parents,(::sofa::core::objectmodel::ClassFlag_External)); \
    SOFA_CLASS_EXTERNAL_DECL

#define SOFA_ABSTRACT_CLASS_UNIQUE(T,Parents) \
    SOFA_ABSTRACT_CLASS_FLAGS(T,Parents,(::sofa::core::objectmodel::ClassFlag_Unique)); \
    SOFA_CLASS_UNIQUE_DECL
#define SOFA_CLASS_UNIQUE(T,Parents) \
    SOFA_CLASS_FLAGS(T,Parents,(::sofa::core::objectmodel::ClassFlag_Unique)); \
    SOFA_CLASS_UNIQUE_DECL

#define SOFA_EVENT_CLASS_EXTERNAL(T,Parents) SOFA_SIMPLE_CLASS_EXTERNAL((::sofa::core::objectmodel::Event),T,Parents)
#define SOFA_EVENT_CLASS_UNIQUE(T,Parents) SOFA_SIMPLE_CLASS_UNIQUE((::sofa::core::objectmodel::Event),T,Parents)

#define SOFA_CLASS_VIRTUAL_PARENTS(...) \
    SOFA_VIRTUAL_PARENTS((__VA_ARGS__))

#define SOFA_ROOT_CLASS_DECL                                                  \
    virtual const RootClass* getClass() const = 0;                            \
    void isClassMacroUsed(MyType*) {}                                         \
    virtual std::pair<const RootClass*,const void*> getClassAndVoidPtr() const = 0; \
    virtual std::pair<const RootClass*,void*> getClassAndVoidPtr() = 0;       \
                                                                              \
    /* Helper to get the full name of a type derived from this class        */\
    /*                                                                      */\
    /* This method should be used as follow :                               */\
    /* \code  std::string type = T::fullTypeName((T*)nullptr); \endcode     */\
    /* This way derived classes can redefine the fullTypeName method        */\
    template<class T>                                                         \
    static std::string fullTypeName(const T* ptr= nullptr)                    \
    {                                                                         \
        return RootClass::defaultFullTypeName(ptr);                           \
    }                                                                         \
                                                                              \
    /* Helper to get the type name of a type derived from this class        */\
    /*                                                                      */\
    /* This method should be used as follow :                               */\
    /* \code  std::string type = T::typeName((T*)nullptr); \endcode         */\
    /* This way derived classes can redefine the typeName method            */\
    template<class T>                                                         \
    static std::string typeName(const T* ptr= nullptr)                        \
    {                                                                         \
        return RootClass::defaultTypeName(ptr);                               \
    }                                                                         \
                                                                              \
    /* Helper to get the class name of a type derived from this class       */\
    /*                                                                      */\
    /* This method should be used as follow :                               */\
    /* \code  std::string type = T::className((T*)nullptr); \endcode        */\
    /* This way derived classes can redefine the className method           */\
    template<class T>                                                         \
    static std::string className(const T* ptr= nullptr)                       \
    {                                                                         \
        return RootClass::defaultClassName(ptr);                              \
    }                                                                         \
                                                                              \
    /* Helper to get the namespace of a type derived from this class        */\
    /*                                                                      */\
    /* This method should be used as follow :                               */\
    /* \code  std::string type = T::namespaceName((T*)nullptr); \endcode    */\
    /* This way derived classes can redefine the namespaceName method       */\
    template<class T>                                                         \
    static std::string namespaceName(const T* ptr= nullptr)                   \
    {                                                                         \
        return RootClass::defaultNamespaceName(ptr);                          \
    }                                                                         \
                                                                              \
    /* Helper to get the template name of a type derived from this class    */\
    /*                                                                      */\
    /* This method should be used as follow :                               */\
    /* \code  std::string type = T::templateName((T*)nullptr); \endcode     */\
    /* This way derived classes can redefine the templateName method        */\
    template<class T>                                                         \
    static std::string templateName(const T* ptr= nullptr)                    \
    {                                                                         \
        return RootClass::defaultTemplateName(ptr);                           \
    }                                                                         \
                                                                              \
    /* Helper to get the shortname of a type derived from this class.       */\
    /* The default implementation return the class name.                    */\
    /*                                                                      */\
    /* This method should be used as follow :                               */\
    /* \code  std::string type = T::shortName((T*)nullptr); \endcode        */\
    /* This way derived classes can redefine the shortName method           */\
    template< class T>                                                        \
    static std::string shortName( const T* ptr = nullptr,                     \
        ::sofa::core::objectmodel::BaseObjectDescription* = nullptr )         \
    {                                                                         \
        std::string shortname = T::className(ptr);                            \
        if( !shortname.empty() )                                              \
        {                                                                     \
            *shortname.begin() = ::tolower(*shortname.begin());               \
        }                                                                     \
        return shortname;                                                     \
    }                                                                         \
    typedef MyType* Ptr

// Do not use this macro directly, use SOFA_*_CLASS instead
#define SOFA_SIMPLE_CLASS_DECL                                                \
    typedef ::sofa::core::objectmodel::BaseRootClass<RootType> RootClass;     \
    template<class TPtr>                                                      \
    static MyType* DynamicCast(TPtr* p) { return MyClass::DynamicCast(p); }   \
    template<class TPtr>                                                      \
    static const MyType* DynamicCast(const TPtr* p)                           \
    { return MyClass::DynamicCast(p); }                                       \
    const RootClass* getClass() const override                                \
    { return GetClass(); }                                                    \
    std::pair<const RootClass*,const void*> getClassAndVoidPtr() const override \
    { const void* ptr = this; return std::make_pair(GetClass(), ptr); }       \
    std::pair<const RootClass*,void*> getClassAndVoidPtr() override           \
    { void* ptr = this; return std::make_pair(GetClass(), ptr); }             \
	static const char* HeaderFileLocation() { return __FILE__; }              \
    typedef MyType* Ptr

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

// Do not use this macro directly, use SOFA_ABSTRACT_CLASS instead
#define SOFA_ABSTRACT_CLASS_DECL                                              \
    /* This method is added to insure derived classes use the CLASS macro */  \
    virtual void isClassMacroUsed(MyType*) = 0

// Do not use this macro directly, use SOFA_CLASS instead
#define SOFA_CLASS_DECL                                                       \
    friend class sofa::core::objectmodel::New<MyType>

#define SOFA_CLASS_DEFAULT_DECL                                               \
    static const MyClass* GetClass()                                          \
    {                                                                         \
        static MyClass singleton;                                             \
        return &singleton;                                                    \
    }                                                                         \
    static std::size_t GetClassId()                                           \
    {                                                                         \
        return 0;                                                             \
    }                                                                         \
    friend MyClass

#define SOFA_CLASS_UNIQUE_DECL                                                \
    static_assert(Inherit1::MyClass::IsExternalClass,                         \
                  "Inherit1 must be unique/external class");                  \
    static const MyClass* GetClass()                                          \
    {                                                                         \
        static const std::shared_ptr<MyClass> singleton =                     \
            MyClass::CreateUnique();                                          \
        return singleton.get();                                               \
    }                                                                         \
    static std::size_t GetClassId()                                           \
    {                                                                         \
        static const std::size_t id = GetClass()->classId;                    \
        return id;                                                            \
    }                                                                         \
    friend MyClass

#define SOFA_CLASS_EXTERNAL_DECL                                              \
private:                                                                      \
    static std::size_t ExternalClassId;                                       \
public:                                                                       \
    static const MyClass* GetClass();                                         \
    static std::size_t GetClassId()                                           \
    {                                                                         \
        static const MyClass* c = GetClass();                                 \
        SOFA_UNUSED(c);                                                       \
        return ExternalClassId;                                               \
        /* return c->classId; */                                              \
    }                                                                         \
    friend MyClass

#define SOFA_CLASS_EXTERNAL_IMPL(T)                                           \
    auto SOFA_UNBUNDLE T::GetClass() -> const MyClass*                        \
    {                                                                         \
        static MyClass singleton(&ExternalClassId);                           \
        return &singleton;                                                    \
    }                                                                         \
    std::size_t SOFA_UNBUNDLE T ::ExternalClassId = 0

#define SOFA_ROOT_CLASS_IMPL(T) SOFA_CLASS_EXTERNAL_IMPL(T)
#define SOFA_ABSTRACT_CLASS_IMPL(T) SOFA_CLASS_EXTERNAL_IMPL(T)
#define SOFA_CLASS_IMPL(T) SOFA_CLASS_EXTERNAL_IMPL(T)
#define SOFA_EVENT_CLASS_IMPL(T) SOFA_CLASS_EXTERNAL_IMPL(T)

template <class T, class Parents, std::size_t N = std::tuple_size<Parents>::value>
class TClassParents
{
public:
    static const std::size_t I = N-1;
    typedef typename std::tuple_element<I,Parents>::type tptr;
    typedef typename std::remove_pointer<tptr>::type type;
    typedef typename type::MyClass MyClass;
    static_assert(std::is_same<type, typename type::MyType>::value, "Incorrect CLASS macro in one of the parent types");

    template<class VecClass>
    static void fillClasses(VecClass& dest)
    {
        TClassParents<T,Parents,I>::fillClasses(dest);
        dest[I] = type::GetClass();
    }
    static void getVecCastSize(std::size_t& res)
    {
        TClassParents<T,Parents,I>::getVecCastSize(res);
        TClassParents<T,typename MyClass::MyParents>::getVecCastSize(res);
        const std::size_t id = type::GetClassId();
        if (id > 0 && id+1 > res) res = id+1;
    }
    typedef StaticCastFunctions<type,T> MyCastFunctions;
    template<class VecCast, class VecConstCast>
    static void fillVecCast(VecCast& vCast, VecConstCast& vConstCast)
    {
        TClassParents<T,Parents,I>::fillVecCast(vCast, vConstCast);
        TClassParents<T,typename MyClass::MyParents>::fillVecCast(vCast, vConstCast);
        const std::size_t id = type::GetClassId();
        if (id > 0)
        {
            vCast[id] = MyCastFunctions::StaticCastVoid;
            vConstCast[id] = MyCastFunctions::StaticCastConstVoid;
        }
    }
};

template <class T, class Parents>
class TClassParents<T,Parents,0>
{
public:
    template<class VecClass>
    static void fillClasses(VecClass&)
    {
    }
    static void getVecCastSize(std::size_t&)
    {
    }
    template<class VecCast, class VecConstCast>
    static void fillVecCast(VecCast&, VecConstCast&)
    {
    }
};

template <class T, class Parents, class RootType, class Flags>
class TClass;

template <class T, class Parents, class RootType>
class TClass<T, Parents, RootType, void> : public BaseRootClass<RootType>
{
public:
    typedef T MyType;
    typedef Parents MyParents;
    typedef RootType MyRootType;
    typedef BaseRootClass<MyRootType> MyRootClass;
    typedef void MyClassFlags;
    typedef TClass<MyType, MyParents, MyRootType, MyClassFlags> MyClass;
    static const bool IsExternalClass = false;
private:
    friend MyType;
    friend MyRootClass;
    TClass()
    : MyRootClass(BaseClassInfo((T*)nullptr))
    {
        this->parents.resize(std::tuple_size<Parents>::value);
        TClassParents<T,Parents >::fillClasses(this->parents);
        this->isExternalClass = IsExternalClass;
        this->classId = T::GetClassId();
        std::size_t vCastSize = 0;
        TClassParents<T,Parents>::getVecCastSize(vCastSize);
        if (this->isExternalClass && this->classId+1 > vCastSize)
        {
            vCastSize = this->classId+1;
        }
        this->staticCastFunctions.resize(vCastSize);
        this->staticCastConstFunctions.resize(vCastSize);
        TClassParents<T,Parents>::fillVecCast(this->staticCastFunctions, this->staticCastConstFunctions);
        if (this->isExternalClass)
        {
            typedef StaticCastFunctions<T,T> MyCastFunctions;
            this->staticCastFunctions[this->classId] = MyCastFunctions::StaticCastVoid;
            this->staticCastConstFunctions[this->classId] = MyCastFunctions::StaticCastConstVoid;
        }

        this->linkNewClass();
    }
    virtual ~TClass() {}

public:
    
    virtual void* dynamicCast(RootType* obj) const override
    {
        return DynamicCast(obj);
    }

    virtual bool isInstance(RootType* obj) const override
    {
        return IsInstance(obj);
    }

    template<class TPtr>
    static T* DynamicCast(TPtr* p)
    {
#ifdef SOFA_CLASS_DCAST_COUNT
        ++T::GetClass()->dynamicCastCount;
#endif
        return dynamic_cast<T*>(p);
    }

    template<class TPtr>
    static const T* DynamicCast(const TPtr* p)
    {
#ifdef SOFA_CLASS_DCAST_COUNT
        ++T::GetClass()->dynamicCastCount;
#endif
        return dynamic_cast<const T*>(p);
    }

    template<class TPtr>
    static bool IsInstance(const TPtr* p)
    {
#ifdef SOFA_CLASS_DCAST_COUNT
        ++T::GetClass()->dynamicCastCount;
#endif
        return dynamic_cast<const T*>(p) != nullptr;
    }
};

template <class T, class Parents, class RootType>
class TClass<T, Parents, RootType, ClassFlag_External> : public BaseRootClass<RootType>
{
public:
    typedef T MyType;
    typedef Parents MyParents;
    typedef RootType MyRootType;
    typedef BaseRootClass<MyRootType> MyRootClass;
    typedef ClassFlag_External MyClassFlags;
    typedef TClass<MyType, MyParents, MyRootType, MyClassFlags> MyClass;
    static const bool IsExternalClass = true;
protected:
    friend MyType;
    friend MyRootClass;
    explicit TClass(std::size_t* pExternalClassId = nullptr)
    : MyRootClass(BaseClassInfo((T*)nullptr))
    {
        this->parents.resize(std::tuple_size<Parents>::value);
        TClassParents<T,Parents >::fillClasses(this->parents);
        this->isExternalClass = IsExternalClass;
        this->classId = MyRootClass::NewClassId();
        if (pExternalClassId)
        {
            *pExternalClassId = this->classId;
        }
        std::size_t vCastSize = 0;
        TClassParents<T,Parents       >::getVecCastSize(vCastSize);
        if (this->isExternalClass && this->classId+1 > vCastSize)
        {
            vCastSize = this->classId+1;
        }
        this->staticCastFunctions.resize(vCastSize);
        this->staticCastConstFunctions.resize(vCastSize);
        TClassParents<T,Parents       >::fillVecCast(this->staticCastFunctions, this->staticCastConstFunctions);
        if (this->isExternalClass)
        {
            typedef StaticCastFunctions<T,T> MyCastFunctions;
            this->staticCastFunctions[this->classId] = MyCastFunctions::StaticCastVoid;
            this->staticCastConstFunctions[this->classId] = MyCastFunctions::StaticCastConstVoid;
        }

        this->linkNewClass();
    }
public:
    virtual ~TClass()
    {
    }

    virtual void* dynamicCast(RootType* obj) const override
    {
        return DynamicCast(obj);
    }

    virtual bool isInstance(RootType* obj) const override
    {
        return IsInstance(obj);
    }

    template<class TPtr>
    static T* DynamicCast(TPtr* p)
    {
#ifdef SOFA_CLASS_DCAST_COUNT
        ++T::GetClass()->dynamicCastCount;
#endif
        T* result;
        if (!p)
        {
            result = nullptr;
        }
        else
        {
            std::pair<const MyRootClass*,void*> pClassAndPtr = p->getClassAndVoidPtr();
            const MyRootClass* pClass = pClassAndPtr.first;
            void* ptr = pClassAndPtr.second;
            typename MyRootClass::StaticCastFunction* pCastFn = pClass->getStaticCast(T::GetClassId());
            if (!pCastFn)
            {
                result = nullptr;
            }
            else
            {
                result = static_cast<T*>((*pCastFn)(ptr));
            }
        }
#ifdef SOFA_CLASS_DCAST_CHECK
        T* dcast = dynamic_cast<T*>(p);
        if (dcast != result)
        {
            std::cerr << __FILE__ << ":" << __LINE__ << ": " << T::GetClass()->className<<"::DynamicCast(" << (p?p->getClass()->className : TPtr::GetClass()->className + " nullptr") << ") " << result << " != dynamic_cast " << dcast << std::endl;
            T::GetClass()->dumpHierarchy(std::cerr);
            if (p)
                p->getClass()->dumpHierarchy(std::cerr);
            TPtr::GetClass()->dumpHierarchy(std::cerr);
            result = dcast;
        }
#endif
        return result;
    }

    template<class TPtr>
    static const T* DynamicCast(const TPtr* p)
    {
#ifdef SOFA_CLASS_DCAST_COUNT
        ++T::GetClass()->dynamicCastCount;
#endif
        const T* result;
        if (!p)
        {
            result = nullptr;
        }
        else
        {
            std::pair<const MyRootClass*,const void*> pClassAndPtr = p->getClassAndVoidPtr();
            const MyRootClass* pClass = pClassAndPtr.first;
            const void* ptr = pClassAndPtr.second;
            typename MyRootClass::StaticCastConstFunction* pCastFn = pClass->getStaticCastConst(T::GetClassId());
            if (!pCastFn)
            {
                result = nullptr;
            }
            else
            {
                result = static_cast<const T*>((*pCastFn)(ptr));
            }
        }
#ifdef SOFA_CLASS_DCAST_CHECK
        const T* dcast = dynamic_cast<const T*>(p);
        if (dcast != result)
        {
            std::cerr << __FILE__ << ":" << __LINE__ << ": " << T::GetClass()->className<<"::DynamicCast(const " << (p?p->getClass()->className : TPtr::GetClass()->className + " nullptr") << ") " << result << " != dynamic_cast " << dcast << std::endl;
            T::GetClass()->dumpHierarchy(std::cerr);
            if (p)
                p->getClass()->dumpHierarchy(std::cerr);
            TPtr::GetClass()->dumpHierarchy(std::cerr);
            result = dcast;
        }
#endif
        return result;

        //if (!p) return nullptr;
        //const MyRootClass* pClass = p->getClass();
        //typename MyRootClass::StaticCastConstFunction* pCastFn = pClass->getStaticCastConst(T::GetClassId());
        //if (!pCastFn) return nullptr;
        //return static_cast<const T*>((*pCastFn)(p));
    }

    template<class TPtr>
    static bool IsInstance(const TPtr* p)
    {
#ifdef SOFA_CLASS_DCAST_COUNT
        ++T::GetClass()->dynamicCastCount;
#endif
        bool result;
        if (!p)
        {
            result = false;
        }
        else
        {
            const MyRootClass* pClass = p->getClass();
            typename MyRootClass::StaticCastConstFunction* pCastFn = pClass->getStaticCastConst(T::GetClassId());
            result = (pCastFn != nullptr);
        }
#ifdef SOFA_CLASS_DCAST_CHECK
        bool dcast = (dynamic_cast<const T*>(p) != nullptr);
        if (dcast != result)
        {
            std::cerr << __FILE__ << ":" << __LINE__ << ": " << T::GetClass()->className<<"::IsInstance(" << (p?p->getClass()->className : TPtr::GetClass()->className + " nullptr") << ") " << result << " != dynamic_cast " << dcast << std::endl;
            T::GetClass()->dumpHierarchy(std::cerr);
            if (p)
                p->getClass()->dumpHierarchy(std::cerr);
            TPtr::GetClass()->dumpHierarchy(std::cerr);
            result = dcast;
        }
#endif
        return result;

        //if (!p) return false;
        //const MyRootClass* pClass = p->getClass();
        //typename MyRootClass::StaticCastConstFunction* pCastFn = pClass->getStaticCastConst(T::GetClassId());
        //if (!pCastFn) return false;
        //return true;
    }
};

template <class T, class Parents, class RootType>
class TClass<T, Parents, RootType, ClassFlag_Unique> :
    public TClass<T, Parents, RootType, ClassFlag_External> ,
    public std::enable_shared_from_this<TClass<T, Parents, RootType, ClassFlag_Unique>>
{
public:
    typedef T MyType;
    typedef BaseRootClass<RootType> RootClass;
    typedef TClass<T, Parents, RootType, ClassFlag_Unique> MyClass;

protected:
    TClass()
    {
    }

public:

    static std::shared_ptr<MyClass> CreateUnique()
    {
        // de-deduplication to support multiple identical classes in different dlls on windows
        // the test below is now done within the macro, so that the error is given at the best place in code
        //static_assert(MyType::Inherit1::MyClass::IsExternalClass, "Inherit1 must be unique/external class");
        typename MyType* ptr = nullptr;
        BaseClassInfo info(ptr);
        const MyClass* existingClass = nullptr;
        std::vector<const RootClass*> existingClasses;
        if (MyType::Inherit1::GetClass()->findDerived(info, existingClasses))
        {
            for (const RootClass* c : existingClasses)
            {
                existingClass = dynamic_cast<const MyClass*>(c);
                if (existingClass) break;
            }
            if (!existingClass)
            {
                std::cerr << "BaseRootClass" << RootType::GetClass()->className
                    << ">::CreateUnique(" << info.fullTypeName << ", "
                    << MyType::Inherit1::GetClass()->fullTypeName << "): "
                    << existingClasses.size() << " entries found but dynamic_cast failed."
                    << std::endl;
                for (const RootClass* c : existingClasses)
                {
                    c->dumpInfo(std::cerr);
                }
            }
        }
        if (!existingClass)
        {
            return std::shared_ptr<MyClass>(new MyClass);
        }
        else
        {
            MyClass* res = const_cast<MyClass*>(existingClass);
            if (!info.targetNames.empty())
            {
                res->targetNames.insert(res->targetNames.end(), info.targetNames.cbegin(), info.targetNames.cend());
            }
            return res->shared_from_this();
        }
    }

};

// DEPRECATED macros, use new syntax instead for new classes

#define SOFA_ABSTRACT_CLASS(T,P1) SOFA_ABSTRACT_CLASS_DEFAULT((T),((P1)))
#define SOFA_ABSTRACT_CLASS2(T,P1,P2) SOFA_ABSTRACT_CLASS_DEFAULT((T),((P1),(P2)))
#define SOFA_ABSTRACT_CLASS3(T,P1,P2,P3) SOFA_ABSTRACT_CLASS_DEFAULT((T),((P1),(P2),(P3)))
#define SOFA_ABSTRACT_CLASS4(T,P1,P2,P3,P4) SOFA_ABSTRACT_CLASS_DEFAULT((T),((P1),(P2),(P3),(P4)))
#define SOFA_ABSTRACT_CLASS5(T,P1,P2,P3,P4,P5) SOFA_ABSTRACT_CLASS_DEFAULT((T),((P1),(P2),(P3),(P4),(P5)))
#define SOFA_ABSTRACT_CLASS6(T,P1,P2,P3,P4,P5,P6) SOFA_ABSTRACT_CLASS_DEFAULT((T),((P1),(P2),(P3),(P4),(P5),(P6)))
#define SOFA_CLASS(T,P1) SOFA_CLASS_DEFAULT((T),((P1)))
#define SOFA_CLASS2(T,P1,P2) SOFA_CLASS_DEFAULT((T),((P1),(P2)))
#define SOFA_CLASS3(T,P1,P2,P3) SOFA_CLASS_DEFAULT((T),((P1),(P2),(P3)))
#define SOFA_CLASS4(T,P1,P2,P3,P4) SOFA_CLASS_DEFAULT((T),((P1),(P2),(P3),(P4)))
#define SOFA_CLASS5(T,P1,P2,P3,P4,P5) SOFA_CLASS_DEFAULT((T),((P1),(P2),(P3),(P4),(P5)))
#define SOFA_CLASS6(T,P1,P2,P3,P4,P5,P6) SOFA_CLASS_DEFAULT((T),((P1),(P2),(P3),(P4),(P5),(P6)))

} // namespace objectmodel

} // namespace core

} // namespace sofa

#endif
