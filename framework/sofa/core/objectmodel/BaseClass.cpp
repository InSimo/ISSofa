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
#include <sofa/core/objectmodel/BaseClass.h>
#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/defaulttype/BaseMatrix.h>
#ifdef SOFA_HAVE_BOOST_THREAD
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp> 
#endif

namespace sofa
{

namespace core
{

namespace objectmodel
{

template<class RootType>
struct BaseRootClass<RootType>::DerivedLock
{
#ifdef SOFA_HAVE_BOOST_THREAD
    boost::mutex updateMutex;
#endif
    DerivedLock()
    {}
};

template<class RootType>
BaseRootClass<RootType>::BaseRootClass(BaseClassInfo&& info)
: BaseClassInfo(std::move(info))
, derivedLock(new DerivedLock)
, isExternalClass(false)
, classId(0)
{
}

template<class RootType>
BaseRootClass<RootType>::~BaseRootClass()
{
#ifdef SOFA_CLASS_DCAST_COUNT
    if (dynamicCastCount>0)
    {
        std::cout << dynamicCastCount << " DynamicCast to " << RootType::className((RootType*)nullptr) << " " << className;
        if (!templateName.empty()) std::cout << "<" << templateName << ">";
        if (isExternalClass) std::cout << " ID " << classId;
        std::cout << std::endl;
    }
#endif
    delete derivedLock;
}

template<class RootType>
std::size_t BaseRootClass<RootType>::NewClassId()
{
    static std::size_t lastId = 0;
    ++lastId;
    return lastId;
}

template<class RootType>
void BaseRootClass<RootType>::dumpHierarchy(std::ostream& out, int indent) const
{
    dumpInfo(out, indent);
    ++indent;
    for (std::size_t i = 0; i < parents.size(); ++i)
    {
        parents[i]->dumpHierarchy(out, indent);
    }
}

template<class RootType>
void BaseRootClass<RootType>::dumpInfo(std::ostream& out, int indent) const
{
    for(;indent > 0;--indent) out << "    ";
    if (isExternalClass)
        out << "ID " << classId << " ";
    out << "class " << className;
    if (!templateName.empty()) out << '<' << templateName << '>';
    if (!namespaceName.empty()) out << " in " << namespaceName;
    if (!shortName.empty()) out << " short " << shortName;
    out << std::endl;
}

template<class RootType>
void BaseRootClass<RootType>::linkNewClass()
{
    for (std::size_t i = 0; i < parents.size(); ++i)
    {
        const_cast<RootClass*>(parents[i])->addDerived(this);
    }
    /*
    if (isExternalClass)
    {
        std::cout << "NEW ";
        dumpInfo(std::cout);
    }
    */
}

template<class TRootType>
void BaseRootClass<TRootType>::addDerived(const RootClass * c)
{
#ifdef SOFA_HAVE_BOOST_THREAD
    boost::lock_guard<boost::mutex> guard(derivedLock->updateMutex);
#endif
    derived.push_back(c);
}

template<class TRootType>
void BaseRootClass<TRootType>::removeDerived(const RootClass * c)
{
#ifdef SOFA_HAVE_BOOST_THREAD
    boost::lock_guard<boost::mutex> guard(derivedLock->updateMutex);
#endif
    auto it = std::find(derived.begin(), derived.end(), c);
    if (it != derived.end())
    {
        derived.erase(it);
    }
}

template<class TRootType>
void BaseRootClass<TRootType>::removeParent(const RootClass * c)
{
    auto it = std::find(parents.begin(), parents.end(), c);
    if (it != parents.end())
    {
        *it = nullptr;
    }
}

template<class TRootType>
auto BaseRootClass<TRootType>::findDerived(const BaseClassInfo & info) const -> const RootClass *
{
#ifdef SOFA_HAVE_BOOST_THREAD
    boost::lock_guard<boost::mutex> guard(derivedLock->updateMutex);
#endif
    auto it = std::find_if(derived.begin(), derived.end(), [&info](const RootClass* c) -> bool
    {
        return info == *c;
    });
    const RootClass* res = nullptr;
    if (it != derived.end())
    {
        res = *it;
    }
    return res;
}

template<class TRootType>
bool BaseRootClass<TRootType>::findDerived(const BaseClassInfo & info, std::vector<const RootClass*>& result) const
{
#ifdef SOFA_HAVE_BOOST_THREAD
    boost::lock_guard<boost::mutex> guard(derivedLock->updateMutex);
#endif
    bool res = false;
    for (const RootClass* c : derived)
    {
        if (info == *c)
        {
            result.push_back(c);
            res = true;
        }
    }
    return res;
}

template<class TRootType>
void BaseRootClass<TRootType>::insertTargetNames(const BaseClassInfo & info)
{
#ifdef SOFA_HAVE_BOOST_THREAD
    boost::lock_guard<boost::mutex> guard(derivedLock->updateMutex);
#endif
    if (!info.targetNames.empty())
    {
        targetNames.insert(targetNames.end(), info.targetNames.cbegin(), info.targetNames.cend());
    }
}

template class SOFA_CORE_API BaseRootClass< Base >;
template class SOFA_CORE_API BaseRootClass< Event >;
template class SOFA_CORE_API BaseRootClass< defaulttype::BaseVector >;
template class SOFA_CORE_API BaseRootClass< defaulttype::BaseMatrix >;


} // namespace objectmodel

} // namespace core

} // namespace sofa

