/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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
#include <sofa/core/objectmodel/BaseLink.h>
#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseData.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/BaseNode.h>

#include <sstream>

namespace sofa
{

namespace core
{

namespace objectmodel
{

//#define CHECK_LINKS_ON_EXIT

#ifdef CHECK_LINKS_ON_EXIT

struct LinkChecker
{

    enum BaseStatus
    {
        NOT_VISITED = 0,
        VISITING,
        VISITED
    };

    static void visit(const Base* cur, const std::map<const Base*, std::set<const Base*>>& src2dst, std::map<const Base*, int>& marks, std::map<const Base*, const Base*> backprop)
    {
        auto it = marks.find(cur);
        if (it != marks.end())
        {
            int mark = it->second;
            if (mark == VISITING)
            {
                std::cerr << "Link cycle : [" << cur->getClassName() << "] " << cur->getName();
                std::vector<const Base*> path;
                const Base* b = backprop[cur];
                while (b != cur)
                {
                    path.push_back(b);
                    b = backprop[b];
                }
                for (auto rit = path.rbegin(); rit != path.rend(); ++rit)
                {
                    const Base* b = *rit;
                    std::cerr << " -> [" << b->getClassName() << "] " << b->getName();
                }
                std::cerr << std::endl;
            }
            return;
        }
        marks[cur] = VISITING;

        auto it2 = src2dst.find(cur);
        if (it2 != src2dst.end())
        {
            for (const Base* next : it2->second)
            {
                backprop[next] = cur;
                visit(next, src2dst, marks, backprop);
            }
        }

        marks[cur] = VISITED;
    }

    ~LinkChecker()
    {
        std::cerr << "================ LINK CHECKER ================" << std::endl;

        std::map<const Base*, std::set<const Base*>> dst2src;
        std::map<const Base*, std::set<const Base*>> src2dst;
        std::cerr << "Remaining links : " << links.size() << std::endl;
        for (const BaseLink* l : links)
        {
            bool linkedBaseFound = false;
            for (unsigned int i = 0u; i < l->getSize(); ++i)
            {
                if (l->getLinkedBase(i))
                {
                    dst2src[l->getLinkedBase(i)].insert(l->getOwnerBase());
                    src2dst[l->getOwnerBase()].insert(l->getLinkedBase(i));
                    linkedBaseFound = true;
                }
            }
            if (!linkedBaseFound)
            {
                src2dst[l->getOwnerBase()].insert(nullptr);
            }
        }

        // Find Bases which own remaining links because something else than a BaseLink keeps their refcount > 0
        for (const auto& p : src2dst)
        {
            const Base* src = p.first;
            auto it = dst2src.find(src);
            if (it == dst2src.end())
            {
                std::cerr << "Source-only link [" << src->getClassName() << "] " << src->getName() << " to: ";
                for (const Base* dst : p.second)
                {
                    if (dst)
                    {
                        std::cerr << "[" << dst->getClassName() << "] " << dst->getName() << ", ";
                    }
                }
                std::cerr << std::endl;
            }
        }

        // Detect circular dependencies between links (which indicates either an error or a missing FLAG_DUPLICATE)
        std::map<const Base*, const Base*> backprop; // used to backpropagate in cycles
        std::map<const Base*, int> marks; // used to mark visitation state of each Base

        // Use topological sorting based on depth-first search (https://en.wikipedia.org/wiki/Topological_sorting#Depth-first_search)
        for (const auto& p : src2dst)
        {
            const Base* src = p.first;

            if (backprop.find(src) != backprop.cend())
            {
                // sub-graph already visited
                continue;
            }

            visit(src, src2dst, marks, backprop);
        }

        std::cerr << "================ LINK CHECKER ================" << std::endl;
    }

    std::set<const BaseLink*> links;
};

/// This static struct will be destroyed when the program exits.
/// Its destructor will check all remaining links and try to find the source(s) of their non-deletion
static LinkChecker checker;
#endif

BaseLink::BaseLink(const BaseInitLink& init, LinkFlags flags)
    : m_flags(flags), m_name(init.name), m_help(init.help)
{
    m_counters.assign(0);
    //m_isSets.assign(false);

#ifdef CHECK_LINKS_ON_EXIT
    if (this->isStrongLink())
    {
        checker.links.insert(this);
    }
#endif
}

BaseLink::~BaseLink()
{
#ifdef CHECK_LINKS_ON_EXIT
    checker.links.erase(this);
#endif
}

/// Print the value of the associated variable
void BaseLink::printValue( std::ostream& o ) const
{
    unsigned int size = getSize();
    bool first = true;
    for (unsigned int i=0; i<size; ++i)
    {
        std::string path = getLinkedPath(i);
        if (path.empty()) continue;
        if (first) first = false;
        else o << ' ';
        o << path;
    }
}

/// Print the value of the associated variable
std::string BaseLink::getValueString() const
{
    std::ostringstream o;
    printValue(o);
    return o.str();
}

/// Provide a string representing the type of the expected destination (Data<T> or Class or Class<T>)
std::string BaseLink::getDestClassName() const
{
    std::string t;
    if (isDataLink())
    {
        const defaulttype::AbstractTypeInfo* c = getDestDataType();
        t = "Data";
        if (c)
        {
            t += '<';
            t += c->name();
            t += '>';
        }
    }
    else
    {
        const BaseClass* c = getDestObjectClass();
        if (!c)
        {
            t = "void";
        }
        else
        {
            t = c->className;
            if (!c->templateName.empty())
            {
                t += '<';
                t += c->templateName;
                t += '>';
            }
        }
    }
    return t;
}

/// Provide a string representing the type of the owner (Data<T> or Class or Class<T>)
std::string BaseLink::getOwnerClassName() const
{
    std::string t;
    if (isOwnerData())
    {
        const defaulttype::AbstractTypeInfo* c = getOwnerDataType();
        t = "Data";
        if (c)
        {
            t += '<';
            t += c->name();
            t += '>';
        }
    }
    else
    {
        const BaseClass* c = getOwnerObjectClass();
        if (!c)
        {
            t = "void";
        }
        else
        {
            t = c->className;
            if (!c->templateName.empty())
            {
                t += '<';
                t += c->templateName;
                t += '>';
            }
        }
    }
    return t;
}

/// Print the value type of the associated variable
std::string BaseLink::getValueTypeString() const
{
    return getDestClassName();
}

/// Copy the value of an aspect into another one.
void BaseLink::copyAspect(int destAspect, int srcAspect)
{
    m_counters[destAspect] = m_counters[srcAspect];
    //m_isSets[destAspect] = m_isSets[srcAspect];
}

/// Release memory allocated for the specified aspect.
void BaseLink::releaseAspect(int aspect)
{
    m_counters[aspect] = -1;
}

bool BaseLink::ParseString(const std::string& text, std::string* path, std::string* data, Base* owner)
{
    if (text.empty())
    {
        if (owner) owner->serr << "ERROR parsing Link \""<<text<<"\": empty path." << owner->sendl;
        else std::cerr << "ERROR parsing Link \""<<text<<"\": empty path." << std::endl;
        return false;
    }
    if (text[0] != '@')
    {
        if (owner) owner->serr << "ERROR parsing Link \""<<text<<"\": first character should be '@'." << owner->sendl;
        else std::cerr << "ERROR parsing Link \""<<text<<"\": first character should be '@'." << std::endl;
        return false;
    }
    std::size_t posPath = text.rfind('/');
    if (posPath == std::string::npos) posPath = 0;
    std::size_t posDot = text.rfind('.');
    if (posDot < posPath) posDot = std::string::npos; // dots can appear within the path
    if (posDot == text.size()-1 && text[posDot-1] == '.') posDot = std::string::npos; // double dots can appear at the end of the path
    if (posDot == text.size()-1 && (text[posDot-1] == '/' || posDot == 1)) posDot = std::string::npos; // a single dot is allowed at the end of the path, although it is better to end it with '/' instead in order to remove any ambiguity
    if (!data && posDot != std::string::npos)
    {
        if (owner) owner->serr << "ERROR parsing Link \""<<text<<"\": a Data field name is specified while an object was expected." << owner->sendl;
        else std::cerr << "ERROR parsing Link \""<<text<<"\": a Data field name is specified while an object was expected." << std::endl;
        return false;
    }

    if (data && data->empty() && posDot == std::string::npos)
    {
        if (owner) owner->serr << "ERROR parsing Link \""<<text<<"\": a Data field name is required." << owner->sendl;
        else std::cerr << "ERROR parsing Link \""<<text<<"\": a Data field name is required." << std::endl;
        return false;
    }

    if (!data || posDot == std::string::npos)
    {
        if (path)
            *path = text.substr(1);
    }
    else
    {
        if (path)
            *path = text.substr(1,posDot-1);
        *data = text.substr(posDot+1);
    }
    if (path && !path->empty())
    {
        if ((*path)[0] == '[' && (*path)[path->size()-1] != ']')
        {
            if (owner) owner->serr << "ERROR parsing Link \""<<text<<"\": missing closing bracket ']'." << owner->sendl;
            else std::cerr << "ERROR parsing Link \""<<text<<"\": missing closing bracket ']'." << std::endl;
            return false;
        }
        if ((*path)[0] == '[' && (*path)[1] != '-' && (*path)[1] != ']')
        {
            if (owner) owner->serr << "ERROR parsing Link \""<<text<<"\": bracket syntax can only be used for self-reference or preceding objects with a negative index." << owner->sendl;
            else std::cerr << "ERROR parsing Link \""<<text<<"\": bracket syntax can only be used for self-reference or preceding objects with a negative index." << std::endl;
            return false;
        }
    }
    //std::cout << "LINK: Parsed \"" << text << "\":";
    //if (path) std::cout << " path=\"" << *path << "\"";
    //if (data) std::cout << " data=\"" << *data << "\"";
    //std::cout << std::endl;
    return true;
}

std::string BaseLink::CreateString(const std::string& path, const std::string& data)
{
    std::string result = "@";
    if (!path.empty()) result += path;
    if (!data.empty())
    {
        if (result[result.size()-1] == '.')
            result += '/'; // path ends at a node designed with '.' or '..', so add '/' in order to separate it from the data part
        result += '.';
        result += data;
    }
    return result;
}

std::string BaseLink::CreateStringPath(Base* dest, Base* from)
{
    if (!dest || dest == from) return std::string("[]");
    BaseObject* o = BaseObject::DynamicCast(dest);
    BaseObject* f = BaseObject::DynamicCast(from);
    BaseContext* ctx = BaseContext::DynamicCast(from);
    if (!ctx && f) ctx = f->getContext();
    if (o)
    {
        std::string objectPath = o->getName();
        BaseObject* master = o->getMaster();
        while (master)
        {
            objectPath = master->getName() + std::string("/") + objectPath;
            master = master->getMaster();
        }
        BaseNode* n = BaseNode::DynamicCast(o->getContext());
        if (f && o->getContext() == ctx)
            return objectPath;
        else if (n)
            return n->getPathName() + std::string("/") + objectPath; // TODO: compute relative path
        else
            return objectPath; // we could not determine destination path, specifying simply its name might be enough to find it back
    }
    else // dest is a context
    {
        if (f && ctx == dest)
            return std::string("./");
        BaseNode* n = BaseNode::DynamicCast(dest);
        if (n) return n->getPathName(); // TODO: compute relative path
        else return dest->getName(); // we could not determine destination path, specifying simply its name might be enough to find it back
    }
}

std::string BaseLink::CreateStringData(BaseData* data)
{
    if (!data) return std::string();
    return data->getName();
}
std::string BaseLink::CreateString(Base* object, Base* from)
{
    return CreateString(CreateStringPath(object,from));
}
std::string BaseLink::CreateString(BaseData* data, Base* from)
{
    return CreateString(CreateStringPath(data->getOwner(),from),CreateStringData(data));
}
std::string BaseLink::CreateString(Base* object, BaseData* data, Base* from)
{
    return CreateString(CreateStringPath(object,from),CreateStringData(data));
}

} // namespace objectmodel

} // namespace core

} // namespace sofa
