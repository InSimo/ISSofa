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
#ifndef SOFA_HELPER_FACTORY_H
#define SOFA_HELPER_FACTORY_H

#include <map>
#include <iostream>
#include <typeinfo>
#include <vector>

#include <sofa/SofaFramework.h>

namespace sofa
{

namespace helper
{

/// Decode the type's name to a more readable form if possible
std::string SOFA_HELPER_API gettypename(const std::type_info& t);

/// Log classes registered in the factory
void SOFA_HELPER_API logFactoryRegister(std::string baseclass, std::string classname, std::string key, bool multi);

/// Print factory log
void SOFA_HELPER_API printFactoryLog(std::ostream& out = std::cout);

template <class Object, class Argument, class ObjectPtr = Object*>
class BaseCreator
{
public:
    virtual ~BaseCreator() { }
    virtual ObjectPtr createInstance(Argument arg) = 0;
    virtual const std::type_info& type() = 0;
    virtual const std::string& description() const = 0;
    virtual const std::vector<std::string>& aliases() const = 0;
    virtual int priority() const { return 0; }
};

template <typename TKey, class TObject, typename TArgument, typename TPtr = TObject* >
class Factory
{
public:
    typedef TKey      Key;
    typedef TObject   Object;
    typedef TPtr      ObjectPtr;
    typedef TArgument Argument;
    typedef BaseCreator<Object, Argument, ObjectPtr> Creator;
    typedef std::multimap<Key, Creator> Registry;

protected:
    std::multimap<Key, Creator*> registry;

public:
    bool registerCreator(Key key, Creator* creator, bool multi=false)
    {
        if(!multi && this->registry.find(key) != this->registry.end())
            return false; // key used
        logFactoryRegister(gettypename(typeid(Object)), gettypename(creator->type()), key, multi);
        //std::cout << gettypename(typeid(Object)) << (multi?" template class ":" class ")
        //          << gettypename(creator->type()) << " registered as " << key << std::endl;
        this->registry.insert(std::pair<Key, Creator*>(key, creator));
        return true;
    }

    ObjectPtr createObject(Key key, Argument arg);
    ObjectPtr createAnyObject(Argument arg);

    template< typename OutIterator >
    void uniqueKeys(OutIterator out) const;

    bool hasKey(Key key);
    bool duplicateEntry( Key existing, Key duplicate, bool multi=false);
    bool resetEntry( Key existingKey);

    static Factory<Key, Object, Argument, ObjectPtr>* getInstance();

    static ObjectPtr CreateObject(Key key, Argument arg)
    {
        return getInstance()->createObject(key, arg);
    }

    static ObjectPtr CreateAnyObject(Argument arg)
    {
        return getInstance()->createAnyObject(arg);
    }

    static bool HasKey(Key key)
    {
        return getInstance()->hasKey(key);
    }

    static bool DuplicateEntry(Key existing, Key duplicate, bool multi=false)
    {
        return getInstance()->duplicateEntry(existing, duplicate, multi);
    }

    static bool ResetEntry(Key existing)
    {
        return getInstance()->resetEntry(existing);
    }


    typedef typename std::multimap<Key, Creator*>::iterator iterator;
    iterator begin() { return registry.begin(); }
    iterator end() { return registry.end(); }
    typedef typename std::multimap<Key, Creator*>::const_iterator const_iterator;
    const_iterator begin() const { return registry.begin(); }
    const_iterator end() const { return registry.end(); }

protected:
    ObjectPtr createObjectRange(iterator begin, iterator end, Argument arg);

};

template <class Factory, class RealObject>
class Creator : public Factory::Creator, public Factory::Key
{
public:
    typedef typename Factory::Object    Object;
    typedef typename Factory::ObjectPtr ObjectPtr;
    typedef typename Factory::Argument  Argument;
    typedef typename Factory::Key       Key;
    explicit Creator(Key key, bool multi=false, int priority = 0, const std::string& description="", const std::vector<Key>& aliases = {})
    : Key(key), m_priority(priority), m_description(description), m_aliases(aliases)
    {
        Factory::getInstance()->registerCreator(key, this, multi);
        for (const Key& k : aliases)
        {
            //we can't use duplicateEntry because other creators might already be registered with the same key
            //Factory::getInstance()->duplicateEntry(key, k, multi);
            Factory::getInstance()->registerCreator(k, this, multi);
        }
    }
    ObjectPtr createInstance(Argument arg)
    {
        RealObject* instance = NULL;
        return RealObject::create(instance, arg);
    }
    const std::type_info& type()
    {
        return typeid(RealObject);
    }

	// Dummy function to avoid dead stripping symbol
	void registerInFactory()
	{
		printf("[SOFA]Registration of class : %s\n", type().name());
	}

    virtual const std::string& description() const
    {
        return m_description;
    }

    virtual int priority() const
    {
        return m_priority;
    }

    virtual const std::vector<Key>& aliases() const
    {
        return m_aliases;
    }

protected:
    int m_priority;
    std::string m_description;
    std::vector<Key> m_aliases;
};
/*
/// Generic object creator. Can be specialized for custom objects creation
template<class Object, class Argument>
Object create(Object* obj, Argument arg)
{
	return new Object(arg);
}
*/
template <class Factory, class RealObject>
class CreatorFn : public Factory::Creator, public Factory::Key
{
public:
    typedef typename Factory::Object    Object;
    typedef typename Factory::ObjectPtr ObjectPtr;
    typedef typename Factory::Argument  Argument;
    typedef typename Factory::Key       Key;
    typedef ObjectPtr Fn(RealObject* obj, Argument arg);

    CreatorFn(Key key, Fn* constructor, bool multi=false, int priority = 0, const std::string& description="", const std::vector<Key>& aliases = {})
        : Key(key), constructor(constructor), m_priority(priority), m_description(description), m_aliases(aliases)
    {
        Factory::getInstance()->registerCreator(key, this, multi);
        for (const Key& k : aliases)
        {
            //we can't use duplicateEntry because other creators might already be registered with the same key
            //Factory::getInstance()->duplicateEntry(key, k, multi);
            Factory::getInstance()->registerCreator(k, this, multi);
        }
    }

    ObjectPtr createInstance(Argument arg)
    {
        RealObject* instance = NULL;
        return (*constructor)(instance, arg);
    }
    const std::type_info& type()
    {
        return typeid(RealObject);
    }

    virtual const std::string& description() const
    {
        return m_description;
    }

    virtual int priority() const
    {
        return m_priority;
    }

    virtual const std::vector<Key>& aliases() const
    {
        return m_aliases;
    }

protected:
    Fn* constructor;
    int m_priority;
    std::string m_description;
    std::vector<Key> m_aliases;
};


} // namespace helper

} // namespace sofa

// Creator is often used without namespace qualifiers
using sofa::helper::Creator;

#endif
