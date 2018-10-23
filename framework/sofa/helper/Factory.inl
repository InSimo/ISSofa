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
#ifndef SOFA_HELPER_FACTORY_INL
#define SOFA_HELPER_FACTORY_INL

#include <sofa/helper/Factory.h>
#include <iostream>
#include <typeinfo>
#include <limits>

// added by Sylvere F.
// this inclusion must be done but not in this part of code. For the moment, I don't know where ;)
#include <string>
#include <sofa/helper/vector.h>

namespace sofa
{

namespace helper
{


template <typename TKey, class TObject, typename TArgument, typename TPtr>
TPtr Factory<TKey, TObject, TArgument, TPtr>::createObject(Key key, Argument arg)
{
    iterator begin = registry.lower_bound(key);
    iterator end   = registry.upper_bound(key);
    TPtr object = createObjectRange(begin, end, arg);
//	if (!object) std::cerr<<"Object type "<<key<<" creation failed."<<std::endl;
    return object;
}

template <typename TKey, class TObject, typename TArgument, typename TPtr>
TPtr Factory<TKey, TObject, TArgument, TPtr>::createAnyObject(Argument arg)
{
    iterator begin = registry.begin();
    iterator end   = registry.end();
    return createObjectRange(begin, end, arg);
}

template <typename TKey, class TObject, typename TArgument, typename TPtr>
TPtr Factory<TKey, TObject, TArgument, TPtr>::createObjectRange(iterator begin, iterator end, Argument arg)
{
    ObjectPtr object;
    Creator* creator;
    // try creators by descending order of priority
    int currentPriority = std::numeric_limits<int>::max();
    int remaining = 0;
    do
    {
        int nextPriority = currentPriority;
        remaining = 0;
        iterator it = begin;
        while (it != end)
        {
            creator = (*it).second;
            int priority = creator->priority();
            if (priority == currentPriority)
            {
                object = creator->createInstance(arg);
                if (object != nullptr)
                {
                    return object;
                }
            }
            else if (priority < currentPriority)
            {
                if (!remaining || priority > nextPriority)
                {
                    nextPriority = priority;
                }
                ++remaining;
            }
            ++it;
        }
        currentPriority = nextPriority;
    }
    while (remaining > 0);
    return nullptr;
}


template <typename TKey, class TObject, typename TArgument, typename TPtr>
template< typename OutIterator >
void Factory<TKey, TObject, TArgument, TPtr>::uniqueKeys(OutIterator out) const
{

    typename std::multimap<Key, Creator*>::const_iterator it;

    const Key* p_key = NULL;
    for ( it = registry.begin(); it != registry.end(); ++it)
    {

        if( p_key && *p_key == it->first ) continue;

        p_key = &(it->first);
        *out = *p_key;
        out++;
    }
}

template <typename TKey, class TObject, typename TArgument, typename TPtr>
bool Factory<TKey, TObject, TArgument, TPtr>::hasKey(Key key)
{
    return (this->registry.find(key) != this->registry.end());
}

template <typename TKey, class TObject, typename TArgument, typename TPtr>
Factory<TKey, TObject, TArgument, TPtr>* Factory<TKey, TObject, TArgument, TPtr>::getInstance()
{
    static Factory<Key, Object, Argument, ObjectPtr> instance;
    return &instance;
}


template <typename TKey, class TObject, typename TArgument, typename TPtr>
bool Factory<TKey, TObject, TArgument, TPtr>::duplicateEntry(Key existing, Key duplicate, bool multi)
{
    if( !hasKey(existing) )
    {
        std::cerr << "ERROR: entry " << existing << " unknown in factory." << std::endl;
        return false;
    }

    if( !multi && hasKey(duplicate) )
    {
        std::cerr << "ERROR: Cannot duplicate "<< duplicate << ", it already exists." << std::endl;
        std::cerr << "ERROR: must call resetEntry(" << duplicate << "," << existing << ") first, or enable multi flag." << std::endl;
        return false;
    }

    typename std::multimap<Key, Creator*>::const_iterator it = registry.lower_bound(existing);
    typename std::multimap<Key, Creator*>::const_iterator end = registry.upper_bound(existing);
    helper::vector<Creator*> entries;
    while (it != end)
    {
        entries.push_back(it->second);
        ++it;
    }

    typename helper::vector<Creator*>::const_iterator iter;
    for( iter = entries.begin(); iter != entries.end() ; ++iter)
    {
        registry.insert(std::make_pair(duplicate,*iter));
    }
    return true;
}

template <typename TKey, class TObject, typename TArgument, typename TPtr>
bool Factory<TKey, TObject, TArgument, TPtr>::resetEntry( Key existingKey)
{
    if( !hasKey(existingKey) )
    {
        std::cerr << "ERROR: Cannot reset entry " << existingKey << ", it does not exist." << std::endl;
        return false;
    }

    registry.erase(existingKey);

    return true;

}


} // namespace helper

} // namespace sofa

#endif
