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
#ifndef SOFA_HELPER_SSOBUFFER_H
#define SOFA_HELPER_SSOBUFFER_H

#include <sofa/SofaFramework.h>
#include <type_traits>
#include <utility>

namespace sofa
{

namespace helper
{

/// Small Size Optimized Buffer
template<std::size_t TSmallSize = 16>
class SSOBuffer
{
public:
    typedef SSOBuffer<TSmallSize> buffer;
    static constexpr std::size_t SmallSize = TSmallSize;
    template<class T>
    using is_small = typename std::enable_if< (sizeof(T)<=SmallSize), int>;
    template<class T>
    using is_large = typename std::enable_if<!(sizeof(T)<=SmallSize), int>;
    union
    {
        void* largePointer;
        typename std::aligned_storage<SmallSize>::type smallArray;
    };
    void (*deleteOrMoveOrCloneFn)(buffer* to, buffer* moveFrom, const buffer* cloneFrom);
public:
    SSOBuffer() : deleteOrMoveOrCloneFn(nullptr) { largePointer = nullptr; }
    SSOBuffer(const buffer& from)
    : deleteOrMoveOrCloneFn(from.deleteOrMoveOrCloneFn)
    {
        if (deleteOrMoveOrCloneFn)
            deleteOrMoveOrCloneFn(this, nullptr, &from);
    }
    SSOBuffer(buffer&& from)
    : deleteOrMoveOrCloneFn(std::move(from.deleteOrMoveOrCloneFn))
    {
        if (deleteOrMoveOrCloneFn)
            deleteOrMoveOrCloneFn(this, &from, nullptr);
    }
    ~SSOBuffer()
    {
        if (deleteOrMoveOrCloneFn)
            deleteOrMoveOrCloneFn(this, nullptr, nullptr);
    }
    buffer& operator=(const buffer& from)
    {
        if (deleteOrMoveOrCloneFn)
            deleteOrMoveOrCloneFn(this, nullptr, nullptr);
        deleteOrMoveOrCloneFn = from.deleteOrMoveOrCloneFn;
        if (deleteOrMoveOrCloneFn)
            deleteOrMoveOrCloneFn(this, nullptr, &from);
        return *this;
    }
    buffer& operator=(buffer&& from)
    {
        if (deleteOrMoveOrCloneFn)
            deleteOrMoveOrCloneFn(this, nullptr, nullptr);
        deleteOrMoveOrCloneFn = std::move(from.deleteOrMoveOrCloneFn);
        if (deleteOrMoveOrCloneFn)
            deleteOrMoveOrCloneFn(this, &from, nullptr);
        return *this;
    }

    template<class T>
    T* create(typename is_small<T>::type=0)
    {
        if (deleteOrMoveOrCloneFn)
        {
            (*deleteOrMoveOrCloneFn)(this, nullptr, nullptr);
            deleteOrMoveOrCloneFn = nullptr;
        }
        T* ptr = new(&smallArray) T;
        deleteOrMoveOrCloneFn = deleteOrMoveOrCloneFnSmall<T>;
        return ptr;
    }

    template<class T>
    T* create(typename is_large<T>::type=0)
    {
        if (deleteOrMoveOrCloneFn)
        {
            (*deleteOrMoveOrCloneFn)(this, nullptr, nullptr);
            deleteOrMoveOrCloneFn = nullptr;
        }
        T* ptr = new T;
        largePointer = ptr;
        deleteOrMoveOrCloneFn = deleteOrMoveOrCloneFnLarge<T>;
        return ptr;
    }

    template<class T>
    T* getOrCreate(typename is_small<T>::type=0)
    {
        T* ptr;
        if (deleteOrMoveOrCloneFn)
        {
            ptr = reinterpret_cast<T*>(&smallArray);
        }
        else
        {
            ptr = new(&smallArray) T;
            deleteOrMoveOrCloneFn = deleteOrMoveOrCloneFnSmall<T>;
        }
        return ptr;
    }
    template<class T>
    T* getOrCreate(typename is_large<T>::type=0)
    {
        T* ptr;
        if (deleteOrMoveOrCloneFn)
        {
            ptr = reinterpret_cast<T*>(largePointer);
        }
        else
        {
            ptr = new T;
            largePointer = ptr;
            deleteOrMoveOrCloneFn = deleteOrMoveOrCloneFnLarge<T>;
        }
        return ptr;
    }

    template<class T>
    const T* get(typename is_small<T>::type=0) const
    {
        const T* ptr = nullptr;
        if (deleteOrMoveOrCloneFn)
        {
            ptr = reinterpret_cast<const T*>(&smallArray);
        }
        return ptr;
    }

    template<class T>
    const T* get(typename is_large<T>::type=0) const
    {
        const T* ptr = nullptr;
        if (deleteOrMoveOrCloneFn)
        {
            ptr = reinterpret_cast<const T*>(largePointer);
        }
        return ptr;
    }

protected:
    template<class T>
    static void deleteOrMoveOrCloneFnSmall(buffer* to, buffer* moveFrom, const buffer* cloneFrom)
    {
        if (moveFrom)
        {
            // Warning this needs to be called only if to->smallArray has not been constructed
            new(&to->smallArray) T(std::move(*reinterpret_cast<T*>(&moveFrom->smallArray)));
        }
        else if (cloneFrom)
        {
            // Warning this needs to be called only if to->smallArray has not been constructed
            new(&to->smallArray) T(*reinterpret_cast<const T*>(&cloneFrom->smallArray));
        }
        else // delete
        {
            reinterpret_cast<T*>(&to->smallArray)->~T();
        }
    }
    template<class T>
    static void deleteOrMoveOrCloneFnLarge(buffer* to, buffer* moveFrom, const buffer* cloneFrom)
    {
        if (moveFrom)
        {
            to->largePointer = moveFrom->largePointer;
            moveFrom->largePointer = nullptr;
        }
        else if (cloneFrom)
        {
            to->largePointer = new T(*static_cast<const T*>(cloneFrom->largePointer));
        }
        else // delete
        {
            delete static_cast<T*>(to->largePointer);
        }
    }
};

} // namespace helper

} // namespace sofa

#endif  // SOFA_HELPER_SSOBUFFER_H
