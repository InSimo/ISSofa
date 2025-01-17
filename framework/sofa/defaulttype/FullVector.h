/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
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
#ifndef SOFA_DEFAULTTYPE_FULLVECTOR_H
#define SOFA_DEFAULTTYPE_FULLVECTOR_H

#include "BaseVector.h"
#include <sofa/helper/rmath.h>
#include <sofa/helper/vector.h>

#include <iostream>
#include <vector>

namespace sofa
{

namespace defaulttype
{

template<typename T>
class FullVector : public defaulttype::BaseVector
{
public:
    SOFA_VECTOR_CLASS_UNIQUE((FullVector<T>),((defaulttype::BaseVector)));

    typedef T Real;
    typedef defaulttype::BaseVector::Index Index;
    typedef T* Iterator;
    typedef const T* ConstIterator;

    typedef Real value_type;
    typedef Index size_type;
    typedef Iterator iterator;
    typedef ConstIterator const_iterator;

protected:
    T* data;
    Index cursize;
    Index allocsize;

#if !defined(NDEBUG) || defined(SOFA_CHECK_CONTAINER_ACCESS)
    void checkIndex(Index n) const
    {
        if (n >= cursize)
            sofa::helper::vector_access_failure(this, cursize, n, typeid(*this));
    }
#else
    void checkIndex(Index) const
    {
    }
#endif


public:

    FullVector()
        : defaulttype::BaseVector()
        , data(NULL), cursize(0), allocsize(0)
    {
    }

    FullVector(const FullVector& vect)
        : defaulttype::BaseVector()
        , data(NULL), cursize(0), allocsize(0)
    {
        (*this) = vect;
    }

    explicit FullVector(Index n)
        : defaulttype::BaseVector()
        , data(new T[n]), cursize(n), allocsize(n)
    {
    }

    FullVector(T* ptr, Index n)
        : defaulttype::BaseVector()
        , data(ptr), cursize(n), allocsize(-n)
    {
    }

    FullVector(T* ptr, Index n, Index nmax)
        : defaulttype::BaseVector()
        , data(ptr), cursize(n), allocsize(-nmax)
    {
    }

    virtual ~FullVector()
    {
        if (allocsize>0)
            delete[] data;
    }

    T* ptr() { return data; }
    const T* ptr() const { return data; }

    void setptr(T* p) { data = p; }

    Index capacity() const { if (allocsize < 0) return -allocsize; else return allocsize; }

    Iterator begin() { return data; }
    Iterator end()   { return data+cursize; }

    ConstIterator begin() const { return data; }
    ConstIterator end()   const { return data+cursize; }

    void fastResize(Index dim)
    {
        if (dim == cursize) return;
        if (allocsize >= 0)
        {
            if (dim > allocsize)
            {
                if (allocsize > 0)
                    delete[] data;
                allocsize = dim*2;
                data = new T[allocsize];
            }
        }
        else
        {
            if (dim > -allocsize)
            {
                std::cerr << "ERROR: cannot resize preallocated vector to size "<<dim<<std::endl;
                return;
            }
        }
        cursize = dim;
    }

    void extend(Index dim)
    {
        assert(dim >= cursize);
        if (dim == cursize) return;
        if (allocsize >= 0)
        {
            if (dim > allocsize)
            {
                // realloc
                allocsize = dim*2;
                T* temp = new T[allocsize];
                if (allocsize > 0)
                {
                    std::copy(data, data + cursize, temp);
                    delete [] data;
                }
                data = temp;
            }
        }
        else
        {
            if (dim > -allocsize)
            {
                std::cerr << "ERROR: cannot extend preallocated vector to size "<<dim<<std::endl;
                return;
            }
        }
        cursize = dim;
    }

    void resize(Index dim)
    {
        fastResize(dim);
        clear();
    }

    void clear()
    {
        if (cursize > 0)
            std::fill( this->begin(), this->end(), T() );
    }

    void swap(FullVector<T>& v)
    {
        Index t;
        t = cursize; cursize = v.cursize; v.cursize = t;
        t = allocsize; allocsize = v.allocsize; v.allocsize = t;
        T* d;
        d = data; data = v.data; v.data = d;
    }

    // for compatibility with baseVector
    void clear(Index dim)
    {
        resize(dim);
    }

    T& operator[](Index i)
    {
        checkIndex(i);
        return data[i];
    }

    const T& operator[](Index i) const
    {
        checkIndex(i);
        return data[i];
    }

    SReal element(Index i) const
    {
        checkIndex(i);
        return (SReal) data[i];
    }

    void set(Index i, SReal v)
    {
        checkIndex(i);
        data[i] = (Real)v;
    }

    void add(Index i, SReal v)
    {
        checkIndex(i);
        data[i] +=  (Real)v;
    }

    Index size() const
    {
        return cursize;
    }

    FullVector<T> sub(Index i, Index n)
    {
        if (n > 0) checkIndex(i+n-1);
        return FullVector<T>(data+i,n);
    }

    template<class TV>
    void getsub(Index i, Index n, TV& v)
    {
        if (n > 0) checkIndex(i+n-1);
        v = FullVector<T>(data+i,n);
    }

    template<class TV>
    void setsub(Index i, Index n, const TV& v)
    {
        if (n > 0) checkIndex(i+n-1);
        FullVector<T>(data+i,n) = v;
    }

    /// v = a
    void operator=(const FullVector<T>& a)
    {
        fastResize(a.size());
        std::copy(a.begin(), a.end(), begin());
    }

    void operator=(const T& a)
    {
        std::fill(begin(), end(), a);
    }

    /// v += a
    template<typename Real2>
    void operator+=(const FullVector<Real2>& a)
    {
        for(Index i=0; i<cursize; ++i)
            (*this)[i] += (Real)a[i];
    }

    /// v -= a
    template<typename Real2>
    void operator-=(const FullVector<Real2>& a)
    {
        for(Index i=0; i<cursize; ++i)
            (*this)[i] -= (Real)a[i];
    }

    /// v = a*f
    template<typename Real2,typename Real3>
    void eq(const FullVector<Real2>& a, Real3 f)
    {
        for(Index i=0; i<cursize; ++i)
            (*this)[i] = (Real)(a[i]*f);
    }

    /// v = a+b*f
    template<typename Real2,typename Real3>
    void eq(const FullVector<Real2>& a, const FullVector<Real2>& b, Real3 f=1.0)
    {
        for(Index i=0; i<cursize; ++i)
            (*this)[i] = (Real)(a[i]+b[i]*f);
    }

    /// v += a*f
    template<typename Real2,typename Real3>
    void peq(const FullVector<Real2>& a, Real3 f)
    {
        for(Index i=0; i<cursize; ++i)
            (*this)[i] += (Real)(a[i]*f);
    }

    /// v *= f
    template<typename Real2>
    void operator*=(Real2 f)
    {
        for(Index i=0; i<cursize; ++i)
            (*this)[i] *= (Real)f;
    }

    /// \return v.a
    Real dot(const FullVector<Real>& a) const
    {
        Real r = 0;
        for(Index i=0; i<cursize; ++i)
            r += (*this)[i]*a[i];
        return r;
    }

    /// \return sqrt(v.v)
    double norm() const
    {
        return helper::rsqrt(dot(*this));
    }

    friend std::ostream& operator << (std::ostream& out, const FullVector<Real>& v )
    {
        for (Index i=0,s=v.size(); i<s; ++i)
        {
            if (i) out << ' ';
            out << v[i];
        }
        return out;
    }

    static const char* Name() { return "FullVector"; }
};

template<> SOFA_DEFAULTTYPE_API void FullVector<bool>::set(Index i, SReal v);
template<> SOFA_DEFAULTTYPE_API void FullVector<bool>::add(Index i, SReal v);
template<> SOFA_DEFAULTTYPE_API bool FullVector<bool>::dot(const FullVector<Real>& a) const;
template<> SOFA_DEFAULTTYPE_API double FullVector<bool>::norm() const;


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_DEFAULTTYPE) 
extern template SOFA_DEFAULTTYPE_API class FullVector<bool>;
extern template SOFA_DEFAULTTYPE_API class FullVector<float>;
extern template SOFA_DEFAULTTYPE_API class FullVector<double>;
#endif

} // namespace defaulttype

} // namespace sofa

#endif
