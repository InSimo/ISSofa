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

#ifndef SOFA_DEFAULTTYPE_UNITS_H
#define SOFA_DEFAULTTYPE_UNITS_H

#include <iostream>
#include <sofa/helper/fixed_array.h>
#include <sofa/defaulttype/QuantityTypeInfo.h>

namespace sofa
{
namespace units
{


//////////////////////////////////////////////////////////////////////////////////////
//
// Vocabulary convention : a physical Quantity (eg : a force) 
//    has got units (or dimensions, eg : kg.m.s-2 for a force),
//      and can have a prefix (a factor which is a multiple or a fraction).
//
// Important : any physical Quantity can be described thanks to a combination of 7
//    "elementary" units which are kilogram, meter, second, Ampere, kelvin, mole, candela.
//      It could be possible to describe a physical Quantity in other units 
//            (eg Celsius instead of Kelvin) thanks to conversion algorithms.
//
//
// Example of usage : 
//    Mass<double>           m0( 1 );
//    Acceleration<Deriv>    a0(Deriv(1, 2, 3);
//
//    Force<Deriv>           f0( m0 * a0 );      // will compile
//    Time<double>           t0( m0 );           // won't compile
//
//    if( m0 == Mass<double>(4) ) {}     // will compile
//    if( m0 == 4 ) {}                    // won't compile
//    if( m0.value() == 4) {}             // will compile
//    if( Scalar<double>(10) == 42.0 ) {}   // will compile thanks to a template specialization on scalars
//
//
//////////////////////////////////////////////////////////////////////////////////////



//////////////////////////// Base class Quantity //////////////////////////////

template<class T, // Precision
            int kg, // Mass
                int m, // Length
                    int s, // Time
                        int A, // Electric current
                            int K, // Thermodynamic temperature
                                int mol, // Amount of substance
                                    int cd> // Luminous intensity
class QuantityBase
{
public:

    // Operator: default constructor
    explicit
        QuantityBase(T initVal = T())
        : m_value(initVal)
    {
    }

    // Get Reference
    T&
        value()
    {
        return m_value;
    }

    // Get Value
    const T&
        value() const
    {
        return m_value;
    }

protected:
    T m_value;
};

template <class T, int kg, int m, int s, int A, int K, int mol, int cd, class dummy = void>
class QuantitySpecialization : public QuantityBase<T, kg, m, s, A, K, mol, cd>
{
public:
    explicit
     QuantitySpecialization(T initVal = T())
     : QuantityBase<T, kg, m, s, A, K, mol, cd>(initVal)
    {}
};

// Specialization for array
// We expect all type using this specialization to have a BidirectionalIterator
template <class T, int kg, int m, int s, int A, int K, int mol, int cd>
class QuantitySpecialization<T, kg, m, s, A, K, mol, cd,
        typename std::enable_if<std::is_base_of<std::forward_iterator_tag, typename std::iterator_traits<typename T::iterator>::iterator_category>::value>::type > : public QuantityBase<T, kg, m, s, A, K, mol, cd>
{
public:
    explicit
        QuantitySpecialization(T initVal = T())
        : QuantityBase<T, kg, m, s, A, K, mol, cd>(initVal)
    {}

    using ValueType = T;

    // We could define a iterator on Quantity<T::value_type, ...>
    // But to do that, we need to be able to reinterpret_cast frome T::reference to Quantity
    // A way to do that is to make Quantity a aggregate to respect type aliasing
    // Unfortunately we cannot use a base class like QuantitySpecialization in a aggregate (C++17 is changing that)
    using iterator = typename T::iterator;
    using const_iterator = typename T::const_iterator;

    iterator begin()
    {
        return this->m_value.begin();
    }

    const_iterator begin() const
    {
        return this->m_value.begin();
    }

    const_iterator cbegin() const
    {
        return this->m_value.cbegin();
    }

    iterator end()
    {
        return this->m_value.end();
    }

    const_iterator end() const
    {
        return this->m_value.end();
    }

    const_iterator cend() const
    {
        return this->m_value.cend();
    }
};

template <class T, int kg, int m, int s, int A, int K, int mol, int cd>
class Quantity : public QuantitySpecialization<T, kg, m, s, A, K, mol, cd>
{
public:
    explicit
        Quantity(T initVal = T())
        : QuantitySpecialization<T, kg, m, s, A, K, mol, cd>(initVal)
    {}

    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator= (const T& rhs)
    {
        this->m_value = rhs;
        return *this;
    }

    // Operator: +=
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator+= (const Quantity<T, kg, m, s, A, K, mol, cd>& rhs)
    {
        this->m_value += rhs.m_value;
        return *this;
    }

    // Operator -=
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator-= (const Quantity<T, kg, m, s, A, K, mol, cd>& rhs)
    {
        this->m_value -= rhs.m_value;
        return *this;
    }

    // Operator *=
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator*= (T rhs)
    {
        this->m_value *= rhs;
        return *this;
    }

    // Operator /=
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator/= (T rhs)
    {
        this->m_value /= rhs;
        return *this;
    }

    // Get Units (/dimensions)
    constexpr std::array<int, 7>
        units() const
    {
        return { {kg, m, s, A, K, mol, cd} };
    }

    // Get Units as text (e.g. returns "kg.m.s-2" for a force)
    const std::string
        unitsAsText() const
    {
        std::string txt = (kg != 0) ? "kg" + ((std::abs(kg) > 1) ? std::to_string(kg) + "." : ".") : "";
        txt += (m != 0) ? "m" + ((std::abs(m) > 1) ? std::to_string(m) + "." : ".") : "";
        txt += (s != 0) ? "s" + ((std::abs(s) > 1) ? std::to_string(s) + "." : ".") : "";
        txt += (A != 0) ? "A" + ((std::abs(A) > 1) ? std::to_string(A) + "." : ".") : "";
        txt += (K != 0)   ? "K"  + ((std::abs(K) > 1) ? std::to_string(K) + "." : ".") : "";
        txt += (mol != 0) ? "mol" + ((std::abs(mol) > 1) ? std::to_string(mol) + "." : ".") : "";
        txt += (cd != 0)  ? "cd" + ((std::abs(cd) > 1) ? std::to_string(cd) + "." : ".") : "";

        return txt.substr(0, txt.size()-1);
    }

    // Output stream
    inline friend std::ostream& operator<< ( std::ostream& os, const Quantity<T, kg, m, s, A, K, mol, cd>& q)
    {
        return os << q.value();
    }

    // Input stream
    inline friend std::istream& operator>> ( std::istream& is, Quantity<T, kg, m, s, A, K, mol, cd>& q)
    {
        return is >> q.value();
    }
};

////////////////////////////// Operators ///////////////////////////////////



////////////////////////
// Operators with scalar


// Operator: Addition
// we don't want to add dimensioned value and a scalar


// Operator: Substraction
// we don't want to substract dimensioned value and a scalar


// Operator: Multiplication
template<class T, int kg, int m, int s, int A, int K, int mol, int cd,
    class Scalar>
    const Quantity<T, kg, m, s, A, K, mol, cd>
    operator* (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
        const Scalar& rhs)
{
    Quantity<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result *= rhs;
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd,
    class Scalar>
    const Quantity<T, kg, m, s, A, K, mol, cd>
    operator* (const Scalar& lhs,
        const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Quantity<T, kg, m, s, A, K, mol, cd> result(rhs);
    return result *= lhs;
}

// Operator: Division
template<class T, int kg, int m, int s, int A, int K, int mol, int cd,
    class Scalar>
    const Quantity<T, kg, m, s, A, K, mol, cd>
    operator/ (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
        const Scalar& rhs)
{
    Quantity<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result /= rhs;
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd,
    class Scalar>
    const Quantity<T, -kg, -m, -s, -A, -K, -mol, -cd>
    operator/ (const Scalar& lhs,
        const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Quantity<T, -kg, -m, -s, -A, -K, -mol, -cd> result(lhs);
    return result /= rhs.value();
}



////////////////////////
// Operators with same types

// Operator: Addition
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Quantity<T, kg, m, s, A, K, mol, cd>
operator+ (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Quantity<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result += rhs;
}

// Operator: Substraction
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Quantity<T, kg, m, s, A, K, mol, cd>
operator- (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Quantity<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result -= rhs;
}

// Operator: Multiplication
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Quantity<T, kg, m, s, A, K, mol, cd>
operator* (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Quantity<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result *= rhs;
}

// Operator: Division
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Quantity<T, kg, m, s, A, K, mol, cd>
operator/ (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Quantity<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result /= rhs;
}



////////////////////////
// Operators between different types

// Operator: Multiplication (Creates New Type)
template<class T1, int kg1, int m1, int s1, int A1, int K1, int mol1, int cd1,
            class T2, int kg2, int m2, int s2, int A2, int K2, int mol2, int cd2>
auto 
operator* (const Quantity<T1, kg1, m1, s1, A1, K1, mol1, cd1>& lhs,
    const Quantity<T2, kg2, m2, s2, A2, K2, mol2, cd2>& rhs) 
        -> Quantity< decltype(lhs.value()*rhs.value()), kg1 + kg2, m1 + m2, s1 + s2, A1 + A2, K1 + K2, mol1 + mol2, cd1 + cd2>
{
    // New Return type
    typedef Quantity< decltype(lhs.value()*rhs.value()),
        kg1 + kg2,
        m1 + m2,
        s1 + s2,
        A1 + A2,
        K1 + K2,
        mol1 + mol2,
        cd1 + cd2> ResultType;

    return ResultType(lhs.value() * rhs.value());
}

// Operator: Division (Creates New Type)
template<class T1, int kg1, int m1, int s1, int A1, int K1, int mol1, int cd1,
            class T2, int kg2, int m2, int s2, int A2, int K2, int mol2, int cd2>
auto
operator/ (const Quantity<T1, kg1, m1, s1, A1, K1, mol1, cd1>& lhs,
    const Quantity<T2, kg2, m2, s2, A2, K2, mol2, cd2>& rhs)
        -> Quantity< decltype(lhs.value()*rhs.value()), kg1 - kg2, m1 - m2, s1 - s2, A1 - A2, K1 - K2, mol1 - mol2, cd1 - cd2>
{
    // New Return type
    typedef Quantity< decltype(lhs.value()*rhs.value()),
        kg1 - kg2,
        m1 - m2,
        s1 - s2,
        A1 - A2,
        K1 - K2,
        mol1 - mol2,
        cd1 - cd2> ResultType;

    return ResultType(lhs.value() / rhs.value());
}



////////////////////////
// Comparison operators between same types

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator< (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() < rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator> (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() > rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator<= (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() <= rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator>= (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() >= rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator== (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() == rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator!= (const Quantity<T, kg, m, s, A, K, mol, cd> & lhs,
    const Quantity<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() != rhs.value();
}


/////////////// Template specialization for the scalars //////////////////

template<class T>
class Quantity<T, 0, 0, 0, 0, 0, 0, 0>
{
public:

    // Operator: default constructor
    explicit
    Quantity(T initVal = 0)
        : m_value(initVal)
    {
    }

    // to/from values of type T
    operator T() const { return m_value; }
    

    // Operator: Assignment from type T
    Quantity<T, 0, 0, 0, 0, 0, 0, 0>&
        operator= (const T rhs)
    {
        m_value = rhs;
        return *this;
    }

    // Operator: +=
    Quantity<T, 0, 0, 0, 0, 0, 0, 0>&
        operator+= (const Quantity<T, 0, 0, 0, 0, 0, 0, 0>& rhs)
    {
        m_value += rhs.m_value;
        return *this;
    }

    // Operator -=
    Quantity<T, 0, 0, 0, 0, 0, 0, 0>&
        operator-= (const Quantity<T, 0, 0, 0, 0, 0, 0, 0>& rhs)
    {
        m_value -= rhs.m_value;
        return *this;
    }

    // Operator *=
    Quantity<T, 0, 0, 0, 0, 0, 0, 0>&
        operator*= (T rhs)
    {
        m_value *= rhs;
        return *this;
    }

    // Operator /=
    Quantity<T, 0, 0, 0, 0, 0, 0, 0>&
        operator/= (T rhs)
    {
        m_value /= rhs;
        return *this;
    }

    // Get Reference
    T&
        value()
    {
        return m_value;
    }

    // Get Value
    const T&
        value() const
    {
        return m_value;
    }

private:
    T m_value;
};


////////////////////////// Some types definitions ///////////////////////////

//  Base Quantities (units from the International System)
template<typename T0> using Scalar =        Quantity<T0, 0, 0, 0, 0, 0, 0, 0>;      // unitless

template<typename T0> using Mass =          Quantity<T0, 1, 0, 0, 0, 0, 0, 0>;      // mass in kilogram (kg)
template<typename T0> using Length =        Quantity<T0, 0, 1, 0, 0, 0, 0, 0>;      // length in meter (m)
template<typename T0> using Time =          Quantity<T0, 0, 0, 1, 0, 0, 0, 0>;      // time in second (s)
template<typename T0> using ECurrent =      Quantity<T0, 0, 0, 0, 1, 0, 0, 0>;      // electric current in ampere (A)
template<typename T0> using Temperature =   Quantity<T0, 0, 0, 0, 0, 1, 0, 0>;      // temperature in kelvin (K)
template<typename T0> using Amount =        Quantity<T0, 0, 0, 0, 0, 0, 1, 0>;      // amount of substance in mole (mol)
template<typename T0> using LIntensity =    Quantity<T0, 0, 0, 0, 0, 0, 0, 1>;      // luminous intensity in candela (cd)


//  Derived Quantities (combinations of units from the International System)
template<typename T0> using Area =          Quantity<T0, 0, 2, 0, 0, 0, 0, 0>;      // Length^2
template<typename T0> using Volume =        Quantity<T0, 0, 3, 0, 0, 0, 0, 0>;      // Length^3

template<typename T0> using Frequency =     Quantity<T0, 0, 0,-1, 0, 0, 0, 0>;      // Time^(-1)

template<typename T0> using Velocity =      Quantity<T0, 0, 1,-1, 0, 0, 0, 0>;      // Length over Time
template<typename T0> using Acceleration =  Quantity<T0, 0, 1,-2, 0, 0, 0, 0>;      // Length over Time^2

template<typename T0> using Force =         Quantity<T0, 1, 1,-2, 0, 0, 0, 0>;      // Mass * Acceleration
template<typename T0> using Momentum =      Quantity<T0, 1, 2,-2, 0, 0, 0, 0>;      // Force * Length

template<typename T0> using Pressure =      Quantity<T0, 1,-1,-2, 0, 0, 0, 0>;      // Force over Area
template<typename T0> using Stress =        Quantity<T0, 1,-1,-2, 0, 0, 0, 0>;      // same as pressure
template<typename T0> using YoungModulus =  Quantity<T0, 1,-1,-2, 0, 0, 0, 0>;      // same as pressure

template<typename T0> using Energy =        Quantity<T0, 1, 2,-2, 0, 0, 0, 0>;      // same as momentum
template<typename T0> using Power =         Quantity<T0, 1, 2,-3, 0, 0, 0, 0>;      // energy over Time

template<typename T0> using Stiffness =     Quantity<T0, 1, 0,-2, 0, 0, 0, 0>;      // Force over Length
template<typename T0> using Complicance =   Quantity<T0,-1, 0, 2, 0, 0, 0, 0>;      // 1 over Stiffness

template<typename T0> using PoissonsRatio = Quantity<T0, 0, 0, 0, 0, 0, 0, 0>;      // unitless
template<typename T0> using Strain =        Quantity<T0, 0, 0, 0, 0, 0, 0, 0>;      // unitless
template<typename T0> using Angle =         Quantity<T0, 0, 0, 0, 0, 0, 0, 0>;      // radian (unitless)


} // namespace units
} // namespace sofa


#endif
