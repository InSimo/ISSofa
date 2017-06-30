/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
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

namespace sofa {
namespace units {


//////////////////////////////////////////////////////////////////////////////////////
//
// Vocabulary convention : a physical Quantity (eg : a force) 
//    has got units (or dimensions, eg : kg.m.s-2 for a force),
//      can have a prefix (a factor which is a multiple or a fraction)
//
// Important : any physical Quantity can be described thanks to a combination of 7
//    "elementary" units which are kilogram, meter, second, Ampere, kelvin, mole, candela.
//      It could be possible to describe a physical Quantity in other units 
//            (eg Celsius instead of Kelvin) thanks to conversion algorithms.
//
//
// Example of usage : 
//    Mass            m0( 1 );
//    Acceleration    a0( 2 );
//
//    Force           f0( m0 * a0 );      // this will compile
//    Time            t0( a0 );           // this won't compile
//
//    if( f0 == Force(4) ) {}             // this will compile
//    if( f0 == 4 ) {}                    // this won't compile
//    if( f0.value() == 4) {}             // this will compile
//    if( Scalar(10) == 42 ) {}           // this will compile as 42 thanks to a template specialization on scalars
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
class Quantity
{
public:

    // Operator: default constructor
    explicit
        Quantity(T initVal = 0)
        : m_value(initVal)
    {
    }

    // Operator: Assignment from type T
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator= (const T rhs)
    {
        m_value = rhs;
        return *this;
    }

    // Operator: +=
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator+= (const Quantity<T, kg, m, s, A, K, mol, cd>& rhs)
    {
        m_value += rhs.m_value;
        return *this;
    }

    // Operator -=
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator-= (const Quantity<T, kg, m, s, A, K, mol, cd>& rhs)
    {
        m_value -= rhs.m_value;
        return *this;
    }

    // Operator *=
    Quantity<T, kg, m, s, A, K, mol, cd>&
        operator*= (T rhs)
    {
        m_value *= rhs;
        return *this;
    }

    // Operator /=
    Quantity<T, kg, m, s, A, K, mol, cd>&
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
template<class T,
    int kg1, int m1, int s1, int A1, int K1, int mol1, int cd1,
    int kg2, int m2, int s2, int A2, int K2, int mol2, int cd2>

    // Return Type
    Quantity<T, kg1 + kg2, m1 + m2, s1 + s2, A1 + A2, K1 + K2, mol1 + mol2, cd1 + cd2>
    operator* (const Quantity<T, kg1, m1, s1, A1, K1, mol1, cd1>& lhs,
        const Quantity<T, kg2, m2, s2, A2, K2, mol2, cd2>& rhs)
{
    // New Return type
    typedef Quantity<T,
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
template<class T,
    int kg1, int m1, int s1, int A1, int K1, int mol1, int cd1,
    int kg2, int m2, int s2, int A2, int K2, int mol2, int cd2>

    // Return Type
    Quantity<T, kg1 - kg2, m1 - m2, s1 - s2, A1 - A2, K1 - K2, mol1 - mol2, cd1 - cd2>
    operator/ (const Quantity<T, kg1, m1, s1, A1, K1, mol1, cd1>& lhs,
        const Quantity<T, kg2, m2, s2, A2, K2, mol2, cd2>& rhs)
{
    // New Return type
    typedef Quantity<T,
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
    operator T() const { return val; }
    

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
typedef Quantity<double, 0, 0, 0, 0, 0, 0, 0> Scalar;             // unitless

typedef Quantity<double, 1, 0, 0, 0, 0, 0, 0> Mass;               // mass in kilogram (kg)
typedef Quantity<double, 0, 1, 0, 0, 0, 0, 0> Length;             // length in meter (m)
typedef Quantity<double, 0, 0, 1, 0, 0, 0, 0> Time;               // time in second (s)
typedef Quantity<double, 0, 0, 0, 1, 0, 0, 0> ECurrent;           // electric current in ampere (A)
typedef Quantity<double, 0, 0, 0, 0, 1, 0, 0> Temperature;        // temperature in kelvin (K)
typedef Quantity<double, 0, 0, 0, 0, 0, 1, 0> Amount;             // amount of substance in mole (mol)
typedef Quantity<double, 0, 0, 0, 0, 0, 0, 1> LIntensity;         // luminous intensity in candela (cd)


//  Derived Quantities (combinations of units from the International System)
typedef Quantity<double, 0, 2, 0, 0, 0, 0, 0> Area;               // Length^2
typedef Quantity<double, 0, 3, 0, 0, 0, 0, 0> Volume;             // Length^3

typedef Quantity<double, 0, 0,-1, 0, 0, 0, 0> Frequency;          // Time^(-1)

typedef Quantity<double, 0, 1,-1, 0, 0, 0, 0> Velocity;           // Length over Time
typedef Quantity<double, 0, 1,-2, 0, 0, 0, 0> Acceleration;       // Length over Time^2

typedef Quantity<double, 1, 1,-2, 0, 0, 0, 0> Force;              // Mass * Acceleration
typedef Quantity<double, 1, 2,-2, 0, 0, 0, 0> Momentum;           // Force * Length

typedef Quantity<double, 1,-1,-2, 0, 0, 0, 0> Pressure;           // Force over Area
typedef Quantity<double, 1,-1,-2, 0, 0, 0, 0> Stress;             // same as pressure
typedef Quantity<double, 1,-1,-2, 0, 0, 0, 0> YoungModulus;       // same as pressure

typedef Quantity<double, 1, 2,-2, 0, 0, 0, 0> Energy;             // same as momentum
typedef Quantity<double, 1, 2,-3, 0, 0, 0, 0> Power;              // energy over Time

typedef Quantity<double, 1, 0,-2, 0, 0, 0, 0> Stiffness;          // Force over Length
typedef Quantity<double,-1, 0, 2, 0, 0, 0, 0> Complicance;        // 1 over Stiffness

typedef Quantity<double, 0, 0, 0, 0, 0, 0, 0> PoissonsRatio;      // unitless
typedef Quantity<double, 0, 0, 0, 0, 0, 0, 0> Strain;             // unitless
typedef Quantity<double, 0, 0, 0, 0, 0, 0, 0> Angle;              // radian (unitless)


} // namespace units
} // namespace sofa


#endif