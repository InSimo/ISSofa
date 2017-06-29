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


////////////////////////////////////////////////////////
//
// Vocabulary : a quantity (eg : a force) has got 
//    units (or dimensions, eg : kg.m.s-2 for a force)
//
// Example of usage : see UnitsTest.cpp
//
////////////////////////////////////////////////////////


//////////////////////////// Base class Units //////////////////////////////

template<class T, // Precision
            int kg, // Mass
                int m, // Length
                    int s, // Time
                        int A, // Electric current
                            int K, // Thermodynamic temperature
                                int mol, // Amount of substance
                                    int cd> // Luminous intensity
class Units
{
public:

    // Operator: default constructor
    explicit
        Units(T initVal = 0)
        : m_value(initVal)
    {
    }

    // Operator: Assignment from type T
    Units<T, kg, m, s, A, K, mol, cd>&
        operator= (const T rhs)
    {
        m_value = rhs;
        return *this;
    }

    // Operator: +=
    Units<T, kg, m, s, A, K, mol, cd>&
        operator+= (const Units<T, kg, m, s, A, K, mol, cd>& rhs)
    {
        m_value += rhs.m_value;
        return *this;
    }

    // Operator -=
    Units<T, kg, m, s, A, K, mol, cd>&
        operator-= (const Units<T, kg, m, s, A, K, mol, cd>& rhs)
    {
        m_value -= rhs.m_value;
        return *this;
    }

    // Operator *=
    Units<T, kg, m, s, A, K, mol, cd>&
        operator*= (T rhs)
    {
        m_value *= rhs;
        return *this;
    }

    // Operator /=
    Units<T, kg, m, s, A, K, mol, cd>&
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
    const Units<T, kg, m, s, A, K, mol, cd>
    operator* (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
        const Scalar& rhs)
{
    Units<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result *= rhs;
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd,
    class Scalar>
    const Units<T, kg, m, s, A, K, mol, cd>
    operator* (const Scalar& lhs,
        const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Units<T, kg, m, s, A, K, mol, cd> result(rhs);
    return result *= lhs;
}

// Operator: Division
template<class T, int kg, int m, int s, int A, int K, int mol, int cd,
    class Scalar>
    const Units<T, kg, m, s, A, K, mol, cd>
    operator/ (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
        const Scalar& rhs)
{
    Units<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result /= rhs;
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd,
    class Scalar>
    const Units<T, -kg, -m, -s, -A, -K, -mol, -cd>
    operator/ (const Scalar& lhs,
        const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Units<T, -kg, -m, -s, -A, -K, -mol, -cd> result(lhs);
    return result /= rhs.value();
}



////////////////////////
// Operators with same types

// Operator: Addition
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Units<T, kg, m, s, A, K, mol, cd>
operator+ (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Units<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result += rhs;
}

// Operator: Substraction
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Units<T, kg, m, s, A, K, mol, cd>
operator- (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Units<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result -= rhs;
}

// Operator: Multiplication
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Units<T, kg, m, s, A, K, mol, cd>
operator* (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Units<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result *= rhs;
}

// Operator: Division
template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
const Units<T, kg, m, s, A, K, mol, cd>
operator/ (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    Units<T, kg, m, s, A, K, mol, cd> result(lhs);
    return result /= rhs;
}



////////////////////////
// Operators between different types

// Operator: Multiplication (Creates New Type)
template<class T,
    int kg1, int m1, int s1, int A1, int K1, int mol1, int cd1,
    int kg2, int m2, int s2, int A2, int K2, int mol2, int cd2>

    // Return Type
    Units<T, kg1 + kg2, m1 + m2, s1 + s2, A1 + A2, K1 + K2, mol1 + mol2, cd1 + cd2>
    operator* (const Units<T, kg1, m1, s1, A1, K1, mol1, cd1>& lhs,
        const Units<T, kg2, m2, s2, A2, K2, mol2, cd2>& rhs)
{
    // New Return type
    typedef Units<T,
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
    Units<T, kg1 - kg2, m1 - m2, s1 - s2, A1 - A2, K1 - K2, mol1 - mol2, cd1 - cd2>
    operator/ (const Units<T, kg1, m1, s1, A1, K1, mol1, cd1>& lhs,
        const Units<T, kg2, m2, s2, A2, K2, mol2, cd2>& rhs)
{
    // New Return type
    typedef Units<T,
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
operator< (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() < rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator> (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() > rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator<= (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() <= rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator>= (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() >= rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator== (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() == rhs.value();
}

template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
bool
operator!= (const Units<T, kg, m, s, A, K, mol, cd> & lhs,
    const Units<T, kg, m, s, A, K, mol, cd> & rhs)
{
    return lhs.value() != rhs.value();
}




////////////////////////// Some types definitions ///////////////////////////

//  Base Quantities (units from the International System)
typedef Units<double, 0, 0, 0, 0, 0, 0, 0> uScalar;             // unitless

typedef Units<double, 1, 0, 0, 0, 0, 0, 0> uMass;               // mass in kilogram (kg)
typedef Units<double, 0, 1, 0, 0, 0, 0, 0> uLength;             // length in meter (m)
typedef Units<double, 0, 0, 1, 0, 0, 0, 0> uTime;               // time in second (s)
typedef Units<double, 0, 0, 0, 1, 0, 0, 0> uECurrent;           // electric current in ampere (A)
typedef Units<double, 0, 0, 0, 0, 1, 0, 0> uTemperature;        // temperature in kelvin (K)
typedef Units<double, 0, 0, 0, 0, 0, 1, 0> uAmount;             // amount of substance in mole (mol)
typedef Units<double, 0, 0, 0, 0, 0, 0, 1> uLIntensity;         // luminous intensity in candela (cd)


//  Derived Quantities (combinations of units from the International System)
typedef Units<double, 0, 2, 0, 0, 0, 0, 0> uArea;               // Length^2
typedef Units<double, 0, 3, 0, 0, 0, 0, 0> uVolume;             // Length^3

typedef Units<double, 0, 0,-1, 0, 0, 0, 0> uFrequency;          // Time^(-1)

typedef Units<double, 0, 1,-1, 0, 0, 0, 0> uVelocity;           // Length over Time
typedef Units<double, 0, 1,-2, 0, 0, 0, 0> uAcceleration;       // Length over Time^2

typedef Units<double, 1, 1,-2, 0, 0, 0, 0> uForce;              // Mass * Acceleration
typedef Units<double, 1, 2,-2, 0, 0, 0, 0> uMomentum;           // Force * Length

typedef Units<double, 1,-1,-2, 0, 0, 0, 0> uPressure;           // Force over Area
typedef Units<double, 1,-1,-2, 0, 0, 0, 0> uStress;             // same as pressure
typedef Units<double, 1,-1,-2, 0, 0, 0, 0> uYoungModulus;       // same as pressure

typedef Units<double, 1, 2,-2, 0, 0, 0, 0> uEnergy;             // same as momentum
typedef Units<double, 1, 2,-3, 0, 0, 0, 0> uPower;              // energy over Time

typedef Units<double, 1, 0,-2, 0, 0, 0, 0> uStiffness;          // Force over Length
typedef Units<double,-1, 0, 2, 0, 0, 0, 0> uComplicance;        // 1 over Stiffness

typedef Units<double, 0, 0, 0, 0, 0, 0, 0> uPoissonsRatio;      // unitless
typedef Units<double, 0, 0, 0, 0, 0, 0, 0> uStrain;             // unitless
typedef Units<double, 0, 0, 0, 0, 0, 0, 0> uAngle;              // radian (unitless)


} // namespace units
} // namespace sofa


#endif