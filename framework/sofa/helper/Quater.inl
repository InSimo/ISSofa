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
#ifndef SOFA_HELPER_QUATER_INL
#define SOFA_HELPER_QUATER_INL

#include "Quater.h"
#include <limits>
#include <math.h>
#include <iostream>
#include <stdio.h>


namespace sofa
{

namespace helper
{

#define QUATER_UNIT_NORM2_THRESHOLD 1e-3 // threshold used to determine if a quaternion magnitude is close enough to a unit quaternion

#define RENORMCOUNT 50

// Constructor
template<class Real>
Quater<Real>::Quater()
{
    _q[0] = _q[1] = _q[2] = 0.0;
    _q[3] = 1.0;
}

template<class Real>
Quater<Real>::Quater(Real x, Real y, Real z, Real w)
{
    _q[0] = x;
    _q[1] = y;
    _q[2] = z;
    _q[3] = w;
}

template<class Real>
Quater<Real>::Quater( const defaulttype::Vec<3,Real>& axis, Real angle )
{
    axisToQuat(axis,angle);
}

// Destructor
template<class Real>
Quater<Real>::~Quater()
{
}

/// Given two rotations, e1 and e2, expressed as quaternion rotations,
/// figure out the equivalent single rotation and stuff it into dest.
/// This routine also normalizes the result every RENORMCOUNT times it is
/// called, to keep error from creeping in.
///   NOTE: This routine is written so that q1 or q2 may be the same
///  	   as dest (or each other).
template<class Real>
Quater<Real> Quater<Real>::operator+(const Quater<Real> &q1) const
{
    Real		t1[4], t2[4], t3[4];
    Real		tf[4];
    Quater<Real>	ret;

    t1[0] = _q[0] * q1._q[3];
    t1[1] = _q[1] * q1._q[3];
    t1[2] = _q[2] * q1._q[3];

    t2[0] = q1._q[0] * _q[3];
    t2[1] = q1._q[1] * _q[3];
    t2[2] = q1._q[2] * _q[3];

    // cross product t3 = q2 x q1
    t3[0] = (q1._q[1] * _q[2]) - (q1._q[2] * _q[1]);
    t3[1] = (q1._q[2] * _q[0]) - (q1._q[0] * _q[2]);
    t3[2] = (q1._q[0] * _q[1]) - (q1._q[1] * _q[0]);
    // end cross product

    tf[0] = t1[0] + t2[0] + t3[0];
    tf[1] = t1[1] + t2[1] + t3[1];
    tf[2] = t1[2] + t2[2] + t3[2];
    tf[3] = _q[3] * q1._q[3] -
            (_q[0] * q1._q[0] + _q[1] * q1._q[1] + _q[2] * q1._q[2]);

    ret._q[0] = tf[0];
    ret._q[1] = tf[1];
    ret._q[2] = tf[2];
    ret._q[3] = tf[3];

/*    if (++count > RENORMCOUNT)
    {
        count = 0;
        ret.normalize();
    } */

	ret.normalize();

    return ret;
}

template<class Real>
//Quater<Real> operator*(const Quater<Real>& q1, const Quater<Real>& q2) const
Quater<Real> Quater<Real>::operator*(const Quater<Real>& q1) const
{
    Quater<Real>	ret;

    ret._q[3] = _q[3] * q1._q[3] -
            (_q[0] * q1._q[0] +
                    _q[1] * q1._q[1] +
                    _q[2] * q1._q[2]);
    ret._q[0] = _q[3] * q1._q[0] +
            _q[0] * q1._q[3] +
            _q[1] * q1._q[2] -
            _q[2] * q1._q[1];
    ret._q[1] = _q[3] * q1._q[1] +
            _q[1] * q1._q[3] +
            _q[2] * q1._q[0] -
            _q[0] * q1._q[2];
    ret._q[2] = _q[3] * q1._q[2] +
            _q[2] * q1._q[3] +
            _q[0] * q1._q[1] -
            _q[1] * q1._q[0];

    return ret;
}

template<class Real>
Quater<Real> Quater<Real>::operator*(Real r) const
{
    Quater<Real>  ret;
    ret[0] = _q[0] * r;
    ret[1] = _q[1] * r;
    ret[2] = _q[2] * r;
    ret[3] = _q[3] * r;
    return ret;
}


template<class Real>
Quater<Real> Quater<Real>::operator/(Real r) const
{
    Quater<Real>  ret;
    ret[0] = _q[0] / r;
    ret[1] = _q[1] / r;
    ret[2] = _q[2] / r;
    ret[3] = _q[3] / r;
    return ret;
}

template<class Real>
Quater<Real>& Quater<Real>::operator*=(Real r)
{
    _q[0] *= r;
    _q[1] *= r;
    _q[2] *= r;
    _q[3] *= r;

    return *this;
}


template<class Real>
Quater<Real>& Quater<Real>::operator/=(Real r)
{
    _q[0] /= r;
    _q[1] /= r;
    _q[2] /= r;
    _q[3] /= r;

    return *this;
}


template<class Real>
Quater<Real> Quater<Real>::quatVectMult(const defaulttype::Vec<3,Real>& vect)
{
    Quater<Real>	ret;

    ret._q[3] = (Real) (-(_q[0] * vect[0] + _q[1] * vect[1] + _q[2] * vect[2]));
    ret._q[0] = (Real) (_q[3] * vect[0] + _q[1] * vect[2] - _q[2] * vect[1]);
    ret._q[1] = (Real) (_q[3] * vect[1] + _q[2] * vect[0] - _q[0] * vect[2]);
    ret._q[2] = (Real) (_q[3] * vect[2] + _q[0] * vect[1] - _q[1] * vect[0]);

    return ret;
}

template<class Real>
Quater<Real> Quater<Real>::vectQuatMult(const defaulttype::Vec<3,Real>& vect)
{
    Quater<Real>	ret;

    ret[3] = (Real) (-(vect[0] * _q[0] + vect[1] * _q[1] + vect[2] * _q[2]));
    ret[0] = (Real) (vect[0] * _q[3] + vect[1] * _q[2] - vect[2] * _q[1]);
    ret[1] = (Real) (vect[1] * _q[3] + vect[2] * _q[0] - vect[0] * _q[2]);
    ret[2] = (Real) (vect[2] * _q[3] + vect[0] * _q[1] - vect[1] * _q[0]);

    return ret;
}

template<class Real>
Quater<Real> Quater<Real>::inverse(bool normalize, Real epsilon) const
{
    (void)epsilon; // FIX warning in Release
    Quater<Real> result = Quater<Real>(-_q[0],-_q[1],-_q[2],_q[3]);
    if(normalize) result.normalize();
    assert(std::abs(result.norm() - 1) <= epsilon); // make sure we are dealing with a unit quaternion.
    return result;
}


template< class Real >
Real Quater<Real>::dot(const Quater<Real>& b) const
{
    return _q[0] * b[0] + _q[1]*b[1] + _q[2]*b[2] + _q[3]*b[3];
}

template< class Real >
Real Quater<Real>::norm2() const
{
    return dot(*this);
}

template< class Real >
Real Quater<Real>::norm() const
{
    return std::sqrt(norm2());
}

/// Quater<Real>s always obey:  a^2 + b^2 + c^2 + d^2 = 1.0
/// If they don't add up to 1.0, dividing by their magnitude will
/// renormalize them.
template<class Real>
void Quater<Real>::normalize()
{
    const Real _norm =  norm();
    Quater<Real>& q  = *this;

    if (_norm  > std::numeric_limits<SReal>::epsilon() )
    {
        q /= _norm;
    }
}

template<class Real>
bool Quater<Real>::isUnit() const
{
    return std::abs(norm2() - 1) < QUATER_UNIT_NORM2_THRESHOLD;
}


/// integrate quaternion using exponential map parametrisation
template<class Real>
void Quater<Real>::integrateExponentialMap(const Vec3& omega)
{
    Quater<Real>& qThis       = *this;
    Quater<Real> exp_omega    = exponentialMap(omega);
    qThis = exp_omega * qThis;
    assert( isUnit() );
}

template<class Real>
void Quater<Real>::integrate(const Vec3& omega)
{
    (*this).normalize();
    Quater<Real> qDot = (*this).vectQuatMult(omega);
    for (int i = 0; i < 4; i++)
    {
        (*this)[i] += qDot[i] * 0.5f;
    }
    (*this).normalize();
}

template<class Real>
void Quater<Real>::fromFrame(const defaulttype::Vec<3,Real>& x, const defaulttype::Vec<3,Real>&y, const defaulttype::Vec<3,Real>&z)
{
    defaulttype::Matrix3 R(x,y,z);
    R.transpose();
    this->fromMatrix(R);
}

template<class Real>
void Quater<Real>::fromMatrix(const defaulttype::Matrix3 &m)
{
    Real tr, s;

    tr = (Real)(m.x().x() + m.y().y() + m.z().z());

    // check the diagonal
    if (tr > 0)
    {
        s = (float)sqrt (tr + 1);
        _q[3] = s * 0.5f; // w OK
        s = 0.5f / s;
        _q[0] = (Real)((m.z().y() - m.y().z()) * s); // x OK
        _q[1] = (Real)((m.x().z() - m.z().x()) * s); // y OK
        _q[2] = (Real)((m.y().x() - m.x().y()) * s); // z OK
    }
    else
    {
        if (m.y().y() > m.x().x() && m.z().z() <= m.y().y())
        {
            s = (Real)sqrt ((m.y().y() - (m.z().z() + m.x().x())) + 1.0f);

            _q[1] = s * 0.5f; // y OK

            if (s != 0.0f)
                s = 0.5f / s;

            _q[2] = (Real)((m.y().z() + m.z().y()) * s); // z OK
            _q[0] = (Real)((m.x().y() + m.y().x()) * s); // x OK
            _q[3] = (Real)((m.x().z() - m.z().x()) * s); // w OK
        }
        else if ((m.y().y() <= m.x().x()  &&  m.z().z() > m.x().x())  ||  (m.z().z() > m.y().y()))
        {
            s = (Real)sqrt ((m.z().z() - (m.x().x() + m.y().y())) + 1.0f);

            _q[2] = s * 0.5f; // z OK

            if (s != 0.0f)
                s = 0.5f / s;

            _q[0] = (Real)((m.z().x() + m.x().z()) * s); // x OK
            _q[1] = (Real)((m.y().z() + m.z().y()) * s); // y OK
            _q[3] = (Real)((m.y().x() - m.x().y()) * s); // w OK
        }
        else
        {
            s = (Real)sqrt ((m.x().x() - (m.y().y() + m.z().z())) + 1.0f);

            _q[0] = s * 0.5f; // x OK

            if (s != 0.0f)
                s = 0.5f / s;

            _q[1] = (Real)((m.x().y() + m.y().x()) * s); // y OK
            _q[2] = (Real)((m.z().x() + m.x().z()) * s); // z OK
            _q[3] = (Real)((m.z().y() - m.y().z()) * s); // w OK
        }
    }
}

// template<class Real> template<class Mat33>
//     void Quater<Real>::toMatrix(Mat33 &m) const
// {
// 	m[0][0] = (1.0 - 2.0 * (_q[1] * _q[1] + _q[2] * _q[2]));
// 	m[0][1] = (2.0 * (_q[0] * _q[1] - _q[2] * _q[3]));
// 	m[0][2] = (2.0 * (_q[2] * _q[0] + _q[1] * _q[3]));
//
// 	m[1][0] = (2.0 * (_q[0] * _q[1] + _q[2] * _q[3]));
// 	m[1][1] = (1.0 - 2.0 * (_q[2] * _q[2] + _q[0] * _q[0]));
// 	m[1][2] = (float) (2.0 * (_q[1] * _q[2] - _q[0] * _q[3]));
//
// 	m[2][0] = (float) (2.0 * (_q[2] * _q[0] - _q[1] * _q[3]));
// 	m[2][1] = (float) (2.0 * (_q[1] * _q[2] + _q[0] * _q[3]));
// 	m[2][2] = (float) (1.0 - 2.0 * (_q[1] * _q[1] + _q[0] * _q[0]));
// }

/// Build a rotation matrix, given a quaternion rotation.
template<class Real>
void Quater<Real>::buildRotationMatrix(Real m[4][4]) const
{
    m[0][0] = (1.0f - 2.0f * (_q[1] * _q[1] + _q[2] * _q[2]));
    m[0][1] = (2.0f * (_q[0] * _q[1] - _q[2] * _q[3]));
    m[0][2] = (2.0f * (_q[2] * _q[0] + _q[1] * _q[3]));
    m[0][3] = 0;

    m[1][0] = (2.0f * (_q[0] * _q[1] + _q[2] * _q[3]));
    m[1][1] = (1.0f - 2.0f * (_q[2] * _q[2] + _q[0] * _q[0]));
    m[1][2] = (2.0f * (_q[1] * _q[2] - _q[0] * _q[3]));
    m[1][3] = 0;

    m[2][0] = (2.0f * (_q[2] * _q[0] - _q[1] * _q[3]));
    m[2][1] = (2.0f * (_q[1] * _q[2] + _q[0] * _q[3]));
    m[2][2] = (1.0f - 2.0f * (_q[1] * _q[1] + _q[0] * _q[0]));
    m[2][3] = 0;

    m[3][0] = 0;
    m[3][1] = 0;
    m[3][2] = 0;
    m[3][3] = 1;
}
/// Write an OpenGL rotation matrix
/*template<class Real>
void Quater<Real>::writeOpenGlMatrix(double *m) const
{
    m[0*4+0] = (1.0 - 2.0 * (_q[1] * _q[1] + _q[2] * _q[2]));
    m[0*4+1] = (2.0 * (_q[0] * _q[1] - _q[2] * _q[3]));
    m[0*4+2] = (2.0 * (_q[2] * _q[0] + _q[1] * _q[3]));
    m[0*4+3] = 0.0f;

    m[1*4+0] = (2.0 * (_q[0] * _q[1] + _q[2] * _q[3]));
    m[1*4+1] = (1.0 - 2.0 * (_q[2] * _q[2] + _q[0] * _q[0]));
    m[1*4+2] = (float) (2.0 * (_q[1] * _q[2] - _q[0] * _q[3]));
    m[1*4+3] = 0.0f;

    m[2*4+0] = (float) (2.0 * (_q[2] * _q[0] - _q[1] * _q[3]));
    m[2*4+1] = (float) (2.0 * (_q[1] * _q[2] + _q[0] * _q[3]));
    m[2*4+2] = (float) (1.0 - 2.0 * (_q[1] * _q[1] + _q[0] * _q[0]));
    m[2*4+3] = 0.0f;

    m[3*4+0] = 0.0f;
    m[3*4+1] = 0.0f;
    m[3*4+2] = 0.0f;
    m[3*4+3] = 1.0f;
}
*/
/// Write an OpenGL rotation matrix
template<class Real>
void Quater<Real>::writeOpenGlMatrix(double *m) const
{
    m[0*4+0] = (1.0f - 2.0f * (_q[1] * _q[1] + _q[2] * _q[2]));
    m[1*4+0] = (2.0f * (_q[0] * _q[1] - _q[2] * _q[3]));
    m[2*4+0] = (2.0f * (_q[2] * _q[0] + _q[1] * _q[3]));
    m[3*4+0] = 0.0f;

    m[0*4+1] = (2.0f * (_q[0] * _q[1] + _q[2] * _q[3]));
    m[1*4+1] = (1.0f - 2.0f * (_q[2] * _q[2] + _q[0] * _q[0]));
    m[2*4+1] = (float) (2.0f * (_q[1] * _q[2] - _q[0] * _q[3]));
    m[3*4+1] = 0.0f;

    m[0*4+2] = (float) (2.0f * (_q[2] * _q[0] - _q[1] * _q[3]));
    m[1*4+2] = (float) (2.0f * (_q[1] * _q[2] + _q[0] * _q[3]));
    m[2*4+2] = (float) (1.0f - 2.0f * (_q[1] * _q[1] + _q[0] * _q[0]));
    m[3*4+2] = 0.0f;

    m[0*4+3] = 0.0f;
    m[1*4+3] = 0.0f;
    m[2*4+3] = 0.0f;
    m[3*4+3] = 1.0f;
}

/// Write an OpenGL rotation matrix
template<class Real>
void Quater<Real>::writeOpenGlMatrix(float *m) const
{
    m[0*4+0] = (float) (1.0f - 2.0f * (_q[1] * _q[1] + _q[2] * _q[2]));
    m[1*4+0] = (float) (2.0f * (_q[0] * _q[1] - _q[2] * _q[3]));
    m[2*4+0] = (float) (2.0f * (_q[2] * _q[0] + _q[1] * _q[3]));
    m[3*4+0] = 0.0f;

    m[0*4+1] = (float) (2.0f * (_q[0] * _q[1] + _q[2] * _q[3]));
    m[1*4+1] = (float) (1.0f - 2.0f * (_q[2] * _q[2] + _q[0] * _q[0]));
    m[2*4+1] = (float) (2.0f * (_q[1] * _q[2] - _q[0] * _q[3]));
    m[3*4+1] = 0.0f;

    m[0*4+2] = (float) (2.0f * (_q[2] * _q[0] - _q[1] * _q[3]));
    m[1*4+2] = (float) (2.0f * (_q[1] * _q[2] + _q[0] * _q[3]));
    m[2*4+2] = (float) (1.0f - 2.0f * (_q[1] * _q[1] + _q[0] * _q[0]));
    m[3*4+2] = 0.0f;

    m[0*4+3] = 0.0f;
    m[1*4+3] = 0.0f;
    m[2*4+3] = 0.0f;
    m[3*4+3] = 1.0f;
}


/// Taken from
/// @ARTICLE{Grassia98practicalparameterization,
/// author ={ F.Sebastian Grassia },
/// title ={ Practical parameterization of rotations using the exponential map },
/// journal ={ Journal of Graphics Tools },
/// year ={ 1998 },
/// volume ={ 3 },
/// pages ={ 29--48 }
template<class Real>
Quater<Real> Quater<Real>::exponentialMap(const defaulttype::Vec<3, Real>& axisAngle, Real epsilon)
{
    const Real theta = axisAngle.norm();
    
    if (theta < epsilon)
    {
        // taylor expansion of the sinc function around zero, maybe overkill to go that far in the expansion
        const Real sinc = Real(0.5) + std::pow(theta, Real(2)) / Real(48)
            - std::pow(theta, Real(4)) / (std::pow(Real(2), Real(5)) * Real(120));

        const Quater<Real> exp(sinc * axisAngle[0],
            sinc * axisAngle[1],
            sinc * axisAngle[2],
            std::cos(theta / Real(2)));

        return exp;
    }
    else
    {
        const Real sinc = std::sin(theta * Real(0.5)) / theta;
        const Quater<Real> exp(sinc * axisAngle[0],
            sinc * axisAngle[1],
            sinc * axisAngle[2],
            std::cos(theta / Real(2)));
        return exp;
    }
}

/// Given an axis and angle, compute quaternion.
template<class Real>
Quater<Real>& Quater<Real>::axisToQuat(const defaulttype::Vec<3,Real>& a, Real phi, Real epsilon)
{
    defaulttype::Vec<3, Real> axis = a;
    axis.normalize();
    axis *= phi;

    *this = exponentialMap(axis,epsilon);

    return *this;
}

/// Given a quaternion, compute an axis and angle
template<class Real>
void Quater<Real>::quatToAxis(defaulttype::Vec<3,Real> & axis, Real &angle, bool normalizeQuaternion, Real epsilon ) const
{
    axis = getLog( normalizeQuaternion, epsilon);
    
    angle = axis.norm();

    if (angle > epsilon)
    {
        axis /= angle;
    }
}


/// Returns the log of a unit quaternion
/// A unit quaternion represents a rotation of angle phi around an axis u
/// q = cos ( phi/2 ) + u . sin( phi / 2 )
///
/// we note theta = phi / 2 to simplify the notation ( conversely phi = 2 * theta )
///
/// log( q ) = log( cos_theta + u sin_theta ) = log( exp( u.theta ) = u.theta = [ 0 , u.theta ]
/// u = q_v / || q_v || where q_v is the vectorial part of the quaternion
/// theta = arctan ( || q_v || / q_w ) or theta = arccos( q_w ) 
/// 
/// log( q ) = q_v * arctan( || q_v || / q_w ) / ||q_v||  
/// using the Taylor series of arctan it can be proven that 
/// log( q ) ~ q_v  when theta is small
/// 
/// @param normalize If set to true a normalization step is performed to work on a unit quaterion
/// @param epsilon  A small value used to evaluate if we are dealing with a small angle.
/// @return The logarithm of the quaternion.
template<class Real>
defaulttype::Vec<3,Real> Quater<Real>::getLog(bool normalize, Real epsilon) const
{
    Quater< Real > q = *this;

    if(normalize)
    {
        q.normalize();
    }

    assert( isUnit() ); // make sure we are dealing with a unit quaternion.

    defaulttype::Vec<3,Real> v(q[0], q[1], q[2]);

    if (q[3] < 0)
    { 
        v = -v; // flip v to get result between 0 and pi
    }

    Real norm = v.norm(); // sin(theta) = sin( phi / 2 )
    Real q_w  = q[3] < 0 ? -q[3] : q[3]; // q[3] = cos_theta : flip to get angle between 0 and pi 

    // even if we are dealing with the unit quaternion
    // these values can be slightly "off" from one
    q_w = std::min(q_w, Real(1.0) ); 
    norm = std::min(norm, Real(1.0) );
    
    // avoid numerical instabilities of arccos when below 5 degrees
    // q[3] > 0.999 => cos( theta ) > 0.999 => theta < 5 degrees
    if (q_w > Real(0.999))
    {
        const Real sin_half_angle = norm;
        const Real angle = Real(2.0) * std::asin(norm);
        if (sin_half_angle > epsilon)
        {
            v *= (angle/sin_half_angle);
        }
    }
    else
    {
        const Real half_angle = std::acos(q_w);
        const Real sin_half_angle = std::sin(half_angle);
        const Real angle = Real(2)*half_angle;
        if (sin_half_angle > epsilon)
        {
            v *= (angle/sin_half_angle);
        }
    }

    return v;
}

// return euler angles with roll and yaw between -pi and +pi
//      and pitch between -pi/2 and pi/2
template<class Real>
defaulttype::Vec<3, Real> Quater<Real>::getEuler(bool normalize, Real epsilon) const
{
    defaulttype::Vec<3, Real> v;
    double qnorm2 = norm2();
    // roll (x-axis rotation)
    double sinr = +2.0 * (_q[3] * _q[0] + _q[1] * _q[2]);
    double cosr = qnorm2 - 2.0 * (_q[0] * _q[0] + _q[1] * _q[1]);
    v[0] = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (_q[3] * _q[1] - _q[2] * _q[0]);
    if (normalize && qnorm2 - 1 > epsilon*epsilon)
        sinp /= qnorm2;
    if (fabs(sinp) >= 1)
        v[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        v[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (_q[3] * _q[2] + _q[0] * _q[1]);
    double cosy = qnorm2 - 2.0 * (_q[1] * _q[1] + _q[2] * _q[2]);
    v[2] = atan2(siny, cosy);

    return v;

}


/// @Deprecated. Use getLog method instead.
template<class Real>
defaulttype::Vec<3,Real> Quater<Real>::toEulerVector() const
{
    return getLog();
}

// get rotation around an axis (alternative to getLog()) which might have issues handling large rotations
template<class Real>
Real Quater<Real>::getRotationAroundAxis(const defaulttype::Vec<3, Real>& axis) const
{
    if (axis.norm() < 1e-6)
    {
        return 0.0;
    }

    Vec3 r = axis;
    r.normalize();
    Vec3 p;

    //get p perpendicular to r
    if (r[2] != 0.0)
    {
        p[0] = Real(1);
        p[1] = Real(1);
        p[2] = (-r[0] * p[0] - r[1] * p[1]) / r[2];
    }
    else
    {
        p[0] = -r[1];
        p[1] = r[0];
        p[2] = Real(0);
    }
    p.normalize();

    // eval rotation of p by q
    Vec3 p_r = (*this).rotate(p);
    p_r.normalize();


    // project p_r on plane of normal vector r
    Vec3 p_r_proj = p_r - sofa::defaulttype::dot(p_r, r) * r;
    p_r_proj.normalize();


    // get angle between p and p_r_proj
    Real theta = std::acos(sofa::defaulttype::dot(p, p_r_proj));

    if (sofa::defaulttype::dot(r, cross(p, p_r_proj)) < 0)
    {
        theta = -theta;
    }
    return theta;
}


// cancel rotation around an axis
template<class Real>
void Quater<Real>::cancelRotationAroundAxis(const defaulttype::Vec<3, Real>& axis)
{
    if (axis.norm() < 1e-6)
    {
        return;
    }

    Real theta = getRotationAroundAxis(axis);

    // get quaternion encoding rotation theta around axis
    Vec3 r = axis;
    r.normalize();
    Quater<Real> q_theta;
    q_theta = q_theta.axisToQuat(r, theta);

    // cancel rotation
    *this = q_theta.inverse() * (*this);
}


/*! Returns the slerp interpolation of Quaternions \p a and \p b, at time \p t.

 \p t should range in [0,1]. Result is \p a when \p t=0 and \p b when \p t=1.

 When \p allowFlip is \c true (default) the slerp interpolation will always use the "shortest path"
 between the Quaternions' orientations, by "flipping" the source Quaternion if needed (see
 negate()). */
template<class Real>
void Quater<Real>::slerp(const Quater& a, const Quater& b, Real t, bool allowFlip)
{
    Real cosAngle =  (Real)(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);

    Real c1, c2;
    // Linear interpolation for close orientations
    if ((1.0 - std::fabs(cosAngle)) < 0.01)
    {
        c1 = 1.0f - t;
        c2 = t;
    }
    else
    {
        // Spherical interpolation
        Real angle    = (Real)std::acos((Real)std::fabs((Real)cosAngle));
        Real sinAngle = (Real)std::sin((Real)angle);
        c1 = (Real)std::sin(angle * (1.0f - t)) / sinAngle;
        c2 = (Real)std::sin(angle * t) / sinAngle;
    }

    // Use the shortest path
    if (allowFlip && (cosAngle < 0.0f))
        c1 = -c1;

    _q[0] = c1*a[0] + c2*b[0];
    _q[1] = c1*a[1] + c2*b[1];
    _q[2] = c1*a[2] + c2*b[2];
    _q[3] = c1*a[3] + c2*b[3];
}

///// Output quaternion
//template<class Real>
//    std::ostream& operator<<(std::ostream& out, Quater<Real> Q)
//{
//	return (out << "(" << Q._q[0] << "," << Q._q[1] << "," << Q._q[2] << ","
//				<< Q._q[3] << ")");
//}

template<class Real>
Quater<Real> Quater<Real>::slerp(Quater<Real> &q1, Real t)
{
    Quater<Real> q0_1;
    for (unsigned int i = 0 ; i<3 ; i++)
        q0_1[i] = -_q[i];

    q0_1[3] = _q[3];

    q0_1 = q1 * q0_1;

    defaulttype::Vec<3,Real> axis, temp;
    Real angle;

    q0_1.quatToAxis(axis, angle);

    temp = axis * sin(t * angle);
    for (unsigned int i = 0 ; i<3 ; i++)
        q0_1[i] = temp[i];

    q0_1[3] = cos(t * angle);
    q0_1 = q0_1 * (*this);
    return q0_1;
}

// Given an axis and angle, compute quaternion.
template<class Real>
Quater<Real> Quater<Real>::slerp2(Quater<Real> &q1, Real t)
{
    // quaternion to return
    Quater<Real> qm;

    // Calculate angle between them.
    double cosHalfTheta = _q[3] * q1[3] + _q[0] * q1[0] + _q[1] * q1[1] + _q[2] * q1[2];
    // if qa=qb or qa=-qb then theta = 0 and we can return qa
    if (std::fabs(cosHalfTheta) >= 1.0)
    {
        qm[3] = _q[3]; qm[0] = _q[0]; qm[1] = _q[1]; qm[2] = _q[2];
        return qm;
    }
    // Calculate temporary values.
    double halfTheta = acos(cosHalfTheta);
    double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
    // if theta = 180 degrees then result is not fully defined
    // we could rotate around any axis normal to qa or qb
    if (std::fabs(sinHalfTheta) < 0.001)  // fabs is floating point absolute
    {
        qm[3] = (Real)(_q[3] * 0.5 + q1[3] * 0.5);
        qm[0] = (Real)(_q[0] * 0.5 + q1[0] * 0.5);
        qm[1] = (Real)(_q[1] * 0.5 + q1[1] * 0.5);
        qm[2] = (Real)(_q[2] * 0.5 + q1[2] * 0.5);
        return qm;
    }
    double ratioA = std::sin((1 - t) * halfTheta) / sinHalfTheta;
    double ratioB = std::sin(t * halfTheta) / sinHalfTheta;
    //calculate Quaternion.
    qm[3] = (Real)(_q[3] * ratioA + q1[3] * ratioB);
    qm[0] = (Real)(_q[0] * ratioA + q1[0] * ratioB);
    qm[1] = (Real)(_q[1] * ratioA + q1[1] * ratioB);
    qm[2] = (Real)(_q[2] * ratioA + q1[2] * ratioB);
    return qm;

}

template<class Real>
Quater<Real> Quater<Real>::createQuaterFromFrame(const defaulttype::Vec<3, Real> &lox, const defaulttype::Vec<3, Real> &loy,const defaulttype::Vec<3, Real> &loz)
{
    Quater<Real> q;
    sofa::defaulttype::Mat<3,3, Real> m;

    for (unsigned int i=0 ; i<3 ; i++)
    {
        m[i][0] = lox[i];
        m[i][1] = loy[i];
        m[i][2] = loz[i];
    }
    q.fromMatrix(m);
    return q;
}

/// Print quaternion (C style)
template<class Real>
void Quater<Real>::print()
{
    printf("(%f, %f ,%f, %f)\n", _q[0], _q[1], _q[2], _q[3]);
}

template<class Real>
Quater<Real>& Quater<Real>::operator+=(const Quater<Real>& q2)
{

    Real t1[4], t2[4], t3[4];
    Quater<Real> q1 = (*this);
    t1[0] = q1._q[0] * q2._q[3];
    t1[1] = q1._q[1] * q2._q[3];
    t1[2] = q1._q[2] * q2._q[3];

    t2[0] = q2._q[0] * q1._q[3];
    t2[1] = q2._q[1] * q1._q[3];
    t2[2] = q2._q[2] * q1._q[3];

    // cross product t3 = q2 x q1
    t3[0] = (q2._q[1] * q1._q[2]) - (q2._q[2] * q1._q[1]);
    t3[1] = (q2._q[2] * q1._q[0]) - (q2._q[0] * q1._q[2]);
    t3[2] = (q2._q[0] * q1._q[1]) - (q2._q[1] * q1._q[0]);
    // end cross product

    _q[0] = t1[0] + t2[0] + t3[0];
    _q[1] = t1[1] + t2[1] + t3[1];
    _q[2] = t1[2] + t2[2] + t3[2];
    _q[3] = q1._q[3] * q2._q[3] -
            (q1._q[0] * q2._q[0] + q1._q[1] * q2._q[1] + q1._q[2] * q2._q[2]);

	normalize();

    return *this;
}

template<class Real>
Quater<Real>& Quater<Real>::operator*=(const Quater<Real>& q1)
{
    Quater<Real> q2 = *this;
    _q[3] = q2._q[3] * q1._q[3] -
            (q2._q[0] * q1._q[0] +
                    q2._q[1] * q1._q[1] +
                    q2._q[2] * q1._q[2]);
    _q[0] = q2._q[3] * q1._q[0] +
            q2._q[0] * q1._q[3] +
            q2._q[1] * q1._q[2] -
            q2._q[2] * q1._q[1];
    _q[1] = q2._q[3] * q1._q[1] +
            q2._q[1] * q1._q[3] +
            q2._q[2] * q1._q[0] -
            q2._q[0] * q1._q[2];
    _q[2] = q2._q[3] * q1._q[2] +
            q2._q[2] * q1._q[3] +
            q2._q[0] * q1._q[1] -
            q2._q[1] * q1._q[0];

    return *this;
}


} // namespace helper

} // namespace sofa

#endif
