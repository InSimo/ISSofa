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
#ifndef SOFA_COMPONENT_MAPPING_IDENTITYMAPPING_INL
#define SOFA_COMPONENT_MAPPING_IDENTITYMAPPING_INL

#include <SofaBaseMechanics/IdentityMapping.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

#include <sofa/core/Mapping.inl>


namespace sofa
{

namespace component
{

namespace mapping
{

template<class T1, class T2>
static inline void eq(T1& dest, const T2& src)
{
    dest = src;
}

template<class T1, class T2>
static inline void peq(T1& dest, const T2& src)
{
    dest += src;
}

// float <-> double (to remove warnings)

//template<>
static inline void eq(float& dest, const double& src)
{
    dest = (float)src;
}

//template<>
static inline void peq(float& dest, const double& src)
{
    dest += (float)src;
}

// Vec <-> Vec

template<int N1, int N2, class T1, class T2>
static inline void eq(defaulttype::Vec<N1,T1>& dest, const defaulttype::Vec<N2,T2>& src)
{
    dest = src;
}

template<int N1, int N2, class T1, class T2>
static inline void peq(defaulttype::Vec<N1,T1>& dest, const defaulttype::Vec<N2,T2>& src)
{
    for (unsigned int i=0; i<(N1>N2?N2:N1); i++)
        dest[i] += (T1)src[i];
}

// RigidDeriv <-> RigidDeriv

template<int N, class T1, class T2>
static inline void eq(defaulttype::RigidDeriv<N,T1>& dest, const defaulttype::RigidDeriv<N,T2>& src)
{
    dest.getVCenter() = src.getVCenter();
    dest.getVOrientation() = (typename defaulttype::RigidDeriv<N,T1>::Rot)src.getVOrientation();
}

template<int N, class T1, class T2>
static inline void peq(defaulttype::RigidDeriv<N,T1>& dest, const defaulttype::RigidDeriv<N,T2>& src)
{
    dest.getVCenter() += src.getVCenter();
    dest.getVOrientation() += (typename defaulttype::RigidDeriv<N,T1>::Rot)src.getVOrientation();
}

// RigidCoord <-> RigidCoord

template<int N, class T1, class T2>
static inline void eq(defaulttype::RigidCoord<N,T1>& dest, const defaulttype::RigidCoord<N,T2>& src)
{
    dest.getCenter() = src.getCenter();
    dest.getOrientation() = (typename defaulttype::RigidCoord<N,T1>::Rot)src.getOrientation();
}

template<int N, class T1, class T2>
static inline void peq(defaulttype::RigidCoord<N,T1>& dest, const defaulttype::RigidCoord<N,T2>& src)
{
    dest.getCenter() += src.getCenter();
    dest.getOrientation() += src.getOrientation();
}

// RigidDeriv <-> Vec

template<int N, class T1, class T2>
static inline void eq(defaulttype::Vec<N,T1>& dest, const defaulttype::RigidDeriv<N,T2>& src)
{
    dest = src.getVCenter();
}

template<int N, class T1, class T2>
static inline void peq(defaulttype::Vec<N,T1>& dest, const defaulttype::RigidDeriv<N,T2>& src)
{
    dest += src.getVCenter();
}

template<int N, class T1, class T2>
static inline void eq(defaulttype::RigidDeriv<N,T1>& dest, const defaulttype::Vec<N,T2>& src)
{
    dest.getVCenter() = src;
    //dest.getVOrientation() = defaulttype::RigidDeriv<N,T1>::Rot(); //.clear();
}

template<int N, class T1, class T2>
static inline void peq(defaulttype::RigidDeriv<N,T1>& dest, const defaulttype::Vec<N,T2>& src)
{
    dest.getVCenter() += src;
}

// RigidCoord <-> Vec

template<int N, class T1, class T2>
static inline void eq(defaulttype::Vec<N,T1>& dest, const defaulttype::RigidCoord<N,T2>& src)
{
    dest = src.getCenter();
}

template<int N, class T1, class T2>
static inline void peq(defaulttype::Vec<N,T1>& dest, const defaulttype::RigidCoord<N,T2>& src)
{
    dest += src.getCenter();
}

template<int N, class T1, class T2>
static inline void eq(defaulttype::RigidCoord<N,T1>& dest, const defaulttype::Vec<N,T2>& src)
{
    dest.getCenter() = src;
    //dest.getOrientation() = defaulttype::RigidCoord<N,T1>::Rot(); //.clear();
}

template<int N, class T1, class T2>
static inline void peq(defaulttype::RigidCoord<N,T1>& dest, const defaulttype::Vec<N,T2>& src)
{
    dest.getCenter() += src;
}

template<class TIn, class TOut>
void IdentityMapping<TIn, TOut>::init()
{
    stateFrom = core::behavior::BaseMechanicalState::DynamicCast(this->fromModel.get());
    stateTo = core::behavior::BaseMechanicalState::DynamicCast(this->toModel.get());

    this->toModel->resize(this->fromModel->getSize());
    Inherit::init();
}

template <class TIn, class TOut>
void IdentityMapping<TIn, TOut>::apply(const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data<VecCoord>& dOut, const Data<InVecCoord>& dIn)
{
    helper::WriteOnlyAccessor< Data<VecCoord> > out = dOut;
    helper::ReadAccessor< Data<InVecCoord> > in = dIn;

    out.resize(in.size());

    for(unsigned int i=0; i<out.size(); i++)
    {
        eq(out[i], in[i]);
    }
}

template <class TIn, class TOut>
void IdentityMapping<TIn, TOut>::applyJ(const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data<VecDeriv>& dOut, const Data<InVecDeriv>& dIn)
{
    helper::WriteOnlyAccessor< Data<VecDeriv> > out = dOut;
    helper::ReadAccessor< Data<InVecDeriv> > in = dIn;

    out.resize(in.size());

    {
        for(unsigned int i=0; i<out.size(); i++)
        {
            eq(out[i], in[i]);
        }
    }
}

template<class TIn, class TOut>
void IdentityMapping<TIn, TOut>::applyJT(const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data<InVecDeriv>& dOut, const Data<VecDeriv>& dIn)
{
    helper::WriteAccessor< Data<InVecDeriv> > out = dOut;
    helper::ReadAccessor< Data<VecDeriv> > in = dIn;

    {
        for(unsigned int i=0; i<in.size(); i++)
        {
            peq(out[i], in[i]);
        }
    }
}

template <class TIn, class TOut>
void IdentityMapping<TIn, TOut>::applyJT(const core::ConstraintParams * /*cparams*/ /* PARAMS FIRST */, Data<InMatrixDeriv>& dOut, const Data<MatrixDeriv>& dIn)
{
    InMatrixDeriv& out = *dOut.beginEdit();
    const MatrixDeriv& in = dIn.getValue();

    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();

        // Creates a constraints if the input constraint is not empty.
        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            while (colIt != colItEnd)
            {
                InDeriv data;
                eq(data, colIt.val());

                o.addCol(colIt.index(), data);

                ++colIt;
            }
        }
    }

    dOut.endEdit();
}

template <class TIn, class TOut>
void IdentityMapping<TIn, TOut>::handleTopologyChange()
{
    if ( stateTo && stateFrom && stateTo->getSize() != stateFrom->getSize()) this->init();
}

template <class TIn, class TOut>
const sofa::defaulttype::BaseMatrix* IdentityMapping<TIn, TOut>::getJ()
{
#ifdef SOFA_HAVE_EIGEN2
    getJs();
    return &eigen;
#else
    return nullptr;
#endif
}

template<int N, int M, class Real>
struct IdentityMappingMatrixHelper
{
    template <class Matrix>
    static void setMatrix(Matrix& mat)
    {
        for(int r = 0; r < N; ++r)
        {
            for(int c = 0; c < M; ++c)
            {
                mat[r][c] = (Real) 0;
            }
            if( r<M ) mat[r][r] = (Real) 1.0;
        }
    }
};


#ifdef SOFA_HAVE_EIGEN2

template <class TIn, class TOut>
const typename IdentityMapping<TIn, TOut>::js_type* IdentityMapping<TIn, TOut>::getJs()
{
	if( !eigen.compressedMatrix.nonZeros() || updateJ ) {
		updateJ = false;

		assert( this->fromModel->getSize() == this->toModel->getSize());

		const unsigned n = this->fromModel->getSize();

		// each block (input i, output j) has only its top-left
		// principal submatrix filled with identity
		
		const unsigned rows = n * NOut;
        const unsigned cols = n * NIn;

        eigen.compressedMatrix.resize( rows, cols );
		eigen.compressedMatrix.setZero();
        eigen.compressedMatrix.reserve( rows );
		
		for(unsigned i = 0 ; i < n; ++i) {

			for(unsigned r = 0; r < std::min<unsigned>(NIn, NOut); ++r) {
				const unsigned row = NOut * i + r;

				eigen.compressedMatrix.startVec( row );

				const unsigned col = NIn * i + r;
				eigen.compressedMatrix.insertBack( row, col ) = 1;
			}
			
		}
		
		eigen.compressedMatrix.finalize();
    }

	// std::cout << eigen.compressedMatrix << std::endl;
	
	js.resize( 1 );
	js[0] = &eigen;
	return &js;
}

#endif

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
