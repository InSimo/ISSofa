/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_LINEARSOLVER_BLOCMATRIXWRITER_H
#define SOFA_COMPONENT_LINEARSOLVER_BLOCMATRIXWRITER_H

#include <sofa/SofaBase.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/MultiMatrixAccessor.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/CompressedRowSparseMatrix.h>

namespace sofa
{

namespace component
{

namespace linearsolver
{

/// This class is a helper to efficiently implement addKToMatrix in forcefields (and is could later be used for mapping, etc.)
/// See TriangularFEMForceFieldOptim for an example.
template<typename TBloc>
class BlocMatrixWriter
{
public:
    typedef TBloc Bloc;
    typedef sofa::defaulttype::matrix_bloc_traits<Bloc> traits;
    typedef typename traits::Real Real;
    enum { NL = traits::NL };
    enum { NC = traits::NC };

    typedef Bloc MatBloc;

    typedef sofa::defaulttype::Vec<NL,Real> DBloc;

    template<class MReal>
    class BlocCRSMatrixWriter
    {
        sofa::defaulttype::CompressedRowSparseMatrix<defaulttype::Mat<NL,NC,MReal> >* m;
        const unsigned int boffsetL, boffsetC;
    public:
        BlocCRSMatrixWriter(sofa::defaulttype::CompressedRowSparseMatrix<defaulttype::Mat<NL,NC,MReal> >* m, unsigned int boffsetL, unsigned int boffsetC) : m(m), boffsetL(boffsetL), boffsetC(boffsetC) {}
        void add(unsigned int bi, unsigned int bj, const MatBloc& b)
        {
            m->add(bi, bj, b, boffsetL, boffsetC);
        }
        void add(unsigned int bi, unsigned int bj, int& rowId, int& colId, const MatBloc& b)
        {
            m->add(bi, bj, rowId, colId, b, boffsetL, boffsetC);
        }
        void addDBloc(unsigned int bi, unsigned int bj, const DBloc& b)
        {
            m->addDBloc(bi, bj, b, boffsetL, boffsetC);
        }
        void addDValue(unsigned int bi, unsigned int bj, const Real b)
        {
            m->addDValue(bi, bj, b, boffsetL, boffsetC);
        }
         void addDValue(unsigned int bi, unsigned int bj, int& rowId, int& colId, const Real b)
        {
            m->addDValue(bi, bj, rowId, colId, b, boffsetL, boffsetC);
        }
        void addDiag(unsigned int bi, const MatBloc& b)
        {
            m->addDiag(bi, b, boffsetL, boffsetC);
        }
        void addDiag(unsigned int bi, int& rowId, int& colId, const MatBloc &b)
        {
            m->addDiag(bi, rowId, colId, b, boffsetL, boffsetC);
        }
        void addDiagDBloc(unsigned int bi, const DBloc& b)
        {
            m->addDiagDBloc(bi, b, boffsetL, boffsetC);
        }
        void addDiagDValue(unsigned int bi, const Real b)
        {
            m->addDiagDValue(bi, b, boffsetL, boffsetC);
        }
        void addDiagDValue(unsigned int bi, int& rowId, int& colId, const Real b)
        {
            m->addDiagDValue(bi, rowId, colId, b, boffsetL, boffsetC);
        }
        void addSym(unsigned int bi, unsigned int bj, const MatBloc& b)
        {
            m->addSym(bi, bj, b, boffsetL, boffsetC);
        }
        void addSym(unsigned int bi, unsigned int bj, int& rowId, int& colId, int& rowIdT, int& colIdT, const MatBloc &b)
        {
            m->addSym(bi, bj, rowId, colId, rowIdT, colIdT, b, boffsetL, boffsetC);
        }
        void addSymDBloc(unsigned int bi, unsigned int bj, const DBloc& b)
        {
            m->addSymDBloc(bi, bj, b, boffsetL, boffsetC);
        }
        void addSymDValue(unsigned int bi, unsigned int bj, const Real b)
        {
            m->addSymDValue(bi, bj, b, boffsetL, boffsetC);
        }
        void addSymDValue(unsigned int bi, unsigned int bj, int& rowId, int& colId, int& rowIdT, int& colIdT, Real b)
        {
            m->addSymDValue(bi, bj, rowId, colId, rowIdT, colIdT, b, boffsetL, boffsetC);
        }
    };

    template<class MReal>
    class CRSMatrixWriter
    {
        sofa::defaulttype::CompressedRowSparseMatrix<MReal>* m;
        const unsigned int offsetL, offsetC;
    public:
        CRSMatrixWriter(sofa::defaulttype::CompressedRowSparseMatrix<MReal>* m, unsigned int offsetL, unsigned int offsetC) : m(m), offsetL(offsetL), offsetC(offsetC) {}
        void add(unsigned int bi, unsigned int bj, const MatBloc& b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
                for (unsigned int j=0; j<NC; ++j)
                    m->addBloc(i0+i,j0+j,(MReal)b[i][j]);
        }
        void add(unsigned int bi, unsigned int bj, int& rowId, int& colId, const MatBloc& b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
                for (unsigned int j=0; j<NC; ++j)
                    m->addBloc(i0+i,j0+j,rowId,colId,(MReal)b[i][j]);
        }
        void addDBloc(unsigned int bi, unsigned int bj, const DBloc& b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
                m->addBloc(i0+i,j0+i,(MReal)b[i]);
        }
        void addDValue(unsigned int bi, unsigned int bj, const Real b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
                m->addBloc(i0+i,j0+i,(MReal)b);
        }
        void addDValue(unsigned int bi, unsigned int bj, int& rowId, int& colId, const Real b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
                m->addBloc(i0+i,j0+i,rowId,colId,(MReal)b);
        }
        void addDiag(unsigned int bi, const MatBloc& b)
        {
            add(bi, bi, b);
        }
        void addDiag(unsigned int bi, int& rowId, int& colId, const MatBloc &b)
        {
            add(bi, bi, rowId, colId, b);
        }
        void addDiagDBloc(unsigned int bi, const DBloc& b)
        {
            addDBloc(bi, bi, b);
        }
        void addDiagDValue(unsigned int bi, const Real b)
        {
            addDValue(bi, bi, b);
        }
        void addDiagDValue(unsigned int bi, int& rowId, int& colId, const Real b)
        {
            addDValue(bi, bi, rowId, colId, b);
        }
        void addSym(unsigned int bi, unsigned int bj, const MatBloc& b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
                for (unsigned int j=0; j<NC; ++j)
                {
                    m->addBloc(i0+i,j0+j,(MReal)b[i][j]);
                    m->addBloc(j0+j,i0+i,(MReal)b[i][j]);
                }
        }
        void addSym(unsigned int bi, unsigned int bj, int& rowId, int& colId, int& rowIdT, int& colIdT, const MatBloc &b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
                for (unsigned int j=0; j<NC; ++j)
                {
                    m->addBloc(i0+i,j0+j,rowId,colId,(MReal)b[i][j]);
                    m->addBloc(j0+j,i0+i,rowIdT,colIdT,(MReal)b[i][j]);
                }
        }
        void addSymDBloc(unsigned int bi, unsigned int bj, const DBloc& b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
            {
                m->addBloc(i0+i,j0+i,(MReal)b[i]);
                m->addBloc(j0+i,i0+i,(MReal)b[i]);
            }
        }
        void addSymDValue(unsigned int bi, unsigned int bj, const Real b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
            {
                m->addBloc(i0+i,j0+i,(MReal)b);
                m->addBloc(j0+i,i0+i,(MReal)b);
            }
        }
        void addSymDValue(unsigned int bi, unsigned int bj, int& rowId, int& colId, int& rowIdT, int& colIdT, Real b)
        {
            unsigned int i0 = offsetL + bi*NL;
            unsigned int j0 = offsetC + bj*NC;
            for (unsigned int i=0; i<NL; ++i)
            {
                m->addBloc(i0+i,j0+i,rowId,colId,(MReal)b);
                m->addBloc(j0+i,i0+i,rowIdT,colIdT,(MReal)b);
            }
        }
    };


    template<class Dispatcher>
    void apply(Dispatcher& dispatch, sofa::defaulttype::BaseMatrix *m, unsigned int offsetL, unsigned int offsetC)
    {
        if ((offsetL % NL) == 0 && (offsetC % NC) == 0 && m->getBlockRows() == NL && m->getBlockCols() == NC)
        {
            unsigned int boffsetL = offsetL / NL;
            unsigned int boffsetC = offsetC / NC;
            if (sofa::defaulttype::CompressedRowSparseMatrix<defaulttype::Mat<NL,NC,double> > * mat = sofa::defaulttype::CompressedRowSparseMatrix<defaulttype::Mat<NL,NC,double> >::DynamicCast(m))
            {
                dispatch(BlocCRSMatrixWriter<double>(mat, boffsetL, boffsetC));
            }
            else if (sofa::defaulttype::CompressedRowSparseMatrix<defaulttype::Mat<NL,NC,float> > * mat = sofa::defaulttype::CompressedRowSparseMatrix<defaulttype::Mat<NL,NC,float> >::DynamicCast(m))
            {
                dispatch(BlocCRSMatrixWriter<float>(mat, boffsetL, boffsetC));
            }
            else
            {
                assert(false);
            }
        }
        else
        {
            if (sofa::defaulttype::CompressedRowSparseMatrix<double> * mat = sofa::defaulttype::CompressedRowSparseMatrix<double>::DynamicCast(m))
            {
                dispatch(CRSMatrixWriter<double>(mat, offsetL, offsetC));
            }
            else if (sofa::defaulttype::CompressedRowSparseMatrix<float> * mat = sofa::defaulttype::CompressedRowSparseMatrix<float>::DynamicCast(m))
            {
                dispatch(CRSMatrixWriter<float>(mat, offsetL, offsetC));
            }
            else
            {
                assert(false);
            }
        }
    }


    template<class FF>
    struct DispatcherForceField_addKToMatrix
    {
        FF* main;
        const sofa::core::MechanicalParams* mparams;
        DispatcherForceField_addKToMatrix(FF* main, const sofa::core::MechanicalParams* mparams) : main(main), mparams(mparams) {}
        template <class MatrixWriter>
        void operator()(const MatrixWriter& m)
        {
            main->addKToMatrixT(mparams, m);
        }
    };

    template<class FF>
    void addKToMatrix(FF* main, const sofa::core::MechanicalParams* mparams, sofa::core::behavior::MultiMatrixAccessor::MatrixRef r)
    {
        if (!r) return;
        DispatcherForceField_addKToMatrix<FF> dispatch(main, mparams);
        apply(dispatch, r.matrix, r.offset, r.offset);
    }

    template<class FF>
    struct DispatcherForceField_addMToMatrix
    {
        FF* main;
        const sofa::core::MechanicalParams* mparams;
        DispatcherForceField_addMToMatrix(FF* main, const sofa::core::MechanicalParams* mparams) : main(main), mparams(mparams) {}
        template <class MatrixWriter>
        void operator()(const MatrixWriter& m)
        {
            main->addMToMatrixT(mparams, m);
        }
    };

    template<class FF>
    void addMToMatrix(FF* main, const sofa::core::MechanicalParams* mparams, sofa::core::behavior::MultiMatrixAccessor::MatrixRef r)
    {
        if (!r) return;
        DispatcherForceField_addMToMatrix<FF> dispatch(main, mparams);
        apply(dispatch, r.matrix, r.offset, r.offset);
    }

    template<class FF>
    struct DispatcherForceField_addMBKToMatrix
    {
        FF* main;
        const sofa::core::MechanicalParams* mparams;
        DispatcherForceField_addMBKToMatrix(FF* main, const sofa::core::MechanicalParams* mparams) : main(main), mparams(mparams) {}
        template <class MatrixWriter>
        void operator()(const MatrixWriter& m)
        {
            main->addMBKToMatrixT(mparams, m);
        }
    };

    template<class FF>
    void addMBKToMatrix(FF* main, const sofa::core::MechanicalParams* mparams, sofa::core::behavior::MultiMatrixAccessor::MatrixRef r)
    {
        if (!r) return;
        DispatcherForceField_addMBKToMatrix<FF> dispatch(main, mparams);
        apply(dispatch, r.matrix, r.offset, r.offset);
    }
};

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_LINEARSOLVER_BLOCMATRIXWRITER_H
