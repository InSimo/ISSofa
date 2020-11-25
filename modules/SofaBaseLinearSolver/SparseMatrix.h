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
#ifndef SOFA_COMPONENT_LINEARSOLVER_SPARSEMATRIX_H
#define SOFA_COMPONENT_LINEARSOLVER_SPARSEMATRIX_H

#include <sofa/defaulttype/BaseMatrix.h>
#include <sofa/defaulttype/FullVector.h>
#include <sofa/defaulttype/MatrixExpr.h>

#include <map>

namespace sofa
{

namespace component
{

namespace linearsolver
{

//#define SPARSEMATRIX_CHECK
//#define SPARSEMATRIX_VERBOSE

/** This is basically a map of map of T, wrapped in a defaulttype::BaseMatrix interface.
 The const access methods avoid creating the entries when they do not exist.
*/
template<typename T>
class SparseMatrix : public defaulttype::BaseMatrix
{
public:
    SOFA_MATRIX_CLASS_UNIQUE((SparseMatrix<T>),((defaulttype::BaseMatrix)));

    typedef T Real;
    typedef std::map<Index,Real> Line;
    typedef std::map<Index,Line> Data;
    typedef typename Line::iterator LElementIterator;
    typedef typename Line::const_iterator LElementConstIterator;
    typedef typename Data::iterator LineIterator;
    typedef typename Data::const_iterator LineConstIterator;

    typedef SparseMatrix<T> Expr;
    typedef SparseMatrix<double> matrix_type;
    enum { category = MATRIX_SPARSE };
    enum { operand = 1 };

protected:
    Data data;
    Index nRow,nCol;

public:

    SparseMatrix()
        : nRow(0), nCol(0)
    {
    }

    SparseMatrix(Index nbRow, Index nbCol)
        : nRow(nbRow), nCol(nbCol)
    {
    }

    LineIterator begin() { return data.begin(); }
    LineIterator end()   { return data.end();   }
    LineConstIterator begin() const { return data.begin(); }
    LineConstIterator end()   const { return data.end();   }

    LineIterator find(Index i) { return data.find(i); }
    LineConstIterator find(Index i) const { return data.find(i); }

    Line& operator[](Index i)
    {
        return data[i];
    }

    const Line& operator[](Index i) const
    {
        static const Line empty;
        LineConstIterator it = data.find(i);
        if (it==data.end())
            return empty;
        else
            return it->second;
    }

    void resize(Index nbRow, Index nbCol)
    {

#ifdef SPARSEMATRIX_VERBOSE
        if (nbRow != rowSize() || nbCol != colSize())
            std::cout << /* this->Name()  <<  */": resize("<<nbRow<<","<<nbCol<<")"<<std::endl;
#endif
        data.clear();
        nRow = nbRow;
        nCol = nbCol;
    }

    Index rowSize(void) const
    {
        return nRow;
    }

    Index colSize(void) const
    {
        return nCol;
    }

    SReal element(Index i, Index j) const
    {
#ifdef SPARSEMATRIX_CHECK
        if (i >= rowSize() || j >= colSize())
        {
            std::cerr << "ERROR: invalid read access to element ("<<i<<","<<j<<") in "<</* this->Name() <<*/" of size ("<<rowSize()<<","<<colSize()<<")"<<std::endl;
            return 0.0;
        }
#endif
        LineConstIterator it = data.find(i);
        if (it==data.end())
            return 0.0;
        LElementConstIterator ite = it->second.find(j);
        if (ite == it->second.end())
            return 0.0;
        return (SReal)ite->second;
    }

    void set(Index i, Index j, double v)
    {
#ifdef SPARSEMATRIX_VERBOSE
        std::cout << /* this->Name()  <<  */"("<<rowSize()<<","<<colSize()<<"): element("<<i<<","<<j<<") = "<<v<<std::endl;
#endif
#ifdef SPARSEMATRIX_CHECK
        if (i >= rowSize() || j >= colSize())
        {
            std::cerr << "ERROR: invalid write access to element ("<<i<<","<<j<<") in "<</* this->Name() <<*/" of size ("<<rowSize()<<","<<colSize()<<")"<<std::endl;
            return;
        }
#endif
        data[i][j] = (Real)v;
    }

    void add(Index i, Index j, double v)
    {
#ifdef SPARSEMATRIX_VERBOSE
        std::cout << /* this->Name()  <<  */"("<<rowSize()<<","<<colSize()<<"): element("<<i<<","<<j<<") += "<<v<<std::endl;
#endif
#ifdef SPARSEMATRIX_CHECK
        if (i >= rowSize() || j >= colSize())
        {
            std::cerr << "ERROR: invalid write access to element ("<<i<<","<<j<<") in "<</* this->Name() <<*/" of size ("<<rowSize()<<","<<colSize()<<")"<<std::endl;
            return;
        }
#endif
        data[i][j] += (Real)v;
    }

    void clear(Index i, Index j)
    {
#ifdef SPARSEMATRIX_VERBOSE
        std::cout << /* this->Name()  <<  */"("<<rowSize()<<","<<colSize()<<"): element("<<i<<","<<j<<") = 0"<<std::endl;
#endif
#ifdef SPARSEMATRIX_CHECK
        if (i >= rowSize() || j >= colSize())
        {
            std::cerr << "ERROR: invalid write access to element ("<<i<<","<<j<<") in "<</* this->Name() <<*/" of size ("<<rowSize()<<","<<colSize()<<")"<<std::endl;
            return;
        }
#endif
        LineIterator it = data.find(i);
        if (it==data.end())
            return;
        LElementIterator ite = it->second.find(j);
        if (ite == it->second.end())
            return;
        it->second.erase(ite);
        if (it->second.empty())
            data.erase(it);
    }

    void clearRow(Index i)
    {
#ifdef SPARSEMATRIX_VERBOSE
        std::cout << /* this->Name()  <<  */"("<<rowSize()<<","<<colSize()<<"): row("<<i<<") = 0"<<std::endl;
#endif
#ifdef SPARSEMATRIX_CHECK
        if (i >= rowSize())
        {
            std::cerr << "ERROR: invalid write access to row "<<i<<" in "<</* this->Name() <<*/" of size ("<<rowSize()<<","<<colSize()<<")"<<std::endl;
            return;
        }
#endif
        LineIterator it = data.find(i);
        if (it==data.end())
            return;
        data.erase(it);
    }

    void clearCol(Index j)
    {
#ifdef SPARSEMATRIX_VERBOSE
        std::cout << /* this->Name()  <<  */"("<<rowSize()<<","<<colSize()<<"): col("<<j<<") = 0"<<std::endl;
#endif
#ifdef SPARSEMATRIX_CHECK
        if (j >= colSize())
        {
            std::cerr << "ERROR: invalid write access to column "<<j<<" in "<</* this->Name() <<*/" of size ("<<rowSize()<<","<<colSize()<<")"<<std::endl;
            return;
        }
#endif
        for(LineIterator it=data.begin(),itend=data.end(); it!=itend; ++it)
        {
            LElementIterator ite = it->second.find(j);
            if (ite != it->second.end())
                it->second.erase(ite);
        }
    }

    void clearRowCol(Index i)
    {
#ifdef SPARSEMATRIX_VERBOSE
        std::cout << /* this->Name()  <<  */"("<<rowSize()<<","<<colSize()<<"): row("<<i<<") = 0 and col("<<i<<") = 0"<<std::endl;
#endif
#ifdef SPARSEMATRIX_CHECK
        if (i >= rowSize() || i >= colSize())
        {
            std::cerr << "ERROR: invalid write access to row and column "<<i<<" in "<</* this->Name() <<*/" of size ("<<rowSize()<<","<<colSize()<<")"<<std::endl;
            return;
        }
#endif
        clearRow(i);
        clearCol(i);
    }

    void clear() { data.clear(); }

    template<class Real2>
    void mul(sofa::defaulttype::FullVector<Real2>& res, const sofa::defaulttype::FullVector<Real2>& v) const
    {
        res.resize(rowSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real2 r = 0;
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                r += (Real2)ite->second * v[ite->first];
            res[itl->first] = r;
        }
    }

    template<class Real2>
    void addMulTranspose(sofa::defaulttype::FullVector<Real2>& res, const sofa::defaulttype::FullVector<Real2>& v) const
    {
        res.resize(colSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real2 vi = v[itl->first];
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                res[ite->first] += (Real2)ite->second * vi;
        }
    }

    template<class Real2>
    void mul(sofa::defaulttype::FullVector<Real2>& res, const defaulttype::BaseVector* v) const
    {
        res.resize(rowSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real2 r = 0;
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                r += (Real2)ite->second * (Real2)v->element(ite->first);
            res[itl->first] = r;
        }
    }

    template<class Real2>
    void addMulTranspose(sofa::defaulttype::FullVector<Real2>& res, const defaulttype::BaseVector* v) const
    {
        res.resize(colSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real2 vi = v->element(itl->first);
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                res[ite->first] += (Real2)ite->second * vi;
        }
    }

    template<class Real2>
    void mul(defaulttype::BaseVector* res, const sofa::defaulttype::FullVector<Real2>& v) const
    {
        res->resize(rowSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real2 r = 0;
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                r += (Real2)ite->second * v[ite->first];
            res->set(itl->first, r);
        }
    }

    template<class Real2>
    void addMulTranspose(defaulttype::BaseVector* res, const sofa::defaulttype::FullVector<Real2>& v) const
    {
        res->resize(colSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real2 vi = v[itl->first];
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                res->add(ite->first, ite->second * vi);
        }
    }

    void mul(defaulttype::BaseVector* res, const defaulttype::BaseVector* v) const
    {
        res->resize(rowSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real r = 0;
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                r += ite->second * (Real)v->element(ite->first);
            res->set(itl->first, r);
        }
    }

    void addMulTranspose(defaulttype::BaseVector* res, const defaulttype::BaseVector* v) const
    {
        res->resize(colSize());
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            Real vi = (Real)v->element(itl->first);
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
                res->add(ite->first, ite->second * vi);
        }
    }

    template<class Real2>
    sofa::defaulttype::FullVector<Real2> operator*(const sofa::defaulttype::FullVector<Real2>& v) const
    {
        sofa::defaulttype::FullVector<Real2> res;
        mul(res,v);
        return res;
    }
    /*
        template<class Real2>
        void mul(SparseMatrix<T>* res, const SparseMatrix<Real2>& m) const
        {
            res->resize(rowSize(), m.colSize());
            for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
            {
    	    const Index this_line = itl->first;
                for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
    	    {
    		Real v = ite->second;
    		const typename SparseMatrix<Real2>::Line& ml = m[ite->first];
    		for (typename SparseMatrix<Real2>::LElementConstIterator ite2 = ml.begin(), ite2end=ml.end(); ite2!=ite2end; ++ite2)
    		{
    		    Real2 v2 = ite2->second;
    		    const Index m_col = ite2->first;
    		    res->add(this_line, m_col, (Real)(v*v2));
    		}
    	    }
            }
        }

        template<class Real2>
        void addmul(SparseMatrix<T>* res, const SparseMatrix<Real2>& m) const
        {
            //res->resize(rowSize(), m.colSize());
            for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
            {
    	    const Index this_line = itl->first;
                for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
    	    {
    		Real v = ite->second;
    		const typename SparseMatrix<Real2>::Line& ml = m[ite->first];
    		for (typename SparseMatrix<Real2>::LElementConstIterator ite2 = ml.begin(), ite2end=ml.end(); ite2!=ite2end; ++ite2)
    		{
    		    Real2 v2 = ite2->second;
    		    const Index m_col = ite2->first;
    		    res->add(this_line, m_col, (Real)(v*v2));
    		}
    	    }
            }
        }
    */


    sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixTranspose< SparseMatrix<T> > > t() const
    {
        return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixTranspose< SparseMatrix<T> > >(sofa::defaulttype::MatrixTranspose< SparseMatrix<T> >(*this));
    }

    sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixNegative< SparseMatrix<T> > > operator-() const
    {
        return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixNegative< SparseMatrix<T> > >(sofa::defaulttype::MatrixNegative< SparseMatrix<T> >(*this));
    }

    template<class Real2>
    sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixProduct< SparseMatrix<T>, SparseMatrix<Real2> > > operator*(const SparseMatrix<Real2>& m) const
    {
        return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixProduct< SparseMatrix<T>, SparseMatrix<Real2> > >(sofa::defaulttype::MatrixProduct< SparseMatrix<T>, SparseMatrix<Real2> >(*this, m));
    }

    sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixScale< SparseMatrix<T>, double > > operator*(const double& r) const
    {
        return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixScale< SparseMatrix<T>, double > >(sofa::defaulttype::MatrixScale< SparseMatrix<T>, double >(*this, r));
    }

    // template<class Expr2>
    // sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixProduct< SparseMatrix<T>, Expr2 > > operator*(const sofa::defaulttype::MatrixExpr<Expr2>& m) const
    // {
    //     return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixProduct< SparseMatrix<T>, Expr2 > >(sofa::defaulttype::MatrixProduct< SparseMatrix<T>, Expr2 >(*this, m));
    // }

    template<class Real2>
    sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixAddition< SparseMatrix<T>, SparseMatrix<Real2> > > operator+(const SparseMatrix<Real2>& m) const
    {
        return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixAddition< SparseMatrix<T>, SparseMatrix<Real2> > >(sofa::defaulttype::MatrixAddition< SparseMatrix<T>, SparseMatrix<Real2> >(*this, m));
    }

    // template<class Expr2>
    // sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixAddition< SparseMatrix<T>, Expr2 > > operator+(const sofa::defaulttype::MatrixExpr<Expr2>& m) const
    // {
    //     return sofa::defaulttype::sofa::defaulttype::MatrixExpr< MatrixAddition< SparseMatrix<T>, Expr2 > >(sofa::defaulttype::MatrixAddition< SparseMatrix<T>, Expr2 >(*this, m));
    // }

    template<class Real2>
    sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixAddition< SparseMatrix<T>, SparseMatrix<Real2> > > operator-(const SparseMatrix<Real2>& m) const
    {
        return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixAddition< SparseMatrix<T>, SparseMatrix<Real2> > >(sofa::defaulttype::MatrixAddition< SparseMatrix<T>, SparseMatrix<Real2> >(*this, m));
    }

    // template<class Expr2>
    // sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixAddition< SparseMatrix<T>, Expr2 > > operator-(const sofa::defaulttype::MatrixExpr<Expr2>& m) const
    // {
    //     return sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixAddition< SparseMatrix<T>, Expr2 > >(sofa::defaulttype::MatrixAddition< SparseMatrix<T>, Expr2 >(*this, m));
    // }

    void swap(SparseMatrix<T>& m)
    {
        data.swap(m.data);
        Index t;
        t = nRow; nRow = m.nRow; m.nRow = t;
        t = nCol; nCol = m.nCol; m.nCol = t;
    }

    template<class M2>
    bool hasRef(const M2* m) const
    {
        return (const void*)this == (const void*)m;
    }

    std::string expr() const
    {
        return std::string(Name());
    }

    bool valid() const
    {
        return true;
    }

    template<class Dest>
    void addTo(Dest* dest) const
    {
        for (LineConstIterator itl = begin(), itlend=end(); itl!=itlend; ++itl)
        {
            const Index l = itl->first;
            for (LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
            {
                const Index c = ite->first;
                Real v = ite->second;
                dest->add(l,c,v);
            }
        }
    }

protected:

    template<class M>
    void equal(const M& m, bool add = false)
    {
        if (m.hasRef(this))
        {
            SparseMatrix<T> tmp;
            tmp.resize(m.rowSize(), m.colSize());
            m.addTo(&tmp);
            if (add)
                tmp.addTo(this);
            else
                swap(tmp);
        }
        else
        {
            if (!add)
                resize(m.rowSize(), m.colSize());
            m.addTo(this);
        }
    }

    /// this += m
    template<class M>
    inline void addEqual( const M& m )
    {
        equal( m, true );
    }

public:

    template<class Real2>
    void operator=(const SparseMatrix<Real2>& m)
    {
        if (&m == this) return;
        resize(m.rowSize(), m.colSize());
        m.addTo(this);
    }

    template<class Real2>
    void operator+=(const SparseMatrix<Real2>& m)
    {
        addEqual(m);
    }

    template<class Real2>
    void operator-=(const SparseMatrix<Real2>& m)
    {
        equal(sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixNegative< SparseMatrix<Real2> > >(sofa::defaulttype::MatrixNegative< SparseMatrix<Real2> >(m)), true);
    }

    template<class Expr2>
    void operator=(const sofa::defaulttype::MatrixExpr< Expr2 >& m)
    {
        equal(m, false);
    }

    template<class Expr2>
    void operator+=(const sofa::defaulttype::MatrixExpr< Expr2 >& m)
    {
        addEqual(m);
    }

    template<class Expr2>
    void operator-=(const sofa::defaulttype::MatrixExpr< Expr2 >& m)
    {
        addEqual(sofa::defaulttype::MatrixExpr< sofa::defaulttype::MatrixNegative< Expr2 > >(sofa::defaulttype::MatrixNegative< Expr2 >(m)));
    }

    static const char* Name();
};

template<> inline const char* SparseMatrix<double>::Name() { return "SparseMatrix"; }
template<> inline const char* SparseMatrix<float>::Name() { return "SparseMatrixf"; }

} // namespace linearsolver

} // namespace component

} // namespace sofa

namespace sofa
{

namespace defaulttype
{

template<class R1, class R2>
class MatrixProductOp<sofa::component::linearsolver::SparseMatrix<R1>, sofa::component::linearsolver::SparseMatrix<R2> >
{
public:
    typedef sofa::component::linearsolver::SparseMatrix<R1> M1;
    typedef sofa::component::linearsolver::SparseMatrix<R2> M2;
    typedef typename type_selector<(sizeof(R2)>sizeof(R1)),M1,M2>::T matrix_type;
    enum { category = matrix_type::category };

    template<class Dest>
    void operator()(const sofa::component::linearsolver::SparseMatrix<R1>& m1, const sofa::component::linearsolver::SparseMatrix<R2>& m2, Dest* d)
    {
        for (typename sofa::component::linearsolver::SparseMatrix<R1>::LineConstIterator itl = m1.begin(), itlend=m1.end(); itl!=itlend; ++itl)
        {
            const int l1 = itl->first;
            for (typename sofa::component::linearsolver::SparseMatrix<R1>::LElementConstIterator ite = itl->second.begin(), iteend=itl->second.end(); ite!=iteend; ++ite)
            {
                const int c1 = ite->first;
                R1 v = ite->second;
                const typename sofa::component::linearsolver::SparseMatrix<R2>::Line& m2l = m2[c1];
                for (typename sofa::component::linearsolver::SparseMatrix<R2>::LElementConstIterator ite2 = m2l.begin(), ite2end=m2l.end(); ite2!=ite2end; ++ite2)
                {
                    R2 v2 = ite2->second;
                    const int c2 = ite2->first;
                    d->add(l1, c2, (v*v2));
                }
            }
        }
    }
};

#ifdef SPARSEMATRIX_CHECK
#undef SPARSEMATRIX_CHECK
#endif
#ifdef SPARSEMATRIX_VERBOSE
#undef SPARSEMATRIX_VERBOSE
#endif

} // namespace defaulttype

} // namespace sofa

#endif
