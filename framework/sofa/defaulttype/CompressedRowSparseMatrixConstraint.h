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
#ifndef SOFA_DEFAULTTYPE_COMPRESSEDROWSPARSEMATRIXCONSTRAINT_H
#define SOFA_DEFAULTTYPE_COMPRESSEDROWSPARSEMATRIXCONSTRAINT_H

#include <sofa/defaulttype/CompressedRowSparseMatrix.h>

namespace sofa
{

namespace defaulttype
{

template<class RowType, class VecDeriv, typename Real = typename VecDeriv::value_type::Real>
Real CompressedRowSparseMatrixVecDerivMult(const RowType row, const VecDeriv& vec)
{
    Real r = 0;
    for (typename RowType::first_type it = row.first, itend = row.second; it != itend; ++it)
        r += it.val() * vec[it.index()];
    return r;
}

template<class RowType, class VecDeriv>
void convertCompressedRowSparseMatrixRowToVecDeriv(const RowType row, VecDeriv& out)
{
    for (typename RowType::first_type it = row.first, itend = row.second; it != itend; ++it)
    {
        out[it.index()] += it.val();
    }
}

/// Constraint policy type, showing the types and flags to give to CompressedRowSparseMatrix
/// for its second template type. The default values correspond to the original implementation.
template<typename TBloc, typename TVecBloc = sofa::helper::vector<TBloc>,
    typename TVecIndex = sofa::helper::vector<int>, typename TVecFlag = sofa::helper::vector<bool> >
class CRSConstraintPolicy : public CRSDefaultPolicy<TBloc, TVecBloc, TVecIndex, TVecFlag>
{
public:
    static constexpr bool AutoSize = true;
    static constexpr bool AutoCompress = true;
    static constexpr bool CompressZeros = false;
    static constexpr bool ClearByZeros = false;
    static constexpr bool OrderedInsertion = true;

    static constexpr int  matrixType = 2;
};

template<typename TBloc, typename TPolicy = CRSConstraintPolicy<TBloc> >
class CompressedRowSparseMatrixConstraint : public sofa::defaulttype::CompressedRowSparseMatrix<TBloc, TPolicy>
{
public:
    typedef CompressedRowSparseMatrixConstraint<TBloc, TPolicy> Matrix;
    typedef CompressedRowSparseMatrix<TBloc, TPolicy> CRSMatrix;
    typedef typename CRSMatrix::Policy Policy;

    typedef typename Policy::VecBloc VecBloc;
    typedef typename Policy::VecIndex VecIndex;
    typedef typename Policy::VecFlag VecFlag;
    typedef typename VecIndex::value_type Index;
    typedef typename VecBloc::value_type Bloc;

    typedef typename CRSMatrix::Bloc Data;
    typedef typename CRSMatrix::Range Range;
    typedef typename CRSMatrix::traits traits;
    typedef typename CRSMatrix::Real Real;
    typedef typename CRSMatrix::Index KeyType;
    typedef typename CRSMatrix::IndexedBloc IndexedBloc;

public:
    CompressedRowSparseMatrixConstraint()
        : CRSMatrix()
    {
    }

    CompressedRowSparseMatrixConstraint(Index nbRow, Index nbCol)
        : CRSMatrix(nbRow, nbCol)
    {
    }

    bool empty() const
    {
        return this->rowIndex.empty();
    }

    class RowType;
    class RowConstIterator;
    /// Row Sparse Matrix columns constant Iterator to match with constraint matrix manipulation
    class ColConstIterator : std::iterator<std::bidirectional_iterator_tag, Index>
    {
    public:
        friend class RowConstIterator;
    protected:

        ColConstIterator(const Index _rowIt, int _internal, const CompressedRowSparseMatrixConstraint* _matrix)
            : m_rowIt(_rowIt)
            , m_internal(_internal)
            , m_matrix(_matrix)
        {}

    public:

        ColConstIterator(const ColConstIterator& it2)
            : m_rowIt(it2.m_rowIt)
            , m_internal(it2.m_internal)
            , m_matrix(it2.m_matrix)
        {}

        void operator=(const ColConstIterator& it2)
        {
            m_rowIt = it2.m_rowIt;
            m_internal = it2.m_internal;
            m_matrix = it2.m_matrix;
        }

        Index row() const
        {
            return m_matrix->rowIndex[m_rowIt];
        }

        /// @return the constraint value
        const TBloc &val() const
        {
            return m_matrix->colsValue[m_internal];
        }

        /// @return the DOF index the constraint is applied on
        Index index() const
        {
            return m_matrix->colsIndex[m_internal];
        }

        const Index getInternal() const
        {
            return m_internal;
        }

        void operator++() // prefix
        {
            m_internal++;
        }

        void operator++(int) // postfix
        {
            m_internal++;
        }

        void operator--() // prefix
        {
            m_internal--;
        }

        void operator--(int) // postfix
        {
            m_internal--;
        }

        bool operator==(const ColConstIterator& it2) const
        {
            return (m_internal == it2.m_internal);
        }

        bool operator!=(const ColConstIterator& it2) const
        {
            return (m_internal != it2.m_internal);
        }

        bool operator<(const ColConstIterator& it2) const
        {
            return m_internal < it2.m_internal;
        }

        bool operator>(const ColConstIterator& it2) const
        {
            return m_internal > it2.m_internal;
        }

    private :

        const Index m_rowIt;
        Index m_internal;
        const CompressedRowSparseMatrixConstraint* m_matrix;
    };

    class RowConstIterator : public std::iterator<std::bidirectional_iterator_tag, Index>
    {
    public:

        friend class CompressedRowSparseMatrixConstraint;

    protected:

        RowConstIterator(const CompressedRowSparseMatrixConstraint* _matrix, int _m_internal)
            : m_internal(_m_internal)
            , m_matrix(_matrix)
        {}

    public:

        RowConstIterator(const RowConstIterator& it2)
            : m_internal(it2.m_internal)
            , m_matrix(it2.m_matrix)
        {}

        RowConstIterator()
        {}

        void operator=(const RowConstIterator& it2)
        {
            m_matrix = it2.m_matrix;
            m_internal = it2.m_internal;
        }

        Index index() const
        {
            return m_matrix->rowIndex[m_internal];
        }

        ColConstIterator begin() const
        {
            Range r = m_matrix->getRowRange(m_internal);
            return ColConstIterator(m_internal, r.begin(), m_matrix);
        }

        ColConstIterator end() const
        {
            Range r = m_matrix->getRowRange(m_internal);
            return ColConstIterator(m_internal, r.end(), m_matrix);
        }

        RowType row() const
        {
            Range r = m_matrix->getRowRange(m_internal);
            return RowType(ColConstIterator(m_internal, r.begin(), m_matrix),
                           ColConstIterator(m_internal, r.end(), m_matrix));
        }

        void operator++() // prefix
        {
            m_internal++;
        }

        void operator++(int) // postfix
        {
            m_internal++;
        }

        void operator--() // prefix
        {
            m_internal--;
        }

        void operator--(int) // postfix
        {
            m_internal--;
        }

        bool operator==(const RowConstIterator& it2) const
        {
            return m_internal == it2.m_internal;
        }

        bool operator!=(const RowConstIterator& it2) const
        {
            return !(m_internal == it2.m_internal);
        }

        bool operator<(const RowConstIterator& it2) const
        {
            return m_internal < it2.m_internal;
        }

        bool operator>(const RowConstIterator& it2) const
        {
            return m_internal > it2.m_internal;
        }

        template <class VecDeriv, typename Real>
        Real operator*(const VecDeriv& v) const
        {
            return CompressedRowSparseMatrixVecDerivMult(row(), v);
        }

    private:

        Index m_internal;
        const CompressedRowSparseMatrixConstraint* m_matrix;
    };

    /// Get the iterator corresponding to the beginning of the rows of blocks
    RowConstIterator begin() const
    {
        SOFA_IF_CONSTEXPR (Policy::AutoCompress) const_cast<Matrix*>(this)->compress();  /// \warning this violates the const-ness of the method !
        return RowConstIterator(this, 0);
    }

    /// Get the iterator corresponding to the end of the rows of blocks
    RowConstIterator end() const
    {
        SOFA_IF_CONSTEXPR (Policy::AutoCompress) const_cast<Matrix*>(this)->compress();  /// \warning this violates the const-ness of the method !
        return RowConstIterator(this, this->rowIndex.size());
    }

    /// Get the iterator corresponding to the beginning of the rows of blocks
    RowConstIterator cbegin() const
    {
        SOFA_IF_CONSTEXPR(Policy::AutoCompress) const_cast<Matrix*>(this)->compress();  /// \warning this violates the const-ness of the method !
        return RowConstIterator(this, 0);
    }

    /// Get the iterator corresponding to the end of the rows of blocks
    RowConstIterator cend() const
    {
        SOFA_IF_CONSTEXPR(Policy::AutoCompress) const_cast<Matrix*>(this)->compress();  /// \warning this violates the const-ness of the method !
        return RowConstIterator(this, this->rowIndex.size());
    }

    class RowWriteAccessor
    {
    public:

        friend class CompressedRowSparseMatrixConstraint;

    protected:

        RowWriteAccessor(CompressedRowSparseMatrixConstraint* _matrix, int _rowIndex)
            : m_rowIndex(_rowIndex)
            , m_matrix(_matrix)
        {}

    public:

        void addCol(Index id, const Bloc& value)
        {
            SOFA_IF_CONSTEXPR (Policy::LogTrace) m_matrix->logCall(FnEnum::addCol, m_rowIndex, id, value);
            *m_matrix->wbloc(m_rowIndex, id, true) += value;
        }

        void setCol(Index id, const Bloc& value)
        {
            SOFA_IF_CONSTEXPR (Policy::LogTrace) m_matrix->logCall(FnEnum::setCol, m_rowIndex, id, value);
            *m_matrix->wbloc(m_rowIndex, id, true) = value;
        }

        bool operator==(const RowWriteAccessor& it2) const
        {
            return m_rowIndex == it2.m_rowIndex;
        }

        bool operator!=(const RowWriteAccessor& it2) const
        {
            return !(m_rowIndex == it2.m_rowIndex);
        }

    private:
        int m_rowIndex;
        CompressedRowSparseMatrixConstraint* m_matrix;
    };

    typedef RowWriteAccessor RowIterator; /// Definition for MapMapSparseMatrix and CompressedRowSparseMatrixConstraint compatibility

    class RowType : public std::pair<ColConstIterator, ColConstIterator>
    {
        typedef std::pair<ColConstIterator, ColConstIterator> Inherit;
    public:
        RowType() : Inherit(0,0) {}
        RowType(ColConstIterator begin, ColConstIterator end) : Inherit(begin,end) {}
        ColConstIterator begin() const { return this->first; }
        ColConstIterator end() const { return this->second; }
        void setBegin(ColConstIterator i) { this->first = i; }
        void setEnd(ColConstIterator i) { this->second = i; }
        bool empty() const { return begin() == end(); }
        Index size() const { return end().getInternal() - begin().getInternal(); }
        void operator++() { ++this->first; }
        void operator++(int) { ++this->first; }
    };

    /// Get the number of constraint
    size_t size() const
    {
        SOFA_IF_CONSTEXPR(Policy::AutoCompress) const_cast<Matrix*>(this)->compress();  /// \warning this violates the const-ness of the method !
        return this->getRowIndex().size();
    }

    /// @return Constant Iterator on specified row
    /// @param lIndex row index
    /// If lIndex row doesn't exist, returns end iterator
    RowConstIterator readLine(Index lIndex) const
    {
        SOFA_IF_CONSTEXPR (Policy::AutoCompress) const_cast<Matrix*>(this)->compress();  /// \warning this violates the const-ness of the method !
        Index rowId = (this->nBlocRow == 0) ? 0 : lIndex * this->rowIndex.size() / this->nBlocRow;
        if (this->sortedFind(this->rowIndex, lIndex, rowId))
        {
            return RowConstIterator(this, rowId);
        }
        else
        {
            return RowConstIterator(this, this->rowIndex.size());
        }
    }

    /// @return Iterator on specified row
    /// @param lIndex row index
    RowWriteAccessor writeLine(Index lIndex)
    {
        return RowWriteAccessor(this, lIndex);
    }

    /// @param lIndex row Index
    /// @param row constraint itself
    /// If lindex already exists, overwrite existing constraint
    void writeLine(Index lIndex, RowType row)
    {
        if (readLine(lIndex) != this->end()) this->clearRowBloc(lIndex);

        RowWriteAccessor it(this, lIndex);
        ColConstIterator colIt = row.first;
        ColConstIterator colItEnd = row.second;

        while (colIt != colItEnd)
        {
            it.setCol(colIt.index(), colIt.val());
            ++colIt;
        }
    }

    /// @param lIndex row Index
    /// @param row constraint itself
    /// If lindex doesn't exists, creates the row
    void addLine(Index lIndex, RowType row)
    {
        RowWriteAccessor it(this, lIndex);

        ColConstIterator colIt = row.first;
        ColConstIterator colItEnd = row.second;

        while (colIt != colItEnd)
        {
            it.addCol(colIt.index(), colIt.val());
            ++colIt;
        }
    }

    template< class VecDeriv>
    void multTransposeBaseVector(VecDeriv& res, const sofa::defaulttype::BaseVector* lambda ) const
    {
        typedef typename VecDeriv::value_type Deriv;

        static_assert(std::is_same<Deriv, TBloc>::value, "res must be contain same type as CompressedRowSparseMatrix type");

        for (auto rowIt = begin(), rowItEnd = end(); rowIt != rowItEnd; ++rowIt)
        {
            const SReal f = lambda->element(rowIt.index());
            for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
            {
                res[colIt.index()] += colIt.val() * f;
            }
        }
    }

    /// write to an output stream
    inline friend std::ostream& operator << ( std::ostream& out, const CompressedRowSparseMatrixConstraint<TBloc, Policy>& sc)
    {
        for (RowConstIterator rowIt = sc.begin(); rowIt !=  sc.end(); ++rowIt)
        {
            out << "Constraint ID : ";
            out << rowIt.index();
            for (ColConstIterator colIt = rowIt.begin(); colIt !=  rowIt.end(); ++colIt)
            {
                out << "  dof ID : " << colIt.index() << "  value : " << colIt.val() << "  ";
            }
            out << "\n";
        }

        return out;
    }

    /// read from an input stream
    inline friend std::istream& operator >> ( std::istream& in, CompressedRowSparseMatrixConstraint<TBloc, Policy>& sc)
    {
        sc.clear();

        unsigned int c_id;
        unsigned int c_number;
        unsigned int c_dofIndex;
        TBloc c_value;

        while (!(in.rdstate() & std::istream::eofbit))
        {
            in >> c_id;
            in >> c_number;

            RowIterator c_it = sc.writeLine(c_id);

            for (unsigned int i = 0; i < c_number; i++)
            {
                in >> c_dofIndex;
                in >> c_value;
                c_it.addCol(c_dofIndex, c_value);
            }
        }

        sc.compress();
        return in;
    }

    static const char* Name()
    {
        static std::string name = std::string("CompressedRowSparseMatrixConstraint") + std::string(traits::Name());
        return name.c_str();
    }
};

} // namespace defaulttype

} // namespace sofa

#endif
