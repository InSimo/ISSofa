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
#ifndef SOFA_DEFAULTTYPE_COMPRESSEDROWSPARSEMATRIXTRACELOGGER_H
#define SOFA_DEFAULTTYPE_COMPRESSEDROWSPARSEMATRIXTRACELOGGER_H

#include <cstdio>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <assert.h>
#include <sofa/defaulttype/matrix_bloc_traits.h>

namespace sofa
{

namespace defaulttype
{

enum FnEnum: char
{
    resizeBloc      = 0,
    compress        = 1,
    fullRows        = 3,
    setBloc         = 4,
    setBlocId       = 5,
    clearRowBloc    = 6,
    clearColBloc    = 7,
    clearRowColBloc = 8,
    clear           = 9,
    add             = 10,
    addId           = 11,
    addDBloc        = 12,
    addDValue       = 13,
    addDValueId     = 14,

    /// Specific method of CRSMatrixMechanical
    fullDiagonal    = 15,
    setVal          = 16,
    addVal          = 17,
    setValId        = 18,
    addValId        = 19,
    clearIndex      = 20,
    clearRow        = 21,
    clearCol        = 22,
    clearRowCol     = 23,

    /// Specific method of CRSMatrixConstraint
    addCol          = 24,
    setCol          = 25
};

template<typename TMatrix >
class CRSTraceWriter
{
public :
    enum { NL = TMatrix::NL };
    enum { NC = TMatrix::NC };

    typedef typename TMatrix::traits traits;
    typedef typename TMatrix::Bloc Bloc;
    typedef typename TMatrix::DBloc DBloc;
    typedef typename TMatrix::Real Real;
    typedef typename TMatrix::Policy Policy;

    template <class F, class First, class... Rest>
    void do_for(F f, First first, Rest... rest)
    {
        f(first);
        do_for(f, rest...);
    }

    template <class F>
    void do_for(F /*f*/) {}

    template <class... Args>
    void logCall_imp(const char fnId, Args... args)
    {
        logT(fnId, m_matrixFile);

        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            std::string msg("Function ID : ");
            msg.append(std::to_string(static_cast<int>(fnId)));
            fwrite(msg.c_str(), msg.length(), 1, m_matrixTextFile);
        }

        unsigned int count = 0;
        do_for([&](auto arg)
        {
            logT(arg, m_matrixFile);
            SOFA_IF_CONSTEXPR (Policy::PrintTrace)
            {
                std::string msg = " arg[";
                msg.append(std::to_string(count));
                msg.append("] = ");
                msg.append(ToString(arg));
                fwrite(msg.c_str(), msg.length(), 1, m_matrixTextFile);
            }
            count++;
        }, args...);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            std::string msg = "\n";
            fwrite(msg.c_str(), msg.length(), 1, m_matrixTextFile);
        }
    }

    std::string ToString (const char         c) { return std::to_string(c); }
    std::string ToString (const unsigned int i) { return std::to_string(i); }
    std::string ToString (const int          i) { return std::to_string(i); }
    std::string ToString (const double       d) { return std::to_string(d); }
    std::string ToString (const float        f) { return std::to_string(f); }

    std::string ToString (const DBloc b)
    {
        std::string bstr = "[";
        for (int i = 0; i < NL; i++)
        {
            for (int j = 0; j < NC; j++)
            {
                bstr.append(std::to_string(matrix_bloc_traits<DBloc>::v(b, i, j)));
                if (i + j != NC + NL - 2)
                {
                    if (j == NC - 1) bstr.append(",");
                    else bstr.append(" ");
                }
            }
        }
        bstr += "]";
        return bstr;
    }

    template< class TBloc, typename std::enable_if< !std::is_same<DBloc, TBloc>::value, int >::type = 0 >
    std::string ToString (const TBloc b)
    {
        std::string bstr = "[";
        for (int i = 0; i < NL; i++)
        {
            for (int j = 0; j < NC; j++)
            {
                bstr.append(std::to_string(traits::v(b, i, j)));
                if (i + j != NC + NL - 2)
                {
                    if (j == NC - 1) bstr.append(",");
                    else bstr.append(" ");
                }
            }
        }
        bstr += "]";
        return bstr;
    }

    std::size_t logT (const char         c, FILE* file) { return fwrite( &c, sizeof(char),         1, file); }
    std::size_t logT (const unsigned int i, FILE* file) { return fwrite( &i, sizeof(unsigned int), 1, file); }
    std::size_t logT (const int          i, FILE* file) { return fwrite( &i, sizeof(int),          1, file); }
    std::size_t logT (const double       d, FILE* file) { return fwrite( &d, sizeof(double),       1, file); }
    std::size_t logT (const float        f, FILE* file) { return fwrite( &f, sizeof(float),        1, file); }

    template< class TBloc, typename std::enable_if< (!std::is_same<double, TBloc>::value && !std::is_same<float, TBloc>::value), int >::type = 0 >
    std::size_t logT (const TBloc b, FILE* file) { return matrix_bloc_traits<TBloc>::logBloc(b, file); }

    void setFile(FILE* file) { m_matrixFile = file; }

    void setTextFile(const std::string fileName) { m_matrixTextFile = fopen(fileName.c_str(), "w"); }

    void endTrace() { if (m_matrixTextFile) fclose(m_matrixTextFile); }

protected :

    int m_numStep = -1;

    FILE* m_matrixFile = nullptr; /// File where matrix trace is logged.
    FILE* m_matrixTextFile = nullptr; /// File where matrix trace is logged as txt for debug purposes.
};

template<typename Bloc, typename DBloc>
struct FnArgs
{
    int i = -1, j = -1, rowId = -1, colId = -1;
    unsigned int bi = 0, bj = 0, boffsetL = 0, boffsetC = 0;
    double v = 0;
    Bloc b = Bloc();
    DBloc bdiag = DBloc();
};

template<typename TMatrix, int matrixType >
class CRSTraceReader
{
public :
    enum { NL = TMatrix::NL };
    enum { NC = TMatrix::NC };

    typedef typename TMatrix::traits traits;
    typedef typename TMatrix::Bloc Bloc;
    typedef typename TMatrix::DBloc DBloc;
    typedef typename TMatrix::Real Real;
    typedef typename TMatrix::Policy Policy;

    CRSTraceReader(TMatrix* m) : m_CRSmatrix(m) {}

    std::size_t readT(char& c, int argId = -1)
    {
        std::size_t processed = fread( &c, sizeof(char), 1, this->m_matrixFile);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            if (argId != -1 && processed != 0) m_matrixTextFile << " arg[" << argId << "] = " << c;
        }
        return processed;
    }
    std::size_t readT(unsigned int& i, int argId = -1)
    {
        std::size_t processed = fread( &i, sizeof(unsigned int), 1, this->m_matrixFile);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            if (argId != -1 && processed != 0) m_matrixTextFile << " arg[" << argId << + "] = " << i;
        }
        return processed;
    }
    std::size_t readT(int& i, int argId = -1)
    {
        std::size_t processed = fread( &i, sizeof(int), 1, this->m_matrixFile);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            if (argId != -1 && processed != 0) m_matrixTextFile << " arg[" << argId << "] = " << i;
        }
        return processed;
    }
    std::size_t readT(double& d, int argId = -1)
    {
        std::size_t processed = fread( &d, sizeof(double), 1, this->m_matrixFile);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            if (argId != -1 && processed != 0) m_matrixTextFile << " arg[" << argId << "] = " << d;
        }
        return processed;
    }
    std::size_t readT(float& f, int argId = -1)
    {
        std::size_t processed = fread( &f, sizeof(float), 1, this->m_matrixFile);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            if (argId != -1 && processed != 0) m_matrixTextFile << " arg[" << argId << "] = " << f;
        }
        return processed;
    }

    template< class TBloc, typename std::enable_if< (!std::is_same<double, TBloc>::value && !std::is_same<float, TBloc>::value), int >::type = 0 >
    std::size_t readT(TBloc& b, int argId = -1)
    {
        using traits = matrix_bloc_traits<TBloc>;
        typename traits::Real vals[traits::NL][traits::NC];
        std::size_t processed = fileRead(&(vals[0][0]),sizeof(vals),1);
        for (int l = 0; l < traits::NL; ++l)
            for (int c = 0; c < traits::NC; ++c)
                traits::vset(b,l,c,vals[l][c]);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            if (argId != -1 && processed != 0) m_matrixTextFile << " arg[" << argId << "] = " << b;
        }
        return processed;
    }

    std::size_t readFn(int& fnId)
    {
        char tempFnId;
        std::size_t processed = readT(tempFnId);
        fnId = static_cast<int>(tempFnId);
        SOFA_IF_CONSTEXPR (Policy::PrintTrace)
        {
            if (processed != 0) m_matrixTextFile << "Function ID : " << fnId;
        }
        return processed;
    }

    FnArgs<Bloc, DBloc> readArgs(int fnId)
    {
        FnArgs<Bloc, DBloc> args;
        switch (fnId)
        {
            case FnEnum::resizeBloc      : readT(args.i, 0); readT(args.j, 1); break;
            case FnEnum::compress        : break;
            case FnEnum::setBloc         : readT(args.i, 0); readT(args.j, 1); readT(args.b, 2); break;
            case FnEnum::setBlocId       : readT(args.i, 0); readT(args.j, 1); readT(args.rowId, 2); readT(args.colId, 3); readT(args.b, 4); break;
            case FnEnum::clearRowBloc    : readT(args.i, 0); break;
            case FnEnum::clearColBloc    : readT(args.i, 0); break;
            case FnEnum::clearRowColBloc : readT(args.i, 0); break;
            case FnEnum::clear           : break;
            case FnEnum::add             : readT(args.bi, 0); readT(args.bj, 1); readT(args.b, 2); readT(args.boffsetL, 3); readT(args.boffsetC, 4); break;
            case FnEnum::addId           : readT(args.bi, 0); readT(args.bj, 1); readT(args.rowId, 2); readT(args.colId, 3); readT(args.b, 4); readT(args.boffsetL, 5); readT(args.boffsetC, 6); break;
            case FnEnum::addDBloc        : readT(args.bi, 0); readT(args.bj, 1); readT(args.bdiag, 2); readT(args.boffsetL, 3); readT(args.boffsetC, 4); break;
            case FnEnum::addDValue       : readT(args.bi, 0); readT(args.bj, 1); readT(args.v, 2); readT(args.boffsetL, 3); readT(args.boffsetC, 4); break;
            case FnEnum::addDValueId     : readT(args.bi, 0); readT(args.bj, 1); readT(args.rowId, 2); readT(args.colId, 3); readT(args.v, 4);readT(args.boffsetL, 5); readT(args.boffsetC, 6); break;
            case FnEnum::fullRows        : break;
            case FnEnum::fullDiagonal    : break;
            case FnEnum::setVal          : readT(args.i, 0); readT(args.j, 1); readT(args.v, 2); break;
            case FnEnum::addVal          : readT(args.i, 0); readT(args.j, 1); readT(args.v, 2); break;
            case FnEnum::setValId        : readT(args.i, 0); readT(args.j, 1); readT(args.rowId, 2); readT(args.colId, 3); readT(args.v, 4); break;
            case FnEnum::addValId        : readT(args.i, 0); readT(args.j, 1); readT(args.rowId, 2); readT(args.colId, 3); readT(args.v, 4); break;
            case FnEnum::clearIndex      : readT(args.i, 0); readT(args.j, 1); break;
            case FnEnum::clearRow        : readT(args.i, 0); break;
            case FnEnum::clearCol        : readT(args.i, 0); break;
            case FnEnum::clearRowCol     : readT(args.i, 0); break;
            case FnEnum::addCol          : readT(args.i, 0); readT(args.j, 1); readT(args.b, 2); break;
            case FnEnum::setCol          : readT(args.i, 0); readT(args.j, 1); readT(args.b, 2); break;
            default                      : assert(false); break; /// unrocognized method id.
        }
        SOFA_IF_CONSTEXPR (Policy::PrintTrace) m_matrixTextFile << std::endl;
        return args;
    }


    void defaultcallFn(int fnId, FnArgs<Bloc, DBloc>& args)
    {
        switch (fnId)
        {
            case FnEnum::resizeBloc      : m_CRSmatrix->resizeBloc(args.i, args.j); break;
            case FnEnum::compress        : m_CRSmatrix->compress(); break;
            case FnEnum::setBloc         : m_CRSmatrix->setBloc(args.i, args.j, args.b); break;
            case FnEnum::setBlocId       : m_CRSmatrix->setBloc(args.i, args.j, args.rowId, args.colId, args.b); break;
            case FnEnum::clearRowBloc    : m_CRSmatrix->clearRowBloc(args.i); break;
            case FnEnum::clearColBloc    : m_CRSmatrix->clearColBloc(args.i); break;
            case FnEnum::clearRowColBloc : m_CRSmatrix->clearRowColBloc(args.i); break;
            case FnEnum::clear           : m_CRSmatrix->clear(); break;
            case FnEnum::fullRows        : m_CRSmatrix->fullRows(); break;
            default : assert(false); return; /// Unrocogized case
        }
    }

    template <int Type = matrixType>
    typename std::enable_if< Type == 0 >::type
    callFn(int fnId, FnArgs<Bloc, DBloc>& args)
    {
        return defaultcallFn(fnId, args);
    }

    template <int Type = matrixType>
    typename std::enable_if< Type == 1 >::type
    callFn(int fnId, FnArgs<Bloc, DBloc>& args)
    {
        switch (fnId)
        {
            case FnEnum::add             : m_CRSmatrix->add(args.bi, args.bj, args.b, args.boffsetL, args.boffsetC); break;
            case FnEnum::addId           : m_CRSmatrix->add(args.bi, args.bj, args.rowId, args.colId, args.b, args.boffsetL, args.boffsetC); break;
            case FnEnum::addDBloc        : m_CRSmatrix->addDBloc(args.bi, args.bj, args.bdiag, args.boffsetL, args.boffsetC); break;
            case FnEnum::addDValue       : m_CRSmatrix->addDValue(args.bi, args.bj, args.v, args.boffsetL, args.boffsetC); break;
            case FnEnum::addDValueId     : m_CRSmatrix->addDValue(args.bi, args.bj, args.rowId, args.colId, args.v, args.boffsetL, args.boffsetC); break;

            case FnEnum::fullDiagonal    : m_CRSmatrix->fullDiagonal();  break;
            case FnEnum::setVal          : m_CRSmatrix->set(args.i, args.j, args.v); break;
            case FnEnum::addVal          : m_CRSmatrix->add(args.i, args.j, args.v); break;
            case FnEnum::setValId        : m_CRSmatrix->set(args.i, args.j, args.rowId, args.colId, args.v); break;
            case FnEnum::addValId        : m_CRSmatrix->add(args.i, args.j, args.rowId, args.colId, args.v); break;
            case FnEnum::clearIndex      : m_CRSmatrix->clear(args.i, args.j); break;
            case FnEnum::clearRow        : m_CRSmatrix->clearRow(args.i); break;
            case FnEnum::clearCol        : m_CRSmatrix->clearCol(args.i); break;
            case FnEnum::clearRowCol     : m_CRSmatrix->clearRowCol(args.i); break;
            default : defaultcallFn(fnId, args); return;
        }
    }

    template <int Type = matrixType>
    typename std::enable_if< Type == 2 >::type
    callFn(int fnId, FnArgs<Bloc, DBloc>& args)
    {
        switch (fnId)
        {
            case FnEnum::addCol          : m_CRSmatrix->writeLine(args.i).addCol(args.j, args.b); break;
            case FnEnum::setCol          : m_CRSmatrix->writeLine(args.i).setCol(args.j, args.b); break;
            default : defaultcallFn(fnId, args); return;
        }
    }

    void setFile(FILE* file) { m_matrixFile = file; }

    void setTextFile(const std::string fileName) { m_matrixTextFile.open(fileName); }

    void endPrint() { if (m_matrixTextFile.is_open()) m_matrixTextFile.close(); }

protected :
    FILE* m_matrixFile; /// File where matrix trace is logged.
    std::ofstream m_matrixTextFile; /// File where matrix trace is logged as txt for debug purposes.
    TMatrix * m_CRSmatrix;

};

} // namespace defaulttype

} // namespace sofa

#endif
