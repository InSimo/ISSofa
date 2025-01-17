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
#ifndef SOFA_COMPONENT_ENGINE_VALUESFROMINDICES_INL
#define SOFA_COMPONENT_ENGINE_VALUESFROMINDICES_INL

#include <SofaEngine/ValuesFromIndices.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>

namespace sofa
{

namespace component
{

namespace engine
{

template <class T>
ValuesFromIndices<T>::ValuesFromIndices()
    : f_in( initData (&f_in, "in", "input values") )
    , f_indices( initData(&f_indices, "indices","Indices of the values") )
    , f_out( initData (&f_out, "out", "Output values corresponding to the indices"))
    , f_singleOut( initData (&f_singleOut, "singleOut", "Output single value selected if indices is of size 1"))
    , f_outStr( initData (&f_outStr, "outStr", "Output values corresponding to the indices, converted as a string"))
    , f_checkSize( initData (&f_checkSize, true, "checkSize", "If false, do not show error message"))
    , d_mandatorySingleOut(initData(&d_mandatorySingleOut, false, "mandatorySingleOut", "Have singleOut mandatory, display error message if no singleOut was set"))
{
    this->addAlias(&f_in, "input"); // 'in' is a reserved word in Python
}

template <class T>
ValuesFromIndices<T>::~ValuesFromIndices()
{
}

template <class T>
void ValuesFromIndices<T>::init()
{
    f_outStr.setParent(&f_out);
    addInput(&f_in);
    addInput(&f_indices);
    addOutput(&f_out);
    addOutput(&f_singleOut);
    setDirtyValue();
}

template <class T>
void ValuesFromIndices<T>::reinit()
{
    this->requestUpdate();
}

template <class T>
void ValuesFromIndices<T>::update()
{
    cleanDirty();
    helper::ReadAccessor<Data<VecValue> > in = f_in;
    helper::ReadAccessor<Data<VecIndex> > indices = f_indices;
    helper::WriteAccessor<Data<VecValue> > out = f_out;

    out.clear();
    out.reserve(indices.size());
    for (unsigned int i=0; i<indices.size(); ++i)
    {
        if ((unsigned)indices[i] < in.size())
            out.push_back(in[indices[i]]);
        else
            if (f_checkSize.getValue()) serr << "Invalid input index " << i <<": " << indices[i] << " >= " << in.size() << sendl;
    }

    if (indices.size() == 1 && indices[0] < in.size())
    {
        f_singleOut.setValue(in[indices[0]]);
    }
    else if (d_mandatorySingleOut.getValue()) //warn if no singleoutput if there: link to singleOut
    {
            serr << this->getName() << ": error, no single output was generated, potential use of deprecated information" << sendl;
    }
}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
