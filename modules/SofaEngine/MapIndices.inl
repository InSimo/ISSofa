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
#ifndef SOFA_COMPONENT_ENGINE_MAPINDICES_INL
#define SOFA_COMPONENT_ENGINE_MAPINDICES_INL

#include <SofaEngine/MapIndices.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>

namespace sofa
{

namespace component
{

namespace engine
{

template <class T>
MapIndices<T>::MapIndices()
    : f_in( initData (&f_in, "in", "input indices") )
    , f_indices( initData(&f_indices, "indices","array containing in ith cell the input index corresponding to the output index i (or reversively if transpose=true)") )
    , f_out( initData (&f_out, "out", "Output indices"))
    , f_outStr( initData (&f_outStr, "outStr", "Output indices, converted as a string"))
    , f_transpose( initData (&f_transpose, false, "transpose", "Should the transposed mapping be used ?"))
{
}

template <class T>
MapIndices<T>::~MapIndices()
{
}

template <class T>
void MapIndices<T>::init()
{
    f_outStr.setParent(&f_out);
    addInput(&f_in);
    addInput(&f_indices);
    addInput(&f_transpose);
    addOutput(&f_out);
    setDirtyValue();
}

template <class T>
void MapIndices<T>::reinit()
{
    this->requestUpdate();
}

template <class T>
inline void MapIndices<T>::apply(Value& v, const MapIndex& m)
{
    for (unsigned int i=0; i<v.size(); ++i)
        applyIndex(v[i], m);
}

template <>
inline void MapIndices<int>::apply(Value& v, const MapIndex& m)
{
    applyIndex(v, m);
}

template <>
inline void MapIndices<unsigned int>::apply(Value& v, const MapIndex& m)
{
    applyIndex(v, m);
}

template <class T>
void MapIndices<T>::update()
{
    cleanDirty();
    helper::ReadAccessor<Data<VecValue> > in = f_in;
    helper::ReadAccessor<Data<VecIndex> > indices = f_indices;
    const bool transpose = f_transpose.getValue();

    helper::WriteAccessor<Data<VecValue> > out = f_out;

    out.clear();
    out.reserve(in.size());

    std::map<Index, Index> old2new;
    for (unsigned int i=0; i<indices.size(); ++i)
    {
        if (transpose) old2new[i] = indices[i];
        else old2new[indices[i]] = i;
    }

    for (unsigned int i=0; i<in.size(); ++i)
    {
        Value v = in[i];
        apply(v, old2new);
        out.push_back(v);
    }
}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
