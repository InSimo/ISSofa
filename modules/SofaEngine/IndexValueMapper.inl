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
#ifndef INDEXVALUEMAPPER_INL_
#define INDEXVALUEMAPPER_INL_

#include "IndexValueMapper.h"

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/system/FileRepository.h>

namespace sofa
{

namespace component
{

namespace engine
{

template <class VecT>
IndexValueMapper<VecT>::IndexValueMapper()
    : f_inputValues(initData(&f_inputValues, "inputValues", "Already existing values (can be empty) "))
    , f_indices(initData(&f_indices, "indices", "Indices to map value on "))
    , f_value(initData(&f_value, "value", "Value to map indices on "))
    , f_outputValues(initData(&f_outputValues, "outputValues", "New map between indices and values"))
    , p_defaultValue(initData(&p_defaultValue, "defaultValue", "Default value for indices without any value"))
{
}

template <class VecT>
void IndexValueMapper<VecT>::init()
{
    addInput(&f_inputValues);
    addInput(&f_indices);
    addInput(&f_value);

    addOutput(&f_outputValues);

    setDirtyValue();
}

template <class VecT>
void IndexValueMapper<VecT>::reinit()
{
    update();
}

template <class VecT>
void IndexValueMapper<VecT>::update()
{
    cleanDirty();

    helper::ReadAccessor< Data< helper::vector<Value> > > inputValues = f_inputValues;
    helper::ReadAccessor< Data< helper::vector<Index> > > indices = f_indices;
    const Value& value = f_value.getValue();

    helper::WriteAccessor< Data< helper::vector<Value> > > outputValues = f_outputValues;

    const Value& defaultValue = p_defaultValue.getValue();

    //copy existing values
    outputValues.clear();
    outputValues.resize(inputValues.size());
    for(unsigned int i=0 ; i<inputValues.size() ; i++)
        outputValues[i] = inputValues[i];

    //add new value
    for(unsigned int i=0 ; i<indices.size() ; i++)
    {
        Index ind = indices[i];
        //new index may be bigger than the current existing map, so have to resize it
        if (ind >= outputValues.size())
        {
            unsigned int oldSize = outputValues.size();
            outputValues.resize(ind+1);
            //if there is hole when resizing, put default value
            for (unsigned int j=oldSize ; j<ind+1 ; j++)
                outputValues[j] = defaultValue;
        }
        outputValues[ind] = value;
    }
}


} // namespace engine

} // namespace component

} // namespace sofa

#endif //INDEXVALUEMAPPER_INL_
