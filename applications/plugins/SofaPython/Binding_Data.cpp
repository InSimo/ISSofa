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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "Binding_Data.h"
#include "Binding_LinearSpring.h"

#include <sofa/core/objectmodel/BaseData.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/dataparser/JsonDataParser.h>
#include "Binding_LinearSpring.h"
#include <sofa/core/objectmodel/DataFileName.h>
#include <SofaDeformable/SpringForceField.h>

using namespace sofa::component::interactionforcefield;
using namespace sofa::core::objectmodel;
using namespace sofa::defaulttype;


// TODO:
// se servir du DataTypeInfo pour utiliser directement les bons type :-)
// Il y a un seul type "Data" exposé en python, le transtypage est géré automatiquement


SP_CLASS_ATTR_GET(Data,name)(PyObject *self, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast
    return PyString_FromString(data->getName().c_str());
}
SP_CLASS_ATTR_SET(Data,name)(PyObject *self, PyObject * args, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast
    char *str = PyString_AsString(args); // pour les setters, un seul objet et pas un tuple....
    data->setName(str);
    return 0;
}

PyObject *GetDataValuePython(BaseData* data)
{
    // depending on the data type, we return the good python type (int, float, sting, array, ...)

    const AbstractTypeInfo *typeinfo = data->getValueTypeInfo();
    const void* valueVoidPtr = data->getValueVoidPtr();
    //int rowWidth = typeinfo->size();
    //int nbRows = typeinfo->size(data->getValueVoidPtr()) / typeinfo->size();

    // special cases...
    Data<sofa::helper::vector<LinearSpring<SReal> > >* vectorLinearSpring = dynamic_cast<Data<sofa::helper::vector<LinearSpring<SReal> > >*>(data);
    if (vectorLinearSpring)
    {
        const int rowWidth = 1;
        const int nbRows = (int)vectorLinearSpring->getValue().size();
        // special type, a vector of LinearSpring objects
        if (nbRows==1)
        {
            // this type is NOT a vector; return directly the proper native type
            const LinearSpring<SReal> value = vectorLinearSpring->getValue()[0];
            LinearSpring<SReal> *obj = new LinearSpring<SReal>(value.m1,value.m2,value.ks,value.kd,value.initpos);
            return SP_BUILD_PYPTR(LinearSpring,LinearSpring<SReal>,obj,true); // "true", because I manage the deletion myself
        }
        else
        {
            PyObject *rows = PyList_New(nbRows);
            for (int i=0; i<nbRows; i++)
            {
                PyObject *row = PyList_New(rowWidth);
                for (int j=0; j<rowWidth; j++)
                {
                    // build each value of the list
                    const LinearSpring<SReal> value = vectorLinearSpring->getValue()[i*rowWidth+j];
                    LinearSpring<SReal> *obj = new LinearSpring<SReal>(value.m1,value.m2,value.ks,value.kd,value.initpos);
                    PyList_SetItem(row,j,SP_BUILD_PYPTR(LinearSpring,LinearSpring<SReal>,obj,true));
                }
                PyList_SetItem(rows,i,row);
            }

            return rows;
        }

    }

    if(typeinfo->ValidInfo() &&  typeinfo->IsMultiValue())
    {
        const AbstractMultiValueTypeInfo* mvinfo = typeinfo->MultiValueType();
        if (mvinfo->finalSize(valueVoidPtr)==1 && mvinfo->FixedFinalSize() 
            && !mvinfo->String() /* Temp condition to keep previous output type for string (array of array of string instead of string) */)
        {
            // this type is NOT a vector; return directly the proper native type
            if (mvinfo->String())
            {
                // it's some text
                return PyString_FromString(mvinfo->getFinalValueString(valueVoidPtr,0).c_str());
            }
            if (mvinfo->Scalar())
            {
                // it's a SReal
                return PyFloat_FromDouble(mvinfo->getFinalValueScalar(valueVoidPtr,0));
            }
            if (mvinfo->Integer())
            {
                // it's some Integer...
                return PyInt_FromLong((long)mvinfo->getFinalValueInteger(valueVoidPtr,0));
            }
        }
        else
        {
            // this is a vector; return a python list of the corrsponding type (ints, scalars or strings)
            const int rowWidth = mvinfo->FinalSize();
            const int nbRows = mvinfo->finalSize(valueVoidPtr) / rowWidth;

            PyObject *rows = PyList_New(nbRows);
            for (int i=0; i<nbRows; i++)
            {
                PyObject *row = PyList_New(rowWidth);
                for (int j=0; j<rowWidth; j++)
                {
                    // build each value of the list
                    if (mvinfo->String())
                    {
                        // it's some text
                        PyList_SetItem(row,j,PyString_FromString(mvinfo->getFinalValueString(valueVoidPtr,i*rowWidth+j).c_str()));
                    }
                    else if (mvinfo->Scalar())
                    {
                        // it's a SReal
                        PyList_SetItem(row,j,PyFloat_FromDouble(mvinfo->getFinalValueScalar(valueVoidPtr,i*rowWidth+j)));
                    }
                    else if (mvinfo->Integer())
                    {
                        // it's some Integer...
                        PyList_SetItem(row,j,PyInt_FromLong((long)mvinfo->getFinalValueInteger(valueVoidPtr,i*rowWidth+j)));
                    }
                    else
                    {
                        // this type is not yet supported
                        printf("<SofaPython> BaseData_getAttr_value WARNING: unsupported native type=%s ; returning string value\n",data->getValueTypeString().c_str());
                        PyList_SetItem(row,j,PyString_FromString(mvinfo->getFinalValueString(valueVoidPtr,i*rowWidth+j).c_str()));
                    }
                }
                PyList_SetItem(rows,i,row);
            }

            return rows;
        }
    }
    // fallback for unknown types, read the content of the data using the output stream operator
    return PyString_FromString(data->getValueString().c_str());
}


bool SetDataValuePython(BaseData* data, PyObject* args)
{
    // de quel type est args ?
    bool isInt = PyInt_Check(args);
    bool isScalar = PyFloat_Check(args);
    bool isString = PyString_Check(args);
    bool isList = PyList_Check(args);
    const AbstractTypeInfo *typeinfo = data->getValueTypeInfo(); // info about the data value
    //int rowWidth = (typeinfo && typeinfo->ValidInfo()) ? typeinfo->size() : 1;
    //int nbRows = (typeinfo && typeinfo->ValidInfo()) ? typeinfo->size(data->getValueVoidPtr()) / typeinfo->size() : 1;

    // special cases...
    Data<sofa::helper::vector<LinearSpring<SReal> > >* dataVectorLinearSpring = dynamic_cast<Data<sofa::helper::vector<LinearSpring<SReal> > >*>(data);
    if (dataVectorLinearSpring)
    {
        // special type, a vector of LinearSpring objects
        const int rowWidth = 1;
        int nbRows = (int)dataVectorLinearSpring->getValue().size();
        if (!isList)
        {
            // one value
            // check the python object type
            if (rowWidth*nbRows<1 || !PyObject_IsInstance(args,reinterpret_cast<PyObject*>(&SP_SOFAPYTYPEOBJECT(LinearSpring))))
            {
                // type mismatch or too long list
                PyErr_BadArgument();
                return false;
            }

            LinearSpring<SReal>* obj=dynamic_cast<LinearSpring<SReal>*>(((PyPtr<LinearSpring<SReal> >*)args)->object);
            sofa::helper::vector<LinearSpring<SReal> >* vectorLinearSpring = dataVectorLinearSpring->beginEdit();

            (*vectorLinearSpring)[0].m1 = obj->m1;
            (*vectorLinearSpring)[0].m2 = obj->m2;
            (*vectorLinearSpring)[0].ks = obj->ks;
            (*vectorLinearSpring)[0].kd = obj->kd;
            (*vectorLinearSpring)[0].initpos = obj->initpos;

            dataVectorLinearSpring->endEdit();

            return true;
        }
        else
        {
            // values list
            // is-it a double-dimension list ?
            //PyObject *firstRow = PyList_GetItem(args,0);

            if (PyList_Check(PyList_GetItem(args,0)))
            {
                // two-dimension array!

                // right number if rows ?
                if (PyList_Size(args)!=nbRows)
                {
                    // only a warning; do not raise an exception...
                    SP_MESSAGE_WARNING( "list size mismatch for data \""<<data->getName()<<"\" (incorrect rows count)" )
                    if (PyList_Size(args)<nbRows)
                        nbRows = PyList_Size(args);
                }

                sofa::helper::vector<LinearSpring<SReal> >* vectorLinearSpring = dataVectorLinearSpring->beginEdit();

                // let's fill our rows!
                for (int i=0; i<nbRows; i++)
                {
                    PyObject *row = PyList_GetItem(args,i);

                    // right number if list members ?
                    int size = rowWidth;
                    if (PyList_Size(row)!=size)
                    {
                        // only a warning; do not raise an exception...
                        SP_MESSAGE_WARNING( "row "<<i<<" size mismatch for data \""<<data->getName()<<"\" (src="<<(int)PyList_Size(row)<<"x"<<nbRows<<" dst="<<size<<"x"<<nbRows<<")" )
                        if (PyList_Size(row)<size)
                            size = PyList_Size(row);
                    }

                    // okay, let's set our list...
                    for (int j=0; j<size; j++)
                    {

                        PyObject *listElt = PyList_GetItem(row,j);
                        if(!PyObject_IsInstance(listElt,reinterpret_cast<PyObject*>(&SP_SOFAPYTYPEOBJECT(LinearSpring))))
                        {
                            // type mismatch
                            dataVectorLinearSpring->endEdit();
                            PyErr_BadArgument();
                            return false;
                        }
                        LinearSpring<SReal>* spring=dynamic_cast<LinearSpring<SReal>*>(((PyPtr<LinearSpring<SReal> >*)listElt)->object);


                        (*vectorLinearSpring)[j+i*rowWidth].m1 = spring->m1;
                        (*vectorLinearSpring)[j+i*rowWidth].m2 = spring->m2;
                        (*vectorLinearSpring)[j+i*rowWidth].ks = spring->ks;
                        (*vectorLinearSpring)[j+i*rowWidth].kd = spring->kd;
                        (*vectorLinearSpring)[j+i*rowWidth].initpos = spring->initpos;

                    }



                }

                dataVectorLinearSpring->endEdit();

                return true;

            }
            else
            {
                // it is a one-dimension only array
                // right number if list members ?
                int size = rowWidth*nbRows;
                if (PyList_Size(args)!=size)
                {
                    // only a warning; do not raise an exception...
                    SP_MESSAGE_WARNING( "list size mismatch for data \""<<data->getName()<<"\" (src="<<(int)PyList_Size(args)<<" dst="<<size<<")" )
                    if (PyList_Size(args)<size)
                        size = PyList_Size(args);
                }

                sofa::helper::vector<LinearSpring<SReal> >* vectorLinearSpring = dataVectorLinearSpring->beginEdit();

                // okay, let's set our list...
                for (int i=0; i<size; i++)
                {

                    PyObject *listElt = PyList_GetItem(args,i);

                    if(!PyObject_IsInstance(listElt,reinterpret_cast<PyObject*>(&SP_SOFAPYTYPEOBJECT(LinearSpring))))
                    {
                        // type mismatch
                        dataVectorLinearSpring->endEdit();
                        PyErr_BadArgument();
                        return false;
                    }

                    LinearSpring<SReal>* spring=dynamic_cast<LinearSpring<SReal>*>(((PyPtr<LinearSpring<SReal> >*)listElt)->object);

                    (*vectorLinearSpring)[i].m1 = spring->m1;
                    (*vectorLinearSpring)[i].m2 = spring->m2;
                    (*vectorLinearSpring)[i].ks = spring->ks;
                    (*vectorLinearSpring)[i].kd = spring->kd;
                    (*vectorLinearSpring)[i].initpos = spring->initpos;


    /*
                    if (PyFloat_Check(listElt))
                    {
                        // it's a scalar
                        if (!typeinfo->Scalar())
                        {
                            // type mismatch
                            PyErr_BadArgument();
                            return false;
                        }
                        SReal value = PyFloat_AsDouble(listElt);
                        void* editVoidPtr = data->beginEditVoidPtr();
                        typeinfo->setScalarValue(editVoidPtr,i,value);
                        data->endEditVoidPtr();
                    }
     */
                }
                dataVectorLinearSpring->endEdit();

                return true;
            }
        }


        return false;
    }

    const AbstractMultiValueTypeInfo* mvinfo = NULL;
    int rowWidth = 1;
    int nbRows = 1;
    if (typeinfo->ValidInfo() && typeinfo->IsMultiValue())
    {
        mvinfo = typeinfo->MultiValueType();
        rowWidth = mvinfo->FinalSize();
        nbRows = mvinfo->finalSize(data->getValueVoidPtr()) / rowWidth;
    }

    if (isInt)
    {
        // it's an int

        if (rowWidth*nbRows<1 || !mvinfo || (!mvinfo->Integer() && !mvinfo->Scalar()))
        {
            // type mismatch or too long list
            PyErr_BadArgument();
            return false;
        }
        long value = PyInt_AsLong(args);
        void* editVoidPtr = data->beginEditVoidPtr();
        if (mvinfo->Scalar())
            mvinfo->setFinalValueScalar(editVoidPtr,0,(SReal)value); // cast int to float
        else
            mvinfo->setFinalValueInteger(editVoidPtr,0,value);
        data->endEditVoidPtr();
        return true;
    }
    else if (isScalar)
    {
        // it's a scalar
        if (rowWidth*nbRows<1 || !mvinfo || !mvinfo->Scalar())
        {
            // type mismatch or too long list
            PyErr_BadArgument();
            return false;
        }
        SReal value = PyFloat_AsDouble(args);
        void* editVoidPtr = data->beginEditVoidPtr();
        mvinfo->setFinalValueScalar(editVoidPtr,0,value);
        data->endEditVoidPtr();
        return true;
    }
    else if (isString)
    {
        // it's a string
        char *str = PyString_AsString(args); // pour les setters, un seul objet et pas un tuple....
        data->read(str);
        return true;
    }
    else if (isList)
    {
        // it's a list
        // check list emptyness
        if (PyList_Size(args)==0)
        {
            // empty list: ignored
            return true;
        }

        // is-it a double-dimension list ?
        //PyObject *firstRow = PyList_GetItem(args,0);

        if (PyList_Check(PyList_GetItem(args,0)))
        {
            // two-dimension array!

            // right number if rows ?
            if (PyList_Size(args)!=nbRows)
            {
                // only a warning; do not raise an exception...
                SP_MESSAGE_WARNING( "list size mismatch for data \""<<data->getName()<<"\" (incorrect rows count)" )
                if (PyList_Size(args)<nbRows)
                    nbRows = PyList_Size(args);
            }

            void* editVoidPtr = data->beginEditVoidPtr();

            // let's fill our rows!
            for (int i=0; i<nbRows; i++)
            {
                PyObject *row = PyList_GetItem(args,i);

                // right number if list members ?
                int size = rowWidth;
                if (PyList_Size(row)!=size)
                {
                    // only a warning; do not raise an exception...
                    SP_MESSAGE_WARNING( "row "<<i<<" size mismatch for data \""<<data->getName()<<"\"" )
                    if (PyList_Size(row)<size)
                        size = PyList_Size(row);
                }

                // okay, let's set our list...
                for (int j=0; j<size; j++)
                {

                    PyObject *listElt = PyList_GetItem(row,j);

                    if (PyInt_Check(listElt))
                    {
                        // it's an int
                        if (mvinfo->Integer())
                        {
                            // integer value
                            long value = PyInt_AsLong(listElt);
                            mvinfo->setFinalValueInteger(editVoidPtr,i*rowWidth+j,value);
                        }
                        else if (mvinfo->Scalar())
                        {
                            // cast to scalar value
                            SReal value = (SReal)PyInt_AsLong(listElt);
                            mvinfo->setFinalValueScalar(editVoidPtr,i*rowWidth+j,value);
                        }
                        else
                        {
                            // type mismatch
                            PyErr_BadArgument();
                            return false;
                        }
                    }
                    else if (PyFloat_Check(listElt))
                    {
                        // it's a scalar
                        if (!mvinfo->Scalar())
                        {
                            // type mismatch
                            PyErr_BadArgument();
                            return false;
                        }
                        SReal value = PyFloat_AsDouble(listElt);
                        mvinfo->setFinalValueScalar(editVoidPtr,i*rowWidth+j,value);
                    }
                    else if (PyString_Check(listElt))
                    {
                        // it's a string
                        if (!mvinfo->String())
                        {
                            // type mismatch
                            PyErr_BadArgument();
                            return false;
                        }
                        char *str = PyString_AsString(listElt); // pour les setters, un seul objet et pas un tuple....
                        mvinfo->setFinalValueString(editVoidPtr,i*rowWidth+j,str);
                    }
                    else
                    {
                        printf("Lists not yet supported...\n");
                        PyErr_BadArgument();
                        return false;

                    }
                }

            }
            data->endEditVoidPtr();
            return true;

        }
        else
        {
            // it is a one-dimension only array
            // right number if list members ?
            int size = rowWidth*nbRows;
            if (PyList_Size(args)!=size)
            {
                // only a warning; do not raise an exception...
                SP_MESSAGE_WARNING( "list size mismatch for data \""<<data->getName()<<"\" (src="<<(int)PyList_Size(args)<<" dst="<<size<<")" )
                if (PyList_Size(args)<size)
                    size = PyList_Size(args);
            }

            void* editVoidPtr = data->beginEditVoidPtr();

            // okay, let's set our list...
            for (int i=0; i<size; i++)
            {

                PyObject *listElt = PyList_GetItem(args,i);

                if (PyInt_Check(listElt))
                {
                    // it's an int
                    if (mvinfo->Integer())
                    {
                        // integer value
                        long value = PyInt_AsLong(listElt);
                        mvinfo->setFinalValueInteger(editVoidPtr,i,value);
                    }
                    else if (mvinfo->Scalar())
                    {
                        // cast to scalar value
                        SReal value = (SReal)PyInt_AsLong(listElt);
                        mvinfo->setFinalValueScalar(editVoidPtr,i,value);
                    }
                    else
                    {
                        // type mismatch
                        PyErr_BadArgument();
                        return false;
                    }
                }
                else if (PyFloat_Check(listElt))
                {
                    // it's a scalar
                    if (!mvinfo->Scalar())
                    {
                        // type mismatch
                        PyErr_BadArgument();
                        return false;
                    }
                    SReal value = PyFloat_AsDouble(listElt);
                    mvinfo->setFinalValueScalar(editVoidPtr,i,value);
                }
                else if (PyString_Check(listElt))
                {
                    // it's a string
                    if (!mvinfo->String())
                    {
                        // type mismatch
                        PyErr_BadArgument();
                        return false;
                    }
                    char *str = PyString_AsString(listElt); // pour les setters, un seul objet et pas un tuple....
                    mvinfo->setFinalValueString(editVoidPtr,i,str);
                }
                else
                {
                    printf("Lists not yet supported...\n");
                    PyErr_BadArgument();
                    return false;

                }
            }
            data->endEditVoidPtr();
            return true;
        }

    }

    return false;

}

PyObject* GetDataJsonPython(BaseData* data)
{
    // depending on the data type, we return the good python type (int, float, sting, array, ...)

    const AbstractTypeInfo *typeinfo = data->getValueTypeInfo();
    const void* valueVoidPtr = data->getValueVoidPtr();
    std::string output;
    if(typeinfo->ValidInfo())
    {
        sofa::core::dataparser::JsonDataParser JsonParser(data->getName());
        JsonParser.fromData(output, valueVoidPtr, typeinfo);
    }
    else
    {
        SP_MESSAGE_ERROR( "Invalide DataType can't be parse in JSON" )
        PyErr_BadArgument();
        output = "{}";
    }
    return PyString_FromString(output.c_str());
}

bool SetDataJsonPython(BaseData* data, PyObject* args)
{
    const AbstractTypeInfo *typeinfo = data->getValueTypeInfo();
    const std::string input = PyString_AsString(args);
    if(typeinfo->ValidInfo())
    {
        sofa::core::dataparser::JsonDataParser JsonParser(data->getName());
        void* editVoidPtr = data->beginEditVoidPtr();
        JsonParser.toData(input, editVoidPtr, typeinfo);
        data->endEditVoidPtr();
        return true;
    }
    else
    {
        SP_MESSAGE_ERROR( "Invalide DataType can't be parse in JSON" )
        PyErr_BadArgument();
        return false;
    }

}

SP_CLASS_ATTR_GET(Data, fullPath)(PyObject *self, void*)
{
    BaseData* data = ((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast

    if (sofa::core::objectmodel::DataFileName* dataFilename = dynamic_cast<sofa::core::objectmodel::DataFileName*>(data))
    {
        return PyString_FromString(dataFilename->getFullPath().c_str());
    }

    Py_RETURN_NONE;
}


SP_CLASS_ATTR_SET(Data, fullPath)(PyObject */*self*/, PyObject * /*args*/, void*)
{
    SP_MESSAGE_ERROR("fullPath attribute is read only")
        PyErr_BadArgument();
    return -1;
}


SP_CLASS_ATTR_GET(Data,value)(PyObject *self, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast
    return GetDataValuePython(data);
}

SP_CLASS_ATTR_GET(Data,json)(PyObject *self, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast
    return GetDataJsonPython(data);
}


SP_CLASS_ATTR_GET(Data,link)(PyObject *self, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast

    std::string path = data->getLinkPath();
    if (path.size() == 0)
    {
        SP_MESSAGE_ERROR( "no link for data " + data->getName());
    }

    return PyString_FromString(path.c_str());
}


SP_CLASS_ATTR_SET(Data,value)(PyObject *self, PyObject * args, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast
    if (SetDataValuePython(data,args))
        return 0;   // OK


    SP_MESSAGE_ERROR( "argument type not supported" )
    PyErr_BadArgument();
    return -1;
}

SP_CLASS_ATTR_SET(Data,json)(PyObject *self, PyObject * args, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast
    if (SetDataJsonPython(data,args))
        return 0;   // OK


    SP_MESSAGE_ERROR( "argument type not supported" )
    PyErr_BadArgument();
    return -1;
}

SP_CLASS_ATTR_SET(Data,link)(PyObject *self, PyObject * args, void*)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object; // TODO: check dynamic cast
    const std::string input = PyString_AsString(args);

    if (data->setParent(input))
        return 0;

    SP_MESSAGE_ERROR( "Bad parents" )
    PyErr_BadArgument();
    return -1;
}

// access ONE element of the vector
extern "C" PyObject * Data_getValue(PyObject *self, PyObject * args)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object;
    const AbstractMultiValueTypeInfo *typeinfo = data->getValueTypeInfo()->MultiValueType(); // info about the data value
    if (!typeinfo)
    {
        SP_MESSAGE_ERROR( "Data.getValue unknown data type" )
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    int index;
    if (!PyArg_ParseTuple(args, "i",&index))
    {
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    const void* valueVoidPtr = data->getValueVoidPtr();
    if ((unsigned int)index>=typeinfo->finalSize(valueVoidPtr))
    {
        // out of bounds!
        SP_MESSAGE_ERROR( "Data.getValue index overflow" )
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    if (typeinfo->Scalar())
        return PyFloat_FromDouble(typeinfo->getFinalValueScalar(valueVoidPtr,index));
    if (typeinfo->Integer())
        return PyInt_FromLong((long)typeinfo->getFinalValueInteger(valueVoidPtr,index));
    if (typeinfo->String())
        return PyString_FromString(typeinfo->getFinalValueString(valueVoidPtr,index).c_str());

    // should never happen....
    SP_MESSAGE_ERROR( "Data.getValue unknown data type" )
    PyErr_BadArgument();
    Py_RETURN_NONE;
}
extern "C" PyObject * Data_setValue(PyObject *self, PyObject * args)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object;
    const AbstractMultiValueTypeInfo *typeinfo = data->getValueTypeInfo()->MultiValueType(); // info about the data value
    if (!typeinfo)
    {
        SP_MESSAGE_ERROR( "Data.setValue unknown data type" )
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    int index;
    PyObject *value;
    if (!PyArg_ParseTuple(args, "iO",&index,&value))
    {
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    void* editVoidPtr = data->beginEditVoidPtr();
    if ((unsigned int)index>=typeinfo->finalSize(editVoidPtr))
    {
        data->endEditVoidPtr();
        // out of bounds!
        SP_MESSAGE_ERROR( "Data.setValue index overflow" )
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    if (typeinfo->Scalar() && PyFloat_Check(value))
    {
        typeinfo->setFinalValueScalar(editVoidPtr,index,PyFloat_AsDouble(value));
        data->endEditVoidPtr();
        return PyInt_FromLong(0);
    }
    if (typeinfo->Integer() && PyInt_Check(value))
    {
        typeinfo->setFinalValueInteger(editVoidPtr,index,PyInt_AsLong(value));
        data->endEditVoidPtr();
        return PyInt_FromLong(0);
    }
    if (typeinfo->String() && PyString_Check(value))
    {
        typeinfo->setFinalValueString(editVoidPtr,index,PyString_AsString(value));
        data->endEditVoidPtr();
        return PyInt_FromLong(0);
    }

    // should never happen....
    SP_MESSAGE_ERROR( "Data.setValue type mismatch" )
    PyErr_BadArgument();
    Py_RETURN_NONE;
}


extern "C" PyObject * Data_getValueTypeString(PyObject *self, PyObject * /*args*/)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object;
    return PyString_FromString(data->getValueTypeString().c_str());
}

extern "C" PyObject * Data_getValueString(PyObject *self, PyObject * /*args*/)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object;
    return PyString_FromString(data->getValueString().c_str());
}

extern "C" PyObject * Data_getSize(PyObject *self, PyObject * /*args*/)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object;

    const AbstractMultiValueTypeInfo *typeinfo = data->getValueTypeInfo()->MultiValueType(); // info about the data value
    long size = 1;
    if (typeinfo)
    {
        int rowWidth = typeinfo->FinalSize();
        int nbRows = typeinfo->finalSize(data->getValueVoidPtr()) / rowWidth;
        size = rowWidth * nbRows;
        printf("Data_getSize rowWidth=%d nbRows=%d\n",rowWidth,nbRows);
    }
    return PyInt_FromLong(size);
}

extern "C" PyObject * Data_setSize(PyObject *self, PyObject * args)
{
    BaseData* data=((PyPtr<BaseData>*)self)->object;
    int size;
    if (!PyArg_ParseTuple(args, "i",&size))
    {
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    const AbstractMultiValueTypeInfo *typeinfo = data->getValueTypeInfo()->MultiValueType(); // info about the data value
    if (typeinfo)
    {
        typeinfo->setFinalSize(data->beginEditVoidPtr(),size);
        data->endEditVoidPtr();
    }
    Py_RETURN_NONE;
}

SP_CLASS_METHODS_BEGIN(Data)
SP_CLASS_METHOD(Data,getValueTypeString)
SP_CLASS_METHOD(Data,getValueString)
SP_CLASS_METHOD(Data,setValue)
SP_CLASS_METHOD(Data,getValue)
SP_CLASS_METHOD(Data,getSize)
SP_CLASS_METHOD(Data,setSize)
SP_CLASS_METHODS_END


SP_CLASS_ATTRS_BEGIN(Data)
SP_CLASS_ATTR(Data,name)
//SP_CLASS_ATTR(BaseData,owner)
SP_CLASS_ATTR(Data,value)
SP_CLASS_ATTR(Data,json)
SP_CLASS_ATTR(Data,link)
SP_CLASS_ATTR(Data,fullPath)
SP_CLASS_ATTRS_END

SP_CLASS_TYPE_BASE_PTR_ATTR(Data,BaseData)
