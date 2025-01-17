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


#include "Binding_BaseContext.h"
#include "Binding_Base.h"
#include "Binding_Vector.h"
#include "ScriptEnvironment.h"

#if defined(__APPLE__) && defined(__MACH__)
#    include <Python/frameobject.h>
#else
#    include <frameobject.h>
#endif

#include <sofa/defaulttype/Vec3Types.h>
using namespace sofa::defaulttype;
#include <sofa/core/ObjectFactory.h>
using namespace sofa::core;
#include <sofa/core/objectmodel/BaseContext.h>
using namespace sofa::core::objectmodel;
#include <sofa/simulation/common/Node.h>
using namespace sofa::simulation;


extern "C" PyObject * BaseContext_setGravity(PyObject *self, PyObject * args)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    PyPtr<Vector3>* pyVec;
    if (!PyArg_ParseTuple(args, "O",&pyVec))
        Py_RETURN_NONE;
    obj->setGravity(*pyVec->object);
    Py_RETURN_NONE;
}

extern "C" PyObject * BaseContext_getGravity(PyObject *self, PyObject * /*args*/)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return SP_BUILD_PYPTR(Vector3,Vector3,new Vector3(obj->getGravity()),true); // "true", because I manage the deletion myself
}

extern "C" PyObject * BaseContext_getTime(PyObject *self, PyObject * /*args*/)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyFloat_FromDouble(obj->getTime());
}

extern "C" PyObject * BaseContext_getDt(PyObject *self, PyObject * /*args*/)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyFloat_FromDouble(obj->getDt());
}

extern "C" PyObject * BaseContext_getRootContext(PyObject *self, PyObject * /*args*/)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return SP_BUILD_PYSPTR(obj->getRootContext());
}

// object factory
extern "C" PyObject * BaseContext_createObject(PyObject * self, PyObject * args, PyObject * kw)
{
    BaseContext* context=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());

//    std::cout << "<PYTHON> BaseContext_createObject PyTuple_Size=" << PyTuple_Size(args) << " PyDict_Size=" << PyDict_Size(kw) << std::endl;

    char *type;
    if (!PyArg_ParseTuple(args, "s",&type))
    {
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }

    // temporarily, the name is set to the type name.
    // if a "name" parameter is provided, it will overwrite it.
    BaseObjectDescription desc(type,type);

    if (kw && PyDict_Size(kw)>0)
    {
        PyObject* keys = PyDict_Keys(kw);
        PyObject* values = PyDict_Values(kw);
        for (int i=0; i<PyDict_Size(kw); i++)
        {
            PyObject *key = PyList_GetItem(keys,i);
            PyObject *value = PyList_GetItem(values,i);
        //    std::cout << PyString_AsString(PyList_GetItem(keys,i)) << "=\"" << PyString_AsString(PyObject_Str(PyList_GetItem(values,i))) << "\"" << std::endl;
            if (PyString_Check(value))
                desc.setAttribute(PyString_AsString(key),PyString_AsString(value));
            else
                desc.setAttribute(PyString_AsString(key),PyString_AsString(PyObject_Str(value)));
        }
        Py_DecRef(keys);
        Py_DecRef(values);
    }

    BaseObject::SPtr obj = ObjectFactory::getInstance()->createObject(context,&desc);
    if (obj==0)
    {
        SP_MESSAGE_ERROR( "createObject " << desc.getName() << " of type " << desc.getAttribute("type","")<< " in node "<<context->getName() )
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }

    // store the source location of this object
    //PyFrameObject* frame = PyEval_GetFrame();
    PyThreadState *tstate = PyThreadState_GET();
    if (NULL != tstate && NULL != tstate->frame)
    {
        PyFrameObject *frame = tstate->frame;
        // int line = frame->f_lineno;
        /*
         frame->f_lineno will not always return the correct line number
         you need to call PyCode_Addr2Line().
        */
        int line = PyCode_Addr2Line(frame->f_code, frame->f_lasti);
        const char *filename = PyString_AsString(frame->f_code->co_filename);
        //const char *funcname = PyString_AsString(frame->f_code->co_name);
        //printf("    %s(%d): %s\n", filename, line, funcname);
        //frame = frame->f_back;
        obj->addSourceFile(filename, line, 0);
    }

    Node *node = Node::DynamicCast(context);
    if (node)
    {
        //SP_MESSAGE_INFO( "Sofa.Node.createObject("<<type<<") node="<<node->getName()<<" isInitialized()="<<node->isInitialized() )
        if (node->isInitialized())
            SP_MESSAGE_WARNING( "Sofa.Node.createObject("<<type<<") called on a node("<<node->getName()<<") that is already initialized" )
        if (!ScriptEnvironment::isNodeCreatedByScript(node))
            SP_MESSAGE_WARNING( "Sofa.Node.createObject("<<type<<") called on a node("<<node->getName()<<") that is not created by the script" )
    }

    return SP_BUILD_PYSPTR(obj.get());
}


extern "C" PyObject * BaseContext_getObject(PyObject * self, PyObject * args)
{
    BaseContext* context=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    char *path;
    if (!PyArg_ParseTuple(args, "s",&path))
    {
        SP_MESSAGE_WARNING( "BaseContext_getObject: wrong argument, should be a string" )
        Py_RETURN_NONE;
    }
    if (!context || !path)
    {
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    BaseObject::SPtr sptr;
    context->get<BaseObject>(sptr,path);
    if (!sptr)
    {
        SP_MESSAGE_WARNING( "BaseContext_getObject: component "<<path<<" not found (the complete relative path is needed)" )
        Py_RETURN_NONE;
    }

    return SP_BUILD_PYSPTR(sptr.get());
}

extern "C" PyObject * BaseContext_getObjects(PyObject * self, PyObject * args)
{
    BaseContext* context=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    char* search_direction= NULL;
    char* type_name= NULL;
    char* name= NULL;
    if ( !PyArg_ParseTuple ( args, "|sss", &search_direction, &type_name, &name ) ) {
        SP_MESSAGE_WARNING( "BaseContext_getObjects: wrong arguments! Expected format: getObjects ( OPTIONAL STRING searchDirection, OPTIONAL STRING typeName, OPTIONAL STRING name )" )
        Py_RETURN_NONE;
    }
    if (!context)
    {
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }

    std::string name_str ( name ? name : "" );
    ObjectFactory::ClassEntry* class_entry = type_name ? &ObjectFactory::getInstance()->getEntry(type_name) : NULL;
    
    sofa::core::objectmodel::BaseContext::SearchDirection search_direction_enum= sofa::core::objectmodel::BaseContext::Local;
    if ( search_direction ) 
    {
        std::string search_direction_str ( search_direction );
        if ( search_direction_str == "SearchUp" )
        {
            search_direction_enum= sofa::core::objectmodel::BaseContext::SearchUp;
        }
        else if ( search_direction_str == "Local" )
        {
            search_direction_enum= sofa::core::objectmodel::BaseContext::Local;
        }
        else if ( search_direction_str == "SearchDown" )
        {
            search_direction_enum= sofa::core::objectmodel::BaseContext::SearchDown;
        }
        else if ( search_direction_str == "SearchRoot" )
        {
            search_direction_enum= sofa::core::objectmodel::BaseContext::SearchRoot;
        }
        else if ( search_direction_str == "SearchParents" )
        {
            search_direction_enum= sofa::core::objectmodel::BaseContext::SearchParents;
        }
        else 
        {
            SP_MESSAGE_WARNING( "BaseContext_getObjects: Invalid search direction, using 'Local'. Expected: 'SearchUp', 'Local', 'SearchDown', 'SearchRoot', or 'SearchParents'." )
        }
    }

    sofa::helper::vector< boost::intrusive_ptr<BaseObject> > list;
    context->get<BaseObject>(&list,search_direction_enum);

    PyObject *pyList = PyList_New(0);
    for (unsigned int i=0; i<list.size(); i++)
    {
        BaseObject* o = list[i].get();
        if ( !class_entry || o->getClassName() == class_entry->className || class_entry->creatorMap.find(o->getClassName()) != class_entry->creatorMap.end())
        {
            if ( !name || name_str == o->getName())
            {
                PyObject* obj= SP_BUILD_PYSPTR(o); // ref 1
                PyList_Append(pyList,obj); // ref 2
                Py_DECREF(obj); // ref 1 (now owned by list)
            }
        }
    }

    return pyList;
}

SP_CLASS_METHODS_BEGIN(BaseContext)
SP_CLASS_METHOD(BaseContext,getRootContext)
SP_CLASS_METHOD(BaseContext,getTime)
SP_CLASS_METHOD(BaseContext,getDt)
SP_CLASS_METHOD(BaseContext,getGravity)
SP_CLASS_METHOD(BaseContext,setGravity)
SP_CLASS_METHOD_KW(BaseContext,createObject)
SP_CLASS_METHOD(BaseContext,getObject)
SP_CLASS_METHOD(BaseContext,getObjects)
SP_CLASS_METHODS_END


extern "C" PyObject * BaseContext_getAttr_animate(PyObject *self, void*)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyBool_FromLong(obj->getAnimate());
}
extern "C" int BaseContext_setAttr_animate(PyObject *self, PyObject * args, void*)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    if (!PyBool_Check(args))
    {
        PyErr_BadArgument();
        return -1;
    }
    obj->setAnimate(args==Py_True);
    return 0;
}

extern "C" PyObject * BaseContext_getAttr_active(PyObject *self, void*)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyBool_FromLong(obj->isActive());
}
extern "C" int BaseContext_setAttr_active(PyObject *self, PyObject * args, void*)
{
    BaseContext* obj=BaseContext::DynamicCast(((PySPtr<Base>*)self)->object.get());
    if (!PyBool_Check(args))
    {
        PyErr_BadArgument();
        return -1;
    }
    obj->setActive(args==Py_True);
    return 0;
}

SP_CLASS_ATTRS_BEGIN(BaseContext)
SP_CLASS_ATTR(BaseContext,active)
SP_CLASS_ATTR(BaseContext,animate)
//SP_CLASS_ATTR(BaseContext,gravity) // attribut objets = problème... le setter ne fonctionne pas
SP_CLASS_ATTRS_END

SP_CLASS_TYPE_SPTR_ATTR(BaseContext,BaseContext,Base)
