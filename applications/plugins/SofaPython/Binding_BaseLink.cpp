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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "Binding_BaseLink.h"

#include <sofa/core/objectmodel/Link.h>

using namespace sofa::core::objectmodel;
using namespace sofa::defaulttype;


SP_CLASS_ATTR_GET(Link, path)(PyObject *self, void*)
{
    BaseLink* link =((PyPtr<BaseLink>*)self)->object;
    return PyString_FromString(link->getLinkedPath().c_str());
}

SP_CLASS_ATTR_SET(Link, path)(PyObject *self, PyObject * args, void*)
{
    BaseLink* link =((PyPtr<BaseLink>*)self)->object;
    if (setLinkePath(link,args))
        return 0;   // OK


    SP_MESSAGE_ERROR( "argument type not supported" )
    PyErr_BadArgument();
    return -1;
}


bool setLinkePath(BaseLink* link, PyObject* args)
{
    bool isString = PyString_Check(args);
    if (! isString)
    {
        return false;
    }

    char *path = PyString_AsString(args);


    if (link->isMultiLink())
    {
        link->releaseAspect(0);

    }

    if (!link->read(path))
    {
        return false;
    }
    return true;
}


extern "C" PyObject * Link_setLinkPath(PyObject* self, PyObject* args)
{
    BaseLink* link =((PyPtr<BaseLink>*)self)->object;
    if (!setLinkePath(link,args))
    {
        SP_MESSAGE_ERROR( "argument type not supported" )
        PyErr_BadArgument();
    }
    Py_RETURN_NONE;
}

SP_CLASS_METHODS_BEGIN(Link)
SP_CLASS_METHOD(Link,setLinkPath)
SP_CLASS_METHODS_END

SP_CLASS_ATTRS_BEGIN(Link)
SP_CLASS_ATTR(Link,path)
SP_CLASS_ATTRS_END

SP_CLASS_TYPE_BASE_PTR_ATTR(Link,BaseLink)
