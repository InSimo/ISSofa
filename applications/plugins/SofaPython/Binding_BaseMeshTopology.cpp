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

#include "Binding_BaseMeshTopology.h"
#include "Binding_Topology.h"

#include <sofa/core/topology/BaseMeshTopology.h>
using namespace sofa::core::topology;

extern "C" PyObject * BaseMeshTopology_getNbEdges(PyObject *self, PyObject * /*args*/)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyInt_FromLong(obj->getNbEdges());
}

extern "C" PyObject * BaseMeshTopology_getNbTriangles(PyObject *self, PyObject * /*args*/)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyInt_FromLong(obj->getNbTriangles());
}

extern "C" PyObject * BaseMeshTopology_getNbQuads(PyObject *self, PyObject * /*args*/)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyInt_FromLong(obj->getNbQuads());
}

extern "C" PyObject * BaseMeshTopology_getNbTetrahedra(PyObject *self, PyObject * /*args*/)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyInt_FromLong(obj->getNbTetrahedra());
}

extern "C" PyObject * BaseMeshTopology_getNbHexahedra(PyObject *self, PyObject * /*args*/)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    return PyInt_FromLong(obj->getNbHexahedra());
}

PyObject * indicesVectorToPyList(const sofa::helper::vector<BaseMeshTopology::index_type>& vector)
{
    PyObject *list = PyList_New(vector.size());
    for (unsigned int i = 0u; i < vector.size(); i++)
        PyList_SetItem(list, i, PyInt_FromLong(vector[i]));
    return list;
}

extern "C" PyObject * BaseMeshTopology_getVerticesAroundVertex(PyObject *self, PyObject * args)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    int pointId;
    if (!PyArg_ParseTuple(args, "i",&pointId))
        Py_RETURN_NONE;
    return indicesVectorToPyList(obj->getVerticesAroundVertex(BaseMeshTopology::PointID(pointId)));
}

extern "C" PyObject * BaseMeshTopology_getEdgesAroundVertex(PyObject *self, PyObject * args)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    int pointId;
    if (!PyArg_ParseTuple(args, "i",&pointId))
        Py_RETURN_NONE;
    return indicesVectorToPyList(obj->getEdgesAroundVertex(BaseMeshTopology::PointID(pointId)));
}

extern "C" PyObject * BaseMeshTopology_getTrianglesAroundVertex(PyObject *self, PyObject * args)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    int pointId;
    if (!PyArg_ParseTuple(args, "i",&pointId))
        Py_RETURN_NONE;
    return indicesVectorToPyList(obj->getTrianglesAroundVertex(BaseMeshTopology::PointID(pointId)));
}


extern "C" PyObject * BaseMeshTopology_getEdgesInTriangle(PyObject *self, PyObject * args)
{
    BaseMeshTopology* obj=BaseMeshTopology::DynamicCast(((PySPtr<Base>*)self)->object.get());
    int triangleId;
    if (!PyArg_ParseTuple(args, "i",&triangleId))
        Py_RETURN_NONE;
    sofa::helper::fixed_array<BaseMeshTopology::EdgeID,3> eint = obj->getEdgesInTriangle(BaseMeshTopology::TriangleID(triangleId));
    PyObject *list = PyList_New(3);
    for (unsigned int i = 0u; i < 3; i++)
        PyList_SetItem(list, i, PyInt_FromLong(eint[i]));
    return list;

}


SP_CLASS_METHODS_BEGIN(BaseMeshTopology)
SP_CLASS_METHOD(BaseMeshTopology,getNbEdges)
SP_CLASS_METHOD(BaseMeshTopology,getNbTriangles)
SP_CLASS_METHOD(BaseMeshTopology,getNbQuads)
SP_CLASS_METHOD(BaseMeshTopology,getNbTetrahedra)
SP_CLASS_METHOD(BaseMeshTopology,getNbHexahedra)
SP_CLASS_METHOD(BaseMeshTopology,getVerticesAroundVertex)
SP_CLASS_METHOD(BaseMeshTopology,getEdgesAroundVertex)
SP_CLASS_METHOD(BaseMeshTopology,getTrianglesAroundVertex)
SP_CLASS_METHOD(BaseMeshTopology,getEdgesInTriangle)
SP_CLASS_METHODS_END


SP_CLASS_TYPE_SPTR(BaseMeshTopology,BaseMeshTopology,Topology)



