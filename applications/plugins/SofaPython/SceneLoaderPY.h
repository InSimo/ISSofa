/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SCENELOADERPY_H
#define SCENELOADERPY_H

#include <sofa/SofaPython.h>
#include <sofa/simulation/common/SceneLoaderFactory.h>


#include <sofa/simulation/common/Visitor.h>
#include <string>
#include <map>

namespace sofa
{

namespace simulation
{


// The scene loader/exporter for python scene files
class SOFA_SOFAPYTHON_API SceneLoaderPY : public SceneLoader
{
public:
    /// Pre-loading check
    virtual bool canLoadFileExtension(const char *extension) override;
    /// Pre-saving check
    virtual bool canWriteFileExtension(const char *extension) override;

    /// load the file
    virtual Node::SPtr load(const char *filename, const std::vector<std::string>& sceneArguments) override;

    bool loadTestWithArguments(const char *filename, const std::vector<std::string>& arguments = {});

    /// write the file
    virtual void write(Node* node, const char *filename) override;

    /// get the file type description
    virtual std::string getFileTypeDesc() override;

    /// get the list of file extensions
    virtual void getExtensionList(ExtensionList* list) override;
};

///////////



/// Export the scene graph in Python format
void SOFA_SOFAPYTHON_API exportPython( Node* node, const char* fileName=NULL );



/// Visitor that exports all nodes/components in python
class SOFA_SOFAPYTHON_API PythonExporterVisitor : public Visitor
{

protected:

    std::ostream& m_out; ///< the output stream

    std::map<core::objectmodel::BaseNode*, std::string > m_mapNodeVariable; ///< gives a python variable name per node
    unsigned m_variableIndex; ///< unique index per node to garanty a unique variablename

public:

    PythonExporterVisitor(std::ostream& out) : Visitor(sofa::core::ExecParams::defaultInstance()), m_out(out), m_variableIndex(0) {}

    template<class T> void processObject( T obj, const std::string& nodeVariable );

    virtual Result processNodeTopDown(Node* node);
    virtual void processNodeBottomUp(Node* node);

    virtual const char* getClassName() const { return "PythonExporterVisitor"; }

};




} // namespace simulation

} // namespace sofa



#endif // SCENELOADERPY_H
