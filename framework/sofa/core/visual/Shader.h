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
#ifndef SOFA_CORE_VISUAL_SHADER_H
#define SOFA_CORE_VISUAL_SHADER_H

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace core
{

namespace visual
{

/**
 *  \brief A basic interface to define a Shader for different system (OpenGL, DirectX, ...).
 *
 *
 *
 */
class SOFA_CORE_API Shader : public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((Shader), ((objectmodel::BaseObject)));
protected:
	Shader() {};
    /// Destructor
    virtual ~Shader() { }
	
private:
	Shader(const Shader& n) ;
	Shader& operator=(const Shader& n) ;
	
public:
    /// Start the shader
    virtual void start() = 0;
    /// Stop the shader
    virtual void stop() = 0;
    ///Tells if it must be activated automatically(value false : the visitor will switch the shader)
    ///or manually (value true : useful when another component wants to use it for itself only)
    virtual bool isActive() = 0;
};

/**
 *  \brief A basic interface to define an element to be used with a Shader.
 */
class SOFA_CORE_API ShaderElement: public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((ShaderElement), ((objectmodel::BaseObject)));
    enum ShaderElementType { SE_NONE = 0, SE_TEXTURE, SE_MACRO, SE_VARIABLE, SE_ATTRIBUTE };
    enum ShaderValueType { SV_NONE = 0, SV_FLOAT, SV_DOUBLE, SV_INT, SV_UNSIGNED_INT };
protected:
	ShaderElement() {};
    /// Destructor
    virtual ~ShaderElement() { }
	
private:
	ShaderElement(const ShaderElement& n) ;
	ShaderElement& operator=(const ShaderElement& n) ;
	
public:
    /// Returns the type of shader element (texture, macro, variable, or attribute)
    virtual ShaderElementType getSEType() const = 0;
    // Returns the ID of the shader element
    virtual const std::string& getSEID() const = 0;
    // Returns the value of the shader element
    virtual const core::objectmodel::BaseData* getSEValue() const = 0;
    // Returns the value of the shader element
    virtual core::objectmodel::BaseData* getSEValue() = 0;
    // For attributes : return the number of values per vertex
    virtual int getSESizePerVertex() { return 0; }
    // For attributes : return the number of values
    virtual int getSETotalSize() { return 0; }
    // For attributes : return the type of values
    virtual ShaderValueType getSEValueType() const { return SV_NONE; }
    // For attributes : return the pointer to the vector of values (castable to the type returned by getSEValueType)
    virtual const void* getSEValuePointer() { return NULL; }
};

} // namespace visual

} // namespace core

} // namespace sofa

#endif //SOFA_CORE_VISUAL_SHADER_H
