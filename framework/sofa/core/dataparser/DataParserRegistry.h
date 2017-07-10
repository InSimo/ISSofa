/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_CORE_DATAPARSERREGISTRY_H
#define SOFA_CORE_DATAPARSERREGISTRY_H

#include "DataParser.h"
#include <sofa/SofaFramework.h>
#include <memory>
#include <vector>

namespace sofa
{

namespace core
{

namespace dataparser
{

class SOFA_CORE_API DataParserRegistry
{
public:
    static bool addParser(std::unique_ptr<DataParser> parser);
    static DataParser* getParser(std::string parserName);
    static DataParser* getParser(DataParser::ParserId id);

private:
    static std::vector<std::unique_ptr<DataParser>> m_parsers;
};

} // namespace dataparser

} // namespace core

} // namespace sofa

#endif //SOFA_CORE_DATAPARSERREGISTRY_H
