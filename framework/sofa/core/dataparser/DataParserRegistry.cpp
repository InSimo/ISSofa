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
#include "DataParserRegistry.h"
#include <cassert>

namespace sofa
{

namespace core
{

namespace dataparser
{

const std::vector<std::unique_ptr<DataParser>>& DataParserRegistry::getParsers()
{
    return parsers();
}
std::vector<std::unique_ptr<DataParser>>& DataParserRegistry::parsers()
{
    static std::vector<std::unique_ptr<DataParser>> instance = {};
    return instance;
}


bool DataParserRegistry::addParser(std::unique_ptr<DataParser> parser)
{
    assert([&]()
    {
        for (auto& existingParser : parsers())
        {
            if (existingParser->getId() == parser->getId())
                return false;
        }
        return true;
    }());

    parsers().emplace_back(std::move(parser));
    return true;
}


DataParser* DataParserRegistry::getParser(std::string parserName)
{
    DataParser* parser = getParser(generateDataParserId(parserName));
    if (parser == nullptr)
    {
        std::cerr << "DataParserRegistry: ERROR parser \"" << parserName << "\" NOT FOUND." << std::endl;
        std::cerr << "DataParserRegistry: Available parsers are:";
        for (const auto& existingParser : parsers())
        {
            std::cerr << " " << existingParser->getName();
        }
        std::cerr << std::endl;
    }
    return parser;
}

DataParser* DataParserRegistry::getParser(DataParser::ParserId id)
{
    for (auto& parser : parsers())
    {
        if (parser->getId() == id)
            return parser.get();
    }
    return nullptr;
}

} // namespace dataparser

} // namespace core

} // namespace sofa