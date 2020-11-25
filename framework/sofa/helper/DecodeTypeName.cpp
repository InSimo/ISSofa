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
#include <sofa/helper/DecodeTypeName.h>
#ifdef __GNUC__
#include <sofa/helper/ps3/cxxabi.h>
#endif
#include <iostream>

namespace sofa
{

namespace helper
{

/// Helper method to decode the type name
std::string DecodeTypeName::decodeFullName(const std::type_info& t)
{
    std::string name;
#ifdef __GNUC__
    int status;
    /* size_t length; */ // although it should, length would not be filled in by the following call
    char* allocname = abi::__cxa_demangle(t.name(), 0, /*&length*/0, &status);
    if(allocname == 0)
    {
        std::cerr << "Unable to demangle symbol: " << t.name() << std::endl;
    }
    else
    {
        int length = 0;
        while(allocname[length] != '\0')
        {
            length++;
        }
        name.resize(length);
        for(int i=0; i<(int)length; i++)
            name[i] = allocname[i];
        free(allocname);
    }
#else
    name = t.name();
#endif
    return name;
}
/// Decode the type's name to a more readable form if possible
std::string DecodeTypeName::decodeTypeName(const std::type_info& t)
{
    std::string name;
    std::string realname = DecodeTypeName::decodeFullName(t);
    size_t len = realname.length();
    name.resize(len+1);
    size_t start = 0;
    size_t dest = 0;
    //char cprev = '\0';
    //sout << "name = "<<realname<<sendl;
    for (size_t i=0; i<len; i++)
    {
        char c = realname[i];
        if (c == ':') // && cprev == ':')
        {
            start = i+1;
        }
        else if (c == ' ' && i >= 5 && realname[i-5] == 'c' && realname[i-4] == 'l' && realname[i-3] == 'a' && realname[i-2] == 's' && realname[i-1] == 's')
        {
            start = i+1;
        }
        else if (c == ' ' && i >= 6 && realname[i-6] == 's' && realname[i-5] == 't' && realname[i-4] == 'r' && realname[i-3] == 'u' && realname[i-2] == 'c' && realname[i-1] == 't')
        {
            start = i+1;
        }
        else if (c != ':' && c != '_' && (c < 'a' || c > 'z') && (c < 'A' || c > 'Z') && (c < '0' || c > '9'))
        {
            // write result
            while (start < i)
            {
                name[dest++] = realname[start++];
            }
        }
        //cprev = c;
        //sout << "i = "<<i<<" start = "<<start<<" dest = "<<dest<<" name = "<<name<<sendl;
    }
    while (start < len)
    {
        name[dest++] = realname[start++];
    }
    name.resize(dest);
    return name;
}

/// Extract the class name (removing namespaces and templates)
std::string DecodeTypeName::decodeClassName(const std::type_info& t)
{
    std::string name;
    std::string realname = DecodeTypeName::decodeFullName(t);
    size_t len = realname.length();
    name.resize(len+1);
    size_t start = 0;
    size_t dest = 0;
    size_t i;
    //char cprev = '\0';
    //sout << "name = "<<realname<<sendl;
    for (i=0; i<len; i++)
    {
        char c = realname[i];
        if (c == '<') break;
        if (c == ':') // && cprev == ':')
        {
            start = i+1;
        }
        else if (c == ' ' && i >= 5 && realname[i-5] == 'c' && realname[i-4] == 'l' && realname[i-3] == 'a' && realname[i-2] == 's' && realname[i-1] == 's')
        {
            start = i+1;
        }
        else if (c == ' ' && i >= 6 && realname[i-6] == 's' && realname[i-5] == 't' && realname[i-4] == 'r' && realname[i-3] == 'u' && realname[i-2] == 'c' && realname[i-1] == 't')
        {
            start = i+1;
        }
        else if (c != ':' && c != '_' && (c < 'a' || c > 'z') && (c < 'A' || c > 'Z') && (c < '0' || c > '9'))
        {
            // write result
            while (start < i)
            {
                name[dest++] = realname[start++];
            }
        }
        //cprev = c;
    }

    while (start < i)
    {
        name[dest++] = realname[start++];
    }
    name.resize(dest);
    return name;
}

/// Extract the namespace (removing class name and templates)
std::string DecodeTypeName::decodeNamespaceName(const std::type_info& t)
{
    std::string name;
    std::string realname = DecodeTypeName::decodeFullName(t);
    size_t len = realname.length();
    size_t start = 0;
    size_t last = len-1;
    size_t i;
    for (i=0; i<len; i++)
    {
        char c = realname[i];
        if (c == ' ' && i >= 5 && realname[i-5] == 'c' && realname[i-4] == 'l' && realname[i-3] == 'a' && realname[i-2] == 's' && realname[i-1] == 's')
        {
            start = i+1;
        }
        else if (c == ' ' && i >= 6 && realname[i-6] == 's' && realname[i-5] == 't' && realname[i-4] == 'r' && realname[i-3] == 'u' && realname[i-2] == 'c' && realname[i-1] == 't')
        {
            start = i+1;
        }
        else if (c == ':' && (i<1 || realname[i-1]!=':'))
        {
            last = i-1;
        }
        else if (c != ':' && c != '_' && (c < 'a' || c > 'z') && (c < 'A' || c > 'Z') && (c < '0' || c > '9'))
        {
            // write result
            break;
        }
    }
    name = realname.substr(start, last-start+1);
    return name;
}

/// Decode the template name (removing namespaces and class name)
std::string DecodeTypeName::decodeTemplateName(const std::type_info& t)
{
    std::string name;
    std::string realname = DecodeTypeName::decodeFullName(t);
    size_t len = realname.length();
    name.resize(len+1);
    size_t start = 0;
    size_t dest = 0;
    size_t i = 0;
    //char cprev = '\0';
    while (i < len && realname[i]!='<')
        ++i;
    start = i+1; ++i;
    for (; i<len; i++)
    {
        char c = realname[i];
        //if (c == '<') break;
        if (c == ':') // && cprev == ':')
        {
            start = i+1;
        }
        else if (c == ' ' && i >= 5 && realname[i-5] == 'c' && realname[i-4] == 'l' && realname[i-3] == 'a' && realname[i-2] == 's' && realname[i-1] == 's')
        {
            start = i+1;
        }
        else if (c == ' ' && i >= 6 && realname[i-6] == 's' && realname[i-5] == 't' && realname[i-4] == 'r' && realname[i-3] == 'u' && realname[i-2] == 'c' && realname[i-1] == 't')
        {
            start = i+1;
        }
        else if (c != ':' && c != '_' && (c < 'a' || c > 'z') && (c < 'A' || c > 'Z') && (c < '0' || c > '9'))
        {
            // write result
            while (start <= i)
            {
                name[dest++] = realname[start++];
            }
        }
        //cprev = c;
    }
    while (start < i)
    {
        name[dest++] = realname[start++];
    }
    name.resize(dest);
    return name.substr(0, name.length()-1);
}

} // namespace helper

} // namespace sofa

