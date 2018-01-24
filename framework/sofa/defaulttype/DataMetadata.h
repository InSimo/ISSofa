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

#ifndef SOFA_DEFAULTTYPE_DATAMETADATA_H
#define SOFA_DEFAULTTYPE_DATAMETADATA_H

#include <iostream>
#include <sofa/helper/fixed_array.h>
#include <sofa/defaulttype/VirtualTypeInfo.h>
#include <sofa/defaulttype/StructTypeInfo.h>
#include <sofa/defaulttype/EnumTypeInfo.h>
#include <sofa/helper/pair.h>
#include <string>
#include <array>

namespace sofa
{
namespace defaulttype
{


//////////////////////////////////////////////////////////////////////////////////////
//
//  This header aims to give metadata information to the Data.
//
//  Convention for the metadata :
//      each piece of metadata is a struct with a single member value named after the struct's name
//      and with two constructors, one of them is the default constructor with no argument and the other
//      fills in the member value. The boolean metadata have "value = true" by their presence in the data.metadata.
//      The structs should also have reflection abilities.
//  eg : 
//      struct Meta1 {
//        T meta1;
//        Units(helper::fixed_array<int, 7> units) : units(units) {};
//        Units() {};
//       }
//
//  Examples of usage for the metadata mechanism :
//      class TestOnData : public core::objectmodel::BaseObject {
//      public:
//          SOFA_CLASS(TestOnData, core::objectmodel::BaseObject);                                  // define initData()
//          Data<double > d_test;
//          TestOnData() : d_test(initData("test", "helpMsg").addMeta(meta::Displayed()))  {}
//          void doStuff() {
//              d_test.addMeta(meta::Range<double>(helper::pair<double, double>{0, 10}));
//              meta::Range<double>* dTestRange;
//              if (d_test.getMeta(dTestRange))  {
//                  dTestRange->range = helper::pair<double, double>{ 1,11 };}}
//      }
//
//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////// AbstractMetadata //////////////////////////////

class AbstractMetadata
{
public:
    virtual int getId() const = 0;
    virtual std::string getName() const = 0;

    virtual ~AbstractMetadata() {}
}; // AbstractProperty



//////////////////////////// VirtualMetadata //////////////////////////////

template<class TMetadata>
class VirtualMetadata : public AbstractMetadata
{
public:
    typedef TMetadata Metadata;
    Metadata m_metadata;

    VirtualMetadata(Metadata p) : m_metadata(std::move(p)) {}
    VirtualMetadata() {}

    int getId() const override
    {
        return VirtualTypeInfo<Metadata>::get()->typeInfoID();
    }

    std::string getName() const override
    {
        return VirtualTypeInfo<Metadata>::get()->name();
    }

    Metadata& getMeta()
    {
        return m_metadata;
    }

}; // VirtualMetadata

} // namespace defaulttype
} // namespace sofa


//////////////////////////// Definition of the metadata //////////////////////////////


namespace sofa
{
namespace meta
{

struct Displayed   // should the data be displayed in the GUI ?
{
public:
    constexpr Displayed() {}

    SOFA_STRUCT_DECL(Displayed, SOFA_EMPTY);
    SOFA_STRUCT_STREAM_METHODS(Displayed);
    bool operator==(const Displayed& rhs) const
    {
        return true;
    }
};

struct ReadOnly   // is the data ReadOnly ?
{
public:
    constexpr ReadOnly() {}

    SOFA_STRUCT_DECL(ReadOnly, SOFA_EMPTY);
    SOFA_STRUCT_STREAM_METHODS(ReadOnly);
    bool operator==(const ReadOnly& rhs) const
    {
        return true;
    }
};

struct ForDebug   // is it a parameter only aimed at debugging ?
{
public:
    constexpr ForDebug() {}

    SOFA_STRUCT_DECL(ForDebug, SOFA_EMPTY);
    SOFA_STRUCT_STREAM_METHODS(ForDebug);
    bool operator==(const ForDebug& rhs) const
    {
        return true;
    }
};

template<typename T> // T should be the type of the data
struct Range   // what is the range of values acceptable for this data ?
{
public:
    helper::pair<T, T> range;

    constexpr Range(helper::pair<T, T> range) : range(range) {}
    Range() {}

    SOFA_STRUCT_DECL(Range, range);
    SOFA_STRUCT_STREAM_METHODS(Range);
    SOFA_STRUCT_COMPARE_METHOD(Range);
};


template<typename T> // T should be the type of the data
struct PossibleValues   // what are the values acceptable for this data ?
{
public:
    helper::vector<T> possibleValues;

    constexpr PossibleValues(const helper::vector<T>& possibleValues) : possibleValues(possibleValues) {}
    PossibleValues() {}

    SOFA_STRUCT_DECL(PossibleValues<T>, possibleValues);
    SOFA_STRUCT_STREAM_METHODS(PossibleValues<T>);
    SOFA_STRUCT_COMPARE_METHOD(PossibleValues<T>);
};


struct Units   // what are the units of the data data ?
{
public:
    std::array<int, 7> units;

    constexpr Units(const std::array<int, 7>& units) : units(units) {}
    Units() {}

    SOFA_STRUCT_DECL(Units, units);
    inline friend std::ostream& operator<<(std::ostream& os, const Units& s) { return os; }
    inline friend std::istream& operator>> (std::istream& in, Units& s) { return in; }
};


struct HelpMsg   // helpMsg stored in the data ?
{
public:
    std::string helpMsg;

    HelpMsg(std::string helpMsg) : helpMsg(helpMsg) {}

    SOFA_STRUCT_DECL(HelpMsg, helpMsg);
    SOFA_STRUCT_STREAM_METHODS(HelpMsg);
    SOFA_STRUCT_COMPARE_METHOD(HelpMsg);
};


} // namespace meta
} // namespace sofa


//////////////////////////// Introspection //////////////////////////////

// introspection of the Property structs
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::meta::ReadOnly);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::meta::Displayed);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::meta::ForDebug);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::meta::HelpMsg);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::meta::Units);

namespace sofa
{
namespace defaulttype
{

inline std::ostream& operator<<(std::ostream& os, const std::array<int, 7>& s)
{
    std::cout << "DataMetadata.h : the operator<< you are trying to use is not correctly implemented" << std::endl;
    return os;
}
inline std::istream& operator >> (std::istream& in, std::array<int, 7>& s)
{
    std::cout << "DataMetadata.h : the operator>> you are trying to use is not correctly implemented" << std::endl;
    return in;
}

template<typename T>
struct DataTypeInfo<sofa::meta::Range<T> > : public StructTypeInfo<sofa::meta::Range<T> > {};

template<typename T>
struct DataTypeInfo<sofa::meta::PossibleValues<T> > : public StructTypeInfo<sofa::meta::PossibleValues<T> > {};

} // namespace defaulttype

} // namespace sofa





#endif // SOFA_DEFAULTTYPE_DATAMETADATA_H
