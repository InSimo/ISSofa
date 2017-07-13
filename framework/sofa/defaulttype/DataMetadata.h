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
#include <sofa/defaulttype/StructTypeInfo.h>
#include <sofa/defaulttype/EnumTypeInfo.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/helper/pair.h>
#include <string>

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
//      fills in the member value.         The structs should also have reflection abilities.
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
    virtual const int getId() const = 0;
    virtual const std::string& getName() const = 0;

}; // AbstractProperty



//////////////////////////// VirtualMetadata //////////////////////////////

template<class TMetadata>
class VirtualMetadata : public AbstractMetadata
{
public:
    typedef TMetadata Metadata;
    Metadata m_metadata;

    VirtualMetadata(Metadata p) : m_metadata(std::move(p)) {};
    VirtualMetadata() {};

    const int getId() const override 
    {
        return VirtualTypeInfo<Metadata>::get()->typeInfoID();
    }

    const std::string& getName() const override
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

struct Displayed
{
public:
    bool displayed;

    Displayed(bool displayed) : displayed(displayed) {};
    Displayed() : displayed(true) {};

    SOFA_STRUCT_DECL(Displayed, displayed);
    SOFA_STRUCT_STREAM_METHODS(Displayed);
    SOFA_STRUCT_COMPARE_METHOD(Displayed);
};

struct ReadOnly
{
public:
    bool readOnly;

    ReadOnly(bool readOnly) : readOnly(readOnly) {};
    ReadOnly() : readOnly(true) {};

    SOFA_STRUCT_DECL(ReadOnly, readOnly);
    SOFA_STRUCT_STREAM_METHODS(ReadOnly);
    SOFA_STRUCT_COMPARE_METHOD(ReadOnly);
};

template<typename T> // T should be the type of the data
struct Range
{
public:
    helper::pair<T, T> range;

    Range(helper::pair<T, T> range) : range(range) {};
    Range() {};

    SOFA_STRUCT_DECL(Range, range);
    SOFA_STRUCT_STREAM_METHODS(Range);
    SOFA_STRUCT_COMPARE_METHOD(Range);
};


template<typename T> // T should be the type of the data
struct PossibleValues
{
public:
    helper::vector<T> possibleValues;

    PossibleValues(helper::vector<T> possibleValues) : possibleValues(possibleValues) {};
    PossibleValues() {};

    SOFA_STRUCT_DECL(PossibleValues<T>, possibleValues);
    SOFA_STRUCT_STREAM_METHODS(PossibleValues<T>);
    SOFA_STRUCT_COMPARE_METHOD(PossibleValues<T>);
};


//template<int kg, int m, int s, int A, int K, int mol, int cd>
//struct Units
//{
//public:
//    helper::fixed_array<int, 7> units;
//
//    Units(helper::fixed_array<int, 7> units) : units(units) {};
//    Units() : units(helper::fixed_array<int, 7>{0, 0, 0, 0, 0, 0, 0}) {};
//
//    using Units_t = Units<kg, m, s, A, K, mol, cd>;
//    SOFA_STRUCT_DECL(Units_t, units);
//    SOFA_STRUCT_STREAM_METHODS(Units_t);
//    SOFA_STRUCT_COMPARE_METHOD(Units_t);
//};

struct Units
{
public:
    helper::fixed_array<int, 7> units;

    Units(helper::fixed_array<int, 7> units) : units(units) {};
    Units() {};

    SOFA_STRUCT_DECL(Units, units);
    SOFA_STRUCT_STREAM_METHODS(Units);
    //SOFA_STRUCT_COMPARE_METHOD(Units);    // ISSUE !
};

struct HelpMsg
{
public:
    std::string helpMsg;

    HelpMsg(std::string helpMsg) : helpMsg(helpMsg) {};

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
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::meta::HelpMsg);

template<typename T>
struct sofa::defaulttype::DataTypeInfo<sofa::meta::Range<T> >               \
    : public sofa::defaulttype::StructTypeInfo<sofa::meta::Range<T> > {};

template<typename T>
struct sofa::defaulttype::DataTypeInfo<sofa::meta::PossibleValues<T> >               \
    : public sofa::defaulttype::StructTypeInfo<sofa::meta::PossibleValues<T> > {};

//template<int kg, int m, int s, int A, int K, int mol, int cd>
//struct sofa::defaulttype::DataTypeInfo<sofa::meta::Units<kg, m, s, A, K, mol, cd> >               \
//    : public sofa::defaulttype::StructTypeInfo<sofa::meta::Units<kg, m, s, A, K, mol, cd> > {};



#endif // SOFA_DEFAULTTYPE_DATAMETADATA_H
