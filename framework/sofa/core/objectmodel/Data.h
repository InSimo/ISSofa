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
#ifndef SOFA_CORE_OBJECTMODEL_DATAFIELD_H
#define SOFA_CORE_OBJECTMODEL_DATAFIELD_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/SofaFramework.h>
#include <sofa/core/objectmodel/BaseData.h>
#include <sofa/defaulttype/VirtualTypeInfo.h>
#include <sofa/helper/accessor.h>
#include <sofa/helper/vector.h>
#include <sofa/core/dataparser/DataParserRegistry.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <map>
#include <memory>

namespace sofa
{

namespace core
{

namespace objectmodel
{

/** \brief Abstract base class template for Data. */
template < class T >
class TData : public BaseData
{
public:
    typedef T value_type;

    /// @name Class reflection system
    /// @{
    static std::string templateName(const TData<T>* = NULL)
    {
        T* ptr = NULL;
        return BaseData::typeName(ptr);
    }
    /// @}

    explicit TData(const BaseInitData& init)
        : BaseData(init), parentData(initLink("parentSameType", "Linked Data in case it stores exactly the same type of Data, and efficient copies can be made (by value or by sharing pointers with Copy-on-Write)"))
    {
        addMetaFromDataType();
    }

    TData( const char* helpMsg=0, bool isDisplayed=true, bool isReadOnly=false)
        : BaseData(helpMsg, isDisplayed, isReadOnly), parentData(initLink("parentSameType", "Linked Data in case it stores exactly the same type of Data, and efficient copies can be made (by value or by sharing pointers with Copy-on-Write)"))
    {
    }

    virtual ~TData()
    {}

    inline void printValue(std::ostream& out) const;
    inline std::string getValueString() const;
    inline std::string getValueTypeString() const; // { return std::string(typeid(m_value).name()); }

    static const sofa::defaulttype::AbstractTypeInfo* GetValueTypeInfo()
    {
        return sofa::defaulttype::VirtualTypeInfo<T>::get();
    }
    
    /// Get info about the value type of the associated variable
    virtual const sofa::defaulttype::AbstractTypeInfo* getValueTypeInfo() const
    {
        return GetValueTypeInfo();
    }

    virtual const T& virtualGetValue() const = 0;
    virtual void virtualSetValue(const T& v) = 0;
    virtual void virtualSetLink(const BaseData& bd) = 0;
    virtual T* virtualBeginEdit() = 0;
    virtual void virtualEndEdit() = 0;

    /// Get current value as a void pointer (use getValueTypeInfo to find how to access it)
    virtual const void* getValueVoidPtr() const
    {
        return &(virtualGetValue());
    }

    /// Begin edit current value as a void pointer (use getValueTypeInfo to find how to access it)
    virtual void* beginEditVoidPtr()
    {
        return virtualBeginEdit();
    }

    /// End edit current value as a void pointer (use getValueTypeInfo to find how to access it)
    virtual void endEditVoidPtr()
    {
        virtualEndEdit();
    }

    /** Try to read argument value from an input stream.
    Return false if failed
     */
    virtual bool read( const std::string& s )
    {
        if (s.empty())
            return false;

        auto* parser = dataparser::DataParserRegistry::getParser(std::string("raw_text_data_parser"));
        auto error_code = parser->toData(s, virtualBeginEdit(), getValueTypeInfo());
        virtualEndEdit();
        if(error_code)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    virtual bool isCounterValid() const {return true;}

    bool copyValue(const TData<T>* parent)
    {
        virtualSetValue(parent->virtualGetValue());
        return true;
    }

    virtual bool copyValue(const BaseData* parent)
    {
        const TData<T>* p = dynamic_cast<const TData<T>*>(parent);
        if (p)
        {
            virtualSetValue(p->virtualGetValue());
            return true;
        }
        return BaseData::copyValue(parent);
    }


    virtual bool validParent(BaseData* parent)
    {
        if (dynamic_cast<TData<T>*>(parent))
            return true;
        return BaseData::validParent(parent);
    }

protected:

    void addMetaFromDataType()
    {
        std::map<int, defaulttype::AbstractMetadata*> metadataTuple = defaulttype::DataTypeInfo<T>::getMetadata();
        m_metadata.insert(metadataTuple.begin(), metadataTuple.end());
    }

    BaseLink::InitLink<TData<T> >
    initLink(const char* name, const char* help)
    {
        return BaseLink::InitLink<TData<T> >(this, name, help);
    }

    void doSetParent(BaseData* parent)
    {
        parentData.set(dynamic_cast<TData<T>*>(parent));
        BaseData::doSetParent(parent);
    }

    bool updateFromParentValue(const BaseData* parent)
    {
        if (parent == parentData.get())
        {
            //virtualSetValue(parentData->virtualGetValue());
            virtualSetLink(*parentData.get());
            return true;
        }
        else
            return BaseData::updateFromParentValue(parent);
    }

    SingleLink<TData<T>,TData<T>, BaseLink::FLAG_DATALINK|BaseLink::FLAG_OWNERDATA|BaseLink::FLAG_DUPLICATE> parentData;
};

template <class T, bool COW>
class DataValue;

template <class T>
class DataValue<T, false>
{
    T data;
public:

    DataValue()
        : data(T())// BUGFIX (Jeremie A.): Force initialization of basic types to 0 (bool, int, float, etc).
    {
    }

    explicit DataValue(const T &value)
        : data(value)
    {
    }

    DataValue(const DataValue& dc)
        : data(dc.getValue())
    {
    }

    DataValue& operator=(const DataValue& dc )
    {
        data = dc.getValue();
        return *this;
    }

    T* beginEdit() { return &data; }
    void endEdit() {}
    const T& getValue() const { return data; }
    void setValue(const T& value)
    {
        data = value;
    }
    void release()
    {
    }
//    T& value() { return data; }
};


template <class T>
class DataValue<T, true>
{
    std::shared_ptr<T> ptr;
public:

    DataValue()
        : ptr(new T(T())) // BUGFIX (Jeremie A.): Force initialization of basic types to 0 (bool, int, float, etc).
    {
    }

    explicit DataValue(const T& value)
        : ptr(new T(value))
    {
    }

    DataValue(const DataValue& dc)
        : ptr(dc.ptr)
    {
    }

    ~DataValue()
    {
    }

    DataValue& operator=(const DataValue& dc )
    {
        //avoid self reference
        if(&dc != this)
        {
            ptr = dc.ptr;
        }

        return *this;
    }

    T* beginEdit()
    {
        if(!ptr.unique())
        {
            ptr.reset(new T(*ptr));
        }
        return ptr.get();
    }

    void endEdit()
    {
    }

    const T& getValue() const
    {
        return *ptr;
    }

    void setValue(const T& value)
    {
        if(!ptr.unique())
        {
            ptr.reset(new T(value));
        }
        else
        {
            *ptr = value;
        }
    }

    void release()
    {
        ptr.reset();
    }

//    T& value()
//    {
//        return *beginEdit();
//    }
};


/** \brief Container that holds a variable for a component.
 *
 * This is a fundamental class template in Sofa.  Data are used to encapsulated
 * member variables of Sofa components (i.e. classes that somehow inherit from
 * Base) in order to access them dynamically and generically: briefly, Data can
 * be retrieved at run-time by their name, and they can be assigned a value from
 * a string, or be printed as a string.
 *
 * More concretely, from the perspective of XML scene files, each Data declared
 * in a component corresponds to an attribute of this component.
 *
 * ### Example ###
 *
 * If a component *Foo* has a boolean parameter *bar*, it does not simply declares it
 * as `bool m_bar`, but rather like this:
 *
 * \code
 *  Data<bool> d_bar;
 * \endcode
 *
 * Then, this Data must be initialized to provide its name and default value.
 * This is typically done in the initialization list of *each constructor* of
 * the component, using the helper function Base::initData():
 *
 * \code
 * Foo::Foo(): d_bar(initData(&d_bar, true, "bar", "Here is a little description of this Data.")) {
 *     // ...
 * }
 * \endcode
 *
 * And this Data can be assigned a value in XML scene files like so:
 * \code
 * <Foo bar="false"/>
 * \endcode
 */
template < class T = void* >
class Data : public TData<T>
{
public:

    /// @name Class reflection system
    /// @{
    static std::string templateName(const Data<T>* = NULL)
    {
        T* ptr = NULL;
        return BaseData::typeName(ptr);
    }
    /// @}

    /// @name Construction / destruction
    /// @{

    /// This internal class is used by the initData() methods to store initialization parameters of a Data
    class InitData : public BaseData::BaseInitData
    {
    public:
        InitData() : value(T()) {}
        InitData(const T& v) : value(v) {}
        InitData(const BaseData::BaseInitData& i) : BaseData::BaseInitData(i), value(T()) {}

        T value;
    };

    /** \copydoc BaseData(const BaseData::BaseInitData& init) */
    explicit Data(const BaseData::BaseInitData& init)
        : TData<T>(init)
        , shared(NULL)
    {
    }

    /** \copydoc Data(const BaseData::BaseInitData&) */
    explicit Data(const InitData& init)
        : TData<T>(init)
        , m_values()
        , shared(NULL)
    {
        m_values[DDGNode::currentAspect()] = ValueType(init.value);
    }

    /** \copydoc BaseData(const char*, bool, bool) */
    explicit Data( const char* helpMsg, bool isDisplayed=true, bool isReadOnly=false)
        : TData<T>(helpMsg, isDisplayed, isReadOnly)
        , m_values()
        , shared(NULL)
    {
        ValueType val;
        m_values.assign(val);
    }

    /** \copydoc BaseData(const char*, bool, bool)
     *  \param value The default value.
     */
    Data( const T& value, const char* helpMsg, bool isDisplayed=true, bool isReadOnly=false)
        : TData<T>(helpMsg, isDisplayed, isReadOnly)
        , m_values()
        , shared(NULL)
    {
        m_values[DDGNode::currentAspect()] = ValueType(value);
    }

    /// Destructor.
    virtual ~Data()
    {}

    /// @}

    /// @name Simple edition and retrieval API
    /// @{

    inline T* beginEdit(const core::ExecParams* params = 0)
    {
        size_t aspect = DDGNode::currentAspect(params);
        this->updateIfDirty(params);
        ++this->m_counters[aspect];
        this->m_isSets[aspect] = true;
        BaseData::setDirtyOutputs(params);
        return m_values[aspect].beginEdit();
    }

    /// BeginEdit method if it is only to write the value
    inline T* beginWriteOnly(const core::ExecParams* params = 0)
    {
        size_t aspect = DDGNode::currentAspect(params);
        ++this->m_counters[aspect];
        this->m_isSets[aspect] = true;
        BaseData::setDirtyOutputs(params);
        return m_values[aspect].beginEdit();
    }

    inline void endEdit(const core::ExecParams* params = 0)
    {
        m_values[DDGNode::currentAspect(params)].endEdit();
    }

    /// @warning writeOnly (the Data is not updated before being set)
    inline void setValue(const T& value)
    {
        *beginWriteOnly()=value;
        endEdit();
    }

    /// @warning writeOnly (the Data is not updated before being set)
    inline void setValue(const core::ExecParams* params, const T& value)
    {
        *beginWriteOnly(params)=value;
        endEdit(params);
    }

    inline const T& getValue(const core::ExecParams* params = 0) const
    {
        this->updateIfDirty(params);
        return m_values[DDGNode::currentAspect(params)].getValue();
    }

    void copyAspect(int destAspect, int srcAspect)
    {
        m_values[destAspect] = m_values[srcAspect];
        BaseData::copyAspect(destAspect, srcAspect);
    }

    void releaseAspect(int aspect)
    {
        m_values[aspect].release();
    }
    /// @}

    /// @name Virtual edition and retrieval API (for generic TData parent API, deprecated)
    /// @{

    virtual const T& virtualGetValue() const { return getValue(); }
    virtual void virtualSetValue(const T& v) { setValue(v); }

    virtual void virtualSetLink(const BaseData& bd)
    {
        const Data<T>* d = dynamic_cast< const Data<T>* >(&bd);
        if (d)
        {
            size_t aspect = DDGNode::currentAspect();
            this->m_values[aspect] = d->m_values[aspect];
            //FIX: update counter
            ++this->m_counters[aspect];
            this->m_isSets[aspect] = true;
            BaseData::setDirtyOutputs();
        }
    }

    virtual T* virtualBeginEdit() { return beginEdit(); }
    virtual void virtualEndEdit() { endEdit(); }


    /// @}

    inline friend std::ostream & operator << (std::ostream &out, const Data& df)
    {
        out<<df.getValue();
        return out;
    }

    inline bool operator ==( const T& value ) const
    {
        return getValue()==value;
    }

    inline bool operator !=( const T& value ) const
    {
        return getValue()!=value;
    }

    inline void operator =( const T& value )
    {
        this->setValue(value);
    }

protected:

    typedef DataValue<T, sofa::defaulttype::DataTypeInfo<T>::CopyOnWrite> ValueType;

    /// Value
    helper::fixed_array<ValueType, SOFA_DATA_MAX_ASPECTS> m_values;

public:
    mutable void* shared;

private:
    Data(const Data& );
    Data& operator=(const Data& );
};

/// Specialization for reading strings
template<>
bool TData<std::string>::read( const std::string& str );


/// Specialization for reading booleans
template<>
bool TData<bool>::read( const std::string& str );


/// General case for printing default value
template<class T>
inline
void TData<T>::printValue( std::ostream& out) const
{
    out << virtualGetValue() << " ";
}

/// General case for printing default value
template<class T>
inline
std::string TData<T>::getValueString() const
{
    std::ostringstream out;
    out << virtualGetValue();
    return out.str();
}

template<class T>
inline
std::string TData<T>::getValueTypeString() const
{
    return BaseData::typeName(&virtualGetValue());
}


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_CORE)

extern template class SOFA_CORE_API TData< std::string >;
extern template class SOFA_CORE_API Data< std::string >;
extern template class SOFA_CORE_API TData< sofa::helper::vector<std::string> >;
extern template class SOFA_CORE_API Data< sofa::helper::vector<std::string> >;
extern template class SOFA_CORE_API TData< bool >;
extern template class SOFA_CORE_API Data< bool >;

#endif

template< typename TData > 
struct DataTraits
{
    typedef typename TData::value_type value_type;///<Wraps the type contained in the Data into an opaque type named value_type
};

} // namespace objectmodel

} // namespace core

// Overload helper::ReadAccessor and helper::WriteAccessor

namespace helper
{


/// @warning the Data is updated (if needed) only by the Accessor constructor
template<class T>
class ReadAccessor< core::objectmodel::Data<T> > : public ReadAccessor<T>
{
public:
    typedef ReadAccessor<T> Inherit;
    typedef core::objectmodel::Data<T> data_container_type;
    typedef T container_type;

//protected:
//    const data_container_type* data;
public:
    ReadAccessor(const data_container_type& d) : Inherit(d.getValue())/*, data(&d)*/ {}
    ReadAccessor(const data_container_type* d) : Inherit(d->getValue())/*, data(d)*/ {}
    ReadAccessor(const core::ExecParams* params, const data_container_type& d) : Inherit(d.getValue(params))/*, data(&d)*/ {}
    ReadAccessor(const core::ExecParams* params, const data_container_type* d) : Inherit(d->getValue(params))/*, data(d)*/ {}
};

/// Read/Write Accessor.
/// The Data is updated before being accessible.
/// This means an expensive chain of Data link and Engine updates can be called
/// For a pure write only Accessor, prefer WriteOnlyAccessor
/// @warning the Data is updated (if needed) only by the Accessor constructor
template<class T>
class WriteAccessor< core::objectmodel::Data<T> > : public WriteAccessor<T>
{
public:
    typedef WriteAccessor<T> Inherit;
    typedef core::objectmodel::Data<T> data_container_type;
    typedef typename Inherit::container_type container_type;

	// these are forbidden (until c++11 move semantics) as they break
	// RAII encapsulation. the reference member 'data' prevents them
	// anyways, but the intent is more obvious like this.
	WriteAccessor(const WriteAccessor& );
	WriteAccessor& operator=(const WriteAccessor& );

protected:
    data_container_type& data;
    const core::ExecParams* dparams;

    /// @internal used by WriteOnlyAccessor
    WriteAccessor( container_type* c, data_container_type& d, const core::ExecParams* params=NULL ) : Inherit(*c), data(d), dparams(params) {}

public:
    WriteAccessor(data_container_type& d) : Inherit(*d.beginEdit()), data(d), dparams(NULL) {}
    WriteAccessor(data_container_type* d) : Inherit(*d->beginEdit()), data(*d), dparams(NULL) {}
    WriteAccessor(const core::ExecParams* params, data_container_type& d) : Inherit(*d.beginEdit(params)), data(d), dparams(params) {}
    WriteAccessor(const core::ExecParams* params, data_container_type* d) : Inherit(*d->beginEdit(params)), data(*d), dparams(params) {}
    ~WriteAccessor() { if (dparams) data.endEdit(dparams); else data.endEdit(); }
};



/** @brief The WriteOnlyAccessor provides an access to the Data without triggering an engine update.
 * This should be the prefered writeAccessor for most of the cases as it avoids uncessary Data updates.
 * @warning read access to the Data is NOT up-to-date
 */
template<class T>
class WriteOnlyAccessor< core::objectmodel::Data<T> > : public WriteAccessor< core::objectmodel::Data<T> >
{
public:
    typedef WriteAccessor< core::objectmodel::Data<T> > Inherit;
    typedef typename Inherit::data_container_type data_container_type;
    typedef typename Inherit::container_type container_type;

    // these are forbidden (until c++11 move semantics) as they break
    // RAII encapsulation. the reference member 'data' prevents them
    // anyways, but the intent is more obvious like this.
    WriteOnlyAccessor(const WriteOnlyAccessor& );
    WriteOnlyAccessor& operator=(const WriteOnlyAccessor& );

    WriteOnlyAccessor(data_container_type& d) : Inherit( d.beginWriteOnly(), d ) {}
    WriteOnlyAccessor(data_container_type* d) : Inherit( d->beginWriteOnly(), *d ) {}
    WriteOnlyAccessor(const core::ExecParams* params, data_container_type& d) : Inherit( d.beginWriteOnly(), d, params ) {}
    WriteOnlyAccessor(const core::ExecParams* params, data_container_type* d) : Inherit( d->beginWriteOnly(), *d, params ) {}
};


template<class TData>
using is_base_of_data = std::is_base_of< sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type >, TData >;

template<class TData>
using enable_if_is_base_of_data = std::enable_if< is_base_of_data<TData>::value >;


/**
* \defgroup Write/Read helper methods to construct accessor objects for Data types
*           these methods work also for types that derive from Data like BaseTopologyData, or DataFileName.
* @brief These methods allow to construct accessor objects using the following syntax
*         auto data_read_access = read(d_data);
*         auto data_write_access = write(d_data);
* @param The data whose accessor is being constructed
* @param An optional ExecParams instance
* @retval An accessor object for the Data
* @{
*/
template<class TData, typename enable_if_is_base_of_data<TData>::type* = nullptr >
inline WriteAccessor<sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> > 
write(TData& data, const core::ExecParams* params)
{
    return WriteAccessor< sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> >(params,data);
}


template<class TData, typename enable_if_is_base_of_data<TData>::type* = nullptr >
inline WriteAccessor<sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> > 
write(TData& data)
{ 
    return write(data,sofa::core::ExecParams::defaultInstance() ); 
}


template<class TData, typename enable_if_is_base_of_data<TData>::type* = nullptr >
inline ReadAccessor<sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> > 
read(const TData& data, const core::ExecParams* params)
{
    return ReadAccessor< sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> >(params, data);
}


template<class TData, typename enable_if_is_base_of_data<TData>::type* = nullptr >
inline ReadAccessor<sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> > 
read(TData& data)
{
    return read(data, sofa::core::ExecParams::defaultInstance());
}

template<class TData, typename enable_if_is_base_of_data<TData>::type* = nullptr >
inline WriteOnlyAccessor<sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> > 
writeOnly(TData& data) 
{ 
    return WriteOnlyAccessor<sofa::core::objectmodel::Data< typename sofa::core::objectmodel::DataTraits<TData>::value_type> >(data);
}
/** @} */

} // namespace helper

// the Data class is used everywhere
using core::objectmodel::Data;

} // namespace sofa

#endif
