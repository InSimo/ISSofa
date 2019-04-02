/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
#include <sofa/core/objectmodel/Base.h>
#include <sofa/helper/Factory.h>
#include <map>
#include <typeinfo>
#include <algorithm>
#include <string.h>
#include <sstream>
#include <atomic>

namespace sofa
{

namespace core
{

namespace objectmodel
{

SOFA_ROOT_CLASS_IMPL((Base));

using std::string;

static const std::string unnamed_label=std::string("unnamed");
static std::atomic<Base::ExecUID> counterExecUID;

namespace
{

#ifdef WIN32
HANDLE winConsole = GetStdHandle(STD_OUTPUT_HANDLE);
CONSOLE_SCREEN_BUFFER_INFO winConsoleInfo;
BOOL winHasConsoleScreenBufferInfo = GetConsoleScreenBufferInfo(winConsole, &winConsoleInfo);
#endif

}


Base::Base()
    : ref_counter(0)
    , m_sourceFileName()
    , m_sourceFilePos(0,0)
    , name(initData(&name,unnamed_label,"name","object name"))
    , f_printLog(initData(&f_printLog, false, "printLog", "if true, print logs at run-time"))
    , f_tags(initData( &f_tags, "tags", "list of the subsets the objet belongs to"))
    , f_bbox(initData( &f_bbox, "bbox", "this object bounding box"))
    , f_execUID(initData( &f_execUID, ++counterExecUID, "execUID", "Unique ID of this Base instance (unique for this run of the application, implemented as a counter starting from 1)"))
{
    name.setOwnerClass("Base");
    name.setAutoLink(false);
    f_printLog.setOwnerClass("Base");
    f_printLog.setAutoLink(false);
    f_tags.setOwnerClass("Base");
    f_tags.setAutoLink(false);
    f_bbox.setOwnerClass("Base");
    f_bbox.setReadOnly(true);
    f_bbox.setPersistent(false);
    f_bbox.setDisplayed(false);
    f_bbox.setAutoLink(false);
    f_execUID.setOwnerClass("Base");
    f_execUID.setReadOnly(true);
    f_execUID.setPersistent(false);
    f_execUID.setAutoLink(false);
    sendl.setParent(this);
}

Base::~Base()
{
}

void Base::addRef()
{
    ++ref_counter;
}

void Base::release()
{
    //if ((--ref_counter) == 0)
    if (ref_counter.dec_and_test_null())
    {
        //serr << "DELETE" << sendl;
        // Deletion of objects can be temporarily disabled by commenting the next line, until smart-pointers usage is corrected

// This delete can cause a crash on Windows with sofa cuda build with msvc
#if !(defined(_MSC_VER) && defined(SOFA_GPU_CUDA))
        delete this;
#endif
    }
}

/// Helper method used by initData()
void Base::initData0(BaseData::BaseInitData& res, const char* name, const char* help)
{
    static const uint32_t draw_fourcc = MAKEFOURCC('d', 'r', 'a', 'w');
    static const uint32_t show_fourcc = MAKEFOURCC('s', 'h', 'o', 'w');

    if (name == NULL || help == NULL)
    {
        std::cerr << "Invalid inputs to " << this->getClassName() << "::initData0(): "
            << " name=" << (name ? name : "NULL")
            << " help=" << (help ? help : "NULL")
            << std::endl;
        if (name == NULL) name = "";
        if (help == NULL) help = "";
    }

    res.owner = this;
    res.name = name;
    res.helpMsg = help;

    if (name[0] && name[1] && name[2] && name[3]) // at least 4 characters
    {
        uint32_t prefix = *(uint32_t*)name;

        if (prefix == draw_fourcc || prefix == show_fourcc)
            res.group = "Visualization";
    }

    //TODO : check on flags
}

/// Helper method used by initData()
void Base::initData0( BaseData* field, BaseData::BaseInitData& res, const char* name, const char* help, bool isDisplayed, bool isReadOnly )
{
    BaseData::DataFlags flags = BaseData::FLAG_DEFAULT;
    if(isDisplayed) flags |= (BaseData::DataFlags)BaseData::FLAG_DISPLAYED; else flags &= ~(BaseData::DataFlags)BaseData::FLAG_DISPLAYED;
    if(isReadOnly)  flags |= (BaseData::DataFlags)BaseData::FLAG_READONLY; else flags &= ~(BaseData::DataFlags)BaseData::FLAG_READONLY;

    initData0(field, res, name, help, flags);
}

/// Helper method used by initData()
void Base::initData0( BaseData* field, BaseData::BaseInitData& res, const char* name, const char* help, BaseData::DataFlags dataFlags )
{
    static const uint32_t draw_fourcc = MAKEFOURCC('d', 'r', 'a', 'w');
    static const uint32_t show_fourcc = MAKEFOURCC('s', 'h', 'o', 'w');

    if (name == NULL || help == NULL)
    {
        std::cerr << "Invalid inputs to "<<this->getClassName()<<"::initData0(): "
                  << " name=" << (name ? name : "NULL")
                  << " help=" << (help ? help : "NULL")
                  << std::endl;
        if (name == NULL) name="";
        if (help == NULL) help="";
    }

    /*
        std::string ln(name);
        if( ln.size()>0 && findField(ln) )
        {
            serr << "field name " << ln << " already used in this class or in a parent class !...aborting" << sendl;
            exit( 1 );
        }
        m_fieldVec.push_back( std::make_pair(ln,field));
        m_aliasData.insert(std::make_pair(ln,field));
    */
    res.owner = this;
    res.data = field;
    res.name = name;
    res.helpMsg = help;
    res.dataFlags = dataFlags;

    if (name[0] && name[1] && name[2] && name[3]) // at least 4 characters
    {
        uint32_t prefix = *(uint32_t*) name;

        if(prefix == draw_fourcc || prefix == show_fourcc)
            res.group = "Visualization";
    }
}

/// Add a data field.
/// Note that this method should only be called if the field was not initialized with the initData method
void Base::addData(BaseData* f)
{
    addData(f, f->getName());
}

/// Add a data field.
/// Note that this method should only be called if the field was not initialized with the initData method
void Base::addData(BaseData* f, const std::string& name)
{
    if (name.size() > 0 && (findData(name) || findLink(name)))
    {
        serr << "Data field name " << name
                << " already used in this class or in a parent class !"
                << sendl;
        //exit(1);
    }
    m_vecData.push_back(f);
    m_aliasData.insert(std::make_pair(name, f));
    f->setOwner(this);
}

/// Remove a data field.
void Base::removeData(BaseData* f)
{
    if (f->getOwner() == this)
    {
        f->setOwner(NULL);
    }
    VecData::iterator itv = std::find(m_vecData.begin(), m_vecData.end(), f);
    MapData::iterator itm = m_aliasData.find(f->getName());
    if (itv == m_vecData.end() && itm == m_aliasData.end())
    {
        serr << "Data field " << f->getName()
                << " cannot be removed as it is not registered !"
                << sendl;
        return;
    }
    if (itv != m_vecData.end())
    {
        m_vecData.erase(itv);
    }
    if (itm != m_aliasData.end())
    {
        m_aliasData.erase(itm);
    }
}

/// Add an alias to a Data
void Base::addAlias( BaseData* field, const char* alias)
{
    m_aliasData.insert(std::make_pair(std::string(alias),field));
}

/// Add a link.
/// Note that this method should only be called if the link was not initialized with the initLink method
void Base::addLink(BaseLink* l)
{
    std::string name = l->getName();
    if (name.size() > 0 && (findData(name) || findLink(name)))
    {
        serr << "Link name " << name
                << " already used in this class or in a parent class !"
                << sendl;
        //exit(1);
    }
    m_vecLink.push_back(l);
    m_aliasLink.insert(std::make_pair(name, l));
    //l->setOwner(this);
}

/// Add an alias to a Link
void Base::addAlias( BaseLink* link, const char* alias)
{
    m_aliasLink.insert(std::make_pair(std::string(alias),link));
}

/// Copy the source aspect to the destination aspect for each Data in the component.
void Base::copyAspect(int destAspect, int srcAspect)
{
    for(VecData::const_iterator iData = m_vecData.begin(); iData != m_vecData.end(); ++iData)
    {
        //std::cout << "  " << iData->first;
        (*iData)->copyAspect(destAspect, srcAspect);
    }
    for(VecLink::const_iterator iLink = m_vecLink.begin(); iLink != m_vecLink.end(); ++iLink)
    {
        //std::cout << "  " << iLink->first;
        (*iLink)->copyAspect(destAspect, srcAspect);
    }
    //std::cout << std::endl;
}

/// Release memory allocated for the specified aspect.
void Base::releaseAspect(int aspect)
{
    for(VecData::const_iterator iData = m_vecData.begin(); iData != m_vecData.end(); ++iData)
    {
        (*iData)->releaseAspect(aspect);
    }
    for(VecLink::const_iterator iLink = m_vecLink.begin(); iLink != m_vecLink.end(); ++iLink)
    {
        (*iLink)->releaseAspect(aspect);
    }
}

/// Get the type name of this object (i.e. class and template types)
std::string Base::getTypeName() const
{
    //return BaseClass::decodeTypeName(typeid(*this));
    std::string c = getClassName();
    std::string t = getTemplateName();
    if (t.empty())
        return c;
    else
        return c + std::string("<") + t + std::string(">");
}

/// Get the class name of this object
std::string Base::getClassName() const
{
    return BaseClass::decodeClassName(typeid(*this));
}

/// Get the template type names (if any) used to instantiate this object
std::string Base::getTemplateName() const
{
    return BaseClass::decodeTemplateName(typeid(*this));
}

void Base::setName(const std::string& na)
{
    name.setValue(na);
}

/// Set the name of this object, adding an integer counter
void Base::setName(const std::string& n, int counter)
{
    std::ostringstream o;
    o << n << counter;
    setName(o.str());
}

#define MAXLOGSIZE 10000000

void Base::processStream(std::ostream& out)
{

#ifdef WIN32

#define SOFA_CONSOLE_BLUE 9
#define SOFA_CONSOLE_GREEN 10
#define SOFA_CONSOLE_CYAN 11
#define SOFA_CONSOLE_RED 12
#define SOFA_CONSOLE_PURPLE 13
#define SOFA_CONSOLE_YELLOW 14
#define SOFA_CONSOLE_WHITE 15

#else

#define SOFA_CONSOLE_BLUE "\033[1;34m "
#define SOFA_CONSOLE_GREEN "\033[1;32m "
#define SOFA_CONSOLE_CYAN "\033[1;36m "
#define SOFA_CONSOLE_RED "\033[1;31m "
#define SOFA_CONSOLE_PURPLE "\033[1;35m "
#define SOFA_CONSOLE_YELLOW "\033[1;33m "
#define SOFA_CONSOLE_WHITE "\033[1;37m "
#define SOFA_CONSOLE_ENDL " \033[0m"

#endif

    const std::string prefix = "[" + getName() + "(" + getClassName() + ")]: ";
    if (&out == &serr)
    {
        serr << "\n";
        const std::string str = prefix + serr.str();
#ifdef WIN32
        SetConsoleTextAttribute(winConsole, SOFA_CONSOLE_RED);
        std::cerr<< " [WARN] ";
        SetConsoleTextAttribute(winConsole, winConsoleInfo.wAttributes);
#else
        std::cerr<< SOFA_CONSOLE_RED <<"[WARN]" << SOFA_CONSOLE_ENDL;
#endif
        std::cerr << str << std::flush;
        if (warnings.size()+str.size() >= MAXLOGSIZE)
        {
            std::cerr<< "LOG OVERFLOW[" << getName() << "(" << getClassName() << ")]: resetting serr buffer." << std::endl;
            warnings.clear();
            warnings = "LOG EVERFLOW: resetting serr buffer\n";
        }
        warnings += str;
        serr.str("");
    }
    else if (&out == &sout)
    {

        sout << "\n";
        const std::string str = prefix + sout.str();
        if (f_printLog.getValue())
        {
#ifdef WIN32
            SetConsoleTextAttribute(winConsole, SOFA_CONSOLE_GREEN);
            std::cout<<" [INFO] ";
            SetConsoleTextAttribute(winConsole, winConsoleInfo.wAttributes);
#else
            std::cerr<< SOFA_CONSOLE_GREEN <<"[INFO]"<< SOFA_CONSOLE_ENDL;
#endif
            std::cout<< str << std::flush;
        }
        if (outputs.size()+str.size() >= MAXLOGSIZE)
        {
            std::cerr<< "LOG OVERFLOW[" << getName() << "(" << getClassName() << ")]: resetting sout buffer." << std::endl;
            outputs.clear();
            outputs = "LOG EVERFLOW: resetting sout buffer\n";
        }
        outputs += str;
        sout.str("");
    }

#undef SOFA_CONSOLE_BLUE
#undef SOFA_CONSOLE_GREEN
#undef SOFA_CONSOLE_CYAN
#undef SOFA_CONSOLE_RED
#undef SOFA_CONSOLE_PURPLE
#undef SOFA_CONSOLE_YELLOW
#undef SOFA_CONSOLE_WHITE

#ifndef WIN32
#undef SOFA_CONSOLE_ENDL
#endif

}

const std::string& Base::getWarnings() const
{
    return warnings;
}

const std::string& Base::getOutputs() const
{
    return outputs;
}

void Base::clearWarnings()
{
    warnings.clear();
}

void Base::clearOutputs()
{
    outputs.clear();
}


bool Base::hasTag(Tag t) const
{
    return (f_tags.getValue().count( t ) > 0 );
}


void Base::addTag(Tag t)
{
    f_tags.beginEdit()->insert(t);
    f_tags.endEdit();
}

void Base::removeTag(Tag t)
{
    f_tags.beginEdit()->erase(t);
    f_tags.endEdit();
}


/// Find a data field given its name.
/// Return NULL if not found. If more than one field is found (due to aliases), only the first is returned.
BaseData* Base::findData( const std::string &name ) const
{
    //Search in the aliases
    if(m_aliasData.size())
    {
        typedef MapData::const_iterator mapIterator;
        std::pair< mapIterator, mapIterator> range = m_aliasData.equal_range(name);
        if (range.first != range.second)
            return range.first->second;
        else
            return NULL;
    }
    else return NULL;
}

/// Find fields given a name: several can be found as we look into the alias map
std::vector< BaseData* > Base::findGlobalField( const std::string &name ) const
{
    std::vector<BaseData*> result;
    //Search in the aliases
    typedef MapData::const_iterator mapIterator;
    std::pair< mapIterator, mapIterator> range = m_aliasData.equal_range(name);
    for (mapIterator itAlias=range.first; itAlias!=range.second; ++itAlias)
        result.push_back(itAlias->second);
    return result;
}


/// Find a link given its name.
/// Return NULL if not found. If more than one link is found (due to aliases), only the first is returned.
BaseLink* Base::findLink( const std::string &name ) const
{
    //Search in the aliases
    typedef MapLink::const_iterator mapIterator;
    std::pair< mapIterator, mapIterator> range = m_aliasLink.equal_range(name);
    if (range.first != range.second)
        return range.first->second;
    else
        return NULL;
}

/// Find links given a name: several can be found as we look into the alias map
std::vector< BaseLink* > Base::findLinks( const std::string &name ) const
{
    std::vector<BaseLink*> result;
    //Search in the aliases
    typedef MapLink::const_iterator mapIterator;
    std::pair< mapIterator, mapIterator> range = m_aliasLink.equal_range(name);
    for (mapIterator itAlias=range.first; itAlias!=range.second; ++itAlias)
        result.push_back(itAlias->second);
    return result;
}

bool Base::findDataLinkDest(BaseData*& ptr, const std::string& path, const BaseLink* link)
{
    std::string pathStr, dataStr;
    if (link)
    {
        if (!link->parseString(path, &pathStr, &dataStr))
            return false;
    }
    else
    {
        if (!BaseLink::ParseString(path, &pathStr, &dataStr, this))
            return false;
    }
    if (pathStr.empty() || pathStr == std::string("[]"))
    {
        ptr = this->findData(dataStr);
        return (ptr != NULL);
    }
    Base* obj = NULL;
    if (!findLinkDest(obj, BaseLink::CreateString(pathStr), link))
        return false;
    if (!obj)
        return false;
    ptr = obj->findData(dataStr);
    return (ptr != NULL);
}

void* Base::findLinkDestClass(const BaseClass* /*destType*/, const std::string& /*path*/, const BaseLink* /*link*/)
{
    std::cerr << "Base: calling unimplemented findLinkDest method" << std::endl;
    return NULL;
}

bool Base::hasField( const std::string& attribute) const
{
    return m_aliasData.find(attribute) != m_aliasData.end()
            || m_aliasLink.find(attribute) != m_aliasLink.end();
}

/// Assign one field value (Data or Link)
bool Base::parseField( const std::string& attribute, const std::string& value)
{
    std::vector< BaseData* > dataVec = findGlobalField(attribute);
    std::vector< BaseLink* > linkVec = findLinks(attribute);
    if (dataVec.empty() && linkVec.empty())
    {
        serr << "Unknown Data field or Link: " << attribute << sendl;
        return false; // no field found
    }
    bool ok = true;
    for (unsigned int d=0; d<dataVec.size(); ++d)
    {
        // test if data is a link and can be linked
        if (value[0] == '@' && dataVec[d]->canBeLinked())
        {
            if (!dataVec[d]->setParent(value))
            {
                serr<<"Could not setup Data link between "<< value << " and " << attribute << "." << sendl;
                ok = false;
                continue;
            }
            else
            {
                BaseData* parentData = dataVec[d]->getParent();
                sout<<"Link from parent Data " << value << " (" << parentData->getValueTypeInfo()->name() << ") to Data " << attribute << "(" << dataVec[d]->getValueTypeInfo()->name() << ") OK" << sendl;
            }
            /* children Data cannot be modified changing the parent Data value */
            dataVec[d]->setReadOnly(true);
            continue;
        }
        if( !(dataVec[d]->read( value )) && !value.empty())
        {
            serr<<"Could not read value for data field "<< attribute <<": " << value << sendl;
            ok = false;
        }
    }
    for (unsigned int l=0; l<linkVec.size(); ++l)
    {
        if( !(linkVec[l]->read( value )) && !value.empty())
        {
            serr<<"Could not read value for link "<< attribute <<": " << value << sendl;
            ok = false;
        }
        sout << "Link " << linkVec[l]->getName() << " = " << linkVec[l]->getValueString() << sendl;
        unsigned int s = linkVec[l]->getSize();
        for (unsigned int i=0; i<s; ++i)
        {
            sout  << "  " << linkVec[l]->getLinkedPath(i) << " = ";
            Base* b = linkVec[l]->getLinkedBase(i);
            BaseData* d = linkVec[l]->getLinkedData(i);
            if (b) sout << b->getTypeName() << " " << b->getName();
            if (d) sout << " . " << d->getValueTypeString() << " " << d->getName();
            sout << sendl;
        }
    }
    return ok;
}

void  Base::parseFields ( const std::list<std::string>& str )
{
    string name,value;
    std::list<std::string>::const_iterator it = str.begin(), itend = str.end();
    while(it != itend)
    {
        name = *it;
        ++it;
        if (it == itend) break;
        value = *it;
        ++it;
        parseField(name, value);
    }
}

void  Base::parseFields ( const std::map<std::string,std::string*>& args )
{
    std::string key,val;
    for( std::map<string,string*>::const_iterator i=args.begin(), iend=args.end(); i!=iend; ++i )
    {
        if( (*i).second!=NULL )
        {
            key=(*i).first;
            val=*(*i).second;
            parseField(key, val);
        }
    }
}

/// Parse the given description to assign values to this object's fields and potentially other parameters
void  Base::parse ( BaseObjectDescription* arg )
{
    std::vector< std::string > attributeList;
    arg->getAttributeList(attributeList);
    for (unsigned int i=0; i<attributeList.size(); ++i)
    {
        std::string attrName = attributeList[i];
        // FIX: "type" is already used to define the type of object to instanciate, any Data with
        // the same name cannot be extracted from BaseObjectDescription
        if (attrName == std::string("type")) continue;
        if (!hasField(attrName)) continue;
        const char* val = arg->getAttribute(attrName);
        if (!val) continue;
        std::string valueString(val);
        parseField(attrName, valueString);
    }
    updateLinks(false);
}

/// Update pointers in case the pointed-to objects have appeared
void Base::updateLinks(bool logErrors)
{
    // update links
    for(VecLink::const_iterator iLink = m_vecLink.begin(); iLink != m_vecLink.end(); ++iLink)
    {
        bool ok = (*iLink)->updateLinks();
        if (!ok && (*iLink)->storePath() && logErrors)
        {
            serr << "Link update failed for " << (*iLink)->getName() << " = " << (*iLink)->getValueString() << sendl;
        }
    }
}

void  Base::writeDatas ( std::map<std::string,std::string*>& args )
{
    for(VecData::const_iterator iData = m_vecData.begin(); iData != m_vecData.end(); ++iData)
    {
        BaseData* field = *iData;
        std::string name = field->getName();
        if( args[name] != NULL )
            *args[name] = field->getValueString();
        else
            args[name] =  new string(field->getValueString());
    }
    for(VecLink::const_iterator iLink = m_vecLink.begin(); iLink != m_vecLink.end(); ++iLink)
    {
        BaseLink* link = *iLink;
        std::string name = link->getName();
        if( args[name] != NULL )
            *args[name] = link->getValueString();
        else
            args[name] =  new string(link->getValueString());
    }
}

static std::string xmlencode(const std::string& str)
{
    std::string res;
    for (unsigned int i=0; i<str.length(); ++i)
    {
        switch(str[i])
        {
        case '<': res += "&lt;"; break;
        case '>': res += "&gt;"; break;
        case '&': res += "&amp;"; break;
        case '"': res += "&quot;"; break;
        case '\'': res += "&apos;"; break;
        default:  res += str[i];
        }
    }
    return res;
}

void  Base::writeDatas (std::ostream& out, const std::string& separator)
{
    for(VecData::const_iterator iData = m_vecData.begin(); iData != m_vecData.end(); ++iData)
    {
        BaseData* field = *iData;
        if (!field->getLinkPath().empty() )
        {
            out << separator << field->getName() << "=\""<< xmlencode(field->getLinkPath()) << "\" ";
        }
        else
        {
            if(field->isPersistent() && field->isSet())
            {
                std::string val = field->getValueString();
                if (!val.empty())
                    out << separator << field->getName() << "=\""<< xmlencode(val) << "\" ";
            }
        }
    }
    for(VecLink::const_iterator iLink = m_vecLink.begin(); iLink != m_vecLink.end(); ++iLink)
    {
        BaseLink* link = *iLink;
        if(link->storePath())
        {
            std::string val = link->getValueString();
            if (!val.empty())
                out << separator << link->getName() << "=\""<< xmlencode(val) << "\" ";
        }
    }
}

void Base::setSourceFile(const std::string& name, int line, int column)
{
    m_sourceFileName = name;
    m_sourceFilePos = std::make_pair(line,column);
}

const std::string& Base::getSourceFileName() const
{
    return m_sourceFileName;
}

std::pair<int,int> Base::getSourceFilePos() const
{
    return m_sourceFilePos;
}

} // namespace objectmodel

} // namespace core

} // namespace sofa
