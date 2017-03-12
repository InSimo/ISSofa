#include "DataTracker.h"
#include "objectmodel/BaseData.h"

namespace sofa
{

namespace core
{





void DataTracker::trackData( const objectmodel::BaseData& data )
{
    m_dataTrackers[&data] = data.getCounter();
}

bool DataTracker::isDirty( const objectmodel::BaseData& data )
{
    return m_dataTrackers[&data] != data.getCounter();
}

bool DataTracker::isDirty()
{
    for( DataTrackers::iterator it=m_dataTrackers.begin(),itend=m_dataTrackers.end() ; it!=itend ; ++it )
        if( it->second != it->first->getCounter() ) return true;
    return false;
}

void DataTracker::clean( const objectmodel::BaseData& data )
{
    m_dataTrackers[&data] = data.getCounter();
}

void DataTracker::clean()
{
    for( DataTrackers::iterator it=m_dataTrackers.begin(),itend=m_dataTrackers.end() ; it!=itend ; ++it )
        it->second = it->first->getCounter();
}



////////////////////



void DataTrackerDDGNode::doCleanDirty(const core::ExecParams* params, bool warnBadUse)
{
    core::objectmodel::DDGNode::doCleanDirty(params, warnBadUse);

    // it is also time to clean the tracked Data
    m_dataTracker.clean();

}



void DataTrackerDDGNode::updateAllInputsIfDirty()
{
    const DDGLinkContainer& inputs = DDGNode::getInputs();
    for(size_t i=0, iend=inputs.size() ; i<iend ; ++i )
    {
        inputs[i]->requestUpdateIfDirty();
    }
}



///////////////////////


void DataTrackerEngine::setUpdateCallback( void (*f)(DataTrackerEngine*) )
{
    m_updateCallback = f;
}

}

}
