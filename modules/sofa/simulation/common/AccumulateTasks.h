/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_ACCUMULATETASKS_H
#define ISPHYSICS_BASE_ACCUMULATETASKS_H

#include <MultiThreading/src/Tasks.h>
#include <ISPhysicsBase/DependencyTask.h>
#include "initPlugin.h"
#include <sofa/core/MultiVecId.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/core/ConstraintParams.h>
#include "SofaDependencyTask.h"

ISPHYSICS_PUBLIC

namespace isphysics
{
namespace base
{

struct SOFA_ISPHYSICS_BASE_API AccumulateTaskInfo
{
    const sofa::core::ConstraintParams* m_cparams;
    sofa::core::MultiMatrixDerivId      m_res;
    bool                                m_reverseOrder;
};

class SOFA_ISPHYSICS_BASE_API AccumulateTask : public TSofaDependencyTask< sofa::core::BaseMapping, AccumulateTaskInfo >
{
public:
    typedef TSofaDependencyTask< sofa::core::BaseMapping, AccumulateTaskInfo > Inherit;

    AccumulateTask(sofa::core::BaseMapping* mapping, std::string baseName="Acc" );

    bool run(sofa::simulation::WorkerThread* thread);
};



} // namespace base
} // namespace isphysics

#endif
