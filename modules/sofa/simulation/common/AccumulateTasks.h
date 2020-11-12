/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_SIMULATION_ACCUMULATETASKS_H
#define SOFA_SIMULATION_ACCUMULATETASKS_H

#include <sofa/SofaSimulation.h>
#include "Tasks.h"
#include "DependencyTask.h"
#include <sofa/core/MultiVecId.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/core/ConstraintParams.h>
#include "SofaDependencyTask.h"

namespace sofa
{
namespace simulation
{

struct SOFA_SIMULATION_COMMON_API AccumulateTaskInfo
{
    const sofa::core::ConstraintParams* m_cparams;
    sofa::core::MultiMatrixDerivId      m_res;
    bool                                m_reverseOrder;
};

class SOFA_SIMULATION_COMMON_API AccumulateTask : public TSofaDependencyTask< sofa::core::BaseMapping, AccumulateTaskInfo >
{
public:
    typedef TSofaDependencyTask< sofa::core::BaseMapping, AccumulateTaskInfo > Inherit;

    AccumulateTask(sofa::core::BaseMapping* mapping, std::string baseName="Acc" );

    bool run(sofa::simulation::WorkerThread* thread);
};



} // namespace simulation
} // namespace sofa

#endif
