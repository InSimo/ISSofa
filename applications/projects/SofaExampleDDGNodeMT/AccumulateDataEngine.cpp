#include "AccumulateDataEngine.h"
#include <thread>
#include <chrono>

namespace sofa
{

AccumulateDataEngine::AccumulateDataEngine()
:d_numInputs(initData(&d_numInputs, "numInputs", "Number of inputs to create for this engine, accumulated in ouputValue during update method call"))
,d_sleepMilliseconds(initData(&d_sleepMilliseconds, "sleepMilliseconds","Number of milliseconds this engine will sleep before exiting update"))
,d_outputValue(initData(&d_outputValue, double(0),"ouptutValue","Output value which is the accmulaton of all value contained in each input"))
{
    this->addOutput(&d_outputValue);
}


void AccumulateDataEngine::init()
{
    std::size_t numInputs = d_numInputs.getValue();

    for (std::size_t i = 0; i < m_dataInputs.size(); ++i)
    {
        this->delInput(m_dataInputs[i].get());
    }

    m_dataInputs.clear();
    m_dataInputs.resize(numInputs);

    for (std::size_t i = 0; i < numInputs; ++i)
    {
        std::string name = "input_" + std::to_string(i);
        std::string help = name;
        m_dataInputs[i].reset(new Data<double>(name.c_str(), true, false));
        m_dataInputs[i]->setName(name);
        addInput(m_dataInputs[i].get());
    }
}

sofa::Data<double>* AccumulateDataEngine::getInput(std::size_t i)
{
    if (i < m_dataInputs.size())
    {
        return m_dataInputs[i].get();
    }
    else
    {
        std::cerr << "invalid access: index " << i << " size: " << m_dataInputs.size() << std::endl;
        return nullptr;
    }
}


void AccumulateDataEngine::update()
{
    cleanDirty();

    double accumulateValue = 0;
    for (std::size_t i = 0; i < m_dataInputs.size(); ++i)
    {
        accumulateValue += m_dataInputs[i]->getValue();
    }

    double* outputValue = d_outputValue.beginEdit();

    std::this_thread::sleep_for(std::chrono::milliseconds(d_sleepMilliseconds.getValue()));

    *outputValue = accumulateValue;

    d_outputValue.endEdit();
}


}
    
    