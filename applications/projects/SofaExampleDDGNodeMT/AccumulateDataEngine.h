#include <sofa/core/DataEngine.h>
#include <memory>
namespace sofa
{

class AccumulateDataEngine : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(AccumulateDataEngine, sofa::core::DataEngine);

    void init() override;

    void update() override;

    sofa::Data<std::size_t> d_numInputs;
    sofa::Data<int>         d_sleepMilliseconds;
    
    sofa::Data<double>      d_outputValue;

    sofa::Data<double>*     getInput(std::size_t i);

protected:

    AccumulateDataEngine();

private:

    std::vector< std::unique_ptr< sofa::Data<double> > > m_dataInputs;

};


}