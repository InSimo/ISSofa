#include "AccumulateDataEngine.h"
#include <future>

int main()
{
    sofa::AccumulateDataEngine::SPtr accumulateDataEngine = sofa::core::objectmodel::New< sofa::AccumulateDataEngine >();


    const std::size_t numInputs   = 100;
    const int         sleepTimeMs = 1;
    std::cout << "Preparing AccumulateDataEngine with " << numInputs << " inputs" << std::endl;

    accumulateDataEngine->d_numInputs.setValue(numInputs);
    accumulateDataEngine->d_sleepMilliseconds.setValue(sleepTimeMs);
    accumulateDataEngine->init();

    for (std::size_t i = 0;  i < numInputs; ++i)
    {
        accumulateDataEngine->getInput(i)->setValue(i);
    }

    auto accumulateDataEngineUpdateLambda = [&accumulateDataEngine] { return accumulateDataEngine->d_outputValue.getValue(); };

    std::cout << "Launching 4 threads concurrently calling the AccumulateDataEngine::update method by fetching its output value" << std::endl;

    auto f1 = std::async(std::launch::async, accumulateDataEngineUpdateLambda);
    auto f2 = std::async(std::launch::async, accumulateDataEngineUpdateLambda);
    auto f3 = std::async(std::launch::async, accumulateDataEngineUpdateLambda);
    auto f4 = std::async(std::launch::async, accumulateDataEngineUpdateLambda);


    std::cout << "Wating for results..." << std::endl;

    f1.wait();
    f2.wait();
    f3.wait();
    f4.wait();

    std::cout << "Done\nResults are: " << f1.get() << " " << f2.get() << " " << f3.get() << " " << f4.get() << std::endl;

    double exepectedResult = numInputs * (numInputs - 1) * 0.5;
    std::cout << "Expected result is: " << exepectedResult << std::endl;

    return 0;
}

