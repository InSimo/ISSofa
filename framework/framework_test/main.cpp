#include <iostream>
#include "gtest/gtest.h"
#include <sofa/helper/test.h>

/// less verbose output, based on https://gist.github.com/elliotchance/8215283
class ConfigurableEventListener : public testing::TestEventListener
{
public:
    using TestEventListener = testing::TestEventListener;
    using UnitTest = testing::UnitTest;
    using TestCase = testing::TestCase;
    using TestInfo = testing::TestInfo;
    using TestPartResult = testing::TestPartResult;

    /// Show the names of each test case.
    bool showTestCases;

    /// Show the names of each test.
    bool showTestNames;

    /// Show each success.
    bool showSuccesses;

    /// Show each failure as it occurs. You will also see it at the bottom after the full suite is run.
    bool showInlineFailures;

    /// Show stdout/stdin from failed tests.
    bool showOutputFailures;

    /// Show stdout/stdin from success.
    bool showOutputSuccesses;

    /// Show the setup of the global environment.
    bool showEnvironment;

    explicit ConfigurableEventListener(TestEventListener* theEventListener) : eventListener(theEventListener)
    {
        showTestCases = true;
        showTestNames = true;
        showSuccesses = true;
        showInlineFailures = true;
        showOutputFailures = true;
        showOutputSuccesses = true;
        showEnvironment = true;
    }

    virtual ~ConfigurableEventListener()
    {
        delete eventListener;
    }

    virtual void OnTestProgramStart(const UnitTest& unit_test)
    {
        if(showEnvironment)
        {
            eventListener->OnTestProgramStart(unit_test);
        }
    }

    virtual void OnTestIterationStart(const UnitTest& unit_test, int iteration)
    {
        eventListener->OnTestIterationStart(unit_test, iteration);
    }
    
    virtual void OnEnvironmentsSetUpStart(const UnitTest& unit_test)
    {
        if(showEnvironment)
        {
            eventListener->OnEnvironmentsSetUpStart(unit_test);
        }
    }
    
    virtual void OnEnvironmentsSetUpEnd(const UnitTest& unit_test)
    {
        if(showEnvironment)
        {
            eventListener->OnEnvironmentsSetUpEnd(unit_test);
        }
    }
    
    virtual void OnTestCaseStart(const TestCase& test_case)
    {
        if(showTestCases)
        {
            eventListener->OnTestCaseStart(test_case);
        }
    }
    
    virtual void OnTestStart(const TestInfo& test_info)
    {
        if (!showOutputFailures || !showOutputSuccesses)
        {
            redirectStart();
        }
        if(showTestNames)
        {
            eventListener->OnTestStart(test_info);
        }
    }

    virtual void OnTestPartResult(const TestPartResult& result)
    {
        //bool wasRedirected = redirectPause();
        eventListener->OnTestPartResult(result);
        //if (wasRedirected) redirectStart();
    }

    virtual void OnTestEnd(const TestInfo& test_info)
    {
        std::string output = redirectEnd();
        if (test_info.result()->Failed())
        {
            if (showOutputFailures && !output.empty())
            {
                std::cerr << output;
                std::cerr.flush();
            }
            if (showInlineFailures)
            {
                eventListener->OnTestEnd(test_info);
            }
        }
        else
        {
            if (showOutputSuccesses && !output.empty())
            {
                std::cout << output;
                std::cout.flush();
            }
            if (showSuccesses)
            {
                eventListener->OnTestEnd(test_info);
            }
        }
    }

    virtual void OnTestCaseEnd(const TestCase& test_case)
    {
        if(showTestCases)
        {
            eventListener->OnTestCaseEnd(test_case);
        }
    }

    virtual void OnEnvironmentsTearDownStart(const UnitTest& unit_test)
    {
        if(showEnvironment)
        {
            eventListener->OnEnvironmentsTearDownStart(unit_test);
        }
    }

    virtual void OnEnvironmentsTearDownEnd(const UnitTest& unit_test)
    {
        if(showEnvironment)
        {
            eventListener->OnEnvironmentsTearDownEnd(unit_test);
        }
    }

    virtual void OnTestIterationEnd(const UnitTest& unit_test, int iteration)
    {
        eventListener->OnTestIterationEnd(unit_test, iteration);
    }

    virtual void OnTestProgramEnd(const UnitTest& unit_test)
    {
        eventListener->OnTestProgramEnd(unit_test);
    }

protected:
    TestEventListener* eventListener;
    bool redirect = false;
    std::ostringstream buffer;
    std::streambuf* cout_buffer = nullptr;
    std::streambuf* cerr_buffer = nullptr;

    void redirectStart()
    {
        if (!redirect)
        {
            redirect = true;
            cout_buffer = std::cout.rdbuf(buffer.rdbuf());
            cerr_buffer = std::cerr.rdbuf(buffer.rdbuf());
        }
    }

    bool redirectPause()
    {
        if (redirect)
        {
            std::cout.rdbuf(cout_buffer);
            std::cerr.rdbuf(cerr_buffer);
            redirect = false;
            return true;
        }
        else
        {
            return false;
        }
        
    }

    std::string redirectEnd()
    {
        if (redirect)
        {
            std::cout.rdbuf(cout_buffer);
            std::cerr.rdbuf(cerr_buffer);
            redirect = false;
        }
        std::string content = buffer.str();
        buffer.str("");
        return content;
    }
};

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    if(argc>1 && argv[1] == std::string("--output-on-failure"))
    {
        // add our listener, turning everything off so we only see the 3 lines for the result
        // plus any failures at the end.

        // remove the default listener
        testing::TestEventListeners& listeners = testing::UnitTest::GetInstance()->listeners();
        auto default_printer = listeners.Release(listeners.default_result_printer());
        ConfigurableEventListener *listener = new ConfigurableEventListener(default_printer);
        listener->showEnvironment = false;
        listener->showTestCases = false;
        listener->showTestNames = false;
        listener->showSuccesses = false;
        listener->showOutputSuccesses = false;
        listener->showInlineFailures = true;
        listeners.Append(listener);
    }
    // Set LC_CTYPE according to the environnement variable.
    setlocale(LC_CTYPE, "");

    sofa::helper::initBeforeTests();
    int ret = RUN_ALL_TESTS();
    sofa::helper::cleanupAfterTests();

    return ret;
}
