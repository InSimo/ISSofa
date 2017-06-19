#include <sofa/helper/preprocessor.h>
#include <gtest/gtest.h>
#include <cstdio>

namespace
{

#define SOFA_PP_TEST_PRINT(Value)  \
 printf(Value);


TEST(SofaPreprocessorTest, FirstTest)
{
    SOFA_FOR_EACH(SOFA_PP_TEST_PRINT, "Hello", " world");
    //TODO
}

}
