#include <sofa/helper/preprocessor.h>
#include <gtest/gtest.h>
#include <cstdio>

namespace
{

#define SOFA_PP_TEST_ADD_VECTOR(Value)  \
 text_vector.push_back(Value);

TEST(SofaPreprocessorTest, VectorSize)
{
    std::vector<const char*> text_vector;

    SOFA_FOR_EACH(SOFA_PP_TEST_ADD_VECTOR, SOFA_EMPTY_DELIMITER, "I", "am", "afraid", "I", "can't", "do", "that", "Dave")
    EXPECT_EQ(8, text_vector.size());
}

TEST(SofaPreprocessorTest, ToString)
{
    std::vector<const char*> text_vector;
    SOFA_FOR_EACH(SOFA_PP_TEST_ADD_VECTOR, SOFA_EMPTY_DELIMITER,
     SOFA_TO_STRING(Never, send, a, human, to, do, a, machine s, job))
        EXPECT_EQ(9, text_vector.size());
}

}
