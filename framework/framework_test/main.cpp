#include <iostream>
#include "gtest/gtest.h"
#include <sofa/helper/test.h>

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    // Set LC_CTYPE according to the environnement variable.
    setlocale(LC_CTYPE, "");

    sofa::helper::initBeforeTests();
    int ret = RUN_ALL_TESTS();
    sofa::helper::cleanupAfterTests();

    return ret;
}
