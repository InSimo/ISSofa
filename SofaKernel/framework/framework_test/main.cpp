#include <sofa/core/init.h>
#include <sofa/helper/test.h>

#include <gtest/gtest.h>

#include <iostream>

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    sofa::core::init();
    sofa::helper::initBeforeTests();
    int ret = RUN_ALL_TESTS();
    sofa::helper::cleanupAfterTests();
    sofa::core::cleanup();
    return ret;
}
