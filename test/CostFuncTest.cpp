#include <gtest/gtest.h>

#include "CostFuncTest.h"


int main(int argc, char** argv)
{
    using namespace orca::test;
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}