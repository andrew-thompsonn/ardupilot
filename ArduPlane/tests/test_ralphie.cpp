#include <AP_gtest.h>

#include "../RALPHIE_UNIT_TEST.h"


TEST(GTEST_RALPHIE, SimpleTest)
{
    EXPECT_EQ(1, 1);
}

TEST(GTEST_RALPHIE, ClassSimpleTest)
{
    RALPHIEUnitTest ralphieTest;
    ralphieTest.testUNIT(1.0, 1.0);
    EXPECT_EQ(1, 1);
}

AP_GTEST_MAIN()
