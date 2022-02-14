#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include "../../../ArduPlane/RALPHIE_UNIT_TEST.h"


TEST(GTEST_RALPHIE, SimpleTest)
{
    EXPECT_EQ(1, 1);
}

TEST(GTEST_RALPHIE, classSimpleTest)
{
    RALPHIE RALPHIEUnitTest;
    EXPECT_EQ(RALPHIE.testUNIT(1,2), 3)
}

AP_GTEST_MAIN()
