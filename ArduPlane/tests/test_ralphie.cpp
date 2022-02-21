#include <AP_gtest.h>

#include "../matrixMath.h"
#include "../RALPHIE_UNIT_TEST.h"


TEST(GTEST_RALPHIE, SimpleTest)
{
    EXPECT_EQ(1, 1);
}

TEST(GTEST_RALPHIE, ClassSimpleTest)
{
    EXPECT_EQ(1, 1);
}

TEST(GTEST_RALPHIE, matrixMathFuncsTest)
{
    matrixMathFuncs* testVar = new matrixMathFuncs();
    EXPECT_FALSE(testVar -> matrixMathTest(1));
}

AP_GTEST_MAIN()
