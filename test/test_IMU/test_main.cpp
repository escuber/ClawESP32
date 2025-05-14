#include <unity.h>
#include "IMU/IMUManager2.h"

void setUp() {}
void tearDown() {}

void test_getPitch_returns_value() {
    int p = getPitch();
    TEST_ASSERT_INT_WITHIN(180, 0, p); // Should be within real pitch range
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_getPitch_returns_value);
    return UNITY_END();
}