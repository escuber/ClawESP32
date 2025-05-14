#include <unity.h>
#include "IMU/WheelieLogic.h"


void setUp() {}
void tearDown() {}

void test_wheelie_suppression_triggered() {
    int result = evaluateThrottle(1700, 25.0, 0.1, 900);
    TEST_ASSERT_INT_WITHIN(10, 1540, result); // Suppressed to ~20% over neutral
}

void test_wheelie_not_triggered() {
    int result = evaluateThrottle(1700, 10.0, 1.0, 300);
    TEST_ASSERT_EQUAL(1700, result);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_wheelie_suppression_triggered);
    RUN_TEST(test_wheelie_not_triggered);
    return UNITY_END();
}