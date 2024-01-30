#include "moving_average.h"
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void test_ma_back() {
    MovingUnsignedAverage* ma = new MovingUnsignedAverage(3, true); // 3 elements (Allocate to IRAM as we don't have PSRAM in test ENV)
    TEST_ASSERT_EQUAL_INT(ma->back(), 0);
    ma->add_sample(1);
    ma->add_sample(2);
    TEST_ASSERT_EQUAL_INT(ma->back(), 1);
    ma->add_sample(3);
    TEST_ASSERT_EQUAL_INT(ma->back(), 1);
    ma->add_sample(4);
    TEST_ASSERT_EQUAL_INT(ma->back(), 2);
}

void test_ma_front() {
    MovingUnsignedAverage* ma = new MovingUnsignedAverage(3, true); // 3 elements (Allocate to IRAM as we don't have PSRAM in test ENV)
    TEST_ASSERT_EQUAL_INT(ma->front(), 0);
    // Push a value
    ma->add_sample(1);
    TEST_ASSERT_EQUAL_INT(ma->front(), 1);
    // Push another value
    ma->add_sample(2);
    TEST_ASSERT_EQUAL_INT(ma->front(), 2);
    // Push another value
    ma->add_sample(3);
    TEST_ASSERT_EQUAL_INT(ma->front(), 3);
    // Push another value (Overwrite idx 0)
    ma->add_sample(4);
    TEST_ASSERT_EQUAL_INT(ma->front(), 4);
}

extern "C" void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ma_back);
    RUN_TEST(test_ma_front);
    UNITY_END();
}