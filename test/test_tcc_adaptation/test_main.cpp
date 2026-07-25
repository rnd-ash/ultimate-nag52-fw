#include <unity.h>

#include "tcc_adaptation.h"

void test_tcc_adaptation_increase_preserves_monotonic_row() {
    int16_t row[] = {1000, 1200, 1400, 1500, 1600};

    update_tcc_adaptation_row(row, 5, 2, 300);

    const int16_t expected[] = {1000, 1200, 1700, 1700, 1700};
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected, row, 5);
}

void test_tcc_adaptation_decrease_cannot_invert_row() {
    int16_t row[] = {1000, 1200, 1400, 1600, 1800};

    update_tcc_adaptation_row(row, 5, 2, -300);

    const int16_t expected[] = {1000, 1200, 1200, 1600, 1800};
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected, row, 5);
}

void test_tcc_adaptation_repairs_downstream_inversions() {
    int16_t row[] = {898, 1173, 929, 1169, 1287, 1283, 1368};

    update_tcc_adaptation_row(row, 7, 2, 1);

    const int16_t expected[] = {898, 1173, 1173, 1173, 1287, 1287, 1368};
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected, row, 7);
}

extern "C" void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_tcc_adaptation_increase_preserves_monotonic_row);
    RUN_TEST(test_tcc_adaptation_decrease_cannot_invert_row);
    RUN_TEST(test_tcc_adaptation_repairs_downstream_inversions);
    UNITY_END();
}
