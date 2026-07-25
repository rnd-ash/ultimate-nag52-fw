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

void test_tcc_adaptation_first_cell_respects_minimum() {
    int16_t row[] = {150, 200, 300};

    update_tcc_adaptation_row(row, 3, 0, -100);

    const int16_t expected[] = {100, 200, 300};
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected, row, 3);
}

void test_tcc_adaptation_last_cell_respects_maximum() {
    int16_t row[] = {1000, 14000, 14900};

    update_tcc_adaptation_row(row, 3, 2, 1000);

    const int16_t expected[] = {1000, 14000, 15000};
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected, row, 3);
}

void test_tcc_adaptation_rejects_out_of_range_index() {
    int16_t row[] = {1000, 1200, 1400};

    update_tcc_adaptation_row(row, 3, 0xFF, 100);

    const int16_t expected[] = {1000, 1200, 1400};
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected, row, 3);
}

void test_tcc_adaptation_corrupt_neighbor_cannot_escape_maximum() {
    int16_t row[] = {16000, 14000, 15100};

    update_tcc_adaptation_row(row, 3, 1, -100);

    const int16_t expected[] = {16000, 15000, 15000};
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected, row, 3);
}

extern "C" void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_tcc_adaptation_increase_preserves_monotonic_row);
    RUN_TEST(test_tcc_adaptation_decrease_cannot_invert_row);
    RUN_TEST(test_tcc_adaptation_repairs_downstream_inversions);
    RUN_TEST(test_tcc_adaptation_first_cell_respects_minimum);
    RUN_TEST(test_tcc_adaptation_last_cell_respects_maximum);
    RUN_TEST(test_tcc_adaptation_rejects_out_of_range_index);
    RUN_TEST(test_tcc_adaptation_corrupt_neighbor_cannot_escape_maximum);
    UNITY_END();
}
