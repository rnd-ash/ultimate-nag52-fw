#include "tcm_maths.h"
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void test_map(void) {

    int16_t x_headers[7] = {1, 2, 4, 8, 16, 32, 64};
    int16_t y_headers[4] = {-5, 0, 5, 10};

    int16_t data[7*4] = {
        0  , 100,  200,  400,  800, 1600, 3200,
        100, 200,  400,  800, 1600, 3200, 6400,
        200, 400,  800, 1600, 3200, 6400, 6400,
        400, 800, 1600, 3200, 6400, 6400, 6400
    };

    TcmMap test_map = TcmMap(7, 4, x_headers, y_headers);
    UNITY_TEST_ASSERT(test_map.allocate_ok(), TEST_LINE_NUM, "Map allocation failed!");
    UNITY_TEST_ASSERT(test_map.add_data(data, 7*4), TEST_LINE_NUM, "Map add data failed!");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(200, test_map.get_value(2, 0), TEST_LINE_NUM, "Data mismatch point lookup");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(300, test_map.get_value(3, 0), TEST_LINE_NUM, "Data mismatch X 50/50 interpolate!");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(150, test_map.get_value(2, -2.5), TEST_LINE_NUM, "Data mismatch Y 50/50 interpolate!");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(500, test_map.get_value(5, 0), TEST_LINE_NUM, "Data mismatch X 25/75 interpolate!");
    UNITY_TEST_ASSERT_EQUAL_FLOAT(500, test_map.get_value(2, 6.25), TEST_LINE_NUM, "Data mismatch Y 25/75 interpolate!");

    // Reverse table lookup
    UNITY_TEST_ASSERT_EQUAL_INT16(150, test_map.lookup_value(800, 4, INT16_MAX), TEST_LINE_NUM, "Reverse lookup Y failed!");
    UNITY_TEST_ASSERT_EQUAL_INT16(150, test_map.lookup_value(400, INT16_MAX, 10), TEST_LINE_NUM, "Reverse lookup X failed!");
}

void test_scale(void) {
    int new_max_large = 100;
    int new_min_large = 0;

    int new_max_small = 3;
    int new_min_small = 1;

    int old_max = 50;
    int old_min = 10;

    UNITY_TEST_ASSERT_EQUAL_FLOAT(50, scale_number(30, new_min_large, new_max_large, old_min, old_max), TEST_LINE_NUM, "Scale invalid");
    // Should be clipped
    UNITY_TEST_ASSERT_EQUAL_FLOAT(0, scale_number(-10, new_min_large, new_max_large, old_min, old_max), TEST_LINE_NUM, "Scale invalid");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(2, scale_number(30, new_min_small, new_max_small, old_min, old_max), TEST_LINE_NUM, "Scale invalid");


} 


extern "C" void app_main(void) {
    vTaskDelay(100);
    UNITY_BEGIN();
    RUN_TEST(test_map);
    RUN_TEST(test_scale);
    UNITY_END();
}