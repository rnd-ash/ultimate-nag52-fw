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

    int16_t* ptr = test_map.get_row(1);

    UNITY_TEST_ASSERT(ptr != nullptr, TEST_LINE_NUM, "Map get row pointer was null");
    int16_t* ptr_src = &data[7];
    for (int i = 0; i < 7; i++) {
        UNITY_TEST_ASSERT_EQUAL_INT16(ptr_src[i], ptr[i], TEST_LINE_NUM, "Data mismatch!");
    }
    UNITY_TEST_ASSERT_EQUAL_FLOAT(200, test_map.get_value(2, 0), TEST_LINE_NUM, "Data mismatch point lookup");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(300, test_map.get_value(3, 0), TEST_LINE_NUM, "Data mismatch X 50/50 interpolate!");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(150, test_map.get_value(2, -2.5), TEST_LINE_NUM, "Data mismatch Y 50/50 interpolate!");

    UNITY_TEST_ASSERT_EQUAL_FLOAT(500, test_map.get_value(5, 0), TEST_LINE_NUM, "Data mismatch X 25/75 interpolate!");
    UNITY_TEST_ASSERT_EQUAL_FLOAT(500, test_map.get_value(2, 6.25), TEST_LINE_NUM, "Data mismatch Y 25/75 interpolate!");

}


extern "C" void app_main(void) {
    vTaskDelay(100);
    UNITY_BEGIN();
    RUN_TEST(test_map);
    UNITY_END();
}