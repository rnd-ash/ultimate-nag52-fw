#include "tcm_maths.h"
#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void test_map(void) {

    int16_t x_headers[7] = {5, 60, 100, 235, 560, 660, 770};
    int16_t y_headers[4] = {-25, 20, 60, 150};

    int16_t data[7*4] = {
        1100, 1085, 954, 700, 450, 350, 200,
        1077,  925, 830, 675, 415, 320,   0,
        1000,  835, 780, 650, 400, 288,   0,
         975,  795, 745, 625, 370, 260,   0
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
    UNITY_TEST_ASSERT_EQUAL_FLOAT(925.0, test_map.get_value(60, 20), TEST_LINE_NUM, "Data mismatch point lookup"); // 60mbar, 20C - Both X and Y are on points in table

    UNITY_TEST_ASSERT_EQUAL_FLOAT(877.5, test_map.get_value(80, 20), TEST_LINE_NUM, "Data mismatch X interpolate!"); // 80mbar, 20C - X axis linear interpolate

    UNITY_TEST_ASSERT_EQUAL_FLOAT(880, test_map.get_value(60, 40), TEST_LINE_NUM, "Data mismatch Y interpolate!");// 60mbar, 40C - Y axis linear interpolation

}


extern "C" void app_main(void) {
    vTaskDelay(100);
    UNITY_BEGIN();
    RUN_TEST(test_map);
    UNITY_END();
}