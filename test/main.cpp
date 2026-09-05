#include <unity.h>
#include <type_traits>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tcu_maths.h"
#include "stored_data.h"
#include "torque_converter.h"
#include "pressure_manager.h"
#include "profiles.h"
#include "adaptation/shift_adaptation.h"
#include "diag/kwp2000.h"
#include "gearbox.h"
#include "shifter/shifter_ewm.h"
#include "shifter/shifter_trrs.h"

// Compile-time regression guards for ownership semantics.
static_assert(std::has_virtual_destructor<StoredData>::value, "StoredData must have a virtual destructor");
static_assert(!std::is_copy_constructible<TorqueConverter>::value, "TorqueConverter must remain non-copyable");
static_assert(!std::is_copy_constructible<PressureManager>::value, "PressureManager must remain non-copyable");
static_assert(!std::is_copy_constructible<AbstractProfile>::value, "AbstractProfile must remain non-copyable");
static_assert(!std::is_copy_constructible<ShiftAdaptationSystem>::value, "ShiftAdaptationSystem must remain non-copyable");
static_assert(!std::is_copy_constructible<Kwp2000_server>::value, "Kwp2000_server must remain non-copyable");
static_assert(!std::is_copy_constructible<Gearbox>::value, "Gearbox must remain non-copyable");
static_assert(!std::is_copy_constructible<ShifterEwm>::value, "ShifterEwm must remain non-copyable");
static_assert(!std::is_copy_constructible<ShifterTrrs>::value, "ShifterTrrs must remain non-copyable");

void test_first_order_filter_int() {
    // Weighted average: (new + n*last)/(n+1)
    TEST_ASSERT_EQUAL_INT32(25, first_order_filter(3, 100, 0));
    TEST_ASSERT_EQUAL_INT32(75, first_order_filter(3, 0, 100));
    // Guard branch for 0xFF sample_count should behave like 0xFE.
    TEST_ASSERT_EQUAL_INT32(0, first_order_filter(0xFF, 255, 0));
}

void test_first_order_filter_float() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, first_order_filter_f(3, 100, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 75.0f, first_order_filter_f(3, 0, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, first_order_filter_f(0xFF, 255, 0.0f));
}

void test_ownership_contracts() {
    TEST_ASSERT_TRUE(std::has_virtual_destructor<StoredData>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<TorqueConverter>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<PressureManager>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<AbstractProfile>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<ShiftAdaptationSystem>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<Kwp2000_server>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<Gearbox>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<ShifterEwm>::value);
    TEST_ASSERT_FALSE(std::is_copy_constructible<ShifterTrrs>::value);
}

extern "C" void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_first_order_filter_int);
    RUN_TEST(test_first_order_filter_float);
    RUN_TEST(test_ownership_contracts);
    UNITY_END();
}