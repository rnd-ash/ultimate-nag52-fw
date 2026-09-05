#include <cmath>
#include <cstdint>
#include <iostream>

#include "tcu_maths.h"

namespace {

int g_failures = 0;

void expect_eq_i32(const char* name, int32_t actual, int32_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_eq_i16(const char* name, int16_t actual, int16_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_near_f(const char* name, float actual, float expected, float eps = 0.001f) {
    if (std::fabs(actual - expected) > eps) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void test_interpolate_float_linear() {
    expect_near_f("interpolate linear midpoint", interpolate_float(50.0f, 0.0f, 100.0f, 0.0f, 100.0f, InterpType::Linear), 50.0f);
    expect_near_f("interpolate linear low clamp", interpolate_float(-5.0f, 10.0f, 20.0f, 0.0f, 100.0f, InterpType::Linear), 10.0f);
    expect_near_f("interpolate linear high clamp", interpolate_float(150.0f, 10.0f, 20.0f, 0.0f, 100.0f, InterpType::Linear), 20.0f);
}

void test_interpolate_float_inverted_axis() {
    // Function swaps ranges when raw bounds are inverted.
    expect_near_f("interpolate inverted range", interpolate_float(75.0f, 100.0f, 0.0f, 100.0f, 0.0f, InterpType::Linear), 75.0f);
}

void test_interpolate_int() {
    expect_eq_i32("interpolate int midpoint", interpolate_int(50, 0, 100, 0, 100), 50);
    expect_eq_i32("interpolate int low clamp", interpolate_int(-100, 0, 100, 0, 100), 0);
    expect_eq_i32("interpolate int high clamp", interpolate_int(200, 0, 100, 0, 100), 100);
}

void test_first_order_filter() {
    // (new + n*last)/(n+1)
    expect_eq_i32("fof int basic", first_order_filter(3, 100, 0), 25);
    expect_eq_i32("fof int previous weight", first_order_filter(3, 0, 100), 75);
    expect_near_f("fof float basic", first_order_filter_f(3, 100, 0.0f), 25.0f);

    // Guard path for 0xFF -> 0xFE.
    expect_eq_i32("fof int sample_count guard", first_order_filter(0xFF, 255, 0), 1);
    expect_near_f("fof float sample_count guard", first_order_filter_f(0xFF, 255, 0.0f), 1.0f);
}

void test_linear_ramp_with_timer() {
    expect_eq_i32("linear ramp up", linear_ramp_with_timer(0, 100, 4), 25);
    expect_eq_i32("linear ramp down", linear_ramp_with_timer(100, 0, 4), 75);
    expect_eq_i32("linear ramp timer zero", linear_ramp_with_timer(0, 100, 0), 100);
}

void test_linear_interp_with_percentage() {
    expect_eq_i16("lin interp 0%", linear_interp_with_percentage(0, 100, 50), 50);
    expect_eq_i16("lin interp 100%", linear_interp_with_percentage(100, 100, 50), 100);
    expect_eq_i16("lin interp 50%", linear_interp_with_percentage(50, 100, 0), 50);
}

} // namespace

int main() {
    test_interpolate_float_linear();
    test_interpolate_float_inverted_axis();
    test_interpolate_int();
    test_first_order_filter();
    test_linear_ramp_with_timer();
    test_linear_interp_with_percentage();

    if (g_failures == 0) {
        std::cout << "PASS: host_math_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_math_tests failures=" << g_failures << std::endl;
    return 1;
}
