// Locks the pedal scaling contract in src/tcu_scaling.h.
//
// Two jobs:
//  1. Pin every conversion to the exact value of the bare literal it replaced
//     (250, 2.5f, 15, 10, 125.0f, 250/4, 64), so the refactor that introduced
//     pedal_pos_t is provably behaviour preserving and stays that way.
//  2. Assert the properties that make the strong type worth having - it is
//     still one byte, and the sentinel cannot collide with a real reading.
//
// What cannot be tested here is the part that matters most: that
// "uint8_t x = pedal;" and "pedal * 2" FAIL TO COMPILE. That is enforced by the
// compiler on every build instead.

#include <cstdint>
#include <iostream>
#include <type_traits>

#include "tcu_scaling.h"

namespace {

int g_failures = 0;

void expect_eq_u32(const char* name, uint32_t actual, uint32_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_near(const char* name, float actual, float expected) {
    const float diff = (actual > expected) ? (actual - expected) : (expected - actual);
    if (diff > 0.0001f) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_true(const char* name, bool value) {
    if (!value) {
        std::cerr << "FAIL: " << name << " expected=true actual=false\n";
        g_failures++;
    }
}

void expect_false(const char* name, bool value) {
    if (value) {
        std::cerr << "FAIL: " << name << " expected=false actual=true\n";
        g_failures++;
    }
}

void test_type_is_zero_overhead_and_strong() {
    expect_eq_u32("pedal_pos_t is one byte", sizeof(pedal_pos_t), sizeof(uint8_t));
    expect_true("underlying type is uint8_t",
        (std::is_same<std::underlying_type<pedal_pos_t>::type, uint8_t>::value));
    // A scoped enum never implicitly converts - that is the whole point.
    expect_false("does not implicitly convert to its underlying type",
        (std::is_convertible<pedal_pos_t, uint8_t>::value));
    expect_false("does not implicitly convert from its underlying type",
        (std::is_convertible<uint8_t, pedal_pos_t>::value));
}

void test_full_scale_is_250_not_100_or_255() {
    // The reason this header exists. HfmCan used to return 0-100.
    expect_eq_u32("pedal full scale is 250", Pedal::raw_u8(Pedal::MAX), 250u);
    expect_near("full scale converts to 100%", Pedal::to_percent(Pedal::MAX), 100.0f);
    expect_near("zero converts to 0%", Pedal::to_percent(Pedal::ZERO), 0.0f);
}

void test_to_percent_matches_the_literals_it_replaced() {
    // profiles.cpp used "pedal_pos / 2.5f"
    // torque_converter.cpp used "(pedal_pos * 100) / 250"
    // profiles.cpp also used "((float)pedal_pos * 100.0f) / 250.0f"
    for (uint32_t raw = 0u; raw <= 250u; raw++) {
        const float via_helper = Pedal::to_percent(Pedal::from_raw((uint8_t)raw));
        const float via_divide = (float)raw / 2.5f;
        const float via_multiply = ((float)raw * 100.0f) / 250.0f;
        const int via_integer = (int)((raw * 100u) / 250u);

        if (via_helper != via_divide || via_helper != via_multiply) {
            std::cerr << "FAIL: to_percent disagrees with original literals at raw=" << raw << "\n";
            g_failures++;
            return;
        }
        if ((int)via_helper != via_integer) {
            std::cerr << "FAIL: to_percent truncates differently to the old integer form at raw=" << raw << "\n";
            g_failures++;
            return;
        }
    }
}

void test_roundtrip_is_lossless() {
    for (uint32_t raw = 0u; raw <= 255u; raw++) {
        if (Pedal::raw_u8(Pedal::from_raw((uint8_t)raw)) != (uint8_t)raw) {
            std::cerr << "FAIL: from_raw/raw roundtrip lost data at " << raw << "\n";
            g_failures++;
            return;
        }
    }
}

void test_percent_thresholds_match_the_literals_they_replaced() {
    // Each of these MUST reproduce the raw literal exactly - changing any of
    // them silently retunes shift behaviour.
    expect_eq_u32("6% is 15 raw (torque_converter, shifting_algo_helpers)", Pedal::raw_u8(Pedal::percent(6.0f)), 15u);
    expect_eq_u32("4% is 10 raw (shift_release coast test)", Pedal::raw_u8(Pedal::percent(4.0f)), 10u);
    expect_eq_u32("25% is 62 raw (gearbox no-signal fallback, was 250/4)", Pedal::raw_u8(Pedal::percent(25.0f)), 250u / 4u);
    expect_eq_u32("50% is 125 raw (shift_release adder ramp)", Pedal::raw_u8(Pedal::percent(50.0f)), 125u);
    expect_eq_u32("25.6% is 64 raw (profiles kickdown jump)", Pedal::raw_u8(Pedal::percent(25.6f)), 64u);
    expect_eq_u32("100% is full scale", Pedal::raw_u8(Pedal::percent(100.0f)), Pedal::raw_u8(Pedal::MAX));
    expect_eq_u32("0% is zero", Pedal::raw_u8(Pedal::percent(0.0f)), 0u);
}

void test_validity_sentinel() {
    // get_pedal_value() returns Pedal::INVALID when there is no reading. It sits
    // OUTSIDE the 0-250 range, so it can never collide with a real value.
    expect_eq_u32("invalid sentinel is 0xFF", Pedal::raw_u8(Pedal::INVALID), 0xFFu);
    expect_false("sentinel is not a valid reading", Pedal::is_valid(Pedal::INVALID));
    expect_true("full scale is a valid reading", Pedal::is_valid(Pedal::MAX));
    expect_true("zero is a valid reading", Pedal::is_valid(Pedal::ZERO));
    // Comparison between two pedal_pos_t needs no cast - this must keep compiling.
    expect_true("sentinel is above full scale", Pedal::INVALID > Pedal::MAX);
}

} // namespace

int main() {
    test_type_is_zero_overhead_and_strong();
    test_full_scale_is_250_not_100_or_255();
    test_to_percent_matches_the_literals_it_replaced();
    test_roundtrip_is_lossless();
    test_percent_thresholds_match_the_literals_they_replaced();
    test_validity_sentinel();

    if (g_failures == 0) {
        std::cout << "PASS: host_pedal_scaling_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_pedal_scaling_tests failures=" << g_failures << std::endl;
    return 1;
}
