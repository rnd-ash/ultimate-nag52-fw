// Locks the temperature scaling contract in src/tcu_scaling.h.
//
// Temperature is carried through the TCU in plain Celsius but leaves it in
// three different encodings, and the encodings differ only by a small integer
// offset - which is exactly why they were so easy to transpose before temp_c_t
// existed. These tests pin each conversion to the literal expression it
// replaced, and pin the one place where the old open-coded form was wrong.
//
// As with the pedal tests, the most valuable property cannot be asserted here:
// that "int16_t x = temp;" and "temp + 50" FAIL TO COMPILE. The compiler
// enforces that on every build instead.

#include <cstdint>
#include <iostream>
#include <type_traits>

#include "tcu_scaling.h"

namespace {

int g_failures = 0;

void expect_eq_i32(const char* name, int32_t actual, int32_t expected) {
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
    expect_eq_i32("temp_c_t is two bytes", (int32_t)sizeof(temp_c_t), (int32_t)sizeof(int16_t));
    expect_true("underlying type is int16_t",
        (std::is_same<std::underlying_type<temp_c_t>::type, int16_t>::value));
    expect_false("does not implicitly convert to its underlying type",
        (std::is_convertible<temp_c_t, int16_t>::value));
    expect_false("does not implicitly convert from its underlying type",
        (std::is_convertible<int16_t, temp_c_t>::value));
    // A temperature must never be assignable from a pedal position, and vice
    // versa. Before these types existed both were bare integers.
    expect_false("a pedal position is not a temperature",
        (std::is_convertible<pedal_pos_t, temp_c_t>::value));
    expect_false("a temperature is not a pedal position",
        (std::is_convertible<temp_c_t, pedal_pos_t>::value));
}

void test_roundtrip_is_lossless() {
    for (int32_t c = -300; c <= 300; c++) {
        if (Temp::celsius_i16(Temp::from_celsius((int16_t)c)) != (int16_t)c) {
            std::cerr << "FAIL: from_celsius/celsius_i16 roundtrip lost data at " << c << "\n";
            g_failures++;
            return;
        }
    }
}

void test_validity_sentinel() {
    // Every get_engine_*_temp() and TCUIO::atf_temperature() returns this when
    // there is no reading. It has to stay INT16_MAX because that is what the
    // open-coded "INT16_MAX == temp" checks used across the TCU compared to.
    expect_eq_i32("invalid sentinel is INT16_MAX", Temp::celsius_i16(Temp::INVALID), INT16_MAX);
    expect_false("sentinel is not a valid reading", Temp::is_valid(Temp::INVALID));
    expect_true("a real reading is valid", Temp::is_valid(Temp::from_celsius(90)));
    expect_true("a sub-zero reading is valid", Temp::is_valid(Temp::from_celsius(-40)));
    // Comparison between two temp_c_t needs no cast - this must keep compiling,
    // because the shift algorithms compare against thresholds this way.
    expect_true("-10C is colder than 30C", Temp::from_celsius(-10) < Temp::from_celsius(30));
}

void test_engine_can_decode_matches_the_literal_it_replaced() {
    // Nine decode sites in can_egs51/52/53 and can_custom used "(int16_t)raw - 40".
    for (uint32_t raw = 0u; raw <= 255u; raw++) {
        const int16_t via_helper = Temp::celsius_i16(Temp::from_can_u8_offset40((uint8_t)raw));
        const int16_t via_literal = (int16_t)((int16_t)raw - 40);
        if (via_helper != via_literal) {
            std::cerr << "FAIL: from_can_u8_offset40 disagrees with the old literal at raw=" << raw << "\n";
            g_failures++;
            return;
        }
    }
    expect_eq_i32("raw 0 decodes to -40C", Temp::celsius_i16(Temp::from_can_u8_offset40(0u)), -40);
    expect_eq_i32("raw 40 decodes to 0C", Temp::celsius_i16(Temp::from_can_u8_offset40(40u)), 0);
    expect_eq_i32("raw 130 decodes to 90C", Temp::celsius_i16(Temp::from_can_u8_offset40(130u)), 90);
}

void test_gearbox_can_encode_saturates_at_both_ends() {
    // The open-coded form was "(uint8_t)(((MAX(temp, -50) + 50) & 0xFF))". It
    // clamped the bottom but masked the top, so anything above 205C wrapped
    // around the byte - 206C reported as 0C, i.e. stone cold on the cluster.
    // In the normal range the helper must reproduce it exactly.
    for (int32_t c = -50; c <= 205; c++) {
        const uint8_t via_helper = Temp::to_can_u8_offset50(Temp::from_celsius((int16_t)c));
        const uint8_t via_literal = (uint8_t)(((c < -50 ? -50 : c) + 50) & 0xFF);
        if (via_helper != via_literal) {
            std::cerr << "FAIL: to_can_u8_offset50 disagrees with the old literal at " << c << "C\n";
            g_failures++;
            return;
        }
    }
    expect_eq_i32("-50C encodes to 0", Temp::to_can_u8_offset50(Temp::from_celsius(-50)), 0);
    expect_eq_i32("0C encodes to 50", Temp::to_can_u8_offset50(Temp::from_celsius(0)), 50);
    expect_eq_i32("90C encodes to 140", Temp::to_can_u8_offset50(Temp::from_celsius(90)), 140);
    expect_eq_i32("205C encodes to 255 (top of range)", Temp::to_can_u8_offset50(Temp::from_celsius(205)), 255);
    // The two ends the old expression got wrong:
    expect_eq_i32("-60C saturates low instead of clamping to -50 then adding",
        Temp::to_can_u8_offset50(Temp::from_celsius(-60)), 0);
    expect_eq_i32("206C saturates high instead of wrapping to 0",
        Temp::to_can_u8_offset50(Temp::from_celsius(206)), 255);
    expect_eq_i32("the INVALID sentinel saturates high, it never looks cold",
        Temp::to_can_u8_offset50(Temp::INVALID), 255);
}

void test_cal_axis_matches_the_plus_50_it_replaced() {
    // pressure_manager used a bare "+ 50.0f" against the factory calibration
    // maps (pcs_map_y, mpc_flush_temp_threshold, atf_density_minus_50c), whose
    // axis origin is -50C rather than 0C.
    for (int32_t c = -60; c <= 220; c++) {
        const float via_helper = Temp::to_cal_axis(Temp::from_celsius((int16_t)c));
        const float via_literal = (float)c + 50.0f;
        if (via_helper != via_literal) {
            std::cerr << "FAIL: to_cal_axis disagrees with the old literal at " << c << "C\n";
            g_failures++;
            return;
        }
    }
    expect_near("-50C is the origin of the calibration axis", Temp::to_cal_axis(Temp::from_celsius(-50)), 0.0f);
    expect_near("0C is 50 on the calibration axis", Temp::to_cal_axis(Temp::from_celsius(0)), 50.0f);
    expect_eq_i32("the cal axis offset is 50", Temp::CAL_AXIS_OFFSET_C, 50);
}

} // namespace

int main() {
    test_type_is_zero_overhead_and_strong();
    test_roundtrip_is_lossless();
    test_validity_sentinel();
    test_engine_can_decode_matches_the_literal_it_replaced();
    test_gearbox_can_encode_saturates_at_both_ends();
    test_cal_axis_matches_the_plus_50_it_replaced();

    if (g_failures == 0) {
        std::cout << "PASS: host_temp_scaling_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_temp_scaling_tests failures=" << g_failures << std::endl;
    return 1;
}
