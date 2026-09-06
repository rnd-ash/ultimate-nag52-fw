// Locks the wheel speed scaling contract in src/tcu_scaling.h.
//
// Every Mercedes bus reports wheel speed at TWICE the real RPM. Before
// wheel_rpm_2x_t, that factor lived only in field names and comments, and the
// two places that had to undo it did so very differently:
//
//   Shifter::set_vehicle_speed()  (front_left + front_right) >> 2
//   TCUIO::update_rpm_sensors()   (rl + rr) / 2 ... then /= 2, forty lines later
//
// Both are correct. Neither says so at the point of use. These tests pin the
// named helpers to those expressions so the equivalence is checked rather than
// remembered.
//
// Shaft speeds (N2, N3, turbine, output, engine) deliberately stay plain
// uint16_t - they are all the same unit with no hidden scaling, so a type there
// would add unwrapping noise without catching anything.

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
    expect_eq_u32("wheel_rpm_2x_t is two bytes", sizeof(wheel_rpm_2x_t), sizeof(uint16_t));
    expect_true("underlying type is uint16_t",
        (std::is_same<std::underlying_type<wheel_rpm_2x_t>::type, uint16_t>::value));
    expect_false("does not implicitly convert to its underlying type",
        (std::is_convertible<wheel_rpm_2x_t, uint16_t>::value));
    expect_false("does not implicitly convert from its underlying type",
        (std::is_convertible<uint16_t, wheel_rpm_2x_t>::value));
    // The point of the type: a doubled wheel speed must not be usable anywhere
    // a plain RPM is expected without going through to_rpm_u16().
    expect_false("a doubled wheel speed is not a torque",
        (std::is_convertible<wheel_rpm_2x_t, torque_nm_t>::value));
}

void test_roundtrip_and_sentinel() {
    for (uint32_t raw = 0u; raw <= 65535u; raw += 7u) {
        if (WheelSpeed::raw_2x_u16(WheelSpeed::from_raw_2x((uint16_t)raw)) != (uint16_t)raw) {
            std::cerr << "FAIL: from_raw_2x/raw_2x_u16 roundtrip lost data at " << raw << "\n";
            g_failures++;
            return;
        }
    }
    expect_eq_u32("invalid sentinel is UINT16_MAX", WheelSpeed::raw_2x_u16(WheelSpeed::INVALID), 65535u);
    expect_eq_u32("zero is stopped", WheelSpeed::raw_2x_u16(WheelSpeed::ZERO), 0u);
    expect_false("sentinel is not a valid reading", WheelSpeed::is_valid(WheelSpeed::INVALID));
    expect_true("zero is a valid reading", WheelSpeed::is_valid(WheelSpeed::ZERO));
    expect_true("a normal reading is valid", WheelSpeed::is_valid(WheelSpeed::from_raw_2x(1200u)));
}

void test_to_rpm_halves_the_bus_value() {
    // diag_data.cpp used "raw; if (UINT16_MAX != raw) raw /= 2;"
    // tcu_io.cpp used a trailing "calc_rpm /= 2; // Since wheel speed is 2x"
    for (uint32_t raw = 0u; raw < 65535u; raw++) {
        const uint16_t via_helper = WheelSpeed::to_rpm_u16(WheelSpeed::from_raw_2x((uint16_t)raw));
        const uint16_t via_literal = (uint16_t)(raw / 2u);
        if (via_helper != via_literal) {
            std::cerr << "FAIL: to_rpm_u16 disagrees with the old /2 at raw=" << raw << "\n";
            g_failures++;
            return;
        }
    }
    expect_eq_u32("1200 on the bus is 600 real RPM", WheelSpeed::to_rpm_u16(WheelSpeed::from_raw_2x(1200u)), 600u);
    expect_eq_u32("a stopped wheel is 0 RPM", WheelSpeed::to_rpm_u16(WheelSpeed::ZERO), 0u);
}

void test_mean_rpm_matches_the_shift_by_two_it_replaced() {
    // Shifter::set_vehicle_speed() used "(front_left + front_right) >> 2".
    // It is a shift by TWO because BOTH inputs are doubled - which is exactly
    // the kind of thing that reads like an off-by-one when written bare.
    for (uint32_t l = 0u; l <= 65000u; l += 251u) {
        for (uint32_t r = 0u; r <= 65000u; r += 997u) {
            const uint16_t via_helper = WheelSpeed::mean_rpm_u16(
                WheelSpeed::from_raw_2x((uint16_t)l), WheelSpeed::from_raw_2x((uint16_t)r));
            const uint16_t via_literal = (uint16_t)((l + r) >> 2);
            if (via_helper != via_literal) {
                std::cerr << "FAIL: mean_rpm_u16 disagrees with the old >>2 at l=" << l << " r=" << r << "\n";
                g_failures++;
                return;
            }
        }
    }
    // Two wheels both reading 1200 on the bus are both turning 600 RPM, so the
    // axle mean is 600 - NOT 1200, and not 300.
    expect_eq_u32("equal wheels give that wheel's real RPM",
        WheelSpeed::mean_rpm_u16(WheelSpeed::from_raw_2x(1200u), WheelSpeed::from_raw_2x(1200u)), 600u);
    expect_eq_u32("the mean sits between the two real speeds",
        WheelSpeed::mean_rpm_u16(WheelSpeed::from_raw_2x(1000u), WheelSpeed::from_raw_2x(1400u)), 600u);
    // The sum of two near-full-scale readings must not wrap a uint16_t.
    expect_eq_u32("a wide sum does not overflow",
        WheelSpeed::mean_rpm_u16(WheelSpeed::from_raw_2x(65000u), WheelSpeed::from_raw_2x(65000u)), 32500u);
}

} // namespace

int main() {
    test_type_is_zero_overhead_and_strong();
    test_roundtrip_and_sentinel();
    test_to_rpm_halves_the_bus_value();
    test_mean_rpm_matches_the_shift_by_two_it_replaced();

    if (g_failures == 0) {
        std::cout << "PASS: host_wheel_speed_scaling_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_wheel_speed_scaling_tests failures=" << g_failures << std::endl;
    return 1;
}
