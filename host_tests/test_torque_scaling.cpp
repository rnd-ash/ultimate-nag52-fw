// Locks the torque scaling contract in src/tcu_scaling.h.
//
// Torque leaves the TCU as an unsigned CAN field encoded (Nm + 500) * 4. The
// decode side appeared open-coded at fifteen sites; the encode side at two, and
// one of those (EGS53 EngTrq_Rq_TCM) had no clamp at all, so an out of range
// request wrapped inside a 13 bit field - a large positive demand could reach
// the engine as a large NEGATIVE one. These tests pin the decode to the literal
// it replaced and pin the encode saturation that fixes the wrap.

#include <cstdint>
#include <cmath>
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
    expect_eq_i32("torque_nm_t is two bytes", (int32_t)sizeof(torque_nm_t), (int32_t)sizeof(int16_t));
    expect_true("underlying type is int16_t",
        (std::is_same<std::underlying_type<torque_nm_t>::type, int16_t>::value));
    expect_false("does not implicitly convert to its underlying type",
        (std::is_convertible<torque_nm_t, int16_t>::value));
    expect_false("does not implicitly convert from its underlying type",
        (std::is_convertible<int16_t, torque_nm_t>::value));
    // Torque and temperature are both int16_t underneath. Before these types
    // existed, nothing stopped one being assigned from the other.
    expect_false("a temperature is not a torque",
        (std::is_convertible<temp_c_t, torque_nm_t>::value));
    expect_false("a torque is not a temperature",
        (std::is_convertible<torque_nm_t, temp_c_t>::value));
}

void test_roundtrip_and_sentinel() {
    for (int32_t nm = -600; nm <= 1600; nm++) {
        if (Torque::nm_i16(Torque::from_nm((int16_t)nm)) != (int16_t)nm) {
            std::cerr << "FAIL: from_nm/nm_i16 roundtrip lost data at " << nm << "\n";
            g_failures++;
            return;
        }
    }
    expect_eq_i32("invalid sentinel is INT16_MAX", Torque::nm_i16(Torque::INVALID), INT16_MAX);
    expect_eq_i32("zero is 0 Nm", Torque::nm_i16(Torque::ZERO), 0);
    expect_false("sentinel is not a valid reading", Torque::is_valid(Torque::INVALID));
    expect_true("zero is a valid reading", Torque::is_valid(Torque::ZERO));
    expect_true("negative overrun torque is a valid reading", Torque::is_valid(Torque::from_nm(-120)));
    // Comparison between two torque_nm_t needs no cast - the shift algorithms
    // compare indicated against min torque this way.
    expect_true("-120Nm is less than 0Nm", Torque::from_nm(-120) < Torque::ZERO);
}

void test_can_decode_matches_the_literal_it_replaced() {
    // Fifteen sites used "(raw / 4) - 500" over a 13 bit field.
    for (int32_t raw = 0; raw <= 8191; raw++) {
        const int16_t via_helper = Torque::nm_i16(Torque::from_can_raw(raw));
        const int16_t via_literal = (int16_t)((raw / 4) - 500);
        if (via_helper != via_literal) {
            std::cerr << "FAIL: from_can_raw disagrees with the old literal at raw=" << raw << "\n";
            g_failures++;
            return;
        }
    }
    expect_eq_i32("raw 0 decodes to -500Nm", Torque::nm_i16(Torque::from_can_raw(0)), -500);
    expect_eq_i32("raw 2000 decodes to 0Nm", Torque::nm_i16(Torque::from_can_raw(2000)), 0);
    expect_eq_i32("raw 3200 decodes to 300Nm", Torque::nm_i16(Torque::from_can_raw(3200)), 300);
    expect_eq_i32("the encoding is 4 counts per Nm", Torque::CAN_COUNTS_PER_NM, 4);
    expect_eq_i32("the encoding zero point is -500Nm", Torque::CAN_OFFSET_NM, 500);
}

void test_can_encode_is_the_inverse_of_decode() {
    for (int32_t nm = -500; nm <= 1547; nm++) {
        const uint16_t raw = Torque::to_can_raw((float)nm, Torque::CAN_RAW_MAX_13BIT);
        if (Torque::nm_i16(Torque::from_can_raw(raw)) != (int16_t)nm) {
            std::cerr << "FAIL: encode/decode is not an identity at " << nm << "Nm\n";
            g_failures++;
            return;
        }
    }
}

void test_can_encode_saturates_instead_of_wrapping() {
    expect_eq_i32("13 bit field max is 8191", Torque::CAN_RAW_MAX_13BIT, 8191);
    expect_eq_i32("-500Nm is the bottom of the range", Torque::to_can_raw(-500.0f, Torque::CAN_RAW_MAX_13BIT), 0);
    expect_eq_i32("0Nm encodes to 2000", Torque::to_can_raw(0.0f, Torque::CAN_RAW_MAX_13BIT), 2000);
    expect_eq_i32("1547Nm is the top of the range", Torque::to_can_raw(1547.0f, Torque::CAN_RAW_MAX_13BIT), 8188);

    // The two cases the unclamped "(amount_nm + 500) * 4" got wrong. Both used
    // to wrap inside the bitfield; a wrapped high demand decodes as a large
    // negative torque request at the engine, which is the dangerous direction.
    expect_eq_i32("beyond the top of the range saturates high, it never wraps",
        Torque::to_can_raw(5000.0f, Torque::CAN_RAW_MAX_13BIT), 8191);
    expect_eq_i32("below the bottom of the range saturates low, it never wraps",
        Torque::to_can_raw(-5000.0f, Torque::CAN_RAW_MAX_13BIT), 0);
    expect_eq_i32("NaN encodes as zero rather than an arbitrary value",
        Torque::to_can_raw(NAN, Torque::CAN_RAW_MAX_13BIT), 0);

    // Saturation must respect whatever field width the caller passes - the diag
    // RLI fields are wider than the 13 bit bus fields.
    expect_eq_i32("saturates to the caller's field width",
        Torque::to_can_raw(50000.0f, 1023u), 1023);
}

} // namespace

int main() {
    test_type_is_zero_overhead_and_strong();
    test_roundtrip_and_sentinel();
    test_can_decode_matches_the_literal_it_replaced();
    test_can_encode_is_the_inverse_of_decode();
    test_can_encode_saturates_instead_of_wrapping();

    if (g_failures == 0) {
        std::cout << "PASS: host_torque_scaling_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_torque_scaling_tests failures=" << g_failures << std::endl;
    return 1;
}
