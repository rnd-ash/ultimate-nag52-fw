#include <cstdint>
#include <iostream>

#include "egs_emulation_rli33_logic.h"

namespace {

int g_failures = 0;

void expect_eq_u8(const char* name, uint8_t actual, uint8_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << (uint16_t)expected << " actual=" << (uint16_t)actual << "\n";
        g_failures++;
    }
}

void expect_eq_u16(const char* name, uint16_t actual, uint16_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void test_rli33_pressure_and_current_mapping() {
    Egs51Rli33DerivedData d = egs51_build_rli33_derived(
        0x1234,
        0x5678,
        111,
        222,
        333,
        444,
        2048,
        0,
        0,
        0
    );

    expect_eq_u16("spc pressure flipped", d.spc_pressure, 0x3412);
    expect_eq_u16("mpc pressure flipped", d.mpc_pressure, 0x7856);
    expect_eq_u16("spc target direct", d.spc_target_current, 111);
    expect_eq_u16("spc actual flipped", d.spc_actual_current, 0xDE00);
    expect_eq_u16("mpc target direct", d.mpc_target_current, 333);
    expect_eq_u16("mpc actual flipped", d.mpc_actual_current, 0xBC01);
    expect_eq_u8("tcc pwm convert", d.tcc_pwm_255, 128);
    expect_eq_u8("no valve state", d.shift_valve_state, 0);
    expect_eq_u8("no valve flag", d.valve_flag, 0);
}

void test_rli33_tcc_pwm_is_clamped() {
    Egs51Rli33DerivedData d = egs51_build_rli33_derived(
        0,
        0,
        0,
        0,
        0,
        0,
        4096,
        0,
        0,
        0
    );
    expect_eq_u8("tcc 4096 clamps to 255", d.tcc_pwm_255, 255);
}

void test_rli33_shift_valve_truth_table() {
    expect_eq_u8("1245", egs51_build_rli33_derived(0,0,0,0,0,0,0,11,0,0).shift_valve_state, 1);
    expect_eq_u8("23", egs51_build_rli33_derived(0,0,0,0,0,0,0,0,11,0).shift_valve_state, 2);
    expect_eq_u8("34", egs51_build_rli33_derived(0,0,0,0,0,0,0,0,0,11).shift_valve_state, 4);
    expect_eq_u8("1245+23", egs51_build_rli33_derived(0,0,0,0,0,0,0,11,11,0).shift_valve_state, 3);
    expect_eq_u8("1245+34", egs51_build_rli33_derived(0,0,0,0,0,0,0,11,0,11).shift_valve_state, 5);
    expect_eq_u8("23+34", egs51_build_rli33_derived(0,0,0,0,0,0,0,0,11,11).shift_valve_state, 6);
    expect_eq_u8("all", egs51_build_rli33_derived(0,0,0,0,0,0,0,11,11,11).shift_valve_state, 7);

    expect_eq_u8("flag set", egs51_build_rli33_derived(0,0,0,0,0,0,0,11,0,0).valve_flag, 1);
    expect_eq_u8("flag clear", egs51_build_rli33_derived(0,0,0,0,0,0,0,0,0,0).valve_flag, 0);
}

} // namespace

int main() {
    test_rli33_pressure_and_current_mapping();
    test_rli33_tcc_pwm_is_clamped();
    test_rli33_shift_valve_truth_table();

    if (g_failures == 0) {
        std::cout << "PASS: host_diag_rli33_logic_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_diag_rli33_logic_tests failures=" << g_failures << std::endl;
    return 1;
}
