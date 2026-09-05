#include <cstdint>
#include <iostream>

#include "can_egs51_logic.h"
#include "egs_emulation_logic.h"
#include "GS51.h"

namespace {

int g_failures = 0;

void expect_eq_u16(const char* name, uint16_t actual, uint16_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_eq_u8(const char* name, uint8_t actual, uint8_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << (uint16_t)expected << " actual=" << (uint16_t)actual << "\n";
        g_failures++;
    }
}

void expect_eq_i16(const char* name, int16_t actual, int16_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void test_tcc_multi_is_full_byte_field() {
    GS_218_EGS51 frame = {};
    frame.TCC_MULTI = 154;
    expect_eq_u8("tcc_multi stores full byte", (uint8_t)frame.TCC_MULTI, 154);
}

void test_tcc_multi_saturation() {
    expect_eq_u8("tcc_multi low clamp", egs51_tcc_multiplier_to_raw(0.20f), 100);
    expect_eq_u8("tcc_multi nominal", egs51_tcc_multiplier_to_raw(1.54f), 154);
    expect_eq_u8("tcc_multi high clamp", egs51_tcc_multiplier_to_raw(3.00f), 254);
}

void test_torque_request_is_saturated() {
    expect_eq_u8("torque req low clamp", egs51_torque_request_to_raw(-9.0f), 0);
    expect_eq_u8("torque req nominal", egs51_torque_request_to_raw(90.0f), 30);
    expect_eq_u8("torque req high clamp", egs51_torque_request_to_raw(1000.0f), 0xFD);
}

void test_freeze_logic_is_not_overwritten() {
    Egs51FreezeResult freeze = egs51_apply_freeze_logic(true, 300, 250, 20);
    expect_eq_i16("freeze adjusts driver torque", freeze.driver_converted, 280);
    expect_eq_i16("freeze keeps delta", freeze.req_static_torque_delta, 20);

    Egs51FreezeResult tracking = egs51_apply_freeze_logic(false, 300, 250, 20);
    expect_eq_i16("non-freeze keeps driver", tracking.driver_converted, 300);
    expect_eq_i16("non-freeze updates delta", tracking.req_static_torque_delta, 50);
}

void test_wheel_decode_tracks_valid_values() {
    expect_eq_u16("wheel valid passes through", egs51_decode_wheel_speed_or_sna(321), 321);
    expect_eq_u16("wheel sentinel maps to sna", egs51_decode_wheel_speed_or_sna(0x3FFF), UINT16_MAX);
}

void test_engine_limp_inference_heuristic() {
    expect_eq_u8("limp false when all clear", (uint8_t)egs51_infer_engine_limp(false, false, false), 0);
    expect_eq_u8("limp false for temp lamp only", (uint8_t)egs51_infer_engine_limp(true, false, false), 0);
    expect_eq_u8("limp true for temp+diag", (uint8_t)egs51_infer_engine_limp(true, false, true), 1);
    expect_eq_u8("limp true for overheat", (uint8_t)egs51_infer_engine_limp(false, true, false), 1);
    expect_eq_u8("limp false for diag lamp only", (uint8_t)egs51_infer_engine_limp(false, false, true), 0);
}

void test_max_torque_factor_scaling() {
    expect_eq_i16("max factor unavailable keeps base", egs51_apply_max_torque_factor(300, 0xFF), 300);
    expect_eq_i16("max factor half scale", egs51_apply_max_torque_factor(300, 64), 149);
    expect_eq_i16("max factor full scale", egs51_apply_max_torque_factor(300, 128), 299);
}

void test_rli31_mapping_uses_n3_and_validity() {
    Egs51Rli31DerivedData derived = egs51_build_rli31_derived(
        1200,
        900,
        2500,
        400,
        UINT16_MAX,
        600,
        800
    );

    expect_eq_u16("n2 pulse source", derived.n2_pulse_count, egs51_flip_uint16_t(1200));
    expect_eq_u16("n3 pulse source", derived.n3_pulse_count, egs51_flip_uint16_t(900));
    expect_eq_u16("engine speed flipped", derived.engine_speed, egs51_flip_uint16_t(2500));
    expect_eq_u16("front-left valid encoded", derived.front_left_wheel_speed, egs51_flip_uint16_t(200));
    expect_eq_u16("front-right invalid sentinel", derived.front_right_wheel_speed, 0xFFFF);
    expect_eq_u16("rear-left valid encoded", derived.rear_left_wheel_speed, egs51_flip_uint16_t(300));
    expect_eq_u16("rear-right valid encoded", derived.rear_right_wheel_speed, egs51_flip_uint16_t(400));
}

} // namespace

int main() {
    test_tcc_multi_is_full_byte_field();
    test_tcc_multi_saturation();
    test_torque_request_is_saturated();
    test_freeze_logic_is_not_overwritten();
    test_wheel_decode_tracks_valid_values();
    test_engine_limp_inference_heuristic();
    test_max_torque_factor_scaling();
    test_rli31_mapping_uses_n3_and_validity();

    if (g_failures == 0) {
        std::cout << "PASS: host_egs51_bugfix_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_egs51_bugfix_tests failures=" << g_failures << std::endl;
    return 1;
}
