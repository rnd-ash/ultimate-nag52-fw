#include <cstdint>
#include <iostream>
#include <limits.h>

#include "ioexpander_logic.h"
#include "kickdownswitch_logic.h"
#include "can_custom_logic.h"
#include "sensors_logic.h"

namespace {

int g_failures = 0;

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

void expect_eq_i16(const char* name, int16_t actual, int16_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_eq_i32(const char* name, int actual, int expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void test_ioexpander_bit_validation() {
    expect_true("valid bit 0", ioexpander_is_valid_bit(0));
    expect_true("valid bit 7", ioexpander_is_valid_bit(7));
    expect_false("invalid bit -1", ioexpander_is_valid_bit(-1));
    expect_false("invalid bit 8", ioexpander_is_valid_bit(8));
}

void test_ioexpander_bit_read_write() {
    uint8_t input = 0x52; // 0b01010010
    expect_true("bit1 true", ioexpander_get_input_bit(input, 1));
    expect_true("bit4 true", ioexpander_get_input_bit(input, 4));
    expect_false("bit0 false", ioexpander_get_input_bit(input, 0));
    expect_false("invalid read returns false", ioexpander_get_input_bit(input, -1));

    uint8_t out = 0;
    out = ioexpander_set_output_bit(out, 2, true);
    expect_eq_u8("set bit2", out, 0x04);
    out = ioexpander_set_output_bit(out, 2, false);
    expect_eq_u8("clear bit2", out, 0x00);

    uint8_t unchanged = 0xAA;
    expect_eq_u8("invalid set keeps state -1", ioexpander_set_output_bit(unchanged, -1, true), unchanged);
    expect_eq_u8("invalid set keeps state 8", ioexpander_set_output_bit(unchanged, 8, false), unchanged);
}

void test_kickdown_rising_edge_only() {
    expect_false("false->false", kickdown_is_new_press(false, false));
    expect_true("false->true", kickdown_is_new_press(true, false));
    expect_false("true->true", kickdown_is_new_press(true, true));
    expect_false("true->false", kickdown_is_new_press(false, true));
}

void test_custom_can_coolant_and_kickdown_decode() {
    ENGINE_100_CUSTOMCAN frame = {};
    frame.T_COOLANT = 90;
    frame.PEDAL = 200;
    expect_eq_i16("coolant uses T_COOLANT", customcan_decode_engine_coolant(frame), 50);

    frame.T_COOLANT = UINT8_MAX;
    expect_eq_i16("coolant sna", customcan_decode_engine_coolant(frame), INT16_MAX);

    frame.KD = true;
    expect_true("kickdown true", customcan_decode_kickdown(frame));
    frame.KD = false;
    expect_false("kickdown false", customcan_decode_kickdown(frame));
}

void test_custom_can_torque_request_encoding() {
    expect_eq_u16("torque req low clamp", customcan_encode_torque_request_nm(-1000.0f), 0);
    expect_eq_u16("torque req nominal", customcan_encode_torque_request_nm(100.0f), 2400);
    expect_eq_u16("torque req high clamp", customcan_encode_torque_request_nm(20000.0f), UINT16_MAX);
}

void test_custom_can_torque_request_control_bits() {
    {
        CustomCanTorqueRequestFields f = customcan_build_torque_request(TorqueRequestControlType::None, TorqueRequestBounds::LessThan, 120.0f);
        expect_false("none ctrl0", f.ctrl0);
        expect_false("none ctrl1", f.ctrl1);
        expect_false("none min", f.min);
        expect_false("none max", f.max);
        expect_eq_u16("none raw", f.raw_torque, 0);
    }
    {
        CustomCanTorqueRequestFields f = customcan_build_torque_request(TorqueRequestControlType::NormalSpeed, TorqueRequestBounds::LessThan, 100.0f);
        expect_true("normal ctrl0", f.ctrl0);
        expect_false("normal ctrl1", f.ctrl1);
        expect_true("normal min", f.min);
        expect_false("normal max", f.max);
        expect_eq_u16("normal raw", f.raw_torque, 2400);
    }
    {
        CustomCanTorqueRequestFields f = customcan_build_torque_request(TorqueRequestControlType::FastAsPossible, TorqueRequestBounds::MoreThan, 100.0f);
        expect_false("fast ctrl0", f.ctrl0);
        expect_true("fast ctrl1", f.ctrl1);
        expect_false("fast min", f.min);
        expect_true("fast max", f.max);
        expect_eq_u16("fast raw", f.raw_torque, 2400);
    }
    {
        CustomCanTorqueRequestFields f = customcan_build_torque_request(TorqueRequestControlType::BackToDemandTorque, TorqueRequestBounds::Exact, 20000.0f);
        expect_true("back ctrl0", f.ctrl0);
        expect_true("back ctrl1", f.ctrl1);
        expect_true("back min", f.min);
        expect_true("back max", f.max);
        expect_eq_u16("back raw saturated", f.raw_torque, UINT16_MAX);
    }
}

void test_atf_resistance_guard() {
    int resistance = -1;
    expect_true("valid divider", sensors_try_calc_atf_resistance(1650, 2000, &resistance));
    expect_eq_i32("valid divider result", resistance, 2000);

    resistance = -1;
    expect_true("zero input is valid", sensors_try_calc_atf_resistance(0, 2000, &resistance));
    expect_eq_i32("zero input resistance", resistance, 0);

    expect_false("3300mv is invalid", sensors_try_calc_atf_resistance(3300, 2000, &resistance));
    expect_false("above rail invalid", sensors_try_calc_atf_resistance(3400, 2000, &resistance));
    expect_false("null output ptr", sensors_try_calc_atf_resistance(1000, 2000, nullptr));
}

} // namespace

int main() {
    test_ioexpander_bit_validation();
    test_ioexpander_bit_read_write();
    test_kickdown_rising_edge_only();
    test_custom_can_coolant_and_kickdown_decode();
    test_custom_can_torque_request_encoding();
    test_custom_can_torque_request_control_bits();
    test_atf_resistance_guard();

    if (g_failures == 0) {
        std::cout << "PASS: host_lowlevel_logic_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_lowlevel_logic_tests failures=" << g_failures << std::endl;
    return 1;
}
