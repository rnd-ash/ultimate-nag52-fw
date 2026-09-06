#include <cstdint>
#include <iostream>
#include <limits.h>
#include <limits>

#include "ioexpander_logic.h"
#include "kickdownswitch_logic.h"
#include "can_custom_logic.h"
#include "sensors_logic.h"
#include "can_egs_scaling_logic.h"
#include "diag_data_logic.h"
#include "kwp2000_logic.h"
#include "kwp2000_defines.h"
#include "flasher_logic.h"
#include "isotp_logic.h"
#include "input_torque_logic.h"
#include "pressure_manager_logic.h"
#include "tcu_io_logic.h"

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

void expect_eq_u32(const char* name, uint32_t actual, uint32_t expected) {
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

void test_egs52_turbine_loss_encoding_is_saturated() {
    expect_eq_u8("egs52 turbine loss zero", egs52_encode_turbine_torque_loss(0), 0);
    expect_eq_u8("egs52 turbine loss nominal", egs52_encode_turbine_torque_loss(50), 200);
    expect_eq_u8("egs52 turbine loss saturates", egs52_encode_turbine_torque_loss(1000), 0xFE);
}

void test_egs52_wheel_ratio_invalid_and_clamp() {
    expect_eq_u16("egs52 ratio invalid negative", egs52_encode_wheel_torque_multi_factor(-1.0f), 0x7FF);
    expect_eq_u16("egs52 ratio nominal", egs52_encode_wheel_torque_multi_factor(20.0f), 1);
    expect_eq_u16("egs52 ratio high clamp", egs52_encode_wheel_torque_multi_factor(100000.0f), 2046);
}

void test_egs53_wheel_ratio_invalid_and_clamp() {
    expect_eq_u16("egs53 ratio invalid negative", egs53_encode_wheel_torque_multi_factor(-0.5f), 0);
    expect_eq_u16("egs53 ratio nominal", egs53_encode_wheel_torque_multi_factor(12.34f), 1234);
    expect_eq_u16("egs53 ratio high clamp", egs53_encode_wheel_torque_multi_factor(500.0f), 16383);
}

void test_kwp_ecu_reset_subfunction_parser() {
    uint8_t sub = ECU_RESET_POWER_ON;
    expect_true("kwp reset accepts power-on", kwp_is_valid_ecu_reset_subfn(&sub, 1));
    sub = ECU_RESET_NON_VOLATILE;
    expect_true("kwp reset accepts non-volatile", kwp_is_valid_ecu_reset_subfn(&sub, 1));
    sub = 0x7F;
    expect_false("kwp reset rejects unknown", kwp_is_valid_ecu_reset_subfn(&sub, 1));
    expect_false("kwp reset rejects missing args", kwp_is_valid_ecu_reset_subfn(nullptr, 0));
}

void test_kwp_tester_present_parser() {
    uint8_t sub = KWP_CMD_RESPONSE_REQUIRED;
    expect_eq_u32("kwp tester-present response-required", (uint32_t)kwp_parse_tester_present_subfn(&sub, 1), (uint32_t)KwpTesterPresentSubfn::ResponseRequired);
    sub = KWP_CMD_NO_RESPONSE_REQUIRED;
    expect_eq_u32("kwp tester-present no-response", (uint32_t)kwp_parse_tester_present_subfn(&sub, 1), (uint32_t)KwpTesterPresentSubfn::NoResponseRequired);
    sub = 0x7E;
    expect_eq_u32("kwp tester-present invalid subfn", (uint32_t)kwp_parse_tester_present_subfn(&sub, 1), (uint32_t)KwpTesterPresentSubfn::Invalid);
    expect_eq_u32("kwp tester-present missing args", (uint32_t)kwp_parse_tester_present_subfn(nullptr, 0), (uint32_t)KwpTesterPresentSubfn::Invalid);
}

void test_kwp_read_data_local_ident_header_validation() {
    uint8_t one[] = {0x90};
    expect_true("kwp rli normal one-byte header", kwp_has_valid_read_data_local_ident_header(one, 1));
    uint8_t map_editor_short[] = {KWP_RLI_MAP_EDITOR, 0x01, 0x02};
    expect_true("kwp rli map editor variable length", kwp_has_valid_read_data_local_ident_header(map_editor_short, 3));
    uint8_t settings_short[] = {KWP_RLI_SETTINGS_EDIT, 0x01};
    expect_true("kwp rli settings variable length", kwp_has_valid_read_data_local_ident_header(settings_short, 2));
    uint8_t invalid[] = {0x90, 0x00};
    expect_false("kwp rli invalid fixed-length payload", kwp_has_valid_read_data_local_ident_header(invalid, 2));
    expect_false("kwp rli missing args", kwp_has_valid_read_data_local_ident_header(nullptr, 0));
}

void test_kwp_big_endian_unpack_helpers() {
    uint8_t raw24[] = {0x12, 0x34, 0x56};
    expect_eq_u32("kwp read u24 be", kwp_read_u24_be(raw24), 0x00123456u);

    uint8_t raw32[] = {0x89, 0xAB, 0xCD, 0xEF};
    expect_eq_u32("kwp read u32 be", kwp_read_u32_be(raw32), 0x89ABCDEFu);

    expect_eq_u32("kwp read u24 null", kwp_read_u24_be(nullptr), 0u);
    expect_eq_u32("kwp read u32 null", kwp_read_u32_be(nullptr), 0u);
}

void test_diag_module_settings_arg_guards() {
    uint16_t out_len = 0u;
    uint8_t* out_buf = nullptr;
    uint8_t write_buf[2] = {0x00, 0xAA};

    expect_eq_u8(
        "diag settings read rejects null len ptr",
        (uint8_t)diag_validate_module_settings_read_args(nullptr, &out_buf),
        NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT
    );
    expect_eq_u8(
        "diag settings read rejects null buffer ptr",
        (uint8_t)diag_validate_module_settings_read_args(&out_len, nullptr),
        NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT
    );
    expect_eq_u8(
        "diag settings read accepts valid args",
        (uint8_t)diag_validate_module_settings_read_args(&out_len, &out_buf),
        NRC_OK
    );

    expect_eq_u8(
        "diag settings write rejects null buffer",
        (uint8_t)diag_get_module_settings_write_action(1u, nullptr),
        (uint8_t)DiagModuleSettingsWriteAction::Invalid
    );
    expect_eq_u8(
        "diag settings write detects reset token",
        (uint8_t)diag_get_module_settings_write_action(1u, write_buf),
        (uint8_t)DiagModuleSettingsWriteAction::Reset
    );
    expect_eq_u8(
        "diag settings write keeps non-reset writes",
        (uint8_t)diag_get_module_settings_write_action(2u, write_buf),
        (uint8_t)DiagModuleSettingsWriteAction::Write
    );
}

void test_diag_runtime_pointer_guard_decisions() {
    uint8_t a = 0x11;
    uint8_t b = 0x22;

    expect_false("diag tcc guard rejects null gearbox", diag_has_tcc_program_sources(nullptr, &a));
    expect_false("diag tcc guard rejects null tcc", diag_has_tcc_program_sources(&a, nullptr));
    expect_true("diag tcc guard accepts valid pointers", diag_has_tcc_program_sources(&a, &b));

    expect_false("diag rx guard rejects null can", diag_has_rx_can_sources(nullptr, &a, &a, &a));
    expect_false("diag rx guard rejects null shifter", diag_has_rx_can_sources(&a, nullptr, &a, &a));
    expect_false("diag rx guard rejects null gearbox", diag_has_rx_can_sources(&a, &a, nullptr, &a));
    expect_false("diag rx guard rejects null global can", diag_has_rx_can_sources(&a, &a, &a, nullptr));
    expect_true("diag rx guard accepts valid pointers", diag_has_rx_can_sources(&a, &a, &a, &a));

    expect_false("diag shift-live guard rejects null can", diag_has_shift_live_sources(nullptr, &a, &a, &a, &a, &a));
    expect_false("diag shift-live guard rejects null gearbox", diag_has_shift_live_sources(&a, nullptr, &a, &a, &a, &a));
    expect_false("diag shift-live guard rejects null pressure manager", diag_has_shift_live_sources(&a, &a, nullptr, &a, &a, &a));
    expect_false("diag shift-live guard rejects null sol y3", diag_has_shift_live_sources(&a, &a, &a, nullptr, &a, &a));
    expect_false("diag shift-live guard rejects null sol y4", diag_has_shift_live_sources(&a, &a, &a, &a, nullptr, &a));
    expect_false("diag shift-live guard rejects null sol y5", diag_has_shift_live_sources(&a, &a, &a, &a, &a, nullptr));
    expect_true("diag shift-live guard accepts valid pointers", diag_has_shift_live_sources(&a, &a, &a, &a, &a, &a));
}

void test_isotp_numeric_guards() {
    expect_eq_u8("isotp sf low nibble len", isotp_single_frame_payload_len(0x04), 4u);
    expect_eq_u8("isotp sf ignores pci high nibble", isotp_single_frame_payload_len(0xF7), 7u);

    expect_true("isotp ff valid min", isotp_first_frame_size_valid(7u, 4095u));
    expect_false("isotp ff invalid short", isotp_first_frame_size_valid(6u, 4095u));
    expect_false("isotp ff invalid too large", isotp_first_frame_size_valid(5000u, 4095u));

    expect_eq_u16("isotp payload len clamped", isotp_clamp_payload_len(5000u, 4095u), 4095u);
    expect_eq_u16("isotp payload len unchanged", isotp_clamp_payload_len(1024u, 4095u), 1024u);

    expect_eq_u8("isotp tx chunk max 7", isotp_tx_chunk_size(0u, 100u), 7u);
    expect_eq_u8("isotp tx chunk tail", isotp_tx_chunk_size(8u, 12u), 4u);
    expect_eq_u8("isotp tx chunk exhausted", isotp_tx_chunk_size(12u, 12u), 0u);

    expect_eq_u8("isotp rx chunk max 7", isotp_rx_chunk_size(0u, 20u), 7u);
    expect_eq_u8("isotp rx chunk tail", isotp_rx_chunk_size(10u, 15u), 5u);
    expect_eq_u8("isotp rx chunk exhausted", isotp_rx_chunk_size(16u, 15u), 0u);
}

void test_flasher_numeric_guards() {
    expect_true("flasher range fits exact tail", flasher_range_fits_u32(0x1000u, 0x200u, 0x1200u));
    expect_false("flasher range rejects overflowed tail", flasher_range_fits_u32(0x1000u, 0x201u, 0x1200u));
    expect_false("flasher range rejects base beyond limit", flasher_range_fits_u32(0x2000u, 0x1u, 0x1000u));

    uint32_t aligned = 0u;
    expect_true("flasher align nominal", flasher_try_align_up_u32(4100u, 4096u, &aligned));
    expect_eq_u32("flasher align nominal value", aligned, 8192u);
    expect_true("flasher align already aligned", flasher_try_align_up_u32(8192u, 4096u, &aligned));
    expect_eq_u32("flasher align keeps value", aligned, 8192u);
    expect_false("flasher align rejects zero alignment", flasher_try_align_up_u32(1u, 0u, &aligned));
    expect_false("flasher align rejects null out", flasher_try_align_up_u32(1u, 4096u, nullptr));
    expect_false("flasher align rejects overflow", flasher_try_align_up_u32(0xFFFFFFFEu, 4096u, &aligned));
}

void test_input_torque_numeric_guards() {
    expect_eq_i32("input torque engine pow2 no overflow", input_torque_engine_pow2_div1000(65535u), 4294836);
    expect_eq_u16("input torque ratio clamp", input_torque_ratio_x1000(65535u, 1u), UINT16_MAX);
    expect_eq_u16("input torque ratio divide zero guard", input_torque_ratio_x1000(100u, 0u), UINT16_MAX);
    expect_eq_i16("input torque positive clamp", input_torque_scaled_clamped(30000, 2.0f), INT16_MAX);
    expect_eq_i16("input torque negative clamp", input_torque_scaled_clamped(-30000, 2.0f), INT16_MIN);
}

void test_pressure_manager_numeric_guards() {
    expect_eq_u16("pressure max shift underflow guard", pressure_manager_calc_max_shift_pressure(1000u, 1200u, 2000u), 0u);
    expect_eq_u16("pressure max shift nominal", pressure_manager_calc_max_shift_pressure(3000u, 1000u, 2000u), 4000u);
    expect_eq_u16("pressure max shift saturates", pressure_manager_calc_max_shift_pressure(UINT16_MAX, 0u, UINT16_MAX), UINT16_MAX);
}

void test_tcuio_turbine_rpm_numeric_guards() {
    expect_eq_u16("tcuio turbine nominal", tcuio_calc_turbine_rpm_safe(1000u, 500u, 1.61f), 1305u);
    expect_eq_u16("tcuio turbine floor zero", tcuio_calc_turbine_rpm_safe(0u, 1000u, 1.61f), 0u);
    expect_eq_u16("tcuio turbine saturates", tcuio_calc_turbine_rpm_safe(UINT16_MAX, UINT16_MAX, 2000.0f), UINT16_MAX);
    expect_eq_u16("tcuio turbine nan guard", tcuio_calc_turbine_rpm_safe(1000u, 1000u, std::numeric_limits<float>::quiet_NaN()), 0u);
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
    test_egs52_turbine_loss_encoding_is_saturated();
    test_egs52_wheel_ratio_invalid_and_clamp();
    test_egs53_wheel_ratio_invalid_and_clamp();
    test_kwp_ecu_reset_subfunction_parser();
    test_kwp_tester_present_parser();
    test_kwp_read_data_local_ident_header_validation();
    test_kwp_big_endian_unpack_helpers();
    test_diag_module_settings_arg_guards();
    test_diag_runtime_pointer_guard_decisions();
    test_isotp_numeric_guards();
    test_flasher_numeric_guards();
    test_input_torque_numeric_guards();
    test_pressure_manager_numeric_guards();
    test_tcuio_turbine_rpm_numeric_guards();

    if (g_failures == 0) {
        std::cout << "PASS: host_lowlevel_logic_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_lowlevel_logic_tests failures=" << g_failures << std::endl;
    return 1;
}
