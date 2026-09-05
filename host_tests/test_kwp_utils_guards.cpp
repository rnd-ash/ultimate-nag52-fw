#include <cstdint>
#include <iostream>

#include "kwp_utils.h"
#include "kwp2000_defines.h"

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

class MockCan final : public EgsBaseCan {
public:
    uint16_t rpm = 0;
    ShifterPosition pos = ShifterPosition::SignalNotAvailable;

    uint16_t get_engine_rpm(uint32_t) override {
        return rpm;
    }

    ShifterPosition get_shifter_position(uint32_t) override {
        return pos;
    }
};

void test_global_make_diag_neg_msg_null_guard() {
    // Guard behavior test: should simply return without crashing.
    global_make_diag_neg_msg(nullptr, 0x22, NRC_GENERAL_REJECT);

    DiagMessage msg = {};
    global_make_diag_neg_msg(&msg, 0x22, NRC_GENERAL_REJECT);
    expect_eq_u16("neg msg id", msg.id, KWP_ECU_TX_ID);
    expect_eq_u16("neg msg len", msg.data_size, 3u);
    expect_eq_u8("neg msg marker", msg.data[0], 0x7Fu);
    expect_eq_u8("neg msg sid", msg.data[1], 0x22u);
    expect_eq_u8("neg msg nrc", msg.data[2], NRC_GENERAL_REJECT);
}

void test_global_make_diag_pos_msg_null_guards() {
    uint8_t payload[2] = {0x11, 0x22};

    // Guard behavior test: should simply return without crashing.
    global_make_diag_pos_msg(nullptr, 0x22, payload, 2u);
    global_make_diag_pos_msg(nullptr, 0x22, 0x99, payload, 2u);

    DiagMessage msg = {};
    global_make_diag_pos_msg(&msg, 0x10, nullptr, 1u);
    expect_eq_u8("pos null payload becomes negative marker", msg.data[0], 0x7Fu);
    expect_eq_u8("pos null payload sid", msg.data[1], 0x10u);
    expect_eq_u8("pos null payload nrc", msg.data[2], NRC_GENERAL_REJECT);

    global_make_diag_pos_msg(&msg, 0x11, nullptr, 0u);
    expect_eq_u16("pos zero-len id", msg.id, KWP_ECU_TX_ID);
    expect_eq_u16("pos zero-len size", msg.data_size, 1u);
    expect_eq_u8("pos zero-len sid", msg.data[0], 0x51u);

    global_make_diag_pos_msg(&msg, 0x21, 0xAA, nullptr, 1u);
    expect_eq_u8("pos pid null payload marker", msg.data[0], 0x7Fu);
    expect_eq_u8("pos pid null payload sid", msg.data[1], 0x21u);
    expect_eq_u8("pos pid null payload nrc", msg.data[2], NRC_GENERAL_REJECT);

    global_make_diag_pos_msg(&msg, 0x21, 0xAA, payload, 2u);
    expect_eq_u16("pos pid id", msg.id, KWP_ECU_TX_ID);
    expect_eq_u16("pos pid size", msg.data_size, 4u);
    expect_eq_u8("pos pid sid", msg.data[0], 0x61u);
    expect_eq_u8("pos pid pid", msg.data[1], 0xAAu);
    expect_eq_u8("pos pid payload0", msg.data[2], 0x11u);
    expect_eq_u8("pos pid payload1", msg.data[3], 0x22u);
}

void test_engine_and_shifter_helpers_null_and_values() {
    expect_true("engine off with null can", is_engine_off(nullptr));
    expect_true("shifter passive with null can", is_shifter_passive(nullptr));

    MockCan can = {};
    can.rpm = 0u;
    expect_true("engine off at 0 rpm", is_engine_off(&can));
    can.rpm = UINT16_MAX;
    expect_true("engine off at SNA rpm", is_engine_off(&can));
    can.rpm = 1000u;
    expect_false("engine running", is_engine_off(&can));

    can.pos = ShifterPosition::P;
    expect_true("shifter P passive", is_shifter_passive(&can));
    can.pos = ShifterPosition::N;
    expect_true("shifter N passive", is_shifter_passive(&can));
    can.pos = ShifterPosition::SignalNotAvailable;
    expect_true("shifter SNA passive", is_shifter_passive(&can));
    can.pos = ShifterPosition::D;
    expect_false("shifter D active", is_shifter_passive(&can));
}

} // namespace

int main() {
    test_global_make_diag_neg_msg_null_guard();
    test_global_make_diag_pos_msg_null_guards();
    test_engine_and_shifter_helpers_null_and_values();

    if (g_failures == 0) {
        std::cout << "PASS: host_kwp_utils_guard_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_kwp_utils_guard_tests failures=" << g_failures << std::endl;
    return 1;
}
