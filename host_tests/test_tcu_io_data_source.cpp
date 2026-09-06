#include <cstdint>
#include <iostream>

#include "tcu_io/tcu_io_data_source.h"
#include "tcu_io/tcu_io_trace_codec.h"

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

void expect_eq_u16(const char* name, uint16_t actual, uint16_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_eq_size(const char* name, size_t actual, size_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void test_playback_holds_last_frame_when_not_looping() {
    TCUIO::TcuIoHardwareFrame frames[2] = {};
    frames[0].rpm_n2 = 111;
    frames[1].rpm_n2 = 222;

    TCUIO::PlaybackTcuIoDataSource source(frames, 2, false);
    TCUIO::TcuIoHardwareFrame out = {};

    expect_true("playback read #1", source.read_frame(&out));
    expect_eq_u16("playback frame #1", out.rpm_n2, 111);

    expect_true("playback read #2", source.read_frame(&out));
    expect_eq_u16("playback frame #2", out.rpm_n2, 222);

    expect_true("playback read #3 still true", source.read_frame(&out));
    expect_eq_u16("playback frame #3 repeats last", out.rpm_n2, 222);
}

void test_playback_loops() {
    TCUIO::TcuIoHardwareFrame frames[2] = {};
    frames[0].battery_mv = 12000;
    frames[1].battery_mv = 13000;

    TCUIO::PlaybackTcuIoDataSource source(frames, 2, true);
    TCUIO::TcuIoHardwareFrame out = {};

    expect_true("loop read #1", source.read_frame(&out));
    expect_eq_u16("loop frame #1", out.battery_mv, 12000);

    expect_true("loop read #2", source.read_frame(&out));
    expect_eq_u16("loop frame #2", out.battery_mv, 13000);

    expect_true("loop read #3", source.read_frame(&out));
    expect_eq_u16("loop frame #3 wraps", out.battery_mv, 12000);
}

void test_ring_capture_sink() {
    TCUIO::TcuIoHardwareFrame buffer[2] = {};
    TCUIO::TcuIoFrameRingBufferCaptureSink sink(buffer, 2);

    TCUIO::TcuIoHardwareFrame f1 = {};
    TCUIO::TcuIoHardwareFrame f2 = {};
    TCUIO::TcuIoHardwareFrame f3 = {};
    f1.rpm_n3 = 10;
    f2.rpm_n3 = 20;
    f3.rpm_n3 = 30;

    sink.on_frame(f1);
    sink.on_frame(f2);
    sink.on_frame(f3);

    expect_eq_size("ring size", sink.size(), 2);
    expect_eq_size("ring writes", sink.samples_written(), 3);

    TCUIO::TcuIoHardwareFrame out = {};
    expect_true("ring get oldest", sink.get_at(0, &out));
    expect_eq_u16("ring oldest is frame2", out.rpm_n3, 20);

    expect_true("ring get latest", sink.get_latest(&out));
    expect_eq_u16("ring latest is frame3", out.rpm_n3, 30);
}

void test_mock_actuator_controller_tracks_last_frame() {
    TCUIO::MockTcuIoActuatorController mock;
    TCUIO::TcuIoActuatorFrame cmd = {};
    cmd.mpc_current_target_ma = 111;
    cmd.spc_current_target_ma = 222;
    cmd.tcc_pwm_12bit = 333;
    cmd.y3_on = true;
    cmd.y4_on = false;
    cmd.y5_on = true;

    mock.apply(cmd);

    TCUIO::TcuIoActuatorFrame out = {};
    expect_true("mock get_last", mock.get_last(&out));
    expect_eq_u16("mock mpc", out.mpc_current_target_ma, 111);
    expect_eq_u16("mock spc", out.spc_current_target_ma, 222);
    expect_eq_u16("mock tcc", out.tcc_pwm_12bit, 333);
    expect_true("mock y3", out.y3_on);
    expect_false("mock y4", out.y4_on);
    expect_true("mock y5", out.y5_on);
}

void test_actuator_ring_capture_sink() {
    TCUIO::TcuIoActuatorFrame buffer[2] = {};
    TCUIO::TcuIoActuatorRingBufferCaptureSink sink(buffer, 2);

    TCUIO::TcuIoActuatorFrame c1 = {};
    TCUIO::TcuIoActuatorFrame c2 = {};
    TCUIO::TcuIoActuatorFrame c3 = {};

    c1.mpc_current_target_ma = 100;
    c2.mpc_current_target_ma = 200;
    c3.mpc_current_target_ma = 300;

    sink.on_frame(c1);
    sink.on_frame(c2);
    sink.on_frame(c3);

    expect_eq_size("actuator ring size", sink.size(), 2);
    expect_eq_size("actuator ring writes", sink.samples_written(), 3);

    TCUIO::TcuIoActuatorFrame out = {};
    expect_true("actuator ring oldest", sink.get_at(0, &out));
    expect_eq_u16("actuator ring oldest is c2", out.mpc_current_target_ma, 200);

    expect_true("actuator ring latest", sink.get_latest(&out));
    expect_eq_u16("actuator ring latest is c3", out.mpc_current_target_ma, 300);
}

void test_hardware_trace_codec_roundtrip() {
    TCUIO::TcuIoHardwareFrame in = {};
    in.rpm_n2 = 1234;
    in.rpm_n3 = 2345;
    in.rpm_out = 456;
    in.battery_mv = 12450;
    in.atf_temp_c = 67;
    in.parking_lock = 0;
    in.wheel_rr_2x_rpm = WheelSpeed::from_raw_2x(1200);
    in.wheel_rl_2x_rpm = WheelSpeed::from_raw_2x(1190);
    in.wheel_fr_2x_rpm = WheelSpeed::from_raw_2x(1210);
    in.wheel_fl_2x_rpm = WheelSpeed::from_raw_2x(1220);
    in.engine_coolant_temp_c = Temp::from_celsius(91);
    in.engine_oil_temp_c = Temp::from_celsius(102);
    in.transfer_case_state = TransferCaseState::Hi;

    char line[256] = {0};
    expect_true("hardware format", TCUIO::format_hardware_frame_csv(in, line, sizeof(line)) > 0);

    TCUIO::TcuIoHardwareFrame out = {};
    expect_true("hardware parse", TCUIO::parse_hardware_frame_csv(line, &out));

    expect_eq_u16("hardware rpm_n2", out.rpm_n2, in.rpm_n2);
    expect_eq_u16("hardware rpm_n3", out.rpm_n3, in.rpm_n3);
    expect_eq_u16("hardware rpm_out", out.rpm_out, in.rpm_out);
    expect_eq_u16("hardware battery", out.battery_mv, in.battery_mv);
    expect_eq_u16("hardware wheel rr", WheelSpeed::raw_2x_u16(out.wheel_rr_2x_rpm), WheelSpeed::raw_2x_u16(in.wheel_rr_2x_rpm));
    expect_eq_u16("hardware wheel rl", WheelSpeed::raw_2x_u16(out.wheel_rl_2x_rpm), WheelSpeed::raw_2x_u16(in.wheel_rl_2x_rpm));
}

void test_actuator_trace_codec_roundtrip() {
    TCUIO::TcuIoActuatorFrame in = {};
    in.mpc_current_target_ma = 700;
    in.spc_current_target_ma = 450;
    in.tcc_pwm_12bit = 1024;
    in.y3_on = true;
    in.y4_on = false;
    in.y5_on = true;

    char line[128] = {0};
    expect_true("actuator format", TCUIO::format_actuator_frame_csv(in, line, sizeof(line)) > 0);

    TCUIO::TcuIoActuatorFrame out = {};
    expect_true("actuator parse", TCUIO::parse_actuator_frame_csv(line, &out));
    expect_eq_u16("actuator mpc", out.mpc_current_target_ma, in.mpc_current_target_ma);
    expect_eq_u16("actuator spc", out.spc_current_target_ma, in.spc_current_target_ma);
    expect_eq_u16("actuator tcc", out.tcc_pwm_12bit, in.tcc_pwm_12bit);
    expect_true("actuator y3", out.y3_on == in.y3_on);
    expect_true("actuator y4", out.y4_on == in.y4_on);
    expect_true("actuator y5", out.y5_on == in.y5_on);
}

} // namespace

int main() {
    test_playback_holds_last_frame_when_not_looping();
    test_playback_loops();
    test_ring_capture_sink();
    test_mock_actuator_controller_tracks_last_frame();
    test_actuator_ring_capture_sink();
    test_hardware_trace_codec_roundtrip();
    test_actuator_trace_codec_roundtrip();

    if (g_failures == 0) {
        std::cout << "All tcu_io data source tests passed\n";
        return 0;
    }

    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
