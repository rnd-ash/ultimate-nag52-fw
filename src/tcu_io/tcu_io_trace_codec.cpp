#include "tcu_io_trace_codec.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

namespace {

bool parse_i32_list(const char* line, int32_t* out, size_t count) {
    if (line == nullptr || out == nullptr || count == 0) {
        return false;
    }

    const char* p = line;
    for (size_t i = 0; i < count; i++) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == '\0' || *p == '\r' || *p == '\n') {
            return false;
        }

        char* end = nullptr;
        long value = strtol(p, &end, 10);
        if (end == p) {
            return false;
        }

        out[i] = (int32_t)value;
        p = end;

        while (*p == ' ' || *p == '\t') {
            p++;
        }

        if (i + 1 < count) {
            if (*p != ',') {
                return false;
            }
            p++;
        }
    }

    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p == '\r' || *p == '\n' || *p == '\0' || *p == '#') {
        return true;
    }
    return false;
}

bool is_data_line(const char* line) {
    if (line == nullptr) {
        return false;
    }
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (*line == '\0' || *line == '\r' || *line == '\n' || *line == '#') {
        return false;
    }
    return (*line == '-' || isdigit((unsigned char)*line) != 0);
}

} // namespace

namespace TCUIO {

size_t format_hardware_frame_csv_header(char* out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return 0;
    }
    int written = snprintf(
        out,
        out_size,
        "rpm_n2,rpm_n3,rpm_out,battery_mv,atf_temp_c,parking_lock,wheel_rr_2x,wheel_rl_2x,wheel_fr_2x,wheel_fl_2x,engine_coolant_temp_c,engine_oil_temp_c,transfer_case_state"
    );
    return written > 0 ? (size_t)written : 0;
}

size_t format_hardware_frame_csv(const TcuIoHardwareFrame& frame, char* out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return 0;
    }
    int written = snprintf(
        out,
        out_size,
        "%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%d,%d,%d",
        (unsigned int)frame.rpm_n2,
        (unsigned int)frame.rpm_n3,
        (unsigned int)frame.rpm_out,
        (unsigned int)frame.battery_mv,
        frame.atf_temp_c,
        (unsigned int)frame.parking_lock,
        (unsigned int)WheelSpeed::raw_2x_u16(frame.wheel_rr_2x_rpm),
        (unsigned int)WheelSpeed::raw_2x_u16(frame.wheel_rl_2x_rpm),
        (unsigned int)WheelSpeed::raw_2x_u16(frame.wheel_fr_2x_rpm),
        (unsigned int)WheelSpeed::raw_2x_u16(frame.wheel_fl_2x_rpm),
        (int)Temp::celsius_i16(frame.engine_coolant_temp_c),
        (int)Temp::celsius_i16(frame.engine_oil_temp_c),
        (int)frame.transfer_case_state
    );
    return written > 0 ? (size_t)written : 0;
}

bool parse_hardware_frame_csv(const char* line, TcuIoHardwareFrame* out) {
    if (out == nullptr || !is_data_line(line)) {
        return false;
    }

    int32_t values[13] = {0};
    if (!parse_i32_list(line, values, 13)) {
        return false;
    }

    out->rpm_n2 = (uint16_t)values[0];
    out->rpm_n3 = (uint16_t)values[1];
    out->rpm_out = (uint16_t)values[2];
    out->battery_mv = (uint16_t)values[3];
    out->atf_temp_c = (int)values[4];
    out->parking_lock = (uint8_t)values[5];
    out->wheel_rr_2x_rpm = WheelSpeed::from_raw_2x((uint16_t)values[6]);
    out->wheel_rl_2x_rpm = WheelSpeed::from_raw_2x((uint16_t)values[7]);
    out->wheel_fr_2x_rpm = WheelSpeed::from_raw_2x((uint16_t)values[8]);
    out->wheel_fl_2x_rpm = WheelSpeed::from_raw_2x((uint16_t)values[9]);
    out->engine_coolant_temp_c = Temp::from_celsius((int16_t)values[10]);
    out->engine_oil_temp_c = Temp::from_celsius((int16_t)values[11]);
    out->transfer_case_state = (TransferCaseState)values[12];
    return true;
}

size_t format_actuator_frame_csv_header(char* out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return 0;
    }
    int written = snprintf(out, out_size, "mpc_ma,spc_ma,tcc_pwm_12bit,y3_on,y4_on,y5_on");
    return written > 0 ? (size_t)written : 0;
}

size_t format_actuator_frame_csv(const TcuIoActuatorFrame& frame, char* out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return 0;
    }
    int written = snprintf(
        out,
        out_size,
        "%u,%u,%u,%u,%u,%u",
        (unsigned int)frame.mpc_current_target_ma,
        (unsigned int)frame.spc_current_target_ma,
        (unsigned int)frame.tcc_pwm_12bit,
        frame.y3_on ? 1u : 0u,
        frame.y4_on ? 1u : 0u,
        frame.y5_on ? 1u : 0u
    );
    return written > 0 ? (size_t)written : 0;
}

bool parse_actuator_frame_csv(const char* line, TcuIoActuatorFrame* out) {
    if (out == nullptr || !is_data_line(line)) {
        return false;
    }

    int32_t values[6] = {0};
    if (!parse_i32_list(line, values, 6)) {
        return false;
    }

    out->mpc_current_target_ma = (uint16_t)values[0];
    out->spc_current_target_ma = (uint16_t)values[1];
    out->tcc_pwm_12bit = (uint16_t)values[2];
    out->y3_on = values[3] != 0;
    out->y4_on = values[4] != 0;
    out->y5_on = values[5] != 0;
    return true;
}

} // namespace TCUIO