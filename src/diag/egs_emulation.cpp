#include "egs_emulation.h"
#include "sensors.h"

RLI_31_DATA get_rli_31(AbstractCan* can_layer) {
    RLI_31_DATA ret = {};
    uint64_t now = esp_timer_get_time() / 1000;
    RpmReading d;
    if (!Sensors::read_input_rpm(&d, false)) {
        ret.n2_pulse_count = 0xFFFF;
        ret.n3_pulse_count = 0xFFFF;
        ret.input_rpm = 0xFFFF;
    } else {
        ret.n2_pulse_count = d.n2_raw;
        ret.n3_pulse_count = d.n3_raw;
        ret.input_rpm = d.calc_rpm;
    }
    ret.vehicle_speed_rear_wheels = 0; // TODO
    ret.vehicle_speed_front_wheels = 0; // TODO

    WheelData wd = {};
    wd = can_layer->get_front_left_wheel(now, 300);
    if (wd.current_dir == WheelDirection::SignalNotAvaliable) {
        ret.front_left_wheel_speed = 0xFFFF;
    } else {
        ret.front_left_wheel_speed = wd.double_rpm / 2.0;
    }

    wd = can_layer->get_front_right_wheel(now, 300);
    if (wd.current_dir == WheelDirection::SignalNotAvaliable) {
        ret.front_right_wheel_speed = 0xFFFF;
    } else {
        ret.front_right_wheel_speed = wd.double_rpm / 2.0;
    }

    wd = can_layer->get_rear_left_wheel(now, 300);
    if (wd.current_dir == WheelDirection::SignalNotAvaliable) {
        ret.rear_left_wheel_speed = 0xFFFF;
    } else {
        ret.rear_left_wheel_speed = wd.double_rpm / 2.0;
    }

    wd = can_layer->get_rear_right_wheel(now, 300);
    if (wd.current_dir == WheelDirection::SignalNotAvaliable) {
        ret.rear_right_wheel_speed = 0xFFFF;
    } else {
        ret.rear_right_wheel_speed = wd.double_rpm / 2.0;
    }

    ret.engine_speed = can_layer->get_engine_rpm(now, 300);

    return ret;
}