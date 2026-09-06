//
// Created by ashcon on 9/2/24.
//

#include "tcu_io.hpp"
#include "tcu_io_logic.h"
#include "sensors.h"
#include "solenoids/solenoids.h"
#include <limits>

float RATIO_2_1 = 1.61f;
float DIFF_RATIO_F = 1.00f;

TCUIO::SmoothedSensor smoothed_sensor_n2_rpm;
TCUIO::SmoothedSensor smoothed_sensor_n3_rpm;
TCUIO::SmoothedSensor smoothed_sensor_out_rpm;
TCUIO::SmoothedSensor smoothed_sensor_atf_temp;
TCUIO::SmoothedSensor smoothed_sensor_vbatt;


TCUIO::OnePollSensor<uint16_t> onepoll_rr_speed;
TCUIO::OnePollSensor<uint16_t> onepoll_rl_speed;
TCUIO::OnePollSensor<uint16_t> onepoll_fr_speed;
TCUIO::OnePollSensor<uint16_t> onepoll_fl_speed;

TCUIO::OnePollSensor<uint8_t> onepoll_parking_lock;
TCUIO::OnePollSensor<int16_t> onepoll_motor_temperature;
TCUIO::OnePollSensor<int16_t> onepoll_motor_oil_temperature;

namespace {

class LiveTcuIoDataSource final : public TCUIO::ITcuIoDataSource {
public:
    const char* name() const override {
        return "live";
    }

    bool read_frame(TCUIO::TcuIoHardwareFrame* out) override {
        if (out == nullptr) {
            return false;
        }

        SensorDataRaw raw = {};
        Sensors::update(&raw);

        out->rpm_n2 = raw.rpm_n2;
        out->rpm_n3 = raw.rpm_n3;
        out->rpm_out = raw.rpm_out;
        out->battery_mv = raw.battery_mv;
        out->atf_temp_c = raw.atf_temp_c;
        out->parking_lock = raw.parking_lock;

        if (egs_can_hal != nullptr) {
            out->wheel_rl_2x_rpm = egs_can_hal->get_rear_left_wheel(100);
            out->wheel_rr_2x_rpm = egs_can_hal->get_rear_right_wheel(100);
            out->wheel_fl_2x_rpm = egs_can_hal->get_front_left_wheel(100);
            out->wheel_fr_2x_rpm = egs_can_hal->get_front_right_wheel(100);
            out->engine_coolant_temp_c = egs_can_hal->get_engine_coolant_temp(100);
            out->engine_oil_temp_c = egs_can_hal->get_engine_oil_temp(100);
            out->transfer_case_state = egs_can_hal->get_transfer_case_state(500);
        } else {
            out->wheel_rl_2x_rpm = WheelSpeed::INVALID;
            out->wheel_rr_2x_rpm = WheelSpeed::INVALID;
            out->wheel_fl_2x_rpm = WheelSpeed::INVALID;
            out->wheel_fr_2x_rpm = WheelSpeed::INVALID;
            out->engine_coolant_temp_c = Temp::INVALID;
            out->engine_oil_temp_c = Temp::INVALID;
            out->transfer_case_state = TransferCaseState::SNA;
        }
        return true;
    }
};

class LiveTcuIoActuatorController final : public TCUIO::ITcuIoActuatorController {
public:
    const char* name() const override {
        return "live";
    }

    void apply(const TCUIO::TcuIoActuatorFrame& frame) override {
        if (sol_mpc != nullptr) {
            sol_mpc->set_current_target(frame.mpc_current_target_ma);
        }
        if (sol_spc != nullptr) {
            sol_spc->set_current_target(frame.spc_current_target_ma);
        }
        if (sol_tcc != nullptr) {
            sol_tcc->set_duty(frame.tcc_pwm_12bit);
        }

        if (sol_y3 != nullptr) {
            if (frame.y3_on) {
                sol_y3->on();
            } else {
                sol_y3->off();
            }
        }
        if (sol_y4 != nullptr) {
            if (frame.y4_on) {
                sol_y4->on();
            } else {
                sol_y4->off();
            }
        }
        if (sol_y5 != nullptr) {
            if (frame.y5_on) {
                sol_y5->on();
            } else {
                sol_y5->off();
            }
        }
        this->last_ = frame;
    }

    bool get_last(TCUIO::TcuIoActuatorFrame* out) const override {
        if (out == nullptr) {
            return false;
        }
        *out = this->last_;
        return true;
    }

private:
    TCUIO::TcuIoActuatorFrame last_{};
};

LiveTcuIoDataSource live_data_source;
TCUIO::ITcuIoDataSource* active_data_source = &live_data_source;
TCUIO::ITcuIoCaptureSink* active_capture_sink = nullptr;
LiveTcuIoActuatorController live_actuator_controller;
TCUIO::ITcuIoActuatorController* active_actuator_controller = &live_actuator_controller;
TCUIO::ITcuIoActuatorCaptureSink* active_actuator_capture_sink = nullptr;

} // namespace

TCUIO::TcuIoHardwareFrame raw_frame;
TransferCaseState last_transfer_case_pos = TransferCaseState::SNA;
bool block_shifting = false;

void init_smoothed_sensor(TCUIO::SmoothedSensor* dest, uint8_t buffer_size, int reset_value = 0) {
    if (dest == nullptr) {
        return;
    }
    dest->e_counter = 0;
    dest->sample_count = buffer_size;
    dest->last_value = reset_value*100;
}

template <typename T>
void init_onepoll(TCUIO::OnePollSensor<T>* dest, T reset_value = 0) {
    if (dest == nullptr) {
        return;
    }
    dest->e_counter = 0;
    dest->current_value = reset_value;
}

template <typename T>
void add_to_smoothed_sensor(TCUIO::SmoothedSensor* dest, T value, bool force_reset = false) {
    if (dest == nullptr) {
        return;
    }
    if (std::numeric_limits<T>::max() == value) {
        // Error condition
        if (dest->e_counter < 254) {
            dest->e_counter++; // Increase cycle error counter
            // TODO - When we create a DTC subsystem, this error counter
            // will increase the DTC occurance counter until its threshold is reached, then we can 
            // decide if the action on error is to substitute a value, or throw the TCU into fail safe mode!
        }
    } else {
        if (0 != dest->e_counter || force_reset) { // There 'was' an error, but now the value is seen. Reset the value
            dest->last_value = value*100;
        } else {
            dest->last_value = first_order_filter(dest->sample_count, value*100, dest->last_value);
        }
        dest->e_counter = 0;
    }
}

template <typename T>
void add_to_onepoll_sensor(TCUIO::OnePollSensor<T>* dest, T value) {
    if (dest == nullptr) {
        return;
    }
    if (std::numeric_limits<T>::max() == value) {
        // Error condition
        if (dest->e_counter < 254) {
            dest->e_counter++; // Increase cycle error counter
            // TODO - When we create a DTC subsystem, this error counter
            // will increase the DTC occurance counter until its threshold is reached, then we can 
            // decide if the action on error is to substitute a value, or throw the TCU into fail safe mode!
        }
    } else {
        dest->current_value = value;
        dest->e_counter = 0;
    }
}

template <typename T>
inline T get_onepoll_sensor_val(TCUIO::OnePollSensor<T>* src, uint8_t ecounter_max) {
    if (src == nullptr) {
        return std::numeric_limits<T>::max();
    }
    return (src->e_counter <= ecounter_max) ? src->current_value : std::numeric_limits<T>::max();
} 

inline uint16_t get_smoothed_sensor_val_unsigned(TCUIO::SmoothedSensor* src, uint8_t ecounter_max) {
    if (src == nullptr) {
        return UINT16_MAX;
    }
    uint16_t ret;
    if (likely(src->e_counter <= ecounter_max)) {
        ret = MIN(UINT16_MAX, src->last_value/100);
    } else {
        ret = UINT16_MAX;
    }
    return ret;
} 

inline int16_t get_smoothed_sensor_val_signed(TCUIO::SmoothedSensor* src, uint8_t ecounter_max) {
    if (src == nullptr) {
        return INT16_MAX;
    }
    int16_t ret;
    if (likely(src->e_counter <= ecounter_max)) {
        ret = MIN(INT16_MAX, src->last_value/100);
    } else {
        ret = INT16_MAX;
    }
    return ret;
} 

esp_err_t TCUIO::setup_io_layer() {
    // Setup PCB Sensor HAL
    esp_err_t ret = ESP_OK;
    if (active_data_source == nullptr) {
        active_data_source = &live_data_source;
    }
    if (active_data_source == &live_data_source) {
        if (nullptr == egs_can_hal) {
            ret = ESP_ERR_INVALID_STATE;
        } else {
            // We have a CAN Layer, continue
            Sensors::init_sensors();
        }
    }
    init_smoothed_sensor(&smoothed_sensor_n2_rpm, 3, 0);
    init_smoothed_sensor(&smoothed_sensor_n3_rpm, 3, 0);
    init_smoothed_sensor(&smoothed_sensor_out_rpm, 3, 0);

    init_smoothed_sensor(&smoothed_sensor_atf_temp, 12, 25); //250ms/20ms
    init_smoothed_sensor(&smoothed_sensor_vbatt, 12, 12000); // 250ms/20ms

    init_onepoll(&onepoll_parking_lock);

    init_onepoll(&onepoll_fl_speed);
    init_onepoll(&onepoll_fr_speed);
    init_onepoll(&onepoll_rl_speed);
    init_onepoll(&onepoll_rr_speed);

    // CAN Matrix inputs
    init_onepoll(&onepoll_motor_temperature);
    init_onepoll(&onepoll_motor_oil_temperature);

    DIFF_RATIO_F = (float)VEHICLE_CONFIG.diff_ratio / 1000.0f;
    return ret;
}

bool was_reading_from_engine = false;
void update_tft_sensor(const TCUIO::TcuIoHardwareFrame& frame) {
    // Quickhand expression here
    // parking_lock == UINT8_MAX -> True (Since PLL is not readable, just use engine temp)
    // parking_lock == 1 -> True (Since PLL is engaged, just use engine temp)
    // parking_lock == 0 -> False (We CAN use TFT temp!)
    bool atf_from_engine_temp = frame.parking_lock != 0;
    add_to_onepoll_sensor(&onepoll_parking_lock, frame.parking_lock);

    bool reset_average = was_reading_from_engine != atf_from_engine_temp; // State change
    int16_t temperature = INT16_MAX;
    if (atf_from_engine_temp) {
        temperature = Temp::celsius_i16(frame.engine_coolant_temp_c);
    } else {
        // Use TFT value
        if (frame.atf_temp_c != INT_MAX) {
            temperature = frame.atf_temp_c;
        }
    }
    // Temperature might be INT16_MAX (Something wrong with the signal)
    add_to_smoothed_sensor(&smoothed_sensor_atf_temp, temperature, reset_average);
    was_reading_from_engine = atf_from_engine_temp;
}

void update_rpm_sensors(const TCUIO::TcuIoHardwareFrame& frame) {
    // INPUT SHAFT CALCULATION
    add_to_smoothed_sensor(&smoothed_sensor_n2_rpm, frame.rpm_n2);
    add_to_smoothed_sensor(&smoothed_sensor_n3_rpm, frame.rpm_n3);
    
    // OUTPUT SHAFT RPM CALCULATION
    if (Sensors::using_dedicated_output_rpm()) {
        add_to_smoothed_sensor(&smoothed_sensor_out_rpm, frame.rpm_out);
    } else {
        uint16_t calc_rpm = UINT16_MAX;
        add_to_onepoll_sensor(&onepoll_rl_speed, WheelSpeed::raw_2x_u16(frame.wheel_rl_2x_rpm));
        add_to_onepoll_sensor(&onepoll_rr_speed, WheelSpeed::raw_2x_u16(frame.wheel_rr_2x_rpm));
        uint16_t rl = WheelSpeed::raw_2x_u16(TCUIO::wheel_rl_2x_rpm());
        uint16_t rr = WheelSpeed::raw_2x_u16(TCUIO::wheel_rr_2x_rpm());
        if (UINT16_MAX != rl || UINT16_MAX != rr) {
            if (unlikely(UINT16_MAX == rl)) {
                // RL signal is faulty
                calc_rpm = rr;
            } else if (unlikely(UINT16_MAX == rr)) {
                // RR signal is faulty
                calc_rpm = rl;
            } else {
                // Both signals OK, take an average
                calc_rpm = (rl+rr)/2;
            }
            calc_rpm *= DIFF_RATIO_F;
            // Check transfer case if present
            if (
                VEHICLE_CONFIG.is_four_matic && 
                (VEHICLE_CONFIG.transfer_case_high_ratio != 0 && VEHICLE_CONFIG.transfer_case_low_ratio != 0)
            ) {
                if (VEHICLE_CONFIG.transfer_case_high_ratio == VEHICLE_CONFIG.transfer_case_low_ratio) {
                    // For 4Matic cars without variable ratio (Like W211)
                    //
                    // NOTE: I have never seen a vehicle with locked ratios that are not 1.0,
                    //       but, we still multiply by one of the ratios, just in case
                    //       this configuration exists somewhere
                    calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0f);
                } else {
                    TransferCaseState state = frame.transfer_case_state;
                    if (TransferCaseState::Switching == state) {
                        // Switching - Use last state
                        state = last_transfer_case_pos;
                        block_shifting = true;
                    } else {
                        block_shifting = false;
                    }
                    switch (state)
                    {
                    case TransferCaseState::Hi:
                        calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0f);
                        last_transfer_case_pos = state;
                        break;
                    case TransferCaseState::Low:
                        calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_low_ratio) / 1000.0f);
                        last_transfer_case_pos = state;
                        break;
                    case TransferCaseState::Neither:
                        last_transfer_case_pos = state;
                        break; // Transfer case is disengaged, ignore
                    case TransferCaseState::Switching:
                        break; // Transfer case is switching, ignore
                    default:
                        calc_rpm = UINT16_MAX; // uh oh (Transfer case in invalid state)
                        break;
                    }
                }
            }
            if (UINT16_MAX != calc_rpm) {
                // Everything above ran in the doubled wheel speed domain, so
                // undo the doubling here - see WheelSpeed:: in tcu_scaling.h.
                calc_rpm = WheelSpeed::to_rpm_u16(WheelSpeed::from_raw_2x(calc_rpm));
            }
        }
        add_to_smoothed_sensor(&smoothed_sensor_out_rpm, calc_rpm);
    }
}

void update_can_values(const TCUIO::TcuIoHardwareFrame& frame) {
    // Motor coolant temperature (Used as a sub for ATF temperature)
    add_to_onepoll_sensor(&onepoll_motor_temperature, Temp::celsius_i16(frame.engine_coolant_temp_c));
    add_to_onepoll_sensor(&onepoll_motor_oil_temperature, Temp::celsius_i16(frame.engine_oil_temp_c));

    //int16_t m_min = egs_can_hal->get_minimum_engine_torque(100);
    //int16_t m_max = egs_can_hal->get_maximum_engine_torque(100);
    //int16_t m_sta = egs_can_hal->get_static_engine_torque(100);
    //int16_t m_esp = egs_can_hal->get_driver_engine_torque(100);

    //if (UINT16_MAX != m_min && UINT16_MAX != m_max && UINT16_MAX != m_sta && UINT16_MAX != m_esp) {
    //    // Calculate motor torques
    //    
    //}
}

void TCUIO::update_io_layer() {
    // Polled every 20ms (Task frequency)
    if (active_data_source == nullptr) {
        active_data_source = &live_data_source;
    }

    if (!active_data_source->read_frame(&raw_frame)) {
        return;
    }

    if (active_capture_sink != nullptr) {
        active_capture_sink->on_frame(raw_frame);
    }

    update_can_values(raw_frame);
    update_rpm_sensors(raw_frame);
    // Now do battery voltage
    add_to_smoothed_sensor(&smoothed_sensor_vbatt, raw_frame.battery_mv);
    // To TFT and parking lock
    update_tft_sensor(raw_frame);
}

void TCUIO::set_data_source(ITcuIoDataSource* data_source) {
    active_data_source = data_source == nullptr ? static_cast<ITcuIoDataSource*>(&live_data_source) : data_source;
}

TCUIO::ITcuIoDataSource* TCUIO::get_data_source() {
    return active_data_source;
}

void TCUIO::set_capture_sink(ITcuIoCaptureSink* sink) {
    active_capture_sink = sink;
}

TCUIO::ITcuIoCaptureSink* TCUIO::get_capture_sink() {
    return active_capture_sink;
}

void TCUIO::set_actuator_controller(ITcuIoActuatorController* controller) {
    active_actuator_controller = controller == nullptr ? static_cast<ITcuIoActuatorController*>(&live_actuator_controller) : controller;
}

TCUIO::ITcuIoActuatorController* TCUIO::get_actuator_controller() {
    return active_actuator_controller;
}

void TCUIO::set_actuator_capture_sink(ITcuIoActuatorCaptureSink* sink) {
    active_actuator_capture_sink = sink;
}

TCUIO::ITcuIoActuatorCaptureSink* TCUIO::get_actuator_capture_sink() {
    return active_actuator_capture_sink;
}

void TCUIO::apply_actuator_frame(const TcuIoActuatorFrame& frame) {
    if (active_actuator_controller == nullptr) {
        active_actuator_controller = &live_actuator_controller;
    }
    active_actuator_controller->apply(frame);
    if (active_actuator_capture_sink != nullptr) {
        active_actuator_capture_sink->on_frame(frame);
    }
}

bool TCUIO::get_last_actuator_frame(TcuIoActuatorFrame* out) {
    if (active_actuator_controller == nullptr) {
        active_actuator_controller = &live_actuator_controller;
    }
    return active_actuator_controller->get_last(out);
}

void TCUIO::set_2_1_ratio(float ratio) {
    RATIO_2_1 = ratio;
}

uint16_t TCUIO::calc_turbine_rpm(const uint16_t n2, const uint16_t n3) {
    return tcuio_calc_turbine_rpm_safe(n2, n3, RATIO_2_1);
}

uint8_t TCUIO::parking_lock() { return get_onepoll_sensor_val(&onepoll_parking_lock, 0); }
temp_c_t TCUIO::atf_temperature() { return Temp::from_celsius(get_smoothed_sensor_val_signed(&smoothed_sensor_atf_temp, 2)); }
uint16_t TCUIO::battery_mv() { return get_smoothed_sensor_val_unsigned(&smoothed_sensor_vbatt, 2); }
uint16_t TCUIO::n2_rpm() { 
    return get_smoothed_sensor_val_unsigned(&smoothed_sensor_n2_rpm, 0); 
}
uint16_t TCUIO::n3_rpm() { 
    return get_smoothed_sensor_val_unsigned(&smoothed_sensor_n3_rpm, 0); 
}

uint16_t TCUIO::output_rpm() {
    return get_smoothed_sensor_val_unsigned(&smoothed_sensor_out_rpm, 0); 
}

wheel_rpm_2x_t TCUIO::wheel_fl_2x_rpm() { return WheelSpeed::from_raw_2x(get_onepoll_sensor_val(&onepoll_fl_speed, 2)); }
wheel_rpm_2x_t TCUIO::wheel_fr_2x_rpm() { return WheelSpeed::from_raw_2x(get_onepoll_sensor_val(&onepoll_fr_speed, 2)); }
wheel_rpm_2x_t TCUIO::wheel_rl_2x_rpm() { return WheelSpeed::from_raw_2x(get_onepoll_sensor_val(&onepoll_rl_speed, 2)); }
wheel_rpm_2x_t TCUIO::wheel_rr_2x_rpm() { return WheelSpeed::from_raw_2x(get_onepoll_sensor_val(&onepoll_rr_speed, 2)); }

temp_c_t TCUIO::motor_temperature() { return Temp::from_celsius(get_onepoll_sensor_val(&onepoll_motor_temperature, 5)); }
temp_c_t TCUIO::motor_oil_temperature() { return Temp::from_celsius(get_onepoll_sensor_val(&onepoll_motor_oil_temperature, 5)); }