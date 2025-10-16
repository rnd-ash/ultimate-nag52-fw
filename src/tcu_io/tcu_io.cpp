//
// Created by ashcon on 9/2/24.
//

#include "tcu_io.hpp"
#include "sensors.h"
#include <limits>

float RATIO_2_1 = 1.61;
float DIFF_RATIO_F = 1.00;
bool INPUT_RPM_SANITY_CHECK = true;

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

SensorDataRaw raw_sensors;
TransferCaseState last_transfer_case_pos = TransferCaseState::SNA;
bool block_shifting = false;

void init_smoothed_sensor(TCUIO::SmoothedSensor* dest, uint8_t buffer_size, int reset_value = 0) {
    dest->e_counter = 0;
    dest->buffer = new FirstOrderAverage(buffer_size, reset_value);
}

template <typename T>
void init_onepoll(TCUIO::OnePollSensor<T>* dest, T reset_value = 0) {
    dest->e_counter = 0;
    dest->current_value = reset_value;
}

template <typename T>
void add_to_smoothed_sensor(TCUIO::SmoothedSensor* dest, T value, bool force_reset = false) {
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
            dest->buffer->reset(value);
        } else {
            dest->buffer->add_sample(value); // Normal operation
        }
        dest->e_counter = 0;
    }
}

template <typename T>
void add_to_onepoll_sensor(TCUIO::OnePollSensor<T>* dest, T value) {
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
    return (src->e_counter <= ecounter_max) ? src->current_value : std::numeric_limits<T>::max();
} 

inline uint16_t get_smoothed_sensor_val_unsigned(TCUIO::SmoothedSensor* src, uint8_t ecounter_max) {
    uint16_t ret;
    if (likely(src->e_counter <= ecounter_max)) {
        ret = MIN(UINT16_MAX, src->buffer->get_average());
    } else {
        ret = UINT16_MAX;
    }
    return ret;
} 

inline int16_t get_smoothed_sensor_val_signed(TCUIO::SmoothedSensor* src, uint8_t ecounter_max) {
    int16_t ret;
    if (likely(src->e_counter <= ecounter_max)) {
        ret = MIN(INT16_MAX, src->buffer->get_average());
    } else {
        ret = INT16_MAX;
    }
    return ret;
} 

esp_err_t TCUIO::setup_io_layer() {
    // Setup PCB Sensor HAL
    esp_err_t ret = ESP_OK;
    if (nullptr == egs_can_hal) {
        ret = ESP_ERR_INVALID_STATE; 
    } else {
        // We have a CAN Layer, continue
        Sensors::init_sensors();
    }
    init_smoothed_sensor(&smoothed_sensor_n2_rpm, 4, 0);
    init_smoothed_sensor(&smoothed_sensor_n3_rpm, 4, 0);
    init_smoothed_sensor(&smoothed_sensor_out_rpm, 4, 0);

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

    DIFF_RATIO_F = (float)VEHICLE_CONFIG.diff_ratio / 1000.0;
    return ret;
}

bool was_reading_from_engine = false;
void update_tft_sensor() {
    // Quickhand expression here
    // parking_lock == UINT8_MAX -> True (Since PLL is not readable, just use engine temp)
    // parking_lock == 1 -> True (Since PLL is engaged, just use engine temp)
    // parking_lock == 0 -> False (We CAN use TFT temp!)
    bool atf_from_engine_temp = raw_sensors.parking_lock != 0;
    add_to_onepoll_sensor(&onepoll_parking_lock, raw_sensors.parking_lock);

    bool reset_average = was_reading_from_engine != atf_from_engine_temp; // State change
    int temperature = 25;
    if (atf_from_engine_temp) {
        // Request value from CAN
        temperature = onepoll_motor_temperature.current_value;
    } else {
        // Use TFT value
        temperature = raw_sensors.atf_temp_c;
    }
    // Temperature might be INT16_MAX (Something wrong with the signal)
    add_to_smoothed_sensor(&smoothed_sensor_atf_temp, temperature, reset_average);
    was_reading_from_engine = atf_from_engine_temp;
}

void update_rpm_sensors() {
    // INPUT SHAFT CALCULATION
    uint16_t calc_rpm = UINT16_MAX;
    add_to_smoothed_sensor(&smoothed_sensor_n2_rpm, raw_sensors.rpm_n2);
    add_to_smoothed_sensor(&smoothed_sensor_n3_rpm, raw_sensors.rpm_n3);
    
    // OUTPUT SHAFT RPM CALCULATION
    if (Sensors::using_dedicated_output_rpm()) {
        add_to_smoothed_sensor(&smoothed_sensor_out_rpm, raw_sensors.rpm_out);
    } else {
        // Poll CANBUS
        add_to_onepoll_sensor(&onepoll_rl_speed, egs_can_hal->get_rear_left_wheel(100));
        add_to_onepoll_sensor(&onepoll_rr_speed, egs_can_hal->get_rear_right_wheel(100));
        uint16_t rl = TCUIO::wheel_rl_2x_rpm();
        uint16_t rr = TCUIO::wheel_rr_2x_rpm();
        calc_rpm = UINT16_MAX;
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
                    calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0);
                } else {
                    TransferCaseState state = egs_can_hal->get_transfer_case_state(500);
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
                        calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0);
                        last_transfer_case_pos = state;
                        break;
                    case TransferCaseState::Low:
                        calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_low_ratio) / 1000.0);
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
                calc_rpm /= 2; // Since wheel speed is 2x
            }
        }
        add_to_smoothed_sensor(&smoothed_sensor_out_rpm, calc_rpm);
    }
}

void update_can_values() {
    // Motor coolant temperature (Used as a sub for ATF temperature)
    add_to_onepoll_sensor(&onepoll_motor_temperature, egs_can_hal->get_engine_coolant_temp(100));
    add_to_onepoll_sensor(&onepoll_motor_oil_temperature, egs_can_hal->get_engine_oil_temp(100));

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
    Sensors::update(&raw_sensors); // 1. Update PCB Hal
    // Poll CAN Layer inputs
    update_can_values();

    update_rpm_sensors();
    // Now do battery voltage
    add_to_smoothed_sensor(&smoothed_sensor_vbatt, raw_sensors.battery_mv);
    // To TFT and parking lock
    update_tft_sensor();
}

void TCUIO::set_input_rpm_perform_sanity_check(bool conduct) {
    INPUT_RPM_SANITY_CHECK = conduct;
}

void TCUIO::set_2_1_ratio(float ratio) {
    RATIO_2_1 = ratio;
}

uint16_t TCUIO::calc_turbine_rpm(const uint16_t n2, const uint16_t n3) {
    return MAX(0,((float)n2 * RATIO_2_1) + ((float)n3 - (RATIO_2_1*(float)n3)));
}

uint8_t TCUIO::parking_lock() { return get_onepoll_sensor_val(&onepoll_parking_lock, 0); }
int16_t TCUIO::atf_temperature() { return smoothed_sensor_atf_temp.buffer->get_average();}
uint16_t TCUIO::battery_mv() { return smoothed_sensor_vbatt.buffer->get_average();}
uint16_t TCUIO::n2_rpm() { 
    return get_smoothed_sensor_val_unsigned(&smoothed_sensor_n2_rpm, 0); 
}
uint16_t TCUIO::n3_rpm() { 
    return get_smoothed_sensor_val_unsigned(&smoothed_sensor_n3_rpm, 0); 
}

uint16_t TCUIO::output_rpm() {
    return get_smoothed_sensor_val_unsigned(&smoothed_sensor_out_rpm, 0); 
}

uint16_t TCUIO::wheel_fl_2x_rpm() { return get_onepoll_sensor_val(&onepoll_fl_speed, 2); }
uint16_t TCUIO::wheel_fr_2x_rpm() { return get_onepoll_sensor_val(&onepoll_fr_speed, 2); }
uint16_t TCUIO::wheel_rl_2x_rpm() { return get_onepoll_sensor_val(&onepoll_rl_speed, 2); }
uint16_t TCUIO::wheel_rr_2x_rpm() { return get_onepoll_sensor_val(&onepoll_rr_speed, 2); }

int16_t TCUIO::motor_temperature() { return get_onepoll_sensor_val(&onepoll_motor_temperature, 5); }
int16_t TCUIO::motor_oil_temperature() { return get_onepoll_sensor_val(&onepoll_motor_oil_temperature, 5); }