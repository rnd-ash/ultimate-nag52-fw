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

TCUIO::OnePollSensor<uint8_t> onepoll_parking_lock;
TCUIO::OnePollSensor<int16_t> onepoll_motor_temperature;

SensorDataRaw raw_sensors;

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

    // CAN Matrix inputs
    init_onepoll(&onepoll_motor_temperature);

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
    if (raw_sensors.rpm_n2 >= 100) {
        add_to_smoothed_sensor(&smoothed_sensor_n2_rpm, raw_sensors.rpm_n2);
    } else {
        add_to_smoothed_sensor(&smoothed_sensor_n2_rpm, 0);
    }
    if (raw_sensors.rpm_n3 >= 100) {
        add_to_smoothed_sensor(&smoothed_sensor_n3_rpm, raw_sensors.rpm_n3);
    } else {
        add_to_smoothed_sensor(&smoothed_sensor_n3_rpm, 0);
    }
    
    // OUTPUT SHAFT RPM CALCULATION
    if (Sensors::using_dedicated_output_rpm()) {
        if (raw_sensors.rpm_out >= 100) {
            add_to_smoothed_sensor(&smoothed_sensor_out_rpm, raw_sensors.rpm_out);
        } else {
            add_to_smoothed_sensor(&smoothed_sensor_out_rpm, 0);
        }
    } else {
        // Poll CANBUS
        uint16_t rl = egs_can_hal->get_rear_left_wheel(100);
        uint16_t rr = egs_can_hal->get_rear_right_wheel(100);
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
            if (VEHICLE_CONFIG.is_four_matic && (VEHICLE_CONFIG.transfer_case_high_ratio != 1000 || VEHICLE_CONFIG.transfer_case_low_ratio != 1000))
            {
                switch (egs_can_hal->get_transfer_case_state(500))
                {
                case TransferCaseState::Hi:
                    calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0);
                    break;
                case TransferCaseState::Low:
                    calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_low_ratio) / 1000.0);
                    break;
                default:
                    calc_rpm = UINT16_MAX; // uh oh (Transfer case in invalid state)
                    break;
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

uint8_t TCUIO::parking_lock() { 
    // Critical signal. If missing, fail (No substitution)
    return onepoll_parking_lock.e_counter > 0 ? UINT8_MAX : onepoll_parking_lock.current_value;
}
int16_t TCUIO::atf_temperature() { 
    return smoothed_sensor_atf_temp.buffer->get_average();
}

uint16_t TCUIO::battery_mv() { 
    return smoothed_sensor_vbatt.buffer->get_average();
}

uint16_t TCUIO::n2_rpm() { 
    // Critical signal
    if (smoothed_sensor_n2_rpm.e_counter > 0) {
        return UINT16_MAX;
    } else {
        uint16_t ret = smoothed_sensor_n2_rpm.buffer->get_average();
        if (ret > 100) {
            return ret;
        } else {
            return 0;
        }
    }
}

uint16_t TCUIO::n3_rpm() { 
    // Critical signal
    if (smoothed_sensor_n3_rpm.e_counter > 0) {
        return UINT16_MAX;
    } else {
        uint16_t ret = smoothed_sensor_n3_rpm.buffer->get_average();
        if (ret > 100) {
            return ret;
        } else {
            return 0;
        }
    }
}

uint16_t TCUIO::output_rpm() { 
    // Critical signal
    if (smoothed_sensor_out_rpm.e_counter > 0) {
        return UINT16_MAX;
    } else {
        uint16_t ret = smoothed_sensor_out_rpm.buffer->get_average();
        if (ret > 100) {
            return ret;
        } else {
            return 0;
        }
    }
}

