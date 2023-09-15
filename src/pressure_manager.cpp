#include "pressure_manager.h"
#include <tcu_maths.h>
#include "solenoids/solenoids.h"
#include "maps.h"
#include "common_structs_ops.h"
#include "nvs/module_settings.h"
#include "nvs/device_mode.h"
#include "nvs/all_keys.h"
PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->req_tcc_clutch_pressure = 0;
    this->req_mpc_clutch_pressure = 0;
    this->req_spc_clutch_pressure = 0;
    this->gb_max_torque = max_torque;

    // For loading maps
    const char* key_name;
    const int16_t* default_data;

    /** Pressure PWM map **/
    const int16_t pwm_x_headers[8] = {0, 50, 600, 1000, 2350, 5600, 6600, 7700};
    const int16_t pwm_y_headers[4] = {-25, 20, 60, 150};
    key_name = NVS_KEY_MAP_NAME_PCS_BROWN;
    default_data = BROWN_PCS_CURRENT_MAP;
    this->pressure_pwm_map = new StoredMap(key_name, PCS_CURRENT_MAP_SIZE, pwm_x_headers, pwm_y_headers, 8, 4, default_data);
    if (this->pressure_pwm_map->init_status() != ESP_OK) {
        delete[] this->pressure_pwm_map;
    }

    /** Pressure PWM map (TCC) **/
    const int16_t pwm_tcc_x_headers[7] = {0, 2000, 4000, 5000, 7500, 10000, 15000};
    const int16_t pwm_tcc_y_headers[5] = {0, 30, 60, 90, 120}; 
    key_name = NVS_KEY_MAP_NAME_TCC_PWM;
    default_data = TCC_PWM_MAP;
    tcc_pwm_map = new StoredMap(key_name, TCC_PWM_MAP_SIZE, pwm_tcc_x_headers, pwm_tcc_y_headers, 7, 5, default_data);
    if (this->tcc_pwm_map->init_status() != ESP_OK) {
        delete[] this->tcc_pwm_map;
    }

    /** Pressure Hold 2 time map **/
    const int16_t hold2_x_headers[4] = {-20, 5, 25, 60};
    const int16_t hold2_y_headers[5] = {1,2,3,4,5}; 
    if (VEHICLE_CONFIG.is_large_nag) { // Large
        key_name = NVS_KEY_MAP_NAME_FILL_TIME_LARGE;
        default_data = LARGE_NAG_FILL_TIME_MAP;
    } else { // Small
        key_name = NVS_KEY_MAP_NAME_FILL_TIME_SMALL;
        default_data = SMALL_NAG_FILL_TIME_MAP;
    }
    hold2_time_map = new StoredMap(key_name, FILL_TIME_MAP_SIZE, hold2_x_headers, hold2_y_headers, 4, 5, default_data);
    if (this->hold2_time_map->init_status() != ESP_OK) {
        delete[] this->hold2_time_map;
    }

    /** Pressure Hold 2 pressure map **/
    const int16_t hold2p_x_headers[1] = {1};
    const int16_t hold2p_y_headers[5] = {1,2,3,4,5};
    key_name = NVS_KEY_MAP_NAME_FILL_PRESSURE;
    default_data = NAG_FILL_PRESSURE_MAP;
    hold2_pressure_map = new StoredMap(key_name, FILL_PRESSURE_MAP_SIZE, hold2p_x_headers, hold2p_y_headers, 1, 5, default_data);
    if (this->hold2_pressure_map->init_status() != ESP_OK) {
        delete[] this->hold2_pressure_map;
    }

    /** Working pressure map **/
    const int16_t wp_x_headers[16] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};
    const int16_t wp_y_headers[7] = {0, 1, 2, 3, 4, 5, 6};
    key_name = NVS_KEY_MAP_NAME_WORKING_MPC;
    default_data = NAG_WORKING_MAP;
    this->mpc_working_pressure = new StoredMap(key_name, WORKING_PRESSURE_MAP_SIZE, wp_x_headers, wp_y_headers, 16, 7, default_data);
    if (this->mpc_working_pressure->init_status() != ESP_OK) {
        delete[] this->mpc_working_pressure;
    }
}

void PressureManager::controller_loop() {
    uint16_t p_last_spc = 0;
    uint16_t p_last_mpc = 0;
    uint16_t spc_now = 0;
    uint16_t mpc_now = 0;
    while(1) {
        // Ignore
        if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {

        } else {
            spc_now = this->req_spc_clutch_pressure;
            mpc_now = this->req_mpc_clutch_pressure;
            int max_spc = 7700;
            if (sol_y3->is_on()) {
                // 1-2 circuit is open (Correct pressure for K1)
                // K1 is controlled by Shift pressure
                if ((this->c_gear == 1 && this->t_gear == 2) || (this->c_gear == 2 && this->t_gear == 1)) {
                    spc_now /= 1.9;
                    max_spc /= 1.9;
                }
            }
            if (spc_now >= 7700) {
                spc_now = 7700;
            }
            if (spc_now >= 7700) {
                spc_now = 7700;
            }
            if (p_last_spc != spc_now) {
                p_last_spc = spc_now;
                if (spc_now >= 7700) {
                    sol_spc->set_current_target(0);
                } else {
                    sol_spc->set_current_target(this->get_p_solenoid_current(spc_now));
                }
            }
            if (p_last_mpc != mpc_now) {
                p_last_mpc = mpc_now;
                if (mpc_now >= 7700) {
                    sol_mpc->set_current_target(0);
                } else {
                    sol_mpc->set_current_target(this->get_p_solenoid_current(mpc_now));
                }
            }
            this->commanded_spc_pressure = spc_now;
            this->commanded_mpc_pressure = mpc_now;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g) {
    if (this->mpc_working_pressure == nullptr) {
        return 7000; // Failsafe!
    }

    uint8_t gear_idx = 0;
    switch(curr_g) {
        case GearboxGear::First:
            gear_idx = 2;
            break;
        case GearboxGear::Second:
            gear_idx = 3;
            break;
        case GearboxGear::Third:
            gear_idx = 4;
            break;
        case GearboxGear::Fourth:
            gear_idx = 5;
            break;
        case GearboxGear::Fifth:
            gear_idx = 6;
            break;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            gear_idx = 1;
            break;
        case GearboxGear::Park:
        case GearboxGear::Neutral:
        case GearboxGear::SignalNotAvailable:
        default: // Already set
            gear_idx = 0;
            break;
    }

    float trq_percent = (float)(sensor_data->input_torque*100.0)/(float)this->gb_max_torque;
    if (sensor_data->output_rpm == 0 && sensor_data->pedal_pos == 0) {
        return 500;
    }
    return this->mpc_working_pressure->get_value(trq_percent, gear_idx);
}

ShiftData PressureManager::get_basic_shift_data(GearboxConfiguration* cfg, ProfileGearChange shift_request, ShiftCharacteristics chars) {
    ShiftData sd; 
    switch (shift_request) {
        case ProfileGearChange::ONE_TWO:
            sd.targ_g = 2; sd.curr_g = 1;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
        case ProfileGearChange::TWO_THREE:
            sd.targ_g = 3; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            break;
        case ProfileGearChange::THREE_FOUR:
            sd.targ_g = 4; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            break;
        case ProfileGearChange::FOUR_FIVE:
            sd.targ_g = 5; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
        case ProfileGearChange::FIVE_FOUR:
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
        case ProfileGearChange::FOUR_THREE:
            sd.targ_g = 3; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            break;
        case ProfileGearChange::THREE_TWO:
            sd.targ_g = 2; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            break;
        case ProfileGearChange::TWO_ONE:
            sd.targ_g = 1; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
    }
    this->c_gear = sd.curr_g;
    this->t_gear = sd.targ_g;
    return sd;
}


PrefillData PressureManager::make_fill_data(ProfileGearChange change) {
    if (nullptr == this->hold2_time_map) {
        return PrefillData {
            .fill_time = 500,
            .fill_pressure_on_clutch = 1500,
            .fill_pressure_off_clutch = 1500,
        };
    } else {
        Clutch to_apply = get_clutch_to_apply(change);
        Clutch to_release = get_clutch_to_release(change);
        return PrefillData {
            .fill_time = (uint16_t)hold2_time_map->get_value(this->sensor_data->atf_temp, (uint8_t)to_apply),
            .fill_pressure_on_clutch = (uint16_t)hold2_pressure_map->get_value(1, (uint8_t)to_apply),
            .fill_pressure_off_clutch = (uint16_t)hold2_pressure_map->get_value(1, (uint8_t)to_release)
        };
    }
}

PressureStageTiming PressureManager::get_max_pressure_timing() {
    return PressureStageTiming {
        .hold_time = (uint16_t)scale_number(this->sensor_data->atf_temp, 1500, 100, -20, 30),
        .ramp_time = 250,
    };
}

// Get PWM value (out of 4096) to write to the solenoid
uint16_t PressureManager::get_p_solenoid_current(uint16_t request_mbar) const {
    if (this->pressure_pwm_map == nullptr) {
        return 0; // 10% (Failsafe)
    }
    return this->pressure_pwm_map->get_value(request_mbar, this->sensor_data->atf_temp);
}

uint16_t PressureManager::get_tcc_solenoid_pwm_duty(uint16_t request_mbar) const {
    if (request_mbar == 0) {
        return 0; // Shortcut for when off
    }
    if (this->tcc_pwm_map == nullptr) {
        return 0; // 0% (Failsafe - TCC off)
    }
    return this->tcc_pwm_map->get_value(request_mbar, this->sensor_data->atf_temp);
}

void PressureManager::set_shift_circuit(ShiftCircuit ss, bool enable) {
    if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {
        return;
    }
    OnOffSolenoid* manipulated = nullptr;
    if (ShiftCircuit::sc_1_2 == ss) {
        manipulated = sol_y3;
    } else if (ShiftCircuit::sc_2_3 == ss) {
        manipulated = sol_y5;
    } else if (ShiftCircuit::sc_3_4 == ss) { // 3-4
        manipulated = sol_y4;
    } else { // No shift circuit (placeholder)
        this->t_gear = 0;
        this->c_gear = 0;
        return;
    }
    // Firstly, check if new value is 0 (Close the solenoid!)
    if (!enable) {
        manipulated->off();
    } else { // Check if current value is 0, if so, write full PWM
        manipulated->on();
    }
}

void PressureManager::set_target_mpc_pressure(uint16_t targ) {
    this->req_mpc_clutch_pressure = targ;
}

void PressureManager::set_target_spc_pressure(uint16_t targ) {
    this->req_spc_clutch_pressure = targ;
}

void PressureManager::set_target_spc_and_mpc_pressure(uint16_t mpc, uint16_t spc) {
    this->req_mpc_clutch_pressure = mpc;
    this->req_spc_clutch_pressure = spc;
}

void PressureManager::set_spc_p_max(void) {
    this->req_spc_clutch_pressure = 7700;
}

void PressureManager::set_target_tcc_pressure(uint16_t targ) {
    if (targ > 15000) {
        targ = 15000;
    }
    this->req_tcc_clutch_pressure = targ;
    sol_tcc->set_duty(this->get_tcc_solenoid_pwm_duty(this->req_tcc_clutch_pressure));
}

uint16_t PressureManager::get_targ_line_pressure(void) {
    return 0; // Unimplemented this->req_line_pressure;
}

uint16_t PressureManager::get_targ_mpc_clutch_pressure(void) const {
    uint16_t ret = 0;
    if (sol_y3->is_on() || sol_y4->is_on() || sol_y5->is_on()) {
        ret = this->req_mpc_clutch_pressure;
    }
    return ret;
}

uint16_t PressureManager::get_off_clutch_hold_pressure(Clutch c) {
    uint16_t min_pressure = 0;
    uint16_t pressure_from_trq;

    switch(c) {
        case Clutch::K1:
            break;
        case Clutch::K2:
            break;
        case Clutch::K3:
            break;
        case Clutch::B1:
            break;
        case Clutch::B2:
        default:
            break;
    }
    pressure_from_trq = 0;
    return MAX(min_pressure, pressure_from_trq);
}

uint16_t PressureManager::get_targ_spc_clutch_pressure(void) const {
    // 0 if no shift circuits are open
    uint16_t ret = 0;
    if (sol_y3->is_on() || sol_y4->is_on() || sol_y5->is_on()) {
        ret = this->req_spc_clutch_pressure;
    }
    return ret;
}

uint16_t PressureManager::get_targ_mpc_solenoid_pressure(void) const {
    return this->commanded_mpc_pressure;
}

uint16_t PressureManager::get_targ_spc_solenoid_pressure(void) const {
    return this->commanded_spc_pressure;
}

uint16_t PressureManager::get_targ_tcc_pressure(void) const {
    return this->req_tcc_clutch_pressure;
}

uint8_t PressureManager::get_active_shift_circuits(void) const {
    uint8_t flg = 0;
    if (sol_y3->is_on()) {
        flg |= (uint8_t)ShiftCircuit::sc_1_2;
    }
    if (sol_y5->is_on()) {
        flg |= (uint8_t)ShiftCircuit::sc_2_3;
    }
    if (sol_y4->is_on()) {
        flg |= (uint8_t)ShiftCircuit::sc_3_4;
    }
    return flg;
}

//uint16_t PressureManager::get_targ_line_pressure(){ return this->req_current_mpc; }

StoredMap* PressureManager::get_pcs_map() { return this->pressure_pwm_map; }
StoredMap* PressureManager::get_tcc_pwm_map() { return this->tcc_pwm_map; }
StoredMap* PressureManager::get_working_map() { return this->mpc_working_pressure; }
StoredMap* PressureManager::get_fill_time_map() { return this->hold2_time_map; }
StoredMap* PressureManager::get_fill_pressure_map() { return  this->hold2_pressure_map; }

PressureManager* pressure_manager = nullptr;