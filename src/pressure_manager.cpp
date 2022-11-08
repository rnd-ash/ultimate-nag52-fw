#include "pressure_manager.h"
#include <tcm_maths.h>
#include "macros.h"
#include "solenoids/constant_current.h"
#include "macros.h"
#include "maps.h"

inline uint16_t locate_pressure_map_value(const pressure_map map, int percent) {
    if (percent <= 0) { return map[0]; }
    else if (percent >= 100) { return map[10]; }
    else {
        int min = percent/10;
        int max = min+1;
        float dy = map[max] - map[min];
        float dx = (max-min)*10;
        return (map[min] + ((dy/dx)) * (percent-(min*10)));
    }
}

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->adapt_map = new AdaptationMap();
    this->req_tcc_pressure = 0;
    this->req_mpc_pressure = 0;
    this->req_mpc_pressure = 0;
    this->req_current_mpc = 0;
    this->req_current_spc = 0;
    this->gb_max_torque = max_torque;
    bool res;

    // For loading maps
    const char* key_name;
    const int16_t* default_data;

    /** Pressure PWM map **/
    const int16_t pwm_x_headers[8] = {0, 50, 600, 1000, 2350, 5600, 6600, 7700};
    const int16_t pwm_y_headers[4] = {-25, 20, 60, 150};
    key_name = MAP_NAME_PCS_BROWN;
    default_data = BROWN_PCS_CURRENT_MAP;
    this->pressure_pwm_map = new StoredTcuMap(key_name, PCS_CURRENT_MAP_SIZE, pwm_x_headers, pwm_y_headers, 8, 4, default_data);
    if (!this->pressure_pwm_map->init_ok()) {
        delete[] this->pressure_pwm_map;
    }

    /** Pressure PWM map (TCC) **/
    const int16_t pwm_tcc_x_headers[7] = {0, 2000, 4000, 5000, 7500, 10000, 15000};
    const int16_t pwm_tcc_y_headers[5] = {-25, 20, 60, 150}; 
    key_name = MAP_NAME_TCC_PWM;
    default_data = TCC_PWM_MAP;
    tcc_pwm_map = new StoredTcuMap(key_name, TCC_PWM_MAP_SIZE, pwm_tcc_x_headers, pwm_tcc_y_headers, 7, 5, default_data);
    if (!this->tcc_pwm_map->init_ok()) {
        delete[] this->tcc_pwm_map;
    }

    /** Pressure Hold 2 time map **/
    const int16_t hold2_x_headers[4] = {-20, 5, 25, 60};
    const int16_t hold2_y_headers[5] = {1,2,3,4,5}; 
    if (VEHICLE_CONFIG.is_large_nag) { // Large
        key_name = MAP_NAME_FILL_TIME_LARGE;
        default_data = LARGE_NAG_FILL_TIME_MAP;
    } else { // Small
        key_name = MAP_NAME_FILL_TIME_SMALL;
        default_data = SMALL_NAG_FILL_TIME_MAP;
    }
    hold2_time_map = new StoredTcuMap(key_name, FILL_TIME_MAP_SIZE, hold2_x_headers, hold2_y_headers, 4, 5, default_data);
    if (!this->hold2_time_map->init_ok()) {
        delete[] this->hold2_time_map;
    }

    /** Working pressure map **/
    const int16_t wp_x_headers[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    const int16_t wp_y_headers[7] = {0, 1, 2, 3, 4, 5, 6};
    if (VEHICLE_CONFIG.is_large_nag) { // Large
        key_name = MAP_NAME_WORKING_LARGE;
        default_data = LARGE_NAG_WORKING_MAP;
    } else { // Small
        key_name = MAP_NAME_WORKING_SMALL;
        default_data = SMALL_NAG_WORKING_MAP;
    }
    this->mpc_working_pressure = new StoredTcuMap(key_name, WORKING_PRESSURE_MAP_SIZE, wp_x_headers, wp_y_headers, 11, 7, default_data);
    if (!this->mpc_working_pressure->init_ok()) {
        delete[] this->mpc_working_pressure;
    }
}

Clutch PressureManager::get_clutch_to_apply(ProfileGearChange change) {
    switch(change) {
        case ProfileGearChange::ONE_TWO:
        case ProfileGearChange::FIVE_FOUR:
            return Clutch::K1;
        case ProfileGearChange::TWO_THREE:
            return Clutch::K2;
        case ProfileGearChange::THREE_FOUR:
        case ProfileGearChange::THREE_TWO:
            return Clutch::K3;
        case ProfileGearChange::FOUR_THREE:
            return Clutch::B2;
        case ProfileGearChange::FOUR_FIVE:
        case ProfileGearChange::TWO_ONE:
        default:
            return Clutch::B1;
    }
}

Clutch PressureManager::get_clutch_to_release(ProfileGearChange change) {
    switch(change) {
        case ProfileGearChange::ONE_TWO:
        case ProfileGearChange::FIVE_FOUR:
            return Clutch::B1;
        case ProfileGearChange::TWO_THREE:
        case ProfileGearChange::FOUR_THREE:
            return Clutch::K3;
        case ProfileGearChange::THREE_FOUR:
            return Clutch::B2;
        case ProfileGearChange::THREE_TWO:
            return Clutch::K2;
        case ProfileGearChange::FOUR_FIVE:
        case ProfileGearChange::TWO_ONE:
        default:
            return Clutch::K1;
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
        case GearboxGear::SignalNotAvaliable:
        default: // Already set
            gear_idx = 0;
            break;
    }

    float trq_percent = (float)(sensor_data->static_torque*100)/(float)this->gb_max_torque;
    return this->mpc_working_pressure->get_value(trq_percent, gear_idx);
}

float PressureManager::get_tcc_temp_multiplier(int atf_temp) {
    if (atf_temp < 40) {
        return 1.0;
    } else if (atf_temp < 60) {
        return 1.2;
    } else if (atf_temp < 80) {
        return 1.4;
    } else if (atf_temp < 100) {
        return 1.6;
    } else {
        return 1.8;
    }
}

ShiftData PressureManager::get_shift_data(ProfileGearChange shift_request, ShiftCharacteristics chars, uint16_t curr_mpc) {
    ShiftData sd; 
    CLAMP(chars.shift_speed, 1, 10);
    switch (shift_request) {
        case ProfileGearChange::ONE_TWO:
            sd.targ_g = 2; sd.curr_g = 1;
            sd.shift_solenoid = sol_y3;
            break;
        case ProfileGearChange::TWO_THREE:
            sd.targ_g = 3; sd.curr_g = 2;
            sd.shift_solenoid = sol_y5;
            break;
        case ProfileGearChange::THREE_FOUR:
            sd.targ_g = 4; sd.curr_g = 3;
            sd.shift_solenoid = sol_y4;
            break;
        case ProfileGearChange::FOUR_FIVE:
            sd.targ_g = 5; sd.curr_g = 4;
            sd.shift_solenoid = sol_y3;
            break;
        case ProfileGearChange::FIVE_FOUR:
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_solenoid = sol_y3;
            break;
        case ProfileGearChange::FOUR_THREE:
            sd.targ_g = 3; sd.curr_g = 4;
            sd.shift_solenoid = sol_y4;
            break;
        case ProfileGearChange::THREE_TWO:
            sd.targ_g = 2; sd.curr_g = 3;
            sd.shift_solenoid = sol_y5;
            break;
        case ProfileGearChange::TWO_ONE:
            sd.targ_g = 1; sd.curr_g = 2;
            sd.shift_solenoid = sol_y3;
            break;
    }


    sd.bleed_data.ramp_time = 0;
    sd.bleed_data.hold_time = 100;
    sd.bleed_data.spc_pressure = 650;
    sd.bleed_data.mpc_pressure = curr_mpc;

    // Make fill phase data
    this->make_fill_data(&sd.fill_data, chars, shift_request, curr_mpc);

    // Recalculated at the time of the gear change
    this->make_torque_data(&sd.torque_data, &sd.fill_data, chars, shift_request, curr_mpc);
    this->make_overlap_data(&sd.overlap_data, &sd.torque_data, chars, shift_request, curr_mpc);
    this->make_max_p_data(&sd.max_pressure_data, &sd.overlap_data, chars, shift_request, curr_mpc);

    float profile_td_amount = ((float)scale_number(chars.shift_speed, 10, 1, 1, 10) / 10.0);
    // <= 500 RPM - 0% profile_td_amount
    // >= 4500 RPM - 100% profile_td_amount
    //float rpm_td_amount = ((float)scale_number(sensor_data->input_rpm, 10, 0, 500, 4500) / 10.0);
    sd.torque_down_amount = profile_td_amount;
    sd.bleed_data.mpc_pressure = sd.fill_data.mpc_pressure;
    return sd;
}

uint16_t get_clutch_fill_pressure(Clutch c) {
    switch(c) {
        case Clutch::K1:
            return 1400;
        case Clutch::K2:
            return 1400;
        case Clutch::K3:
            return 1300;
        case Clutch::B1:
            return 1200;
        case Clutch::B2:
        default:
            return 1400;           
    }
}

void PressureManager::make_fill_data(ShiftPhase* dest, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->ramp_time = 100;
    if (this->hold2_time_map == nullptr) {
        dest->hold_time = 500;
        dest->spc_pressure = 1500;
    } else {
        switch (get_clutch_to_apply(change)) {
                case Clutch::K1:
                    dest->hold_time = hold2_time_map->get_value(this->sensor_data->atf_temp, 1);
                    dest->spc_pressure = 1200;
                    break;
                case Clutch::K2:
                    dest->hold_time = hold2_time_map->get_value(this->sensor_data->atf_temp, 2);
                    dest->spc_pressure = 1400;
                    break;
                case Clutch::K3:
                    dest->hold_time = hold2_time_map->get_value(this->sensor_data->atf_temp, 3);
                    dest->spc_pressure = 1300;
                    break;
                case Clutch::B1:
                    dest->hold_time = hold2_time_map->get_value(this->sensor_data->atf_temp, 4);
                    dest->spc_pressure = 1200;
                    break;
                case Clutch::B2:
                default:
                    dest->hold_time = hold2_time_map->get_value(this->sensor_data->atf_temp, 5);
                    dest->spc_pressure = 1400;
                    break;            
        }
    }
    const AdaptationCell* cell = this->adapt_map->get_adapt_cell(sensor_data, change, this->gb_max_torque);
    //dest->hold_time += cell->fill_time_adder;
    dest->hold_time -= dest->ramp_time;
    dest->mpc_pressure = curr_mpc + dest->spc_pressure;
}

void PressureManager::make_torque_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->hold_time = 10;
    dest->ramp_time = 00;
    dest->spc_pressure = prev->spc_pressure;
    dest->mpc_pressure = prev->mpc_pressure;
}

void PressureManager::make_overlap_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    
    dest->hold_time = scale_number(abs(sensor_data->static_torque), 300, 150, gb_max_torque/3, gb_max_torque);
    if (change == ProfileGearChange::ONE_TWO) {
        dest->hold_time += scale_number(abs(sensor_data->static_torque), 20, 0, gb_max_torque/3, (gb_max_torque/3)*2);
    }
    dest->hold_time *= (float)scale_number(chars.shift_speed*10, 1200, 750, 0, 100)/1000.0;
    dest->ramp_time = 100;

    uint16_t spc_addr = scale_number(abs(sensor_data->static_torque), 50, 750, 0, gb_max_torque);
    spc_addr *= (float)scale_number(chars.shift_speed*10, 1000, 2000, 0, 100)/1000.0;

    dest->spc_pressure = prev->spc_pressure+spc_addr;
    //dest->mpc_pressure = scale_number(sensor_data->static_torque, dest->spc_pressure*0.9, 900, 0, gb_max_torque);
    dest->mpc_pressure = dest->spc_pressure*0.9;
}

void PressureManager::make_max_p_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->ramp_time = 250;
    dest->hold_time = scale_number(sensor_data->atf_temp, 1500, 100, -20, 30);
    dest->spc_pressure = MIN(6000, prev->spc_pressure*1.5);
    dest->mpc_pressure = prev->mpc_pressure;
}

// Get PWM value (out of 4096) to write to the solenoid
uint16_t PressureManager::get_p_solenoid_current(uint16_t request_mbar, bool is_spc) {
    if (this->pressure_pwm_map == nullptr) {
        return 0; // 10% (Failsafe)
    }
    uint16_t c = this->pressure_pwm_map->get_value(request_mbar, this->sensor_data->atf_temp);
    if (is_spc) {
        this->req_current_spc = c;
    } else {
        this->req_current_mpc = c;
    }
    return c;
}

uint16_t PressureManager::get_tcc_solenoid_pwm_duty(uint16_t request_mbar) {
    if (request_mbar == 0) {
        return 0; // Shortcut for when off
    }
    if (this->tcc_pwm_map == nullptr) {
        return 0; // 0% (Failsafe - TCC off)
    }
    return this->tcc_pwm_map->get_value(request_mbar, this->sensor_data->atf_temp);
}

void PressureManager::set_target_mpc_pressure(uint16_t targ) {
    egs_can_hal->set_mpc_pressure(targ);
    this->req_mpc_pressure = targ;
    mpc_cc->set_target_current(this->get_p_solenoid_current(this->req_mpc_pressure, false));
}

void PressureManager::set_target_spc_pressure(uint16_t targ) {
    egs_can_hal->set_spc_pressure(targ);
    this->req_spc_pressure = targ;
    spc_cc->set_target_current(this->get_p_solenoid_current(this->req_spc_pressure, true));
}

void PressureManager::disable_spc() {
    this->req_spc_pressure = 7000;
    this->req_current_spc = 0;
    egs_can_hal->set_spc_pressure(7000);
    spc_cc->set_target_current(0);
}

void PressureManager::set_target_tcc_pressure(uint16_t targ) {
    egs_can_hal->set_tcc_pressure(targ);
    this->req_tcc_pressure = targ;
    sol_tcc->write_pwm_12_bit(this->get_tcc_solenoid_pwm_duty(this->req_tcc_pressure)); // TCC is just raw PWM, no voltage compensating
}

uint16_t PressureManager::get_targ_mpc_pressure(){ return this->req_mpc_pressure; }
uint16_t PressureManager::get_targ_spc_pressure(){ return this->req_spc_pressure; }
uint16_t PressureManager::get_targ_tcc_pressure(){ return this->req_tcc_pressure; }
uint16_t PressureManager::get_targ_spc_current(){ return this->req_current_spc; }
uint16_t PressureManager::get_targ_mpc_current(){ return this->req_current_mpc; }