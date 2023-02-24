#include "pressure_manager.h"
#include <tcu_maths.h>
#include "solenoids/constant_current.h"
#include "maps.h"

// inline uint16_t locate_pressure_map_value(const pressure_map map, int percent) {
//     if (percent <= 0) { return map[0]; }
//     else if (percent >= 100) { return map[10]; }
//     else {
//         int min = percent/10;
//         int max = min+1;
//         float dy = map[max] - map[min];
//         float dx = (max-min)*10;
//         return (map[min] + ((dy/dx)) * (percent-(min*10)));
//     }
// }

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->pressure_adapt_system = new ShiftAdaptationSystem();
    this->req_tcc_pressure = 0;
    this->req_mpc_pressure = 0;
    this->req_spc_pressure = 0;
    this->req_current_mpc = 0;
    this->req_current_spc = 0;
    this->gb_max_torque = max_torque;

    // For loading maps
    const char* key_name;
    const int16_t* default_data;

    /** Pressure PWM map **/
    const int16_t pwm_x_headers[8] = {0, 50, 600, 1000, 2350, 5600, 6600, 7700};
    const int16_t pwm_y_headers[4] = {-25, 20, 60, 150};
    key_name = MAP_NAME_PCS_BROWN;
    default_data = BROWN_PCS_CURRENT_MAP;
    this->pressure_pwm_map = new StoredTcuMap(key_name, PCS_CURRENT_MAP_SIZE, pwm_x_headers, pwm_y_headers, 8, 4, default_data);
    if (this->pressure_pwm_map->init_status() != ESP_OK) {
        delete[] this->pressure_pwm_map;
    }

    /** Pressure PWM map (TCC) **/
    const int16_t pwm_tcc_x_headers[7] = {0, 2000, 4000, 5000, 7500, 10000, 15000};
    const int16_t pwm_tcc_y_headers[5] = {0, 30, 60, 90, 120}; 
    key_name = MAP_NAME_TCC_PWM;
    default_data = TCC_PWM_MAP;
    tcc_pwm_map = new StoredTcuMap(key_name, TCC_PWM_MAP_SIZE, pwm_tcc_x_headers, pwm_tcc_y_headers, 7, 5, default_data);
    if (this->tcc_pwm_map->init_status() != ESP_OK) {
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
    if (this->hold2_time_map->init_status() != ESP_OK) {
        delete[] this->hold2_time_map;
    }

    /** Pressure Hold 2 pressure map **/
    const int16_t hold2p_x_headers[1] = {1};
    const int16_t hold2p_y_headers[5] = {1,2,3,4,5};
    if (VEHICLE_CONFIG.is_large_nag) { // Large
        key_name = MAP_NAME_FILL_PRESSURE_LARGE;
        default_data = LARGE_NAG_FILL_PRESSURE_MAP;
    } else { // Small
        key_name = MAP_NAME_FILL_PRESSURE_SMALL;
        default_data = SMALL_NAG_FILL_PRESSURE_MAP;
    }
    hold2_pressure_map = new StoredTcuMap(key_name, FILL_PRESSURE_MAP_SIZE, hold2p_x_headers, hold2p_y_headers, 1, 5, default_data);
    if (this->hold2_pressure_map->init_status() != ESP_OK) {
        delete[] this->hold2_pressure_map;
    }

    /** Pressure Hold 2 pressure map **/
    const int16_t hold2mpc_p_x_headers[11] = {0,10,20,30,40,50,60,70,80,90,100};
    const int16_t hold2mpc_p_y_headers[5] = {1,2,3,4,5};
    key_name = MAP_NAME_FILL_MPC_ADDER;
    default_data = FILL_MPC_ADDER_MAP;
    hold2_pressure_mpc_adder_map = new StoredTcuMap(key_name, FILL_PRESSURE_ADDER_MAP_SIZE, hold2mpc_p_x_headers, hold2mpc_p_y_headers, 11, 5, default_data);
    if (this->hold2_pressure_mpc_adder_map->init_status() != ESP_OK) {
        delete[] this->hold2_pressure_mpc_adder_map;
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
    if (this->mpc_working_pressure->init_status() != ESP_OK) {
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
        case GearboxGear::SignalNotAvailable:
        default: // Already set
            gear_idx = 0;
            break;
    }

    float trq_percent = (float)(sensor_data->input_torque*100.0)/(float)this->gb_max_torque;
    return this->mpc_working_pressure->get_value(trq_percent, gear_idx);
}

float PressureManager::get_tcc_temp_multiplier(int atf_temp) {
    return (float)scale_number(sensor_data->atf_temp, 100, 200, 40, 100) / 100.0;
}

ShiftData PressureManager::get_shift_data(GearboxConfiguration* cfg, ProfileGearChange shift_request, ShiftCharacteristics chars, uint16_t curr_mpc) {
        ShiftData sd; 
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
    sd.bleed_data.hold_time = 200;
    sd.bleed_data.spc_pressure = 650;
    sd.bleed_data.mpc_pressure = curr_mpc;
    sd.bleed_data.mpc_offset_mode = false;
    sd.bleed_data.spc_offset_mode = false;

    // Make fill phase data
    this->make_fill_data(&sd.fill_data, chars, shift_request, curr_mpc);

    // Recalculated at the time of the gear change
    this->make_torque_and_overlap_data(&sd.overlap_data, &sd.torque_data, &sd.overlap_data, chars, shift_request, curr_mpc);
    this->make_max_p_data(&sd.max_pressure_data, &sd.overlap_data, chars, shift_request, curr_mpc);

    //float profile_td_amount = ((float)scale_number(chars.shift_speed, 10, 1, 1, 10) / 10.0);
    // <= 500 RPM - 0% profile_td_amount
    // >= 4500 RPM - 100% profile_td_amount
    //float rpm_td_amount = ((float)scale_number(sensor_data->input_rpm, 10, 0, 500, 4500) / 10.0);
    sd.torque_down_amount = 0;
    sd.bleed_data.mpc_pressure = sd.fill_data.mpc_pressure;
    return sd;
}

void PressureManager::make_fill_data(ShiftPhase* dest, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    if (this->hold2_time_map == nullptr) {
        dest->hold_time = 500;
        dest->spc_pressure = 1500;
        dest->mpc_pressure = curr_mpc;
        dest->mpc_offset_mode = false;
        dest->spc_offset_mode = false;
    } else {
        Clutch to_change = get_clutch_to_apply(change);
        Clutch to_release = get_clutch_to_release(change);
        dest->ramp_time = 0;
        dest->hold_time = hold2_time_map->get_value(this->sensor_data->atf_temp, (uint8_t)to_change);
        dest->mpc_pressure = 100;
        dest->spc_pressure = hold2_pressure_map->get_value(1, (uint8_t)to_change) + scale_number(chars.target_shift_time, 200, 0, 100, 500);
        dest->mpc_offset_mode = true;
        dest->spc_offset_mode = false;
    }
    //const AdaptationCell* cell = tvirhis->adapt_map->get_adapt_cell(sensor_data, change, this->gb_max_torque);
}

void PressureManager::make_torque_and_overlap_data(ShiftPhase* dest_torque, ShiftPhase* dest_overlap, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    //int div = scale_number(abs(sensor_data->static_torque), 2, 5, 100, this->gb_max_torque);
    dest_torque->hold_time = 100;
    dest_torque->ramp_time = 0;
    dest_overlap->ramp_time = (float)chars.target_shift_time/2;
    dest_overlap->hold_time = (float)chars.target_shift_time/2;
    uint16_t spc_addr =  MAX(100, abs(sensor_data->input_torque)*2.5); // 2mBar per Nm
    dest_torque->mpc_pressure = 0;
    dest_overlap->mpc_pressure = 0;
    dest_torque->mpc_offset_mode = true;
    dest_overlap->mpc_offset_mode = true;
    dest_torque->spc_offset_mode = true;
    dest_overlap->spc_offset_mode = true;
    dest_torque->spc_pressure = 0;
    dest_overlap->spc_pressure = spc_addr;
}

void PressureManager::make_max_p_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->ramp_time = 250;
    dest->hold_time = scale_number(sensor_data->atf_temp, 1500, 100, -20, 30);
    dest->spc_pressure = 7000;
    dest->mpc_pressure = 0; // 0 offset
    dest->spc_offset_mode = false;
    dest->mpc_offset_mode = true;
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
    if (targ > 7000) {
        targ = 7000;
    }
    egs_can_hal->set_mpc_pressure(targ);
    this->req_mpc_pressure = targ;
    mpc_cc->set_target_current(this->get_p_solenoid_current(this->req_mpc_pressure, false));
}

void PressureManager::set_target_spc_pressure(uint16_t targ) {
    if (targ > 7000) {
        targ = 7000;
    }
    egs_can_hal->set_spc_pressure(targ);
    this->req_spc_pressure = targ;
    spc_cc->set_target_current(this->get_p_solenoid_current(this->req_spc_pressure, true));
}

void PressureManager::disable_spc() {
    this->req_spc_pressure = 0;
    this->req_current_spc = 0;
    egs_can_hal->set_spc_pressure(7000);
    spc_cc->set_target_current(0);
}

void PressureManager::set_target_tcc_pressure(uint16_t targ) {
    if (targ > 15000) {
        targ = 15000;
    }
    egs_can_hal->set_tcc_pressure(targ);
    this->req_tcc_pressure = targ;
    sol_tcc->write_pwm_12_bit(this->get_tcc_solenoid_pwm_duty(this->req_tcc_pressure)); // TCC is just raw PWM, no voltage compensating
}

uint16_t PressureManager::get_mpc_hold_adder(Clutch to_apply) {
    uint16_t ret = 0;
    if (this->hold2_pressure_mpc_adder_map != nullptr) {
        float trq_percent = (float)(sensor_data->input_torque*100.0)/(float)this->gb_max_torque;
        ret = hold2_pressure_mpc_adder_map->get_value(trq_percent, (uint8_t)to_apply);
    }
    return ret;
}

uint16_t PressureManager::get_targ_mpc_pressure(){ return this->req_mpc_pressure; }
uint16_t PressureManager::get_targ_spc_pressure(){ return this->req_spc_pressure; }
uint16_t PressureManager::get_targ_tcc_pressure(){ return this->req_tcc_pressure; }
uint16_t PressureManager::get_targ_spc_current(){ return this->req_current_spc; }
uint16_t PressureManager::get_targ_mpc_current(){ return this->req_current_mpc; }

StoredTcuMap* PressureManager::get_pcs_map() { return this->pressure_pwm_map; }
StoredTcuMap* PressureManager::get_tcc_pwm_map() { return this->tcc_pwm_map; }
StoredTcuMap* PressureManager::get_working_map() { return this->mpc_working_pressure; }
StoredTcuMap* PressureManager::get_fill_time_map() { return this->hold2_time_map; }
StoredTcuMap* PressureManager::get_fill_pressure_map() { return  this->hold2_pressure_map; }
StoredTcuMap* PressureManager::get_fill_pressure_mpc_adder_map() { return  this->hold2_pressure_mpc_adder_map; }

PressureManager* pressure_manager = nullptr;