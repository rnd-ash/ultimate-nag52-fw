#include "pressure_manager.h"


float find_temp_multiplier(int temp_c) {
    int temp_raw = temp_c+50;
    if (temp_raw < 0) { return pressure_temp_normalizer[0]; }
    else if (temp_raw > 160) { return pressure_temp_normalizer[16]; }
    int min = temp_raw/10;
    int max = min+1;
    float dy = pressure_temp_normalizer[max] - pressure_temp_normalizer[min];
    float dx = (max-min)*10;
    return (pressure_temp_normalizer[min] + ((dy/dx)) * (temp_raw-(min*10)));
}

float find_temp_ramp_speed_multiplier(int temp_c) {
    int temp_raw = temp_c+50;
    if (temp_raw < 0) { return ramp_speed_temp_normalizer[0]; }
    else if (temp_raw > 160) { return ramp_speed_temp_normalizer[16]; }
    int min = temp_raw/10;
    int max = min+1;
    float dy = ramp_speed_temp_normalizer[max] - ramp_speed_temp_normalizer[min];
    float dx = (max-min)*10;
    return (ramp_speed_temp_normalizer[min] + ((dy/dx)) * (temp_raw-(min*10)));
}

float find_rpm_pressure_multiplier(int engine_rpm) {
    if (engine_rpm <= 0) { return rpm_normalizer[0]; }
    else if (engine_rpm > 8000) { return rpm_normalizer[8]; }
    int min = engine_rpm/1000;
    int max = min+1;
    float dy = rpm_normalizer[max] - rpm_normalizer[min];
    float dx = (max-min)*1000;
    return (rpm_normalizer[min] + ((dy/dx)) * (engine_rpm-(min*1000)));
}

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

uint16_t find_spc_pressure(const pressure_map map, SensorData* sensors) {
    int load = (sensors->pedal_pos*100/250);
    return locate_pressure_map_value(map, load) * find_temp_multiplier(sensors->atf_temp);
}

uint16_t find_mpc_pressure(const pressure_map map, SensorData* sensors) {
    // MPC reacts to Torque (Also sets pressure for SPC. Shift firmness can be increased)
    //int load = sensors->static_torque*100/MAX_TORQUE_RATING_NM;
    //if (load < 0) { load *= -0.25; } // Pulling engine
    int load = (sensors->pedal_pos*100/250); // Try same as SPC
    // For now forget about shift_firmness (TODO work on shift_firmness 1-10 to 1.1-0.9)
    return locate_pressure_map_value(map, load);
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g, SensorData* sensors) {
    int torque = sensors->static_torque;
    if (torque < 0) {
        torque *= -1;
    }
    int torque_load = (torque*100/MAX_TORQUE_RATING_NM);
    uint16_t raw = locate_pressure_map_value(working_norm_pressure, torque_load);
    uint16_t engine_rpm = sensors->engine_rpm;
    if (engine_rpm <= 0) { return rpm_normalizer[0]; }
    else if (engine_rpm > 8000) { return rpm_normalizer[8]; }
    int min = engine_rpm/1000;
    int max = min+1;
    float dy = rpm_working_normalizer[max] - rpm_working_normalizer[min];
    float dx = (max-min)*1000;
    return raw*(rpm_working_normalizer[min] + ((dy/dx)) * (engine_rpm-(min*1000)));
}

float PressureManager::get_tcc_temp_multiplier(int atf_temp) {
    return find_temp_multiplier(atf_temp);
}

ShiftData PressureManager::get_shift_data(SensorData* sensors, ProfileGearChange shift_request, ShiftCharacteristics chars) {
    ShiftData sd; 
    switch (shift_request) {
        case ProfileGearChange::ONE_TWO:
            sd.initial_spc_pwm = find_spc_pressure(spc_1_2, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_1_2, this->sensor_data);
            sd.targ_g = 2; sd.curr_g = 1;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = TC_1_2;
            sd.sip_threshold = SIP_1_2;
            break;
        case ProfileGearChange::TWO_THREE:
            sd.initial_spc_pwm = find_spc_pressure(spc_2_3, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_2_3, this->sensor_data);
            sd.targ_g = 3; sd.curr_g = 2;
            sd.shift_solenoid = sol_y5;
            sd.torque_cut_multiplier = TC_2_3;
            sd.sip_threshold = SIP_2_3;
            break;
        case ProfileGearChange::THREE_FOUR:
            sd.initial_spc_pwm = find_spc_pressure(spc_3_4, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_3_4, this->sensor_data);
            sd.targ_g = 4; sd.curr_g = 3;
            sd.shift_solenoid = sol_y4;
            sd.torque_cut_multiplier = TC_3_4;
            sd.sip_threshold = SIP_3_4;
            break;
        case ProfileGearChange::FOUR_FIVE:
            sd.initial_spc_pwm = find_spc_pressure(spc_4_5, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_4_5, this->sensor_data);
            sd.targ_g = 5; sd.curr_g = 4;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = TC_4_5;
            sd.sip_threshold = SIP_4_5;
            break;
        case ProfileGearChange::FIVE_FOUR:
            sd.initial_spc_pwm = find_spc_pressure(spc_5_4, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_5_4, this->sensor_data);
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = 0.9;
            sd.sip_threshold = SIP_5_4;
            break;
        case ProfileGearChange::FOUR_THREE:
            sd.initial_spc_pwm = find_spc_pressure(spc_4_3, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_4_3, this->sensor_data);
            sd.targ_g = 3; sd.curr_g = 4;
            sd.shift_solenoid = sol_y4;
            sd.torque_cut_multiplier = 0.9;
            sd.sip_threshold = SIP_4_3;
            break;
        case ProfileGearChange::THREE_TWO:
            sd.initial_spc_pwm = find_spc_pressure(spc_3_2, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_3_2, this->sensor_data);
            sd.targ_g = 2; sd.curr_g = 3;
            sd.shift_solenoid = sol_y5;
            sd.torque_cut_multiplier = 0.9;
            sd.sip_threshold = SIP_3_2;
            break;
        case ProfileGearChange::TWO_ONE:
            sd.initial_spc_pwm = find_spc_pressure(spc_2_1, this->sensor_data);
            sd.initial_mpc_pwm = find_mpc_pressure(mpc_2_1, this->sensor_data);
            sd.targ_g = 1; sd.curr_g = 2;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = 0.9;
            sd.sip_threshold = SIP_2_1;
            break;
    }
    sd.spc_dec_speed = (chars.shift_speed * find_temp_ramp_speed_multiplier(sensors->atf_temp));
    //if (this->adapt_map != nullptr) {
    //    sd.initial_spc_pwm += adapt_map->get_adaptation_offset(sensors, shift_request);
    //}
    if (sd.targ_g < sd.curr_g) {
        sd.spc_dec_speed *= 0.75; // Make downshifting a little smoother
    }
    sd.mpc_dec_speed = sd.spc_dec_speed/2.0; // For now
    return sd;
}

float get_tcc_temp_multiplier(int atf_temp) {
    return find_temp_multiplier(atf_temp);
}