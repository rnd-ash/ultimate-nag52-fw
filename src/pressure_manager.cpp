#include "pressure_manager.h"

/*
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
*/

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

uint16_t find_spc_pressure(const pressure_map map, SensorData* sensors, int max_rated_torque) {
    // MPC reacts to Torque (Also sets pressure for SPC. Shift firmness can be increased)
    int load = sensors->static_torque*100/max_rated_torque;
    if (load < 0) { load *= -1; } // Pulling engine
    //int load = (sensors->pedal_pos*100/250); // Try same as SPC
    // For now forget about shift_firmness (TODO work on shift_firmness 1-10 to 1.1-0.9)
    return locate_pressure_map_value(map, load);
    //int load = (sensors->pedal_pos*100/250);
    //return locate_pressure_map_value(map, load); //* find_temp_multiplier(sensors->atf_temp);
}

uint16_t find_mpc_pressure(const pressure_map map, SensorData* sensors, int max_rated_torque) {
    // MPC reacts to Torque (Also sets pressure for SPC. Shift firmness can be increased)
    int load = sensors->static_torque*100/max_rated_torque;
    if (load < 0) { load *= -1; } // Pulling engine
    //int load = (sensors->pedal_pos*100/250); // Try same as SPC
    // For now forget about shift_firmness (TODO work on shift_firmness 1-10 to 1.1-0.9)
    return locate_pressure_map_value(map, load);
}

#define COPY_MAP(name) \
    memcpy(this->map_data.name, large_nag ? &name##_large : &name##_small, sizeof(pressure_map));

PressureManager::PressureManager(SensorData* sensor_ptr) {
    this->sensor_data = sensor_ptr;
    this->adapt_map = new AdaptationMap();
    this->map_data = PressureMgrData{};
    bool large_nag = VEHICLE_CONFIG.is_large_nag;
    COPY_MAP(mpc_1_2)
    COPY_MAP(spc_1_2)
    COPY_MAP(mpc_2_3)
    COPY_MAP(spc_2_3)
    COPY_MAP(mpc_3_4)
    COPY_MAP(spc_3_4)
    COPY_MAP(mpc_4_5)
    COPY_MAP(spc_4_5)
    COPY_MAP(mpc_5_4)
    COPY_MAP(spc_5_4)
    COPY_MAP(mpc_4_3)
    COPY_MAP(spc_4_3)
    COPY_MAP(mpc_3_2)
    COPY_MAP(spc_3_2)
    COPY_MAP(mpc_2_1)
    COPY_MAP(spc_2_1)
    memcpy(this->map_data.ramp_speed_multiplier, rpm_normalizer, sizeof(rpm_modifier_map));
    memcpy(this->map_data.working_mpc_p, working_norm_pressure_p, sizeof(pressure_map));
    memcpy(this->map_data.working_mpc_r, working_norm_pressure_r, sizeof(pressure_map));
    memcpy(this->map_data.working_mpc_1, working_norm_pressure_1, sizeof(pressure_map));
    memcpy(this->map_data.working_mpc_2, working_norm_pressure_2, sizeof(pressure_map));
    memcpy(this->map_data.working_mpc_3, working_norm_pressure_3, sizeof(pressure_map));
    memcpy(this->map_data.working_mpc_4, working_norm_pressure_4, sizeof(pressure_map));
    memcpy(this->map_data.working_mpc_5, working_norm_pressure_5, sizeof(pressure_map));
    memset(&this->tcc, 0, sizeof(PSolenoidData));
    memset(&this->spc, 0, sizeof(PSolenoidData));
    memset(&this->mpc, 0, sizeof(PSolenoidData));
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g, SensorData* sensors, int max_rated_torque) {
    int torque = sensors->static_torque;
    if (torque < 0) {
        torque *= -1;
    }

    int16_t* targ_map = map_data.working_mpc_p;
    switch(curr_g) {
        case GearboxGear::First:
            targ_map = map_data.working_mpc_1;
            break;
        case GearboxGear::Second:
            targ_map = map_data.working_mpc_2;
            break;
        case GearboxGear::Third:
            targ_map = map_data.working_mpc_3;
            break;
        case GearboxGear::Fourth:
            targ_map = map_data.working_mpc_4;
            break;
        case GearboxGear::Fifth:
            targ_map = map_data.working_mpc_5;
            break;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            targ_map = map_data.working_mpc_r;
            break;
        case GearboxGear::Park:
        case GearboxGear::Neutral:
        case GearboxGear::SignalNotAvaliable:
        default: // Already set
            break;
    }

    int torque_load = (torque*100/max_rated_torque);
    uint16_t raw = locate_pressure_map_value(targ_map, torque_load);
    return raw;
}

float PressureManager::get_tcc_temp_multiplier(int atf_temp) {
    return 1.0/find_temp_ramp_speed_multiplier(atf_temp);
}

ShiftData PressureManager::get_shift_data(SensorData* sensors, ProfileGearChange shift_request, ShiftCharacteristics chars, int max_rated_torque) {
    ShiftData sd; 
    switch (shift_request) {
        case ProfileGearChange::ONE_TWO:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_1_2, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_1_2, this->sensor_data, max_rated_torque);
            sd.targ_g = 2; sd.curr_g = 1;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = 0.8;
            break;
        case ProfileGearChange::TWO_THREE:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_2_3, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_2_3, this->sensor_data, max_rated_torque);
            sd.targ_g = 3; sd.curr_g = 2;
            sd.shift_solenoid = sol_y5;
            sd.torque_cut_multiplier = 0.8;
            break;
        case ProfileGearChange::THREE_FOUR:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_3_4, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_3_4, this->sensor_data, max_rated_torque);
            sd.targ_g = 4; sd.curr_g = 3;
            sd.shift_solenoid = sol_y4;
            sd.torque_cut_multiplier = 0.8;
            break;
        case ProfileGearChange::FOUR_FIVE:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_4_5, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_4_5, this->sensor_data, max_rated_torque);
            sd.targ_g = 5; sd.curr_g = 4;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = 0.8;
            break;
        case ProfileGearChange::FIVE_FOUR:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_5_4, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_5_4, this->sensor_data, max_rated_torque);
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = 0.9;
            break;
        case ProfileGearChange::FOUR_THREE:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_4_3, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_4_3, this->sensor_data, max_rated_torque);
            sd.targ_g = 3; sd.curr_g = 4;
            sd.shift_solenoid = sol_y4;
            sd.torque_cut_multiplier = 0.9;
            break;
        case ProfileGearChange::THREE_TWO:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_3_2, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_3_2, this->sensor_data, max_rated_torque);
            sd.targ_g = 2; sd.curr_g = 3;
            sd.shift_solenoid = sol_y5;
            sd.torque_cut_multiplier = 0.9;
            break;
        case ProfileGearChange::TWO_ONE:
            sd.initial_spc_pwm = find_spc_pressure(map_data.spc_2_1, this->sensor_data, max_rated_torque);
            sd.initial_mpc_pwm = find_mpc_pressure(map_data.mpc_2_1, this->sensor_data, max_rated_torque);
            sd.targ_g = 1; sd.curr_g = 2;
            sd.shift_solenoid = sol_y3;
            sd.torque_cut_multiplier = 0.9;
            break;
    }
    sd.spc_dec_speed = chars.shift_speed;// * find_temp_ramp_speed_multiplier(sensors->atf_temp));
    //if (this->adapt_map != nullptr) {
    //    sd.initial_spc_pwm += adapt_map->get_adaptation_offset(sensors, shift_request);
    //}
    sd.temp_multi =  find_temp_ramp_speed_multiplier(sensors->atf_temp);
    return sd;
}


void PressureManager::set_target_mpc_percent(uint16_t targ, int speed) {
    if (targ == this->mpc.curr_pwm_percent) {
        this->mpc.write_pwm = false;
        return;
    }
    this->mpc.write_pwm = true;
    if (speed < 0) {
        this->mpc.curr_pwm_percent = targ;
        this->mpc.targ_pwm_percent = targ;
        this->mpc.ramp_speed = 0;
    } else {
        this->mpc.targ_pwm_percent = targ;
        this->mpc.ramp_speed = speed;
    }
}

void PressureManager::set_target_spc_percent(uint16_t targ, int speed) {
    if (targ == this->spc.curr_pwm_percent) {
        this->mpc.write_pwm = false;
        return;
    }
    this->spc.write_pwm = true;
    if (speed < 0) {
        this->spc.curr_pwm_percent = targ;
        this->spc.targ_pwm_percent = targ;
        this->spc.ramp_speed = 0;
    } else {
        this->spc.targ_pwm_percent = targ;
        this->spc.ramp_speed = speed;
    }
}

void PressureManager::set_target_tcc_percent(uint16_t targ, int speed) {
    if (targ == this->tcc.curr_pwm_percent) {
        this->mpc.write_pwm = false;
        return;
    }
    this->tcc.write_pwm = true;
    if (speed < 0) {
        this->tcc.curr_pwm_percent = targ;
        this->tcc.targ_pwm_percent = targ;
        this->tcc.ramp_speed = 0;
    } else {
        this->tcc.targ_pwm_percent = targ;
        this->tcc.ramp_speed = speed;
    }
}

void PressureManager::set_spc_compensation_factor(float amount) {
    if (amount < 0) {
        amount = 0;
    }
    this->spc_compensation_amount = amount;
}

void PressureManager::update(GearboxGear curr_gear, GearboxGear targ_gear) {

    if (this->spc.write_pwm) {
        if (abs(this->spc.curr_pwm_percent - this->spc.targ_pwm_percent) < this->spc.ramp_speed) {
            this->spc.curr_pwm_percent = this->spc.targ_pwm_percent;
            this->spc.ramp_speed = 0;
            this->spc.write_pwm = false;
        } else {
            if (this->spc.curr_pwm_percent > this->spc.targ_pwm_percent) {
                this->spc.curr_pwm_percent -= this->spc.ramp_speed;
            } else {
                this->spc.curr_pwm_percent += this->spc.ramp_speed;
            }
        }
    }
    if (this->tcc.write_pwm) {
        if (abs(this->tcc.curr_pwm_percent - this->tcc.targ_pwm_percent) < this->tcc.ramp_speed) {
            this->tcc.curr_pwm_percent = this->tcc.targ_pwm_percent;
            this->tcc.write_pwm = false;
            this->tcc.ramp_speed = 0;
        } else {
            if (this->tcc.curr_pwm_percent > this->tcc.targ_pwm_percent) {
                this->tcc.curr_pwm_percent -= this->tcc.ramp_speed;
            } else {
                this->tcc.curr_pwm_percent += this->tcc.ramp_speed;
            }
        }
    }
    if (this->mpc.write_pwm) {
        if (abs(this->mpc.curr_pwm_percent - this->mpc.targ_pwm_percent) < this->mpc.ramp_speed) {
            this->mpc.curr_pwm_percent = this->mpc.targ_pwm_percent;
            this->mpc.write_pwm = false;
            this->mpc.ramp_speed = 0;
        } else {
            if (this->mpc.curr_pwm_percent > this->mpc.targ_pwm_percent) {
                this->mpc.curr_pwm_percent -= this->mpc.ramp_speed;
            } else {
                this->mpc.curr_pwm_percent += this->mpc.ramp_speed;
            }
        }
    }
    float spc_compensation = ((float)(this->spc.curr_pwm_percent/4.0)) * this->spc_compensation_amount;
    if (spc_compensation > this->mpc.curr_pwm_percent) {
        spc_compensation = this->mpc.curr_pwm_percent;
    }
    // Always write MPC as it regulates everyone else
    sol_spc->write_pwm_percent_with_voltage(this->spc.curr_pwm_percent, this->sensor_data->voltage);
    sol_mpc->write_pwm_percent_with_voltage(this->mpc.curr_pwm_percent-spc_compensation, this->sensor_data->voltage);
    sol_tcc->write_pwm_percent_with_voltage(this->tcc.curr_pwm_percent, this->sensor_data->voltage);
}