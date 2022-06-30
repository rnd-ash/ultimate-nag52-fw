#include "pressure_manager.h"
#include <tcm_maths.h>
#include "macros.h"

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

PressureManager::PressureManager(SensorData* sensor_ptr) {
    this->sensor_data = sensor_ptr;
    this->adapt_map = new AdaptationMap();
    this->req_tcc_pwm = 0;
    this->req_mpc_pressure = 0;
    this->req_mpc_pressure = 0;

    /** Pressure PWM map **/

    int16_t pwm_x_headers[8] = {0, 50, 600, 1000, 2350, 5600, 6600, 7700};
    int16_t pwm_y_headers[4] = {-25, 20, 60, 150}; 

    pressure_pwm_map = new TcmMap(8, 4, pwm_x_headers, pwm_y_headers);

    if (!this->pressure_pwm_map->allocate_ok()) {
        this->pressure_pwm_map = nullptr;
        ESP_LOGE("PM", "Allocation of pressure pwm map failed!");
        return;
    }
    // Allocation OK, add the data to the map

    // Values are PWM 12bit (0-1000) for SPC and MPC solenoid
    // This table takes into account resistance change based on ATF temp
    int16_t pwm_map[8*4] = {
    /*               0    50   600  1000  2350  5600  6600  7700 <- mBar */
    /* -25C */     2000, 1625, 1603, 1409, 1034,  665,  517, 295,
    /*  20C */     2100, 1935, 1662, 1662, 1213,  746,  575,   0,
    /*  60C */     2200, 2081, 1738, 1738, 1353,  833,  600,   0,
    /* 150C */     3000, 2653, 2163, 2163, 1701, 1007,  708,   0
    };
    pressure_pwm_map->add_data(pwm_map, 8*4);

    /** Pressure PWM map (TCC) **/

    int16_t pwm_tcc_x_headers[7] = {0, 2000, 4000, 5000, 7500, 10000, 15000};
    int16_t pwm_tcc_y_headers[5] = {-25, 20, 60, 150}; 

    tcc_pwm_map = new TcmMap(7, 5, pwm_tcc_x_headers, pwm_tcc_y_headers);

    if (!this->tcc_pwm_map->allocate_ok()) {
        this->tcc_pwm_map = nullptr;
        ESP_LOGE("PM", "Allocation of TCC pressure pwm map failed!");
        return;
    }
    // Allocation OK, add the data to the map
    // Values are PWM 12bit (0-1000) for TCC solenoid
    int16_t tc_pwm_map[7*5] = {
    /*               0   2000  4000  5000  7500  10000  15000   <- mBar */
    /*   0C */       0,   480,  960, 1280, 1920,  2560,  4096,
    /*  30C */       0,   560, 1040, 1280, 1920,  2560,  4096,
    /*  60C */       0,   640, 1120, 1280, 1920,  2560,  4096,
    /*  90C */       0,   640, 1120, 1280, 1920,  2560,  4096,
    /* 120C */       0,   640, 1120, 1280, 1920,  2560,  4096,
    };
    tcc_pwm_map->add_data(tc_pwm_map, 7*5);


    /** Pressure Hold 2 time map **/
    int16_t hold2_x_headers[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    int16_t hold2_y_headers[4] = {-20, 5, 25, 60}; 

    hold2_time_map = new TcmMap(8, 4, hold2_x_headers, hold2_y_headers);

    if (!this->hold2_time_map->allocate_ok()) {
        this->hold2_time_map = nullptr;
        ESP_LOGE("PM", "Allocation of pressure pwm map failed!");
        return;
    }
    // Allocation OK, add the data to the map

    // Values are in millsecond for SPC to hold at Hold2 phase
    // Derived from this table
    /*              K1    K2    K3    B1    B2 */
    /* -20C        600, 1620,  860,  600,  820, */
    /*   5C        300,  600,  440,  380,  400, */
    /*  25C        180,  240,  160,  220,  180, */
    /*  60C        160,  140,  140,  180,  120  */
    /**
     * 1 -         K3, B1, B2
     * 2 - K1,     K3,     B2 
     * 3 - K1, K2,         B2
     * 4 - K1. K2, K3 
     * 5 -     K2, K3, B1
     * 
     * 
        * 1-2 prefill K1 release B1
     * 2-1 prefill B1 release K1
     * 
     * 2-3 prefill K2 release K3
     * 3-2 prefill K3 release K2
     * 
     * 3-4 prefill K3 release B2
     * 4-3 prefill B2 release K3
     * 
     * 4-5 prefill B1 release K1
     * 5-4 prefill K1 release B1
     */
    int16_t hold2_map[8*4] = {
    /* Curr gear   1-2   2-3   3-4   4-5   5-4   4-3   3-2   2-1 */
    /* -20C */     600, 1620,  860,  600,  600,  820,  860,  600, 
    /*   5C */     300,  600,  440,  380,  300,  400,  440,  380,
    /*  25C */     180,  240,  180,  220,  180,  180,  180,  220,
    /*  60C */     160,  140,  160,  180,  160,  120,  160,  180
    };
    hold2_time_map->add_data(hold2_map, 8*4);
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g, SensorData* sensors, int max_rated_torque) {
    const int16_t* targ_map = working_norm_pressure_p;
    switch(curr_g) {
        case GearboxGear::First:
            targ_map = working_norm_pressure_1;
            break;
        case GearboxGear::Second:
            targ_map = working_norm_pressure_2;
            break;
        case GearboxGear::Third:
            targ_map = working_norm_pressure_3;
            break;
        case GearboxGear::Fourth:
            targ_map = working_norm_pressure_4;
            break;
        case GearboxGear::Fifth:
            targ_map = working_norm_pressure_5;
            break;
        case GearboxGear::Reverse_First:
        case GearboxGear::Reverse_Second:
            targ_map = working_norm_pressure_r;
            break;
        case GearboxGear::Park:
        case GearboxGear::Neutral:
        case GearboxGear::SignalNotAvaliable:
        default: // Already set
            break;
    }

    int pedal_pos = ((int)sensor_data->pedal_pos*100/250);
    return locate_pressure_map_value(targ_map, pedal_pos);
}

float PressureManager::get_tcc_temp_multiplier(int atf_temp) {
    return 1.0;
}



ShiftData PressureManager::get_shift_data(SensorData* sensors, ProfileGearChange shift_request, ShiftCharacteristics chars, int max_rated_torque, uint16_t curr_mpc) {
    ShiftData sd; 
    uint8_t map_gear = 1;
    CLAMP(chars.shift_speed, 1, 10);
    switch (shift_request) {
        case ProfileGearChange::ONE_TWO:
            sd.targ_g = 2; sd.curr_g = 1; map_gear = 1;
            sd.shift_solenoid = sol_y3;
            break;
        case ProfileGearChange::TWO_THREE:
            sd.targ_g = 3; sd.curr_g = 2; map_gear = 2;
            sd.shift_solenoid = sol_y5;
            break;
        case ProfileGearChange::THREE_FOUR:
            sd.targ_g = 4; sd.curr_g = 3; map_gear = 3;
            sd.shift_solenoid = sol_y4;
            break;
        case ProfileGearChange::FOUR_FIVE:
            sd.targ_g = 5; sd.curr_g = 4; map_gear = 4;
            sd.shift_solenoid = sol_y3;
            break;
        case ProfileGearChange::FIVE_FOUR:
            sd.targ_g = 4; sd.curr_g = 5; map_gear = 5;
            sd.shift_solenoid = sol_y3;
            break;
        case ProfileGearChange::FOUR_THREE:
            sd.targ_g = 3; sd.curr_g = 4; map_gear = 6;
            sd.shift_solenoid = sol_y4;
            break;
        case ProfileGearChange::THREE_TWO:
            sd.targ_g = 2; sd.curr_g = 3; map_gear = 7;
            sd.shift_solenoid = sol_y5;
            break;
        case ProfileGearChange::TWO_ONE:
            sd.targ_g = 1; sd.curr_g = 2; map_gear = 8;
            sd.shift_solenoid = sol_y3;
            break;
    }


    sd.hold1_data.ramp_time = 100;
    sd.hold1_data.hold_time = 0;
    sd.hold1_data.spc_pressure = 10;
    sd.hold1_data.mpc_pressure = curr_mpc;

    sd.hold2_data.ramp_time = 0; // Hold 2 has no ramp time
    sd.hold2_data.hold_time = hold2_time_map->get_value(map_gear, sensors->atf_temp);
    sd.hold2_data.spc_pressure = 200;
    sd.hold2_data.mpc_pressure = curr_mpc;

    //sd.hold3_data.ramp_time = 60;
    //sd.hold3_data.hold_time = 100;
    //sd.hold3_data.spc_pressure = scale_number(abs(sensor_data->static_torque), 300, 650, 0, 330);//= 650;
    //sd.hold3_data.mpc_pressure = curr_mpc;

    // MPC MUST be higher than SPC for all the hold phases
    // to prevent an early shift. For now keep the offset to 200mBar
    // (MPC > SPC+200mBar)
    //sd.hold3_data.mpc_pressure = sd.hold3_data.spc_pressure + 200;
    this->make_hold3_data(&sd.hold3_data, chars, shift_request, curr_mpc);

    //sd.torque_data.ramp_time = scale_number(chars.shift_speed, 150, 0, 0, 10);
    //sd.torque_data.hold_time = sd.torque_data.ramp_time;
    //
    //sd.torque_data.spc_pressure = scale_number(abs(sensor_data->static_torque), 650, 900, 0, 330);
    //sd.torque_data.mpc_pressure = sd.torque_data.spc_pressure+150;
    this->make_torque_data(&sd.torque_data, chars, shift_request, curr_mpc);

    //sd.overlap_data.hold_time = 0; // No hold time
    //if (shift_request == ProfileGearChange::ONE_TWO) {
    //    sd.overlap_data.ramp_time += scale_number(sensor_data->static_torque, 20, 0, 100, 200);
    //}
    //sd.overlap_data.ramp_time = scale_number(sensor_data->static_torque, 1000, 150, 0, 200);
    //sd.overlap_data.spc_pressure = scale_number(abs(sensor_data->static_torque), 1200, 3500, 0, 330);
    //sd.overlap_data.mpc_pressure = (sd.overlap_data.spc_pressure / 4) * 3;
    this->make_overlap_data(&sd.overlap_data, chars, shift_request, curr_mpc);

    sd.max_pressure_data.ramp_time = 200;
    sd.max_pressure_data.hold_time  = scale_number(sensor_data->atf_temp, 1500, 100, -20, 30);
    sd.max_pressure_data.spc_pressure = MAX(4000, curr_mpc*2);
    sd.max_pressure_data.mpc_pressure = curr_mpc;

    float profile_td_amount = (float)scale_number(chars.shift_speed, 0, 10, 10, 0) / 10.0;
    return sd;
}

void PressureManager::make_hold3_data(ShiftPhase* dest, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->ramp_time = 60;
    dest->hold_time = 100;
    dest->spc_pressure = scale_number(abs(sensor_data->static_torque), 300, 650, 0, 330);//= 650;
    dest->mpc_pressure = dest->spc_pressure + 200;
}

void PressureManager::make_torque_data(ShiftPhase* dest, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->ramp_time = 100;
    dest->hold_time = dest->ramp_time;
    dest->spc_pressure = scale_number(abs(sensor_data->static_torque), 650, 900, 0, 330);
    dest->mpc_pressure = dest->spc_pressure;
}

void PressureManager::make_overlap_data(ShiftPhase* dest, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->hold_time = 0; // No hold time
    if (change == ProfileGearChange::ONE_TWO) {
        dest->ramp_time += scale_number(sensor_data->static_torque, 20, 0, 100, 200);
    }
    dest->ramp_time = scale_number(sensor_data->static_torque, 1000, 100, 0, 200);
    dest->spc_pressure = scale_number(abs(sensor_data->static_torque), 1200, 3500, 0, 330);
    dest->spc_pressure = (float)(dest->spc_pressure) * (1.0 + ((float)chars.shift_speed/20.0));
    float multi = (float)scale_number(chars.shift_speed*10, 6, 9, 0, 100) / 10.0;
    dest->mpc_pressure = (float)dest->spc_pressure*multi;


}

uint16_t PressureManager::get_p_solenoid_pwm_duty(uint16_t request_mbar) {
    if (this->pressure_pwm_map == nullptr) {
        return 100; // 10% (Failsafe)
    }
    return this->pressure_pwm_map->get_value(request_mbar, this->sensor_data->atf_temp);
}




void PressureManager::set_target_mpc_pressure(uint16_t targ) {
    egs_can_hal->set_mpc_pressure(targ);
    this->req_mpc_pressure = targ;
    sol_mpc->write_pwm_12bit_with_voltage(this->get_p_solenoid_pwm_duty(this->req_mpc_pressure), this->sensor_data->voltage);
}

void PressureManager::set_target_spc_pressure(uint16_t targ) {
    this->spc_off = false;
    egs_can_hal->set_spc_pressure(targ);
    sol_spc->write_pwm_12bit_with_voltage(this->get_p_solenoid_pwm_duty(this->req_spc_pressure), this->sensor_data->voltage);
    this->req_spc_pressure = targ;
}

void PressureManager::disable_spc() {
    this->req_spc_pressure = 8000;
    egs_can_hal->set_spc_pressure(8000);
    this->spc_off = true;
}

void PressureManager::set_target_tcc_pwm(uint16_t targ) {
    this->req_tcc_pwm = targ;
}

void PressureManager::update(GearboxGear curr_gear, GearboxGear targ_gear) {
    sol_tcc->write_pwm_12bit_with_voltage(this->req_tcc_pwm, this->sensor_data->voltage);
    if (spc_off) {
        sol_spc->write_pwm_12_bit(0);
    } else {
        sol_spc->write_pwm_12bit_with_voltage(this->get_p_solenoid_pwm_duty(this->req_spc_pressure), this->sensor_data->voltage);
    }
    sol_mpc->write_pwm_12bit_with_voltage(this->get_p_solenoid_pwm_duty(this->req_mpc_pressure), this->sensor_data->voltage);
}