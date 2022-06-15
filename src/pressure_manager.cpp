#include "pressure_manager.h"
#include <tcm_maths.h>

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
     * 1-2 prefill K1
     * 2-1 prefill B1
     * 
     * 2-3 prefill K2
     * 3-2 prefill K1 and K3
     * 
     * 3-4 prefill K1 and K3
     * 4-3 prefill B2
     * 
     * 4-5 prefill B1
     * 5-4 prefill K1
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
    int torque = sensors->static_torque;
    if (torque < 0) {
        torque *= -1;
    }

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
    uint8_t map_gear;
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
    sd.hold1_data.hold_time = 250;
    sd.hold1_data.pressure = 50;

    sd.hold2_data.ramp_time = 0; // Hold 2 has no ramp time
    sd.hold2_data.hold_time = hold2_time_map->get_value(sd.targ_g, sensors->atf_temp);
    sd.hold2_data.pressure = 200;

    sd.hold3_data.ramp_time = 60;
    sd.hold3_data.hold_time = 100;
    sd.hold3_data.pressure = 800;

    sd.torque_data.ramp_time = 400;
    sd.torque_data.hold_time = 500;
    sd.torque_data.pressure = 1500;

    sd.overlap_data.ramp_time = 200;
    if (shift_request == ProfileGearChange::ONE_TWO) {
        sd.overlap_data.hold_time = 20;
    } else {
        sd.overlap_data.hold_time = 0;
    }
    sd.overlap_data.pressure = 2500;

    sd.max_pressure_data.ramp_time = 300;
    sd.max_pressure_data.hold_time = 500;
    sd.max_pressure_data.pressure = 7000;

    return sd;
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