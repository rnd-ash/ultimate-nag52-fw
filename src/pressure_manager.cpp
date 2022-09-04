#include "pressure_manager.h"
#include <tcm_maths.h>
#include "macros.h"
#include "solenoids/constant_current.h"
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

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->adapt_map = new AdaptationMap();
    this->req_tcc_pressure = 0;
    this->req_mpc_pressure = 0;
    this->req_mpc_pressure = 0;
    this->req_current_mpc = 0;
    this->req_current_spc = 0;

    /** Pressure PWM map **/

    int16_t pwm_x_headers[8] = {0, 50, 600, 1000, 2350, 5600, 6600, 7700};
    int16_t pwm_y_headers[4] = {-25, 20, 60, 150}; 

    pressure_pwm_map = new TcmMap(8, 4, pwm_x_headers, pwm_y_headers);

    if (!this->pressure_pwm_map->allocate_ok()) {
        this->pressure_pwm_map = nullptr;
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "PM", "Allocation of pressure pwm map failed!");
        return;
    }
    // Allocation OK, add the data to the map
    // Values are current (mA) for SPC and MPC solenoid
    int16_t pcs_current_map[8*4] = {
    /*               0    50    600  1000  2350  5600  6600  7700 <- mBar */
    /* -25C */     1300, 1100, 1085,  954,  700,  450,  350, 200,
    /*  20C */     1277, 1077,  925,  830,  675,  415,  320,   0,
    /*  60C */     1200, 1000,  835,  780,  650,  400,  288,   0,
    /* 150C */     1175,  975,  795,  745,  625,  370,  260,   0
    };
    
    pressure_pwm_map->add_data(pcs_current_map, 8*4);

    /** Pressure PWM map (TCC) **/

    int16_t pwm_tcc_x_headers[7] = {0, 2000, 4000, 5000, 7500, 10000, 15000};
    int16_t pwm_tcc_y_headers[5] = {-25, 20, 60, 150}; 

    tcc_pwm_map = new TcmMap(7, 5, pwm_tcc_x_headers, pwm_tcc_y_headers);

    if (!this->tcc_pwm_map->allocate_ok()) {
        this->tcc_pwm_map = nullptr;
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "PM", "Allocation of TCC pressure pwm map failed!");
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
    int16_t hold2_x_headers[5] = {1, 2, 3, 4, 5};
    int16_t hold2_y_headers[4] = {-20, 5, 25, 60}; 

    hold2_time_map = new TcmMap(5, 4, hold2_x_headers, hold2_y_headers);

    if (!this->hold2_time_map->allocate_ok()) {
        this->hold2_time_map = nullptr;
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "PM", "Allocation of pressure pwm map failed!");
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
    int16_t hold2_map[5*4] = {
    /* Clutch grp  K1    K2    K3    B1   B2 */
    /* -20C */     600, 1620,  860,  600, 820,
    /*   5C */     300,  600,  440,  380, 400,
    /*  25C */     180,  240,  180,  220, 180,
    /*  60C */     160,  140,  160,  180, 120
    };
    hold2_time_map->add_data(hold2_map, 5*4);
    this->gb_max_torque = max_torque;
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
            targ_map = working_norm_pressure_p;
            break;
    }

    int pedal_pos = ((int)sensor_data->pedal_pos*100/250);
    return locate_pressure_map_value(targ_map, pedal_pos);
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


    sd.hold1_data.ramp_time = 0;
    sd.hold1_data.hold_time = 50;
    sd.hold1_data.spc_pressure = 50;
    sd.hold1_data.mpc_pressure = (curr_mpc*1.5) + sd.hold1_data.spc_pressure;

    sd.hold2_data.ramp_time = 100;
    sd.hold2_data.hold_time = 100;
    sd.hold2_data.spc_pressure = 500;
    sd.hold2_data.mpc_pressure = (curr_mpc*1.2) + sd.hold2_data.spc_pressure;

    //sd.hold3_data.ramp_time = 60;
    //sd.hold3_data.hold_time = 100;
    //sd.hold3_data.spc_pressure = scale_number(abs(sensor_data->static_torque), 300, 650, 0, 330);//= 650;
    //sd.hold3_data.mpc_pressure = curr_mpc;

    // MPC MUST be higher than SPC for all the hold phases
    // to prevent an early shift. For now keep the offset to 200mBar
    // (MPC > SPC+200mBar)
    //sd.hold3_data.mpc_pressure = sd.hold3_data.spc_pressure + 200;
    this->make_hold3_data(&sd.hold3_data, &sd.hold2_data, chars, shift_request, curr_mpc);

    //sd.torque_data.ramp_time = scale_number(chars.shift_speed, 150, 0, 0, 10);
    //sd.torque_data.hold_time = sd.torque_data.ramp_time;
    //
    //sd.torque_data.spc_pressure = scale_number(abs(sensor_data->static_torque), 650, 900, 0, 330);
    //sd.torque_data.mpc_pressure = sd.torque_data.spc_pressure+150;
    this->make_torque_data(&sd.torque_data, &sd.hold3_data, chars, shift_request, curr_mpc);

    //sd.overlap_data.hold_time = 0; // No hold time
    //if (shift_request == ProfileGearChange::ONE_TWO) {
    //    sd.overlap_data.ramp_time += scale_number(sensor_data->static_torque, 20, 0, 100, 200);
    //}
    //sd.overlap_data.ramp_time = scale_number(sensor_data->static_torque, 1000, 150, 0, 200);
    //sd.overlap_data.spc_pressure = scale_number(abs(sensor_data->static_torque), 1200, 3500, 0, 330);
    //sd.overlap_data.mpc_pressure = (sd.overlap_data.spc_pressure / 4) * 3

    this->make_overlap_data(&sd.overlap_data, &sd.torque_data, chars, shift_request, curr_mpc);

    sd.max_pressure_data.ramp_time = 250;
    sd.max_pressure_data.hold_time = scale_number(sensor_data->atf_temp, 1500, 100, -20, 30);
    sd.max_pressure_data.spc_pressure = MIN(6000, sd.overlap_data.spc_pressure*2);
    sd.max_pressure_data.mpc_pressure = sd.overlap_data.mpc_pressure;

    // All hold times should be changed based on engine RPM (Assumed 2K RPM when selecting the pressures)
    //float multi = (float)sensor_data->input_rpm / 3000.0;
    //sd.max_pressure_data.hold_time = (float)sd.max_pressure_data.hold_time / multi;
    //sd.overlap_data.hold_time = (float)sd.max_pressure_data.hold_time * multi; // overlap ramp time is always 0
    //sd.torque_data.hold_time = (float)sd.torque_data.hold_time / multi;
    //sd.hold3_data.hold_time = (float)sd.hold3_data.hold_time / multi;
    //sd.hold2_data.hold_time = (float)sd.hold2_data.hold_time / multi;
    //sd.hold1_data.hold_time = (float)sd.hold1_data.hold_time / multi;

    // <=  0 -> 100% (1.0)
    // >= 10 -> 10% (0.0)
    float profile_td_amount = ((float)scale_number(chars.shift_speed, 10, 1, 1, 10) / 10.0);
    // <= 500 RPM - 0% profile_td_amount
    // >= 4500 RPM - 100% profile_td_amount
    //float rpm_td_amount = ((float)scale_number(sensor_data->input_rpm, 10, 0, 500, 4500) / 10.0);
    sd.torque_down_amount = profile_td_amount;
    sd.hold1_data.mpc_pressure = sd.torque_data.mpc_pressure;
    sd.hold2_data.mpc_pressure = sd.torque_data.mpc_pressure;
    sd.hold3_data.mpc_pressure = sd.torque_data.mpc_pressure;
    return sd;
}

void PressureManager::make_hold3_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->ramp_time = 100;
    dest->hold_time = 200;
    const AdaptationCell* cell = this->adapt_map->get_adapt_cell(sensor_data, change);
    dest->spc_pressure = 650;
    dest->hold_time += cell->fill_time_adder;
    dest->mpc_pressure = 650;
}

void PressureManager::make_torque_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    dest->hold_time = dest->ramp_time = scale_number(abs(sensor_data->static_torque), 100, 30, 0, this->gb_max_torque);
        /*
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
    switch (change) {
        case ProfileGearChange::ONE_TWO: // Prefilling K1
            dest->hold_time = hold2_time_map->get_value(1, this->sensor_data->atf_temp);
            dest->spc_pressure = 1200;
            break;
        case ProfileGearChange::FIVE_FOUR:
            dest->hold_time = hold2_time_map->get_value(1, this->sensor_data->atf_temp);
            dest->spc_pressure = 1500;
            break;
        case ProfileGearChange::TWO_ONE:
        case ProfileGearChange::FOUR_FIVE: // Prefilling B1
            dest->hold_time = hold2_time_map->get_value(4, this->sensor_data->atf_temp);
            dest->spc_pressure = 1300;
            break;
        case ProfileGearChange::TWO_THREE: // Prefilling K2
            dest->hold_time = hold2_time_map->get_value(2, this->sensor_data->atf_temp);
            dest->spc_pressure = 1400;
            break;
        case ProfileGearChange::THREE_TWO: // Prefilling K3
        case ProfileGearChange::THREE_FOUR:
            dest->hold_time = hold2_time_map->get_value(3, this->sensor_data->atf_temp);
            dest->spc_pressure = 1600;
            break;
        case ProfileGearChange::FOUR_THREE: // Prefilling B2
        default:
            dest->hold_time = hold2_time_map->get_value(5, this->sensor_data->atf_temp);
            dest->spc_pressure = 1400;
            break;
    }
    const AdaptationCell* cell = this->adapt_map->get_adapt_cell(sensor_data, change);
    dest->spc_pressure += cell->fill_pressure_mbar;
    dest->mpc_pressure = dest->spc_pressure+(curr_mpc/1.2);
}

void PressureManager::make_overlap_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc) {
    
    dest->ramp_time = scale_number(sensor_data->static_torque, 300, 100, gb_max_torque/3, gb_max_torque);
    //dest->hold_time = 0;
    if (change == ProfileGearChange::ONE_TWO) {
        dest->ramp_time += scale_number(sensor_data->static_torque, 20, 0, 100, 200);
    }

    dest->spc_pressure = prev->mpc_pressure + scale_number(sensor_data->static_torque, 50, 1000, 0, this->gb_max_torque);
    dest->mpc_pressure = prev->mpc_pressure;
    dest->ramp_time *= (float)scale_number(chars.shift_speed*10, 1200, 800, 0, 100)/1000.0;
    dest->hold_time = 250;
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