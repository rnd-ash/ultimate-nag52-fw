#include "pressure_manager.h"
#include <tcu_maths.h>
#include "solenoids/solenoids.h"
#include "maps.h"
#include "common_structs_ops.h"
#include "nvs/module_settings.h"
#include "nvs/device_mode.h"
#include "nvs/all_keys.h"

const int16_t friction_coefficient_0c = 185;
const int16_t friction_coefficient_80C = 140;

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->gb_max_torque = max_torque;

    // For loading maps
    const char* key_name;
    const int16_t* default_data;

    const uint8_t hydralic_idx = CAL_CURRENT_SETTINGS.hydralic_set == HydralicCalibration::HydralicSet1 ? 1 : 0;

    /** Pressure PWM map **/
    const int16_t* pwm_x_headers = HYDRALIC_PCS_MAP_X_HEADER[hydralic_idx];
    const int16_t pwm_y_headers[4] = {-25, 20, 60, 150};
    this->solenoid_max_pressure = pwm_x_headers[7];
    default_data = HYDRALIC_PCS_MAP[hydralic_idx];

    /** Clutch friction data map */
    // heaviest loaded clutch index lookup
    this->heaviest_loaded_clutch_idx_map = VEHICLE_CONFIG.is_large_nag ? STRONGEST_LOADED_CLUTCH_LARGE_NAG : STRONGEST_LOADED_CLUTCH_SMALL_NAG;

    // Friction lookup table
    this->clutch_friction_coefficient_map = CLUTCH_FRICTION_MAP[(uint8_t)CAL_CURRENT_SETTINGS.clutch_friction_set];
    this->clutch_spring_release_map = CLUTCH_RELEASE_SPRING_MAP[(uint8_t)CAL_CURRENT_SETTINGS.clutch_release_spring_set];


    // Set pointer to valve body settings
    if (CAL_CURRENT_SETTINGS.hydralic_set == HydralicCalibration::HydralicSet1) {
        this->valve_body_settings = &HYD_CURRENT_SETTINGS.type1;
    } else {
        this->valve_body_settings = &HYD_CURRENT_SETTINGS.type0;
    }
    this->pressure_pwm_map = new LookupMap(pwm_x_headers, 8, pwm_y_headers, 4, default_data, 8*4);

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

    // Init MPC and SPC req pressures
    this->target_shift_pressure = this->solenoid_max_pressure;
    this->target_modulating_pressure = this->solenoid_max_pressure;
    this->target_tcc_pressure = 0;
}

uint16_t PressureManager::calc_working_pressure(GearboxGear current_gear, uint16_t in_mpc, uint16_t in_spc) {
    float fac = valve_body_settings->multiplier_all_gears;
    // Only when not shifting and constantly in 1 or R1
    if ((current_gear == GearboxGear::First || current_gear == GearboxGear::Reverse_First) && (c_gear == 0 && t_gear == 0)) {
        fac = valve_body_settings->multiplier_in_1st_gear;
    }
    uint16_t regulator_pressure = in_mpc + valve_body_settings->lp_regulator_force_mbar;
    float k1_factor = 0;
    uint16_t p_adder = valve_body_settings->inlet_pressure_offset_mbar_other_gears;
    if (this->shift_circuit_flag == (uint8_t)ShiftCircuit::sc_1_2) {
        p_adder = valve_body_settings->inlet_pressure_offset_mbar_first_gear;
        k1_factor = valve_body_settings->k1_engaged_factor;
    }
    uint16_t extra_pressure = interpolate_float(
        sensor_data->engine_rpm, 
        0,
        p_adder,
        valve_body_settings->pressure_correction_pump_speed_min,
        valve_body_settings->pressure_correction_pump_speed_max,
        InterpType::Linear
    );
    float spc_reduction = in_spc * k1_factor;
    return (fac * regulator_pressure) + extra_pressure - spc_reduction;
}

uint16_t PressureManager::calc_input_pressure(uint16_t working_pressure) {
    return interpolate_float(
        (float)working_pressure,
        &valve_body_settings->working_pressure_compensation,
        InterpType::Linear
    );
}

float PressureManager::calc_inlet_factor(uint16_t inlet_pressure) {
    return 0.03 * ((float)valve_body_settings->working_pressure_compensation.new_max - (float)inlet_pressure) / 1000.0;
}

void PressureManager::update_pressures(GearboxGear current_gear) {
    // Ignore
    if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {

    } else {
        uint16_t spc_in = this->target_shift_pressure;
        uint16_t mpc_in = this->target_modulating_pressure;
        uint16_t mpcc_in = this->target_modulating_clutch_pressure;

        // Shift solenoid 1-2 active, reduce SPC and mpc(clutch release) influence
        if ((uint8_t)ShiftCircuit::sc_1_2 == this->shift_circuit_flag || (c_gear == 1 && t_gear == 2) || (c_gear == 2 && t_gear == 1)) {
            spc_in /= valve_body_settings->shift_circuit_factor_1_2;
            mpcc_in /= valve_body_settings->shift_circuit_factor_1_2;
        }
        mpc_in += mpcc_in;

        uint16_t wp = this->calc_working_pressure(current_gear, mpc_in, spc_in);
        uint16_t pump = this->calc_input_pressure(wp);

        this->calculated_inlet_pressure = pump;
        this->calculated_working_pressure = wp;

        float factor = this->calc_inlet_factor(pump);
        
        this->corrected_mpc_pressure = mpc_in + (factor * (mpc_in + 1000.0));

        if (spc_in > pump) {
            this->corrected_spc_pressure = this->solenoid_max_pressure;
        } else {
            this->corrected_spc_pressure = spc_in + (factor * (spc_in + 1000.0));
        }

        // Now actuate solenoids
        if (this->corrected_spc_pressure >= this->solenoid_max_pressure) {
            this->corrected_spc_pressure = this->solenoid_max_pressure;
            sol_spc->set_current_target(0);
        } else {
            sol_spc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_spc_pressure, sensor_data->atf_temp));
        }

        if (this->corrected_mpc_pressure >= this->solenoid_max_pressure) {
            this->corrected_mpc_pressure = this->solenoid_max_pressure;
            sol_mpc->set_current_target(0);
        } else {
            sol_mpc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_mpc_pressure, sensor_data->atf_temp));
        }
    }
}

uint8_t gear_to_idx_lookup(GearboxGear g) {
    uint8_t gear_idx = 0;
    switch(g) {
        case GearboxGear::First:
            gear_idx = 1;
            break;
        case GearboxGear::Second:
            gear_idx = 2;
            break;
        case GearboxGear::Third:
            gear_idx = 3;
            break;
        case GearboxGear::Fourth:
            gear_idx = 4;
            break;
        case GearboxGear::Fifth:
            gear_idx = 5;
            break;
        case GearboxGear::Reverse_First:
            gear_idx = 6;
            break;
        case GearboxGear::Reverse_Second:
            gear_idx = 7;
            break;
        case GearboxGear::Park:
        case GearboxGear::Neutral:
        case GearboxGear::SignalNotAvailable:
        default:
            gear_idx = 0;
            break;
    }
    return gear_idx;
}

uint16_t PressureManager::find_working_pressure_for_clutch(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm) {
    uint16_t ret = this->solenoid_max_pressure;
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float friction_coefficient = interpolate_float(
        sensor_data->atf_temp, 
        friction_coefficient_0c,
        friction_coefficient_80C,
        0,
        80,
        InterpType::Linear
    );
    float friction_val = this->clutch_friction_coefficient_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = (friction_val / friction_coefficient) * (float)abs_torque_nm;
    if (calc < this->valve_body_settings->minimum_mpc_pressure) {
        calc = this->valve_body_settings->minimum_mpc_pressure;
    } else if (calc > this->solenoid_max_pressure) {
        calc = this->solenoid_max_pressure;
    }
    ret = calc;
    return ret;
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g) {
    uint8_t gear_idx = gear_to_idx_lookup(curr_g);
    Clutch heaviest_loaded_clutch = (Clutch)heaviest_loaded_clutch_idx_map[gear_idx];
    return find_working_pressure_for_clutch(curr_g, heaviest_loaded_clutch, abs(sensor_data->input_torque));
}

void PressureManager::notify_shift_end() {
    this->c_gear = 0;
    this->t_gear = 0;
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
            sd.shift_circuit = ShiftCircuit::sc_4_5;
            break;
        case ProfileGearChange::FIVE_FOUR:
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_circuit = ShiftCircuit::sc_4_5;
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
    // Shift start notify for pm internal algo
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
        .hold_time = (uint16_t)interpolate_float(this->sensor_data->atf_temp, 1500, 200, -20, 30, InterpType::Linear),
        .ramp_time = 400,
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
    } else if (ShiftCircuit::sc_4_5 == ss) {
        manipulated = sol_y3;
    } else { // No shift circuit (placeholder)
        this->t_gear = 0;
        this->c_gear = 0;
        return;
    }
    // Firstly, check if new value is 0 (Close the solenoid!)
    if (!enable) {
        manipulated->off();
        this->shift_circuit_flag &= ~(uint8_t)ss; //
    } else { // Check if current value is 0, if so, write full PWM
        manipulated->on();
        this->shift_circuit_flag |= (uint8_t)ss; //
    }
}

void PressureManager::set_target_shift_clutch_pressure(uint16_t targ) {
    this->target_shift_pressure = targ;
}

void PressureManager::set_target_modulating_working_pressure(uint16_t targ) {
    this->target_modulating_pressure = targ;
}

void PressureManager::set_target_modulating_releasing_pressure(uint16_t targ) {
    this->target_modulating_clutch_pressure = targ;
}

void PressureManager::set_spc_p_max() {
    this->target_shift_pressure = this->solenoid_max_pressure;
}

void PressureManager::set_target_tcc_pressure(uint16_t targ) {
    if (targ > 3000) {
        targ = 3000;
    }
    this->target_tcc_pressure = targ;
    sol_tcc->set_duty(this->get_tcc_solenoid_pwm_duty(this->target_tcc_pressure));
}

uint16_t PressureManager::get_spring_pressure(Clutch c) {
    return this->clutch_spring_release_map[(uint8_t)c];
}

uint16_t PressureManager::get_calc_line_pressure(void) const {
    return this->calculated_working_pressure;
}

uint16_t PressureManager::get_calc_inlet_pressure(void) const {
    return this->calculated_inlet_pressure;
}

uint16_t PressureManager::get_input_shift_pressure(void) const {
    return this->target_shift_pressure;
}

uint16_t PressureManager::get_input_modulating_pressure(void) const {
    return this->target_modulating_pressure + this->target_modulating_clutch_pressure;
}

uint16_t PressureManager::get_corrected_spc_pressure(void) const {
        return this->corrected_spc_pressure;
}

uint16_t PressureManager::get_corrected_modulating_pressure(void) const {
    return this->corrected_mpc_pressure;
}

uint16_t PressureManager::get_targ_tcc_pressure(void) const {
    return this->target_tcc_pressure;
}

uint8_t PressureManager::get_active_shift_circuits(void) const {
    return this->shift_circuit_flag;
}

uint16_t PressureManager::get_max_solenoid_pressure() {
    return this->solenoid_max_pressure;
}

StoredMap* PressureManager::get_tcc_pwm_map() { return this->tcc_pwm_map; }
StoredMap* PressureManager::get_fill_time_map() { return this->hold2_time_map; }
StoredMap* PressureManager::get_fill_pressure_map() { return  this->hold2_pressure_map; }

PressureManager* pressure_manager = nullptr;