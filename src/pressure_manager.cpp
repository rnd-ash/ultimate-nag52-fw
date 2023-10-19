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
    this->gb_max_torque = max_torque;

    // For loading maps
    const char* key_name;
    const int16_t* default_data;

    /** Pressure PWM map **/
    const int16_t* pwm_x_headers = PRM_CURRENT_SETTINGS.hydralic_variant == HydralicVariant::Variant0 ? PCS_CURRENT_MAP_X_VARIANT0 : PCS_CURRENT_MAP_X_VARIANT1;
    const int16_t pwm_y_headers[4] = {-25, 20, 60, 150};
    this->solenoid_max_pressure = pwm_x_headers[7];
    default_data = PRM_CURRENT_SETTINGS.hydralic_variant == HydralicVariant::Variant0 ? PCS_CURRENT_MAP_VARIANT0 : PCS_CURRENT_MAP_VARIANT1;

    // Set pointer to valve body settings
    if (PRM_CURRENT_SETTINGS.hydralic_variant == HydralicVariant::Variant1) {
        this->valve_body_settings = &HYD_CURRENT_SETTINGS.type2;
    } else {
        this->valve_body_settings = &HYD_CURRENT_SETTINGS.type1;
    }
    this->pressure_pwm_map = new LookupMap(pwm_x_headers, 8, pwm_y_headers, 4, default_data, 8*4);

    /** Pressure PWM map (TCC) **/
    const int16_t pwm_tcc_x_headers[7] = {0, 400, 800, 1000, 1500, 2000, 3000};
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

void PressureManager::update_pressures(GearboxGear current_gear) {
    // Ignore
    if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {

    } else {
        uint16_t spc_in = this->target_shift_pressure;
        uint16_t mpc_in = this->target_modulating_pressure;

        uint16_t max_spc = this->solenoid_max_pressure;
        uint16_t max_mpc = this->solenoid_max_pressure + valve_body_settings->lp_regulator_force_mbar;

        uint16_t extra_pressure = 0;
        float k1_multi = 1.0;
        if (0 != this->shift_circuit_flag) { // A shift circuit is active
            // Add spring pressure if shift circuit is open
            //if (spc_in > valve_body_settings->shift_regulator_force_mbar) {
            //    spc_in -= valve_body_settings->shift_regulator_force_mbar;
            //} else {
            //    spc_in = 0; // Cannot achieve target pressure as spring is working on regulator
            //}
            
            if (this->shift_circuit_flag == (uint8_t)ShiftCircuit::sc_1_2) {

                spc_in /= valve_body_settings->shift_circuit_factor_1_2;
                extra_pressure = interpolate_int(
                    sensor_data->engine_rpm, 
                    0,
                    valve_body_settings->inlet_pressure_offset_mbar_first_gear,
                    valve_body_settings->pressure_correction_pump_speed_min,
                    valve_body_settings->pressure_correction_pump_speed_max
                );
                k1_multi = valve_body_settings->k1_engaged_factor;
            } 
        } else {
            extra_pressure = interpolate_int(
                sensor_data->engine_rpm, 
                0, 
                valve_body_settings->inlet_pressure_offset_mbar_other_gears,
                valve_body_settings->pressure_correction_pump_speed_min,
                valve_body_settings->pressure_correction_pump_speed_max
            );
        }

        float pressure_multiplier = valve_body_settings->multiplier_all_gears;
        if (current_gear == GearboxGear::First || current_gear == GearboxGear::Reverse_First) {
            pressure_multiplier = valve_body_settings->multiplier_in_1st_gear;
        }

        uint16_t shift_pressure_reduction = spc_in * k1_multi;

        mpc_in = pressure_multiplier * (mpc_in + valve_body_settings->lp_regulator_force_mbar);
        this->calc_working_pressure = mpc_in + extra_pressure - shift_pressure_reduction;

        // Calculate solenoid inlet pressure

        this->calc_inlet_pressure = interpolate_float(
            this->calc_working_pressure,
            &valve_body_settings->working_pressure_compensation,
            InterpType::Linear
        );

        float inlet_factor = (0.03 * (valve_body_settings->working_pressure_compensation.new_max - this->calc_inlet_pressure)) / 1000.0;
        // Constant 1000mBar
        this->corrected_mpc_pressure = mpc_in + (inlet_factor * (mpc_in + 1000));

        if (this->corrected_mpc_pressure < 500) { // Prevent reduction too far!
            this->corrected_mpc_pressure = 500;
        }

        if (spc_in < this->calc_inlet_pressure) {
            this->corrected_spc_pressure = spc_in + (inlet_factor * (spc_in + 1000));
        } else {
            this->corrected_spc_pressure = this->solenoid_max_pressure;
        }
        //printf("%.3f %d %d\n", inlet_factor, this->target_modulating_pressure, this->target_shift_pressure);

        // Now actuate solenoids
        if (this->corrected_spc_pressure >= this->solenoid_max_pressure) {
            sol_spc->set_current_target(0);
        } else {
            sol_spc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_spc_pressure, sensor_data->atf_temp));
        }

        if (this->corrected_mpc_pressure >= this->solenoid_max_pressure) {
            sol_mpc->set_current_target(0);
        } else {
            sol_mpc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_mpc_pressure, sensor_data->atf_temp));
        }
    }
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g) {
    if (this->mpc_working_pressure == nullptr) {
        return this->solenoid_max_pressure; // Failsafe!
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

    float trq_percent = (float)(abs(sensor_data->input_torque)*100.0)/(float)this->gb_max_torque;
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

void PressureManager::set_target_modulating_pressure(uint16_t targ) {
    this->target_modulating_pressure = targ;
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
    uint16_t spring_pressure = 1000;

    switch(c) {
        case Clutch::K1:
            spring_pressure = 1270;
            break;
        case Clutch::K2:
            spring_pressure = 846;
            break;
        case Clutch::K3:
            spring_pressure = VEHICLE_CONFIG.is_large_nag ? 1646 : 1205;
            break;
        case Clutch::B1:
            spring_pressure = 1139;
            break;
        case Clutch::B2:
            spring_pressure = 1289;
            break;
        default:
            break;
    }
    return spring_pressure;
}

uint16_t PressureManager::get_calc_line_pressure(void) const {
    return this->calc_working_pressure;
}

uint16_t PressureManager::get_calc_inlet_pressure(void) const {
    return this->calc_inlet_pressure;
}

uint16_t PressureManager::get_input_shift_pressure(void) const {
    return this->target_shift_pressure;
}

uint16_t PressureManager::get_input_modulating_pressure(void) const {
    return this->target_modulating_pressure;
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

StoredMap* PressureManager::get_tcc_pwm_map() { return this->tcc_pwm_map; }
StoredMap* PressureManager::get_working_map() { return this->mpc_working_pressure; }
StoredMap* PressureManager::get_fill_time_map() { return this->hold2_time_map; }
StoredMap* PressureManager::get_fill_pressure_map() { return  this->hold2_pressure_map; }

PressureManager* pressure_manager = nullptr;