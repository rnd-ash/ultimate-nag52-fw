#include "pressure_manager.h"
#include <tcu_maths.h>
#include "solenoids/solenoids.h"
#include "maps.h"
#include "common_structs_ops.h"
#include "nvs/module_settings.h"
#include "nvs/device_mode.h"
#include "nvs/all_keys.h"
#include "egs_calibration/calibration_structs.h"

const int16_t friction_coefficient_0c = 185;
const int16_t friction_coefficient_80C = 140;

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->gb_max_torque = max_torque;

    // For loading maps
    const char* key_name;
    const int16_t* default_data;

    /** Pressure PWM map **/

    // Friction lookup table
    this->pressure_pwm_map = new LookupRefMap((int16_t*)HYDR_PTR->pcs_map_x, 7, (int16_t*)HYDR_PTR->pcs_map_y, 4, (int16_t*)HYDR_PTR->pcs_map_z, 7*4);

    this->momentum_upshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_1_2_x, 3, SHIFT_ALGO_CFG_PTR->momentum_1_2_y, 2, SHIFT_ALGO_CFG_PTR->momentum_1_2_z, 3*2);
    this->momentum_upshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_2_3_x, 3, SHIFT_ALGO_CFG_PTR->momentum_2_3_y, 2, SHIFT_ALGO_CFG_PTR->momentum_2_3_z, 3*2);
    this->momentum_upshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_3_4_x, 3, SHIFT_ALGO_CFG_PTR->momentum_3_4_y, 2, SHIFT_ALGO_CFG_PTR->momentum_3_4_z, 3*2);
    this->momentum_upshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_4_5_x, 3, SHIFT_ALGO_CFG_PTR->momentum_4_5_y, 2, SHIFT_ALGO_CFG_PTR->momentum_4_5_z, 3*2);
    for (auto i = 0; i < 4; i++) {
        if (!this->momentum_upshifts[i]->is_allocated()) {
            ESP_LOGE("PM", "Momentum upshift map %d failed to allocate!", i);
            delete this->momentum_upshifts[i];
        }
    }

    this->momentum_downshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_2_1_x, 6, SHIFT_ALGO_CFG_PTR->momentum_2_1_y, 10, SHIFT_ALGO_CFG_PTR->momentum_2_1_z, 6*10);
    this->momentum_downshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_3_2_x, 6, SHIFT_ALGO_CFG_PTR->momentum_3_2_y, 10, SHIFT_ALGO_CFG_PTR->momentum_3_2_z, 6*10);
    this->momentum_downshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_4_3_x, 6, SHIFT_ALGO_CFG_PTR->momentum_4_3_y, 10, SHIFT_ALGO_CFG_PTR->momentum_4_3_z, 6*10);
    this->momentum_downshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_5_4_x, 6, SHIFT_ALGO_CFG_PTR->momentum_5_4_y, 10, SHIFT_ALGO_CFG_PTR->momentum_5_4_z, 6*10);
    for (auto i = 0; i < 4; i++) {
        if (!this->momentum_downshifts[i]->is_allocated()) {
            ESP_LOGE("PM", "Momentum downshift map %d failed to allocate!", i);
            delete this->momentum_downshifts[i];
        }
    }

    this->torque_adder_upshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_1_2_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_1_2_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_1_2_z, 8*6);
    this->torque_adder_upshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_2_3_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_2_3_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_2_3_z, 8*6);
    this->torque_adder_upshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_3_4_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_3_4_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_3_4_z, 8*6);
    this->torque_adder_upshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_4_5_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_4_5_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_4_5_z, 8*6);
    for (auto i = 0; i < 4; i++) {
        if (!this->torque_adder_upshifts[i]->is_allocated()) {
            ESP_LOGE("PM", "Torque adder upshift map %d failed to allocate!", i);
            delete this->torque_adder_upshifts[i];
        }
    }

    this->torque_adder_downshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_2_1_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_2_1_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_2_1_z, 3*4);
    this->torque_adder_downshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_3_2_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_3_2_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_3_2_z, 3*4);
    this->torque_adder_downshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_4_3_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_4_3_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_4_3_z, 3*4);
    this->torque_adder_downshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_5_4_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_5_4_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_5_4_z, 3*4);
    for (auto i = 0; i < 4; i++) {
        if (!this->torque_adder_downshifts[i]->is_allocated()) {
            ESP_LOGE("PM", "Torque adder doownshift map %d failed to allocate!", i);
            delete this->torque_adder_downshifts[i];
        }
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

    /** Pressure fill time map **/
    const int16_t fill_t_x_headers[4] = {-20, 5, 25, 60};
    const int16_t fill_t_y_headers[5] = {1,2,3,4,5}; 
    if (MECH_PTR->gb_ty == 0) { // Large
        key_name = NVS_KEY_MAP_NAME_FILL_TIME_LARGE;
        default_data = LARGE_NAG_FILL_TIME_MAP;
    } else { // Small
        key_name = NVS_KEY_MAP_NAME_FILL_TIME_SMALL;
        default_data = SMALL_NAG_FILL_TIME_MAP;
    }
    fill_time_map = new StoredMap(key_name, FILL_TIME_MAP_SIZE, fill_t_x_headers, fill_t_y_headers, 4, 5, default_data);
    if (this->fill_time_map->init_status() != ESP_OK) {
        delete[] this->fill_time_map;
    }

    /** Pressure fill pressure map **/
    const int16_t fill_p_x_headers[1] = {1};
    const int16_t fill_p_y_headers[6] = {1,2,3,4,5,6};
    key_name = NVS_KEY_MAP_NAME_FILL_PRESSURE;
    default_data = NAG_FILL_PRESSURE_MAP;
    fill_pressure_map = new StoredMap(key_name, FILL_PRESSURE_MAP_SIZE, fill_p_x_headers, fill_p_y_headers, 1, 6, default_data);
    if (this->fill_pressure_map->init_status() != ESP_OK) {
        delete[] this->fill_pressure_map;
    }

    /** Pressure fill pressure map **/
    const int16_t fill_lp_x_headers[1] = {1};
    const int16_t fill_lp_y_headers[5] = {1,2,3,4,5};
    key_name = NVS_KEY_MAP_NAME_FILL_LOW_PRESSURE;
    default_data = NAG_FILL_LOW_PRESSURE_MAP;
    fill_low_pressure_map = new StoredMap(key_name, LOW_FILL_PRESSURE_MAP_SIZE, fill_lp_x_headers, fill_lp_y_headers, 1, 5, default_data);
    if (this->fill_low_pressure_map->init_status() != ESP_OK) {
        delete[] this->fill_low_pressure_map;
    }

    // Init MPC and SPC req pressures
    this->target_shift_pressure = this->get_max_solenoid_pressure();
    this->target_modulating_pressure = this->get_max_solenoid_pressure();
    this->target_tcc_pressure = 0;
}

uint16_t PressureManager::get_shift_regulator_pressure(void) {
    return HYDR_PTR->shift_reg_spring_pressure;
}

/*

MERCEDES-BENZS PRESSURE NAMES (WIS)

p-A - Working pressure
p-RV - Regulating valve pressure
p-SV - Shift valve pressure
p-S/RMV - Shift pressure control solenoid valve pressure (Solenoid output)
p-S/KUB - TCC solenoid output
p-Mod - Modulating pressure (Solenoid output)
p-U - Overlap pressure
p-KUB - TCC lockup clutch working pressure
p-Sm - Lubrication pressure

Solenoids:             In     Out
Shift solenoid(s):    p-SV -> p-SV
Modulating pressure:  p-RV -> p-Mod
Shift pressure:       p-RV -> p-S/RMV
TCC pressure:         p-SV -> p-S/KUB

Correlation:
p-A is directly controlled by p-Rv (Linear correlation)
p-U is derrived by p-S and p-Mod, which controls p-A
p-KUB is derrived from p-S/KUB and p-A
p-SV is created by Shift valve pressure regulator, it converts a variable p-RV into constant p-SV
*/
void PressureManager::update_pressures(GearboxGear current_gear, GearChange change_state) {
    // Ignore
    if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {

    } else {
        // EGS Func 0da5f6 (MB Layer)
        // This is my best guess at interpreting the assembly (Decompiler view messes a lot up with this function due to indirections)
        uint16_t mpc_in = this->target_modulating_pressure;
        uint16_t spc_in = this->target_shift_pressure;

        float factor;
        uint16_t extra_p = 0;
        if (GearChange::_IDLE == change_state) { // Not shifting
            // Not shifting
            if (GearboxGear::First == current_gear || GearboxGear::Reverse_First == current_gear) {
                factor = (float)HYDR_PTR->p_multi_1 / 1000.0;
            } else {
                factor = (float)HYDR_PTR->p_multi_other / 1000.0;
            }
        } else {
            // Shifting
            uint16_t extra_p_interp_max;
            if (GearChange::_1_2 == change_state || GearChange::_2_1 == change_state) {
                factor = (float)HYDR_PTR->p_multi_1 / 1000.0;
                extra_p_interp_max = HYDR_PTR->extra_pressure_adder_r1_1;
            } else {
                factor = (float)HYDR_PTR->p_multi_other / 1000.0;
                extra_p_interp_max = HYDR_PTR->extra_pressure_adder_other_gears;
            }
            extra_p = interpolate_float(sensor_data->engine_rpm, 0, extra_p_interp_max, HYDR_PTR->extra_pressure_pump_speed_min, HYDR_PTR->extra_pressure_pump_speed_max, InterpType::Linear);
        }

        float line_pressure = HYDR_PTR->lp_reg_spring_pressure + mpc_in;
        
        float wp = extra_p + ((float)line_pressure / factor);
        this->calculated_working_pressure = wp;

        float interpolated = interpolate_float(
            wp,
            HYDR_PTR->inlet_pressure_output_min,
            HYDR_PTR->inlet_pressure_output_max,
            HYDR_PTR->inlet_pressure_input_min,
            HYDR_PTR->inlet_pressure_input_max,
            InterpType::Linear
        );
        this->calculated_inlet_pressure = interpolated;

        float inlet_factor = (((float)HYDR_PTR->shift_pressure_addr_percent/1000.0) * ((float)HYDR_PTR->inlet_pressure_output_max - (float)interpolated));
        inlet_factor /= 1000.0;

        // -- Correct solenoids --

        // MPC solenoid is not affected by limits
        this->corrected_mpc_pressure = mpc_in + (inlet_factor * (float)(mpc_in + HYDR_PTR->inlet_pressure_offset));

        if (spc_in < interpolated) {
            this->corrected_spc_pressure = spc_in + (inlet_factor * (float)(spc_in + HYDR_PTR->inlet_pressure_offset));
        } else {
            this->corrected_spc_pressure = this->get_max_solenoid_pressure();
        }

        // -- Set solenoid currents --
        if (this->corrected_spc_pressure >= get_max_solenoid_pressure()) {
            sol_spc->set_current_target(0);
        } else {
            sol_spc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_spc_pressure, sensor_data->atf_temp+50.0));
        }
        sol_mpc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_mpc_pressure, sensor_data->atf_temp+50.0));
        sol_tcc->set_duty(this->get_tcc_solenoid_pwm_duty(this->target_tcc_pressure));
    }
}

float PressureManager::calculate_centrifugal_force_for_clutch(Clutch clutch, uint16_t input, uint16_t rear_sun) {
    float speed = 0;
    uint8_t sel_idx = 0xFF;
    float ret = 0;
    switch (clutch) {
        // OBSERVE. K1 is missing from this list.
        // on EGS52, it is listed as 0 for the factor table. Perhaps during
        // testing, they found calculating this force for K1 created some issues?
        case Clutch::K2:
            sel_idx = 1;
            speed = input;
            break;
        case Clutch::K3:
            sel_idx = 2;
            speed = rear_sun;
            break;
        default:
            break;
    }
    if (sel_idx != 0xFF) {
        float clutch_factor = MECH_PTR->atf_density_centrifugal_force_factor[sel_idx];
        if (clutch_factor != 0) {
            float density_now = MECH_PTR->atf_density_minus_50c - ((sensor_data->atf_temp + 50) * ((float)(MECH_PTR->atf_density_drop_per_c) / 100.0));
            ret = density_now * ((speed * speed) / clutch_factor);
            ret /= 10000.0; // To convert to mbar
        }
    }
    return ret;
}

uint16_t PressureManager::p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, CoefficientTy coef_ty) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coef = 1.F;
    switch (coef_ty) {
        case CoefficientTy::Static:
            coef = this->stationary_coefficient();
            break;
        case CoefficientTy::Sliding:
            coef = this->sliding_coefficient();
            break;
        case CoefficientTy::Release:
            coef = this->release_coefficient();
            break;
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = (friction_val / coef) * (float)abs_torque_nm;
    return calc;
}

int16_t PressureManager::p_clutch_with_coef_signed(GearboxGear gear, Clutch clutch, int16_t abs_torque_nm, CoefficientTy coef_ty) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coef;
    switch (coef_ty) {
        case CoefficientTy::Static:
            coef = this->stationary_coefficient();
            break;
        case CoefficientTy::Sliding:
            coef = this->sliding_coefficient();
            break;
        case CoefficientTy::Release:
            coef = this->release_coefficient();
            break;
        default:
            coef = 1.F;
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = (friction_val / coef) * (float)abs_torque_nm;
    return calc;
}

// Clutches that are held (Not moving) during shifts
const Clutch HOLDING_CLUTCHES[8][2] = {
    {Clutch::B2, Clutch::K3}, // 1-2 (B2 + K3)
    {Clutch::B2, Clutch::K1}, // 2-3 (B2 + K1)
    {Clutch::K1, Clutch::K2}, // 3-4 (K1 + K2)
    {Clutch::K2, Clutch::K3}, // 4-5 (K2 + K3)

    {Clutch::B2, Clutch::K3}, // 2-1 (B2 + K3)
    {Clutch::B2, Clutch::K1}, // 3-2 (B2 + K1)
    {Clutch::K1, Clutch::K2}, // 4-3 (K1 + K2)
    {Clutch::K2, Clutch::K3}, // 5-4 (K2 + K3)
};

uint16_t PressureManager::find_pressure_holding_other_clutches_in_change(GearChange change, GearboxGear current_g, uint16_t abs_torque_nm) {
    const uint8_t idx = fwd_gearchange_egs_map_lookup_idx(change);
    const Clutch* lookup = HOLDING_CLUTCHES[idx];
    uint16_t max = 0;
    for(int i = 0; i < 2; i++) {
        uint16_t calc = p_clutch_with_coef(current_g, lookup[i], abs_torque_nm, CoefficientTy::Static);
        if (calc > max) {
            max = calc;
        }
    }
    return max;
}

float PressureManager::sliding_coefficient() const {
    return interpolate_float(
        sensor_data->atf_temp, 
        friction_coefficient_0c,
        friction_coefficient_80C,
        0,
        80,
        InterpType::Linear
    );
}

float PressureManager::release_coefficient() const {
    return 120.0;
}

float PressureManager::stationary_coefficient() const {
    return 100.0;
}

uint16_t PressureManager::find_decent_adder_torque(GearChange change, uint16_t abs_motor_torque, uint16_t output_rpm) {
    LookupByteMap* map = nullptr;
    switch(change) {
        case GearChange::_1_2:
            map = this->torque_adder_upshifts[0];
            break;
        case GearChange::_2_3:
            map = this->torque_adder_upshifts[1];
            break;
        case GearChange::_3_4:
            map = this->torque_adder_upshifts[2];
            break;
        case GearChange::_4_5:
            map = this->torque_adder_upshifts[3];
            break;
        case GearChange::_2_1:
            map = this->torque_adder_downshifts[0];
            break;
        case GearChange::_3_2:
            map = this->torque_adder_downshifts[1];
            break;
        case GearChange::_4_3:
            map = this->torque_adder_downshifts[2];
            break;
        case GearChange::_5_4:
            map = this->torque_adder_downshifts[3];
            break;
        default:
            break;
    }
    if (nullptr == map) {
        return 0;
    } else {
        uint16_t ret = map->get_value((float)output_rpm/30.0, (float)abs_motor_torque/5.0); 
        return ret;
    }
}

uint16_t PressureManager::find_freeing_torque(GearChange change, uint16_t motor_torque, uint16_t output_rpm) {
    LookupByteMap* map = nullptr;
    switch(change) {
        case GearChange::_1_2:
            map = this->momentum_upshifts[0];
            break;
        case GearChange::_2_3:
            map = this->momentum_upshifts[1];
            break;
        case GearChange::_3_4:
            map = this->momentum_upshifts[2];
            break;
        case GearChange::_4_5:
            map = this->momentum_upshifts[3];
            break;
        case GearChange::_2_1:
            map = this->momentum_downshifts[0];
            break;
        case GearChange::_3_2:
            map = this->momentum_downshifts[1];
            break;
        case GearChange::_4_3:
            map = this->momentum_downshifts[2];
            break;
        case GearChange::_5_4:
            map = this->momentum_downshifts[3];
            break;
        default:
            break;
    }
    if (nullptr == map) {
        return 0;
    } else {
        uint16_t ret = map->get_value((float)output_rpm/30.0, (float)motor_torque/5.0); 
        return ret*5;
    }
}

uint16_t PressureManager::find_turbine_drag(uint8_t map_idx) {
    if (map_idx > 7) {
        return 1;
    } else {
        return MECH_PTR->turbine_drag[map_idx];
    }
}

uint16_t PressureManager::calc_max_torque_for_clutch(GearboxGear gear, Clutch clutch, uint16_t pressure, CoefficientTy coef_val) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coef = 1.F;
    switch (coef_val) {
        case CoefficientTy::Static:
            coef = this->stationary_coefficient();
            break;
        case CoefficientTy::Sliding:
            coef = this->sliding_coefficient();
            break;
        case CoefficientTy::Release:
            coef = this->release_coefficient();
            break;
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc =  ((float)pressure * coef) / (float)friction_val;
    return calc;
}

// FUNC_0d8092 (MB Layer EGS)
uint16_t PressureManager::correct_shift_shift_pressure(uint8_t shift_idx, uint32_t pressure) {
    // TODO - Move max_p to global constant so it can be referred in other functions
    uint16_t max_p = this->get_max_shift_pressure(shift_idx);
    // Bypass EEPROM adaptation offsets for shift circuits
    // Bypass offset for Mclaren
    if (pressure > max_p) {
        pressure = max_p;
    }
    // P*1000 as shift_spc_gain is *1000
    return (uint16_t)(((pressure*1000) / HYDR_PTR->shift_spc_gain[shift_idx]) + HYDR_PTR->shift_reg_spring_pressure);
}

uint16_t PressureManager::get_max_shift_pressure(uint8_t shift_idx) {
    uint32_t max_p = (this->get_max_solenoid_pressure() - HYDR_PTR->shift_reg_spring_pressure) * HYDR_PTR->shift_spc_gain[shift_idx];
    max_p /= 1000; // shift_spc_gain is *1000;
    return max_p;
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g) {
    uint8_t gear_idx = gear_to_idx_lookup(curr_g);
    uint16_t output = 0;
    uint8_t clutch_idx = MECH_PTR->strongest_loaded_clutch_idx[gear_idx];
    if (gear_idx == 0 || clutch_idx >= 6) {
        // N,P,SNV
        output = 0;
    } else {   
        float ret = p_clutch_with_coef(curr_g, (Clutch)clutch_idx, abs(sensor_data->input_torque), CoefficientTy::Static);
        ret += MECH_PTR->release_spring_pressure[clutch_idx];
        if (curr_g == GearboxGear::First || curr_g == GearboxGear::Reverse_First) {
            ret *= (HYDR_PTR->p_multi_1 / 1000.0);
        } else {
            ret *= (HYDR_PTR->p_multi_other / 1000.0);
        }
        if (ret < HYDR_PTR->lp_reg_spring_pressure) {
            ret = 0;
        } else {
            ret -= HYDR_PTR->lp_reg_spring_pressure;
        }
        output = ret;
    }
    // Clamping pressures
    if (output > get_max_solenoid_pressure()) {
        output = get_max_solenoid_pressure();
    } else if (output < HYDR_PTR->min_mpc_pressure) {
        output = HYDR_PTR->min_mpc_pressure;
    }
    return output;
}

void PressureManager::notify_shift_end() {
    this->c_gear = 0;
    this->t_gear = 0;
    this->ptr_shift_pressures = nullptr;
}

// TODO pull this from calibration tables
const float C_C_FACTOR[8] = {1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 1.0, 1.0};

CircuitInfo PressureManager::get_basic_shift_data(GearboxConfiguration* cfg, GearChange shift_request, ShiftCharacteristics chars) {
    CircuitInfo sd; 
    uint8_t lookup_valve_info = fwd_gearchange_egs_map_lookup_idx(shift_request);
    switch (shift_request) {
        case GearChange::_1_2:
            sd.targ_g = 2; sd.curr_g = 1;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
        case GearChange::_2_3:
            sd.targ_g = 3; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            break;
        case GearChange::_3_4:
            sd.targ_g = 4; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            break;
        case GearChange::_4_5:
            sd.targ_g = 5; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_4_5;
            break;
        case GearChange::_5_4:
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_circuit = ShiftCircuit::sc_4_5;
            break;
        case GearChange::_4_3:
            sd.targ_g = 3; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            break;
        case GearChange::_3_2:
            sd.targ_g = 2; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            break;
        case GearChange::_2_1:
            sd.targ_g = 1; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
        default:
            break;
    }
    sd.pressure_multi_mpc = (float)HYDR_PTR->overlap_circuit_factor_mpc[lookup_valve_info]/1000.0;
    sd.pressure_multi_spc = (float)HYDR_PTR->overlap_circuit_factor_spc[lookup_valve_info]/1000.0;
    sd.mpc_pressure_spring_reduction = HYDR_PTR->overlap_circuit_spring_pressure[lookup_valve_info];
    sd.centrifugal_factor_off_clutch = C_C_FACTOR[lookup_valve_info];
    // Shift start notify for pm internal algo
    this->c_gear = sd.curr_g;
    this->t_gear = sd.targ_g;
    return sd;
}

uint16_t PressureManager::get_b3_prefill_pressure(void) const {
    if (this->fill_pressure_map) {
        return fill_pressure_map->get_value(1, (uint8_t)Clutch::B3);
    } else {
        return 1500;
    }
}

PrefillData PressureManager::make_fill_data(Clutch applying) {
    if (nullptr == this->fill_time_map) {
        return PrefillData {
            .fill_time = 500,
            .fill_pressure_on_clutch = 1500,
            .low_fill_pressure_on_clutch = 700,
        };
    } else {
        PrefillData ret =  PrefillData {
            .fill_time = (uint16_t)fill_time_map->get_value(this->sensor_data->atf_temp, (uint8_t)applying),
            .fill_pressure_on_clutch = (uint16_t)fill_pressure_map->get_value(1, (uint8_t)applying),
            .low_fill_pressure_on_clutch = (uint16_t)fill_low_pressure_map->get_value(1, (uint8_t)applying),
        };
        return ret;
    }
}

PressureStageTiming PressureManager::get_max_pressure_timing() {
    return PressureStageTiming {
        .hold_time = (uint16_t)interpolate_float(this->sensor_data->atf_temp, 1500, 200, -20, 30, InterpType::Linear),
        .ramp_time = 300,
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
    } else if (ShiftCircuit::sc_3_4 == ss) {
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

void PressureManager::set_target_shift_pressure(uint16_t targ) {
    this->target_shift_pressure = targ;
}

void PressureManager::set_target_modulating_pressure(uint16_t targ) {
    this->target_modulating_pressure = targ;
}

void PressureManager::set_spc_p_max() {
    this->target_shift_pressure = get_max_solenoid_pressure();
}

void PressureManager::set_target_tcc_pressure(uint16_t targ) {
    if (targ > 15000) {
        targ = 15000;
    }
    this->target_tcc_pressure = targ;
}

uint16_t PressureManager::get_spring_pressure(Clutch c) {
    return MECH_PTR->release_spring_pressure[(uint8_t)c];
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

uint16_t PressureManager::get_max_solenoid_pressure() {
    return HYDR_PTR->pcs_map_x[6];
}

StoredMap* PressureManager::get_tcc_pwm_map() { return this->tcc_pwm_map; }
StoredMap* PressureManager::get_fill_time_map() { return this->fill_time_map; }
StoredMap* PressureManager::get_fill_pressure_map() { return  this->fill_pressure_map; }
StoredMap* PressureManager::get_low_fill_pressure_map() { return  this->fill_low_pressure_map; }

PressureManager* pressure_manager = nullptr;