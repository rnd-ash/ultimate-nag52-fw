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

/**
 * @brief Calculates the current working pressure (p-A) of the valve body of the gearbox, taking into account influence of regulators
 * @param current_gear The current gear the gearbox is in
 * @param in_mpc Target Modulating valve pressure (p-RV)
 * @param in_spc Target Shift valve pressure (p-SV)
 * @return Working pressure in mBar (p-A)
 */
uint16_t PressureManager::calc_working_pressure(GearboxGear current_gear, uint16_t in_mpc, uint16_t in_spc) {
    float fac = 1.0/((float)HYDR_PTR->p_multi_other/1000.0);
    uint16_t p_adder = HYDR_PTR->extra_pressure_adder_other_gears;
    // Only when not shifting and constantly in 1 or R1
    if ((current_gear == GearboxGear::First || current_gear == GearboxGear::Reverse_First) && this->shift_circuit_flag == 0) {
        fac = 1.0/((float)HYDR_PTR->p_multi_1/1000.0);
        p_adder = HYDR_PTR->extra_pressure_adder_r1_1;
    }
    float regulator_pressure = in_mpc + HYDR_PTR->lp_reg_spring_pressure; // Using just p-RV, this is what the working pressure should be
    // Now calculate influence of shift pressure and additional pressure caused by RPM
    float k1_factor = 0;
    if (this->shift_circuit_flag == (uint8_t)ShiftCircuit::sc_1_2) { // 1-2 or 2-1
        k1_factor = (float)HYDR_PTR->shift_pressure_factor_percent/1000.0;
    }
    uint16_t extra_pressure = interpolate_float(
        sensor_data->engine_rpm, // Engine RPM drives the pump, not input RPM
        0,
        p_adder,
        HYDR_PTR->extra_pressure_pump_speed_min,
        HYDR_PTR->extra_pressure_pump_speed_max,
        InterpType::Linear
    );
    float spc_reduction = in_spc * k1_factor; // Influence of Shift pressure on Working pressure regulator when K1 is applied and shift circuit is active
    return (fac * regulator_pressure) + extra_pressure - spc_reduction;
}

uint16_t PressureManager::calc_input_pressure(uint16_t working_pressure) {
    return interpolate_float(
        working_pressure,
        HYDR_PTR->inlet_pressure_output_min,
        HYDR_PTR->inlet_pressure_output_max,
        HYDR_PTR->inlet_pressure_input_min,
        HYDR_PTR->inlet_pressure_input_max,
        InterpType::Linear
    );
}

#define CLAMP(in, min, max) MAX(min, MIN(in, max))

// Number returns is 0..0.1
float PressureManager::calc_inlet_factor(uint16_t inlet_pressure) {
    return CLAMP(
        (((float)HYDR_PTR->shift_pressure_addr_percent/1000.0) * ((float)HYDR_PTR->inlet_pressure_output_max - (float)inlet_pressure)),
        0,
        100.0
    ) / 1000.0;
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
void PressureManager::update_pressures(GearboxGear current_gear) {
    // Ignore
    if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE)) {

    } else {
        float amplifier = 1.0;
        if ((this->shift_circuit_flag == (uint8_t)ShiftCircuit::sc_1_2)) {
            amplifier = 1.993;
        }

        uint16_t spc_in = this->target_shift_pressure;
        uint16_t spc_sol_in = ((float)this->target_shift_pressure / amplifier) + HYDR_PTR->shift_reg_spring_pressure;
        uint16_t mpc_in = this->target_modulating_pressure;

        uint16_t p_a = this->calc_working_pressure(current_gear, mpc_in, spc_in);
        uint16_t inlet = this->calc_input_pressure(p_a);

        this->calculated_inlet_pressure = inlet;
        this->calculated_working_pressure = p_a;

        float factor = this->calc_inlet_factor(inlet);
        this->corrected_mpc_pressure = mpc_in + (factor * (float)(mpc_in + HYDR_PTR->inlet_pressure_offset));
        if (spc_sol_in > inlet) {
            this->corrected_spc_pressure = this->get_max_solenoid_pressure();
        } else {
            this->corrected_spc_pressure = spc_sol_in + (factor * (float)(spc_sol_in + HYDR_PTR->inlet_pressure_offset));
        }

        // Now actuate solenoids
        this->corrected_spc_pressure = MIN(this->corrected_spc_pressure, get_max_solenoid_pressure());
        this->corrected_mpc_pressure = MIN(this->corrected_mpc_pressure, get_max_solenoid_pressure());
        if (get_max_solenoid_pressure() == this->corrected_spc_pressure) {
            sol_spc->set_current_target(0);
        } else {
            sol_spc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_spc_pressure, sensor_data->atf_temp+50.0));
        }
        sol_mpc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_mpc_pressure, sensor_data->atf_temp+50.0));
        sol_tcc->set_duty(this->get_tcc_solenoid_pwm_duty(this->target_tcc_pressure));
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

uint16_t PressureManager::find_working_pressure_for_clutch(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, bool clamp_to_min_mpc) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float friction_coefficient = this->friction_coefficient();
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = (friction_val / friction_coefficient) * (float)abs_torque_nm;
    if (calc < HYDR_PTR->min_mpc_pressure && clamp_to_min_mpc) {
        calc = HYDR_PTR->min_mpc_pressure;
    } else if (calc > get_max_solenoid_pressure()) {
        calc = get_max_solenoid_pressure();
    }
    return calc;
}

uint16_t PressureManager::find_releasing_pressure_for_clutch(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    // Normal EGS uses hard coded 120 for temp coefficient when releasing clutch...
    // 120 = 0.85% of 140 (Coefficient at 80C)
    // Experiment. See how 85% of temp coefficient feels across temperature range rather than hard coded static val.
    float release_coefficient = this->release_coefficient();
    //float friction_coefficient = 120.0;
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = (friction_val / release_coefficient) * (float)abs_torque_nm;
    return calc;
}

float PressureManager::friction_coefficient() {
    return interpolate_float(
        sensor_data->atf_temp, 
        friction_coefficient_0c,
        friction_coefficient_80C,
        0,
        80,
        InterpType::Linear
    );
}

float PressureManager::release_coefficient() {
    return this->friction_coefficient() * 0.85;
}

uint16_t PressureManager::find_decent_adder_torque(ProfileGearChange change, uint16_t abs_motor_torque, uint16_t output_rpm) {
    LookupByteMap* map = nullptr;
    switch(change) {
        case ProfileGearChange::ONE_TWO:
            map = this->torque_adder_upshifts[0];
            break;
        case ProfileGearChange::TWO_THREE:
            map = this->torque_adder_upshifts[1];
            break;
        case ProfileGearChange::THREE_FOUR:
            map = this->torque_adder_upshifts[2];
            break;
        case ProfileGearChange::FOUR_FIVE:
            map = this->torque_adder_upshifts[3];
            break;
        case ProfileGearChange::TWO_ONE:
            map = this->torque_adder_downshifts[0];
            break;
        case ProfileGearChange::THREE_TWO:
            map = this->torque_adder_downshifts[1];
            break;
        case ProfileGearChange::FOUR_THREE:
            map = this->torque_adder_downshifts[2];
            break;
        case ProfileGearChange::FIVE_FOUR:
            map = this->torque_adder_downshifts[3];
            break;
    }
    if (nullptr == map) {
        return 0;
    } else {
        uint16_t ret = map->get_value((float)output_rpm/30.0, (float)abs_motor_torque/5.0); 
        return ret;
    }
}

uint16_t PressureManager::find_freeing_torque(ProfileGearChange change, uint16_t motor_torque, uint16_t output_rpm) {
    LookupByteMap* map = nullptr;
    switch(change) {
        case ProfileGearChange::ONE_TWO:
            map = this->momentum_upshifts[0];
            break;
        case ProfileGearChange::TWO_THREE:
            map = this->momentum_upshifts[1];
            break;
        case ProfileGearChange::THREE_FOUR:
            map = this->momentum_upshifts[2];
            break;
        case ProfileGearChange::FOUR_FIVE:
            map = this->momentum_upshifts[3];
            break;
        case ProfileGearChange::TWO_ONE:
            map = this->momentum_downshifts[0];
            break;
        case ProfileGearChange::THREE_TWO:
            map = this->momentum_downshifts[1];
            break;
        case ProfileGearChange::FOUR_THREE:
            map = this->momentum_downshifts[2];
            break;
        case ProfileGearChange::FIVE_FOUR:
            map = this->momentum_downshifts[3];
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

uint16_t PressureManager::calc_max_torque_for_clutch(GearboxGear gear, Clutch clutch, uint16_t pressure, bool use_release_coefficient) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coefficient = use_release_coefficient ? this->release_coefficient() : this->friction_coefficient();
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = ((float)pressure * coefficient) / (float)friction_val;
    return calc;
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g) {
    uint8_t gear_idx = gear_to_idx_lookup(curr_g);
    uint8_t clutch_idx = MECH_PTR->strongest_loaded_clutch_idx[gear_idx];
    if (clutch_idx < 6) {
        return find_working_pressure_for_clutch(curr_g, (Clutch)clutch_idx, abs(sensor_data->input_torque));
    } else {
        return HYDR_PTR->min_mpc_pressure; // For neutral or park
    }
}

void PressureManager::notify_shift_end() {
    this->c_gear = 0;
    this->t_gear = 0;
    this->ptr_shift_pressures = nullptr;
}

// TODO pull this from calibration tables
const float C_C_FACTOR[8] = {1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 1.0, 1.0};

CircuitInfo PressureManager::get_basic_shift_data(GearboxConfiguration* cfg, ProfileGearChange shift_request, ShiftCharacteristics chars) {
    CircuitInfo sd; 
    uint8_t lookup_valve_info = 0;
    switch (shift_request) {
        case ProfileGearChange::ONE_TWO:
            sd.targ_g = 2; sd.curr_g = 1;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            lookup_valve_info = 0;
            break;
        case ProfileGearChange::TWO_THREE:
            sd.targ_g = 3; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            lookup_valve_info = 1;
            break;
        case ProfileGearChange::THREE_FOUR:
            sd.targ_g = 4; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            lookup_valve_info = 2;
            break;
        case ProfileGearChange::FOUR_FIVE:
            sd.targ_g = 5; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_4_5;
            lookup_valve_info = 3;
            break;
        case ProfileGearChange::FIVE_FOUR:
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_circuit = ShiftCircuit::sc_4_5;
            lookup_valve_info = 7;
            break;
        case ProfileGearChange::FOUR_THREE:
            sd.targ_g = 3; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            lookup_valve_info = 6;
            break;
        case ProfileGearChange::THREE_TWO:
            sd.targ_g = 2; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            lookup_valve_info = 5;
            break;
        case ProfileGearChange::TWO_ONE:
            sd.targ_g = 1; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            lookup_valve_info = 4;
            break;
    }
    sd.pressure_multi_mpc = (float)HYDR_PTR->overlap_circuit_factor_mpc[lookup_valve_info]/1000.0;
    sd.pressure_multi_spc = (float)HYDR_PTR->overlap_circuit_factor_spc[lookup_valve_info]/1000.0;
    sd.mpc_pressure_spring_reduction = HYDR_PTR->overlap_circuit_spring_pressure[lookup_valve_info];
    sd.centrifugal_factor_off_clutch = C_C_FACTOR[lookup_valve_info];
    sd.map_idx = lookup_valve_info;
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
        return PrefillData {
            .fill_time = (uint16_t)fill_time_map->get_value(this->sensor_data->atf_temp, (uint8_t)applying),
            .fill_pressure_on_clutch = (uint16_t)fill_pressure_map->get_value(1, (uint8_t)applying),
            .low_fill_pressure_on_clutch = (uint16_t)fill_low_pressure_map->get_value(1, (uint8_t)applying),
        };
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

PressureManager* pressure_manager = nullptr;