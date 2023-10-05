#ifndef PRESSURE_MANAGER_H
#define PRESSURE_MANAGER_H

#include <common_structs.h>
#include "tcu_maths.h"
#include "profiles.h"
#include "adaptation/shift_adaptation.h"
#include "nvs/eeprom_config.h"
#include "stored_map.h"
#include "sensors.h"

typedef struct {
    uint16_t fill_time;
    uint16_t fill_pressure_on_clutch;
    uint16_t fill_pressure_off_clutch;
} PrefillData;

typedef struct {
    uint16_t hold_time;
    uint16_t ramp_time_1;
    uint16_t ramp_time_2;
} PressureStageTiming;

class PressureManager {

public:
    [[noreturn]]
    static void start_pm_internal(void *_this) {
        static_cast<PressureManager*>(_this)->controller_loop();
    }

    /**
     * @brief Toggle the state of a shift solenoid
     * @param ss Shifting hydralic circuit to engage
     * @param enable Set circuit state to on
     */
    void set_shift_circuit(ShiftCircuit ss, bool enable);
    /**
     * @brief Set the target Working pressure of the gearbox. 
     * This pressure affects the actively engaged clutches in any gear, 
     * as well as hydralic elements in the valve body
     * 
     * @param targ Target Working pressure to achieve in mBar
     */
    void set_target_working_pressure(uint16_t targ);

    /**
     * @brief Set the target Shift pressure clutch pressure.
     * Via means of the overlap valve, when a shift command solenoid is engaged,
     * this pressure is sent to the engaging clutch in the gearbox
     * 
     * @param targ Target pressure to achieve in mBar
     */
    void set_target_shift_clutch_pressure(uint16_t targ);

    /**
     * @brief Set the target Modulating pressure clutch pressure.
     * Via means of the overlap valve, when a shift command solenoid is engaged,
     * this pressure is sent to the releasing clutch in the gearbox
     * 
     * @param targ Target pressure to achieve in mBar
     */
    void set_target_modulating_clutch_pressure(uint16_t targ);

    /**
     * @brief Set the target TCC pressure (Torque converter)
     * 
     * @param targ Target TCC pressure to achieve in mBar
     */
    void set_target_tcc_pressure(uint16_t targ);

   uint16_t get_off_clutch_hold_pressure(Clutch c);

    uint16_t get_targ_line_pressure(void);
    uint16_t get_targ_mpc_clutch_pressure(void) const;
    uint16_t get_targ_spc_clutch_pressure(void) const;
    uint16_t get_targ_mpc_solenoid_pressure(void) const;
    uint16_t get_targ_spc_solenoid_pressure(void) const;
    uint16_t get_targ_tcc_pressure(void) const;
    uint8_t get_active_shift_circuits(void) const;

    /**
     * Force SPC solenoid to turn off
    */
    void set_spc_p_max(void);

    PressureManager(SensorData* sensor_ptr, uint16_t max_torque);

    /**
     * @brief Get the shift data object for the requested gear change
     * 
     * @param shift_request Which gear shift is being requested
     * @param chars Shift requested characteristics
     * @param curr_mpc Current MPC working pressure at the time of shift
     * @return ShiftData 
     */
    ShiftData get_basic_shift_data(GearboxConfiguration* cfg, ProfileGearChange shift_request, ShiftCharacteristics chars);

    uint16_t find_working_mpc_pressure(GearboxGear curr_g);

    PrefillData make_fill_data(ProfileGearChange change);
    PressureStageTiming get_max_pressure_timing();
    StoredMap* get_pcs_map(void);
    StoredMap* get_tcc_pwm_map(void);
    StoredMap* get_working_map(void);
    StoredMap* get_fill_time_map(void);
    StoredMap* get_fill_pressure_map(void);
private:

    void controller_loop();

     /**
     * Returns the estimated PWM to send to either SPC or MPC solenoid
     * Based on the requested pressure that is needed withint either pressure rail.
     */
    uint16_t get_p_solenoid_current(uint16_t request_mbar) const;

    /**
     * Returns the estimated PWM to send to the TCC solenoid
     * based on the requested pressure that is needed
     */
    uint16_t get_tcc_solenoid_pwm_duty(uint16_t request_mbar) const;

    uint8_t shift_circuit_flag = 0;

    SensorData* sensor_data;
    // At the clutch
    uint16_t req_tcc_clutch_pressure;
    // At the engaging clutch (When shifting)
    uint16_t req_spc_clutch_pressure;
    // At the releasing clutch (When shifting)
    uint16_t req_mpc_clutch_pressure;
    // For all clutches
    uint16_t req_working_pressure = 0u;

    // At SPC solenoid
    uint16_t commanded_spc_pressure = 0u;
    // At MPC solenoid
    uint16_t commanded_mpc_pressure = 0u;
    // Shift circuit currently open
    ShiftCircuit currently_open_circuit = ShiftCircuit::None;

    StoredMap* pressure_pwm_map;
    StoredMap* tcc_pwm_map;
    StoredMap* mpc_working_pressure;
    StoredMap* hold2_time_map;
    StoredMap* hold2_pressure_map;
    uint16_t gb_max_torque;
    uint8_t c_gear = 0;
    uint8_t t_gear = 0;

    bool init_ss_recovery = false;
    uint64_t last_ss_on_time = 0;
};

extern PressureManager* pressure_manager;

#endif