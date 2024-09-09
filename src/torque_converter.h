#ifndef TORQUE_CONVERTER_H__
#define TORQUE_CONVERTER_H__

#include <stdint.h>
#include "canbus/can_hal.h"
#include "common_structs.h"
#include "nvs/eeprom_config.h"
#include "esp_log.h"
#include <string.h>
#include "pressure_manager.h"
#include "canbus/can_hal.h"
#include "nvs/module_settings.h"
#include "firstorder_average.h"

enum class InternalTccState {
    Open = 0,
    Slipping = 1,
    Closed = 2
};


class TorqueConverter {
    public:
        TorqueConverter(uint16_t max_gb_rating);

        /**
         * @brief Lets the torque converter code poll and see what is next to do with the converters
         * clutch
         * 
         * @param curr_gear The current gear the transmission is in
         * @param max_lockup The maximum allowed lockup type for the torque converter
         * @param sensors Sensor data used as input
         * @param shifting True if the car is currently transitioning to new gear
         */
        void update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors);
        TccClutchStatus get_clutch_state(void);
        void save() {
            if (this->tcc_lock_map) {
                this->tcc_lock_map->save_to_eeprom();
            }
            if (this->tcc_slip_map) {
                this->tcc_slip_map->save_to_eeprom();
            }
        };
        void set_shift_target_state(SensorData* sd, InternalTccState target_state);
        void on_shift_ending(void);

        void diag_toggle_tcc_sol(bool en);

        void set_stationary();

        int16_t get_slip_filtered();
        InternalTccState __get_internal_state(void);
        uint8_t get_current_state();
        uint8_t get_target_state();
        uint8_t get_can_req_bits();
        uint16_t get_current_pressure();
        uint16_t get_target_pressure();
        uint16_t get_slip_targ() {
            return this->slip_target;
        }

        inline StoredMap* get_slip_map() {
            return this->tcc_slip_map;
        }

        inline StoredMap* get_lock_map() {
            return this->tcc_lock_map;
        }

        inline StoredMap* get_rpm_slip_map() {
            return this->slip_rpm_target_map;
        }
        
        inline uint32_t get_engine_power() {
            return this->engine_output_joule;
        }
        
        inline uint32_t get_absorbed_power() {
            return this->absorbed_power_joule;
        }

    private:
        int rated_max_torque;
        bool is_shifting = false;
        bool tcc_solenoid_enabled = true;
        int tcc_pressure_target = 0;
        int tcc_pressure_current = 0;
        int prev_state_tcc_pressure = 0;
        uint32_t prefill_start_time = 0;
        InternalTccState current_tcc_state = InternalTccState::Open;
        InternalTccState target_tcc_state = InternalTccState::Open;
        InternalTccState shift_req_tcc_state = InternalTccState::Open;
        StoredMap* slip_rpm_target_map;
        bool pending_changes = false;
        uint32_t last_adapt_check = 0;
        FirstOrderAverage* slip_average = nullptr;
        
        bool init_tables_ok = false;

        StoredMap* tcc_slip_map = nullptr;
        StoredMap* tcc_lock_map = nullptr;

        bool was_stationary = true;
        uint32_t last_state_stable_time = 0;
        uint16_t slip_target = 100;
        uint32_t absorbed_power_joule = 0;
        uint32_t engine_output_joule = 0;
};

#endif