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
#include "moving_average.h"

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
            if (this->tcc_learn_lockup_map != nullptr && this->pending_changes) {
                this->tcc_learn_lockup_map->save_to_eeprom();
            }
        };
        StoredMap* tcc_learn_lockup_map;
        void set_shift_target_state(InternalTccState target_state);
        void on_shift_ending(void);

        void diag_toggle_tcc_sol(bool en);

        // In % (0-100) - 100 is returned if already at next phase
        uint8_t progress_to_next_phase(void);

        void set_stationary();

        int16_t get_slip_filtered();
        uint8_t get_current_state();
        uint8_t get_target_state();
        uint8_t get_can_req_bits();
        uint16_t get_current_pressure();
        uint16_t get_target_pressure();

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
        bool pending_changes = false;
        uint32_t last_adapt_check = 0;
        uint32_t last_slip_add_time = 0;
        MovingAverage* slip_average = nullptr;
        
        bool init_tables_ok = false;
        // -%, 0%, 25%, 50%, 75%, 100%, 150%

        const int16_t load_header[7] = {-25, 0, 25, 50, 75, 100, 150};

        LookupTable* slip_2_3;
        LookupTable* lock_2_3;

        LookupTable* slip_4_5;
        LookupTable* lock_4_5;

        const int16_t slip_data_default[7] = {700, 750, 800, 850, 950, 1000, 1050};
        const int16_t lock_data_default[7] = {850, 850, 1000, 1050, 1100, 1150, 1200};

        bool was_stationary = true;
        uint32_t last_state_stable_time = 0;
};

#endif