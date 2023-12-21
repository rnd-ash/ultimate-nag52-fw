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
        void update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting);
        TccClutchStatus get_clutch_state(void);
        void save() {
            if (this->tcc_learn_lockup_map != nullptr && this->pending_changes) {
                this->tcc_learn_lockup_map->save_to_eeprom();
            }
        };

        void adjust_map_cell(GearboxGear g, uint16_t new_pressure);
        StoredMap* tcc_learn_lockup_map;
        void set_shift_target_state(InternalTccState target_state);
        void on_shift_ending(void);

        void diag_toggle_tcc_sol(bool en);

        // In % (0-100) - 100 is returned if already at next phase
        uint8_t progress_to_next_phase(void);

        void set_stationary();

    private:
        bool tcc_solenoid_enabled = true;
        inline void reset_rpm_samples(SensorData* sensors);
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
        uint16_t slip_offset[5] = {400, 400, 400, 400, 400};
        uint16_t lock_offset[5] = {700, 700, 700, 700, 700};
        bool was_stationary = true;
        uint32_t last_state_stable_time = 0;
};

#endif