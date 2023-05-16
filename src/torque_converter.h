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
    private:
        inline void reset_rpm_samples(SensorData* sensors);
        bool neg_torque_zone = false;
        uint16_t adapt_lock_count = 0;
        uint16_t strike_count = 0;
        bool initial_ramp_done = false;
        float curr_tcc_target = 0;
        float curr_tcc_pressure = 0;
        float base_tcc_pressure = 0;
        bool inhibit_increase = false;
        bool was_idle = false;
        bool prefilling = false;
        uint64_t prefill_start_time = 0;
        TccClutchStatus state = TccClutchStatus::Open;
        uint64_t last_inc_time = 0;
        bool is_temp_pressure = false;
        uint16_t tmp_pressure = 0;
        uint64_t last_adj_time = 0;
        bool pending_changes = false;
        bool was_shifting = false;
        uint64_t last_idle_timestamp = 0;
};

#endif