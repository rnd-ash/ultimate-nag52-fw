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

enum class InternalTccState {
    Open,
    Slipping,
    Closed
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
        void on_shift_starting(void);
        void on_shift_ending(void);
    private:
        inline void reset_rpm_samples(SensorData* sensors);
        float tcc_pressure_target = 0;
        float tcc_pressure_current = 0;
        uint64_t prefill_start_time = 0;
        InternalTccState current_tcc_state = InternalTccState::Open;
        InternalTccState target_tcc_state = InternalTccState::Open;
        InternalTccState preshift_tcc_state = InternalTccState::Open;
        bool pending_changes = false;
};

#endif