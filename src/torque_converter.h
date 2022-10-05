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

class TorqueConverter {
    public:
        TorqueConverter() {}
        /**
         * @brief Lets the torque converter code poll and see what is next to do with the converters
         * clutch
         * 
         * @param curr_gear The current gear the transmission is in
         * @param max_lockup The maximum allowed lockup type for the torque converter
         * @param sensors Sensor data used as input
         * @param shifting True if the car is currently transitioning to new gear
         */
        void update(GearboxGear curr_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors, bool is_shifting);
        void modify_lockup_data(GearboxGear gear, uint16_t slip_rpm, uint16_t lock_rpm);
        void on_shift_start(uint64_t now, bool is_downshift, SensorData* sensors, float shift_firmness);
        void on_shift_complete(uint64_t now);
        ClutchStatus get_clutch_state();
    private:
        uint16_t strike_count = 0;
        float curr_tcc_pressure = 0;
        float tcc_shift_adder = 0;
        bool inhibit_increase = false;
        bool was_idle = false;
        bool prefilling = false;
        uint64_t prefill_start_time = 0;
        ClutchStatus state = ClutchStatus::Open;
        uint64_t last_inc_time = 0;
        bool is_temp_pressure = false;
        uint16_t tmp_pressure = 0;
};

#endif