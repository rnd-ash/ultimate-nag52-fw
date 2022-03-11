#ifndef TORQUE_CONVERTER_H__
#define TORQUE_CONVERTER_H__

#include <stdint.h>
#include "canbus/can_hal.h"
#include "common_structs.h"
#include "nvs/eeprom_config.h"
#include "esp_log.h"
#include <string.h>
#include "pressure_manager.h"

enum class LockupType {
    Open, // Torque converter must remian open
    Slip, // Torque converter must at most remain between 100-200RPM slip
    Shut  // Torque converter can be full locked
};

class TorqueConverter {
    public:
        TorqueConverter() {
            ESP_LOGI("TCC","Start Torque converter. Adaptation map:");
            for (int i = 0; i < NUM_GEARS; i++) {
                ESP_LOGI("TCC", "Gear %d:", i+1);
                char buffer[256];
                memset(buffer, 0x00, sizeof(buffer));
                int x = 0;
                for (int j = 0; j < 17; j++) {
                    x += sprintf(buffer+x, "%d ", torque_converter_adaptation[i].slip_values[j]);
                }
                ESP_LOGI("TCC", "Lockup thresholds: %s", buffer);
                memset(buffer, 0x00, sizeof(buffer));
                x = 0;
                for (int j = 0; j < 17; j++) {
                    x += sprintf(buffer+x, "%d ", torque_converter_adaptation[i].learned[j]);
                }
                ESP_LOGI("TCC", "Learned stats: %s", buffer);
            }
        }
        /**
         * @brief Lets the torque converter code poll and see what is next to do with the converters
         * clutch
         * 
         * @param curr_gear The current gear the transmission is in
         * @param max_lockup The maximum allowed lockup type for the torque converter
         * @param sensors Sensor data used as input
         * @param shifting True if the car is currently transitioning to new gear
         */
        void update(GearboxGear curr_gear, PressureManager* pm, LockupType max_lockup,SensorData* sensors, bool is_shifting);
        void modify_lockup_data(GearboxGear gear, uint16_t slip_rpm, uint16_t lock_rpm);
        void save_adaptation_data();
        void on_shift_start(uint64_t now, bool is_downshift, float shift_firmness, SensorData* sensors);
        void on_shift_complete(uint64_t now);
    private:
        LockupType current_lockup = LockupType::Open;
        uint8_t gear_idx = 0;
        uint16_t curr_tcc_pwm = 0;
        uint16_t targ_tcc_pwm = 0;
        bool pending_changes = false;
        bool was_idle = false;
        bool adapting_idle = false;
        bool adapting_load = false;
        unsigned long idle_adapt_time = 0;
        unsigned long load_adapt_time = 0;
        unsigned long last_modify_time = 0;
};

#endif