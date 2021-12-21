#ifndef TORQUE_CONVERTER_H__
#define TORQUE_CONVERTER_H__

#include <stdint.h>
#include "canbus/can_hal.h"
#include "common_structs.h"
#include "nvs/eeprom_config.h"
#include "esp_log.h"
#include <string.h>

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
                ESP_LOGI("TCC", "Gear %d:", i);
                ESP_LOGI("TCC", "Slip RPM threshold: %d", torque_converter_adaptation[i].slip_rpm_limit);
                ESP_LOGI("TCC", "Lock RPM threshold: %d", torque_converter_adaptation[i].lock_rpm_limit);
                char buffer[256];
                memset(buffer, 0x00, sizeof(buffer));
                int x = 0;
                for (int j = 0; j < 17; j++) {
                    x += sprintf(buffer+x, "%d ", torque_converter_adaptation[i].slip_values[j]);
                }
                ESP_LOGI("TCC", "Slip values: %s", buffer);
                memset(buffer, 0x00, sizeof(buffer));
                x = 0;
                for (int j = 0; j < 17; j++) {
                    x += sprintf(buffer+x, "%d ", torque_converter_adaptation[i].lockup_values[j]);
                }
                ESP_LOGI("TCC", "Lock values: %s", buffer);
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
        void update(GearboxGear curr_gear, LockupType max_lockup,SensorData* sensors, bool shifting);
        void modify_lockup_data(GearboxGear gear, uint16_t slip_rpm, uint16_t lock_rpm);
        void save_adaptation_data();
    private:
        LockupType current_lockup = LockupType::Open;
        uint8_t gear_idx = 0;
        uint16_t curr_tcc_percent = 0;
        uint16_t targ_tcc_percent = 0;
        bool was_idle = false;
        unsigned long last_modify_time = 0;
};

#endif