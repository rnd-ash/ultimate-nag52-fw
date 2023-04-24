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


#define TCC_ADV_OPTS_NVS_KEY "TCC_A2"
typedef struct {
    bool adapt_enable;
    bool enable_d1;
    bool enable_d2;
    bool enable_d3;
    bool enable_d4;
    bool enable_d5;
    uint16_t prefill_pressure;
    uint16_t lock_rpm_threshold;
    uint16_t min_locking_rpm;
    uint16_t adjust_interval_ms;
    uint16_t tcc_stall_speed;
    uint16_t min_torque_adapt;
    uint16_t max_torque_adapt;
    uint16_t prefill_min_engine_rpm;
    uint16_t base_pressure_offset_start_ramp;
    LinearInterpSetting pressure_increase_ramp_settings;
    uint8_t adapt_pressure_inc;
    uint16_t adapt_lock_detect_time;
    uint16_t pulling_slip_rpm_low_threshold;
    uint16_t pulling_slip_rpm_high_threhold;
    float reaction_torque_multiplier;
    uint16_t trq_consider_coasting;
    LinearInterpSetting load_dampening;
    LinearInterpSetting pressure_multiplier_output_rpm; 
    uint16_t max_allowed_bite_pressure;
    uint16_t max_allowed_pressure_longterm;
} __attribute__ ((packed)) TCC_ADV_OPTS;

const TCC_ADV_OPTS TCC_ADV_OPTS_DEFAULT = {
    .adapt_enable = true,
    .enable_d1 = true,
    .enable_d2 = true,
    .enable_d3 = true,
    .enable_d4 = true,
    .enable_d5 = true,
    .prefill_pressure = 500,
    .lock_rpm_threshold = 50,
    .min_locking_rpm = 1100,
    .adjust_interval_ms = 500,
    .tcc_stall_speed = 2500,
    .min_torque_adapt = 50,
    .max_torque_adapt = 110,
    .prefill_min_engine_rpm = 900,
    .base_pressure_offset_start_ramp = 300,
    .pressure_increase_ramp_settings = {
        .new_min = 1,
        .new_max = 5,
        .raw_min = 100,
        .raw_max = 1000,
    },
    .adapt_pressure_inc = 10,
    .adapt_lock_detect_time = 2000,
    .pulling_slip_rpm_low_threshold = 20,
    .pulling_slip_rpm_high_threhold = 100,
    .reaction_torque_multiplier = 15,
    .trq_consider_coasting = 40,
    .load_dampening = {
        .new_min = 100,
        .new_max = 50,
        .raw_min = -40,
        .raw_max = 40,
    },
    .pressure_multiplier_output_rpm = {
        .new_min = 1.00,
        .new_max = 1.25,
        .raw_min = 1500,
        .raw_max = 2500,
    },
    .max_allowed_bite_pressure = 1800,
    .max_allowed_pressure_longterm = 7000,
};

const size_t TCC_ADV_OPTS_LEN = sizeof(TCC_ADV_OPTS);

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
        ClutchStatus get_clutch_state(void);
        void save() {
            if (this->tcc_learn_lockup_map != nullptr && this->pending_changes) {
                this->tcc_learn_lockup_map->save_to_eeprom();
            }
        };

        void adjust_map_cell(GearboxGear g, uint16_t new_pressure);
        StoredMap* tcc_learn_lockup_map;

        const TCC_ADV_OPTS* get_running_opts();
        esp_err_t set_running_opts(TCC_ADV_OPTS opts);
        esp_err_t reset_opts();
        esp_err_t write_opts(TCC_ADV_OPTS opts);
    private:
        esp_err_t check_running_opts(TCC_ADV_OPTS opts);
        inline void reset_rpm_samples(SensorData* sensors);
        bool neg_torque_zone = false;
        uint16_t adapt_lock_count = 0;
        uint16_t strike_count = 0;
        bool initial_ramp_done = false;
        uint32_t curr_tcc_target = 0;
        uint32_t curr_tcc_pressure = 0;
        uint32_t base_tcc_pressure = 0;
        bool inhibit_increase = false;
        bool was_idle = false;
        bool prefilling = false;
        uint64_t prefill_start_time = 0;
        ClutchStatus state = ClutchStatus::Open;
        uint64_t last_inc_time = 0;
        bool is_temp_pressure = false;
        uint16_t tmp_pressure = 0;
        uint64_t last_adj_time = 0;
        bool pending_changes = false;
        bool was_shifting = false;
        uint64_t last_idle_timestamp = 0;
        TCC_ADV_OPTS tcc_settings;
};

#endif