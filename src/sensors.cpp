#include "sensors.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "board_config.h"
#include "esp_check.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_private/adc_private.h"

#define N_SENSOR_PULSES_PER_REV 60 // N2 and N3 are 60 pulses per revolution
#define MAX_RPM_PCNT 10000

pcnt_unit_handle_t PCNT_HANDLE_N2;
pcnt_channel_handle_t PCNT_C_HANDLE_N2;

pcnt_unit_handle_t PCNT_HANDLE_N3;
pcnt_channel_handle_t PCNT_C_HANDLE_N3;

pcnt_unit_handle_t PCNT_HANDLE_OUTPUT;
pcnt_channel_handle_t PCNT_C_HANDLE_OUTPUT;

const pcnt_glitch_filter_config_t glitch_filter = {
    .max_glitch_ns = 1000
};

struct PcntCallbackData {
    uint64_t last_time_us;
    uint64_t t_val;
    uint16_t pulses_per_rev;
    uint16_t max_val;
    uint64_t max_time_100rpm;
};

PcntCallbackData cb_data_n2 = {};
PcntCallbackData cb_data_n3 = {};
PcntCallbackData cb_data_out = {};

adc_oneshot_unit_handle_t adc2_handle;

const adc_oneshot_unit_init_cfg_t init_adc2 = {
    .unit_id = ADC_UNIT_2,
    .clk_src = soc_periph_adc_rtc_clk_src_t::ADC_RTC_CLK_SRC_DEFAULT,
    .ulp_mode = adc_ulp_mode_t::ADC_ULP_MODE_DISABLE
};

const adc_oneshot_chan_cfg_t adc2_chan_config = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
};

adc_cali_handle_t adc2_cal = nullptr;

bool output_rpm_ok = false;

static bool IRAM_ATTR on_pulse_reached(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    PcntCallbackData* cbd = (PcntCallbackData*)(user_ctx);
    uint64_t now = esp_timer_get_time();
    uint64_t t_seg = (now - cbd->last_time_us)/cbd->max_val;
    cbd->t_val = t_seg;
    cbd->last_time_us = now;
    return true;
}

static uint16_t calc_rpm(PcntCallbackData* cb) {
    uint16_t val = 0;
    if (cb->t_val != 0 && (esp_timer_get_time() - cb->last_time_us) < cb->max_time_100rpm) {
        val = (60*1000*1000)/(cb->t_val*cb->pulses_per_rev);
    }
    return val;
}

bool Sensors::using_dedicated_output_rpm() {
    return output_rpm_ok;
}

void Sensors::update(SensorDataRaw* dest) {
    dest->rpm_n2 = UINT16_MAX;
    dest->rpm_n3 = UINT16_MAX;
    dest->rpm_out = UINT16_MAX;
    dest->battery_mv = UINT16_MAX;
    dest->atf_temp_c = INT_MAX;
    dest->parking_lock = UINT8_MAX;

    // RPM Sensors
    dest->rpm_n2 = calc_rpm(&cb_data_n2);
    dest->rpm_n3 = calc_rpm(&cb_data_n3);
    if (output_rpm_ok) {
        dest->rpm_out = calc_rpm(&cb_data_out);
    }
    // Battery voltage gathering
    int adc_res = 0;
    int adc_voltage = 0;
    if (ESP_OK == adc_oneshot_read(adc2_handle, pcb_gpio_matrix->sensor_data.adc_batt, &adc_res)) {
        if (ESP_OK == adc_cali_raw_to_voltage(adc2_cal, adc_res, &adc_voltage)) {
            // Vin = Vout(R1+R2)/R2
            adc_voltage *= 5.54; // 5.54 = (100+22)/22
            dest->battery_mv = adc_voltage;
        }
    }

    // TFT/PL lock
    if (ESP_OK == adc_oneshot_read(adc2_handle, pcb_gpio_matrix->sensor_data.adc_atf, &adc_res)) {
        if (adc_res > 3000) {
            dest->parking_lock = 1;
            dest->atf_temp_c = INT_MAX;
        } else {
            dest->parking_lock = 0;
            adc_cali_raw_to_voltage(adc2_cal, adc_res, &adc_voltage);
            const temp_reading_t *atf_temp_lookup = pcb_gpio_matrix->sensor_data.atf_calibration_curve;
            if (adc_voltage <= atf_temp_lookup[0].v)
            {
                dest->atf_temp_c = (int16_t)(atf_temp_lookup[0].temp) / 10.0;
            }
            else if (adc_voltage >= atf_temp_lookup[NUM_TEMP_POINTS - 1].v)
            {   
                dest->atf_temp_c = (int16_t)((atf_temp_lookup[NUM_TEMP_POINTS - 1].temp)) / 10.0;
            }
            else
            {
                
                for (uint8_t i = 0; i < NUM_TEMP_POINTS - 1; i++)
                {
                    // Found! Interpolate linearly to get a better estimate of ATF Temp
                    if (atf_temp_lookup[i].v <= adc_voltage && atf_temp_lookup[i + 1].v >= adc_voltage)
                    {
                        dest->atf_temp_c = interpolate_int(
                            adc_voltage, // Read voltage
                            atf_temp_lookup[i].temp, // Min temp for this range
                            atf_temp_lookup[i+1].temp, // Max temp for this range
                            atf_temp_lookup[i].v, // Min voltage for this boundary
                            atf_temp_lookup[i+1].v // Max voltage for this boundary
                        ) / 10.0;
                        break;
                    }
                }
            }
        }
    }
}

esp_err_t configure_pcnt(const char* name, uint16_t pulses_per_rpm, gpio_num_t gpio, pcnt_unit_handle_t* UNIT_HANDLE, pcnt_channel_handle_t* CHANNEL_HANDLE, PcntCallbackData* cbd) {
#define RPM_DIVIDER 8
    int max = MAX(1, pulses_per_rpm/RPM_DIVIDER);
    memset(cbd, 0x00, sizeof(PcntCallbackData));
    cbd->max_val = max;
    cbd->max_time_100rpm = (((60*1000*1000)/100)/pulses_per_rpm)*RPM_DIVIDER;
    cbd->pulses_per_rev = pulses_per_rpm;
    const pcnt_unit_config_t RPM_UNIT_CFG __attribute__((used)) = {
        .low_limit = INT16_MIN,
        .high_limit = max,
        .flags {
            .accum_count = 0
        }
    };
    ESP_RETURN_ON_ERROR(gpio_set_direction(gpio, GPIO_MODE_INPUT), "SENSORS", "Failed to set %s Pin to Input", name);
    ESP_RETURN_ON_ERROR(gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY), "SENSORS", "Failed to set %s Pin to pullup", name);
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&RPM_UNIT_CFG, UNIT_HANDLE), "SENSORS", "Failed to setup %s RPM PCNT Unit", name);
    const pcnt_chan_config_t rpm_chan_config = {
        .edge_gpio_num = gpio,
        .level_gpio_num = -1,
        .flags {
            .invert_edge_input = 0,
            .invert_level_input = 0,
            .virt_edge_io_level = 0,
            .virt_level_io_level = 0,
            .io_loop_back = 0,
        }     
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(*UNIT_HANDLE, &rpm_chan_config, CHANNEL_HANDLE), "SENSORS", "Failed to setup %s RPM PCNT Channel", name);
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(*CHANNEL_HANDLE, 
            pcnt_channel_edge_action_t::PCNT_CHANNEL_EDGE_ACTION_HOLD,
            pcnt_channel_edge_action_t::PCNT_CHANNEL_EDGE_ACTION_INCREASE
        ),
        "SENSORS",
        "Failed to set PCNT actions for %s PCNT",
        name
    );
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(*UNIT_HANDLE, &glitch_filter), "SENSORS", "Failed to set glitch filter for PCNT unit %s", name);
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(*UNIT_HANDLE, max), "SENSORS", "Failed to add watch point for PCNT unit %s", name);
    pcnt_event_callbacks_t cb = {
        .on_reach = on_pulse_reached
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_register_event_callbacks(*UNIT_HANDLE, &cb, (void*)(cbd)), "SENSORS", "Failed to set callbacks for PCNT unit %s", name);
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(*UNIT_HANDLE), "SENSORS", "Failed to enable PCNT unit %s", name);
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(*UNIT_HANDLE), "SENSORS", "Failed to clear PCNT unit %s", name);
    ESP_RETURN_ON_ERROR(pcnt_unit_start(*UNIT_HANDLE), "SENSORS", "Failed to start PCNT unit %s", name);
    cbd->last_time_us = esp_timer_get_time();
    return ESP_OK;
}

esp_err_t Sensors::init_sensors(void){
    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->vsense_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_VBATT to Input!");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->atf_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_ATF to Input!");

    // Configure ADC2 for analog readings
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_adc2, &adc2_handle), "SENSORS", "Failed to init oneshot ADC2 driver");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc2_handle, pcb_gpio_matrix->sensor_data.adc_atf, &adc2_chan_config), "SENSORS", "Failed to setup oneshot config for ATF channel");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc2_handle, pcb_gpio_matrix->sensor_data.adc_batt, &adc2_chan_config), "SENSORS", "Failed to setup oneshot config for VBATT channel");
    
    // Characterise ADC2       
    adc_cali_line_fitting_efuse_val_t x;
    ESP_RETURN_ON_ERROR(adc_cali_scheme_line_fitting_check_efuse(&x), "SENSORS", "Failed to get ADC Cal type");
    ESP_RETURN_ON_FALSE(x != adc_cali_line_fitting_efuse_val_t::ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF, ESP_ERR_NOT_SUPPORTED, "SENSORS", "Default VREF ADC mode is NOT supported");
    const adc_cali_line_fitting_config_t cali = {
        .unit_id = ADC_UNIT_2,
        .atten = adc_atten_t::ADC_ATTEN_DB_12,
        .bitwidth = adc_bitwidth_t::ADC_BITWIDTH_12,
        .default_vref = 0 // Since we do not support this, set to 0
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali, &adc2_cal), "SENSORS", "Failed to create ADC cal");
    
    // Now configure PCNT to begin counting!
    ESP_RETURN_ON_ERROR(configure_pcnt("N2", N_SENSOR_PULSES_PER_REV, pcb_gpio_matrix->n2_pin, &PCNT_HANDLE_N2, &PCNT_C_HANDLE_N2, &cb_data_n2), "SENSORS", "N2 PCNT Setup failed");
    ESP_RETURN_ON_ERROR(configure_pcnt("N3", N_SENSOR_PULSES_PER_REV, pcb_gpio_matrix->n3_pin, &PCNT_HANDLE_N3, &PCNT_C_HANDLE_N3, &cb_data_n3), "SENSORS", "N3 PCNT Setup failed");

    // Enable output RPM reading if needed
    if (VEHICLE_CONFIG.io_0_usage == 1) {
        if (VEHICLE_CONFIG.input_sensor_pulses_per_rev == 0) {
            ESP_LOGE("SENSORS", "Cannot init output sensor with 0 pulses/rev specified");
            return ESP_ERR_INVALID_ARG;
        } else {
            ESP_LOGI("SENSORS", "Will init OUTPUT RPM sensor");
            ESP_RETURN_ON_ERROR(configure_pcnt("OUT", VEHICLE_CONFIG.input_sensor_pulses_per_rev, pcb_gpio_matrix->io_pin, &PCNT_HANDLE_OUTPUT, &PCNT_C_HANDLE_OUTPUT, &cb_data_n3), "SENSORS", "OUTPUT PCNT Setup failed");
            output_rpm_ok = true;
        }
    }
    ESP_LOGI("SENSORS", "Init complete");
    return ESP_OK;
}