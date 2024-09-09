#include "sensors.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "esp_check.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "tcu_maths.h"
#include "esp_timer.h"
#include "esp_private/adc_private.h"
#include "firstorder_average.h"

#define N_SENSOR_PULSES_PER_REV 60 // N2 and N3 are 60 pulses per revolution
#define MAX_RPM_PCNT 10000

const pcnt_unit_config_t RPM_UNIT_CFG __attribute__((used)) = {
    .low_limit = INT16_MIN,
    .high_limit = INT16_MAX,
    .flags {
        .accum_count = 0
    }
};

pcnt_unit_handle_t PCNT_HANDLE_N2;
pcnt_channel_handle_t PCNT_C_HANDLE_N2;

pcnt_unit_handle_t PCNT_HANDLE_N3;
pcnt_channel_handle_t PCNT_C_HANDLE_N3;

pcnt_unit_handle_t PCNT_HANDLE_OUTPUT;
pcnt_channel_handle_t PCNT_C_HANDLE_OUTPUT;

adc_oneshot_unit_handle_t adc2_handle;
adc_oneshot_unit_init_cfg_t init_adc2 = {
    .unit_id = ADC_UNIT_2,
    .clk_src = soc_periph_adc_rtc_clk_src_t::ADC_RTC_CLK_SRC_DEFAULT,
    .ulp_mode = adc_ulp_mode_t::ADC_ULP_MODE_DISABLE
};
adc_oneshot_chan_cfg_t adc2_chan_config = {
    .atten = ADC_ATTEN_DB_11,
    .bitwidth = ADC_BITWIDTH_12,
};
adc_cali_handle_t adc2_cal = nullptr;

// For RPM sensor debouncing / smoothing

#define RPM_CHANGE_MAX 1000
#define SENSOR_TIMER_INTERVAL_MS 20

FirstOrderAverage* n2_avg_buffer = nullptr;
FirstOrderAverage* n3_avg_buffer = nullptr;
FirstOrderAverage* out_avg_buffer = nullptr;
FirstOrderAverage* tft_filter = nullptr;
FirstOrderAverage* batt_filter = nullptr;
bool output_rpm_ok = false;

// Good enough for both boxes, but will be corrected as soon as the gearbox code boots up
// to a more accurate value
float RATIO_2_1 = 1.61f;

inline static void read_and_reset_pcnt(pcnt_unit_handle_t unit, int* dest) {
    pcnt_unit_get_count(unit, dest);
    pcnt_unit_clear_count(unit);
}

int batt_adc_res = 0;
int tft_adc_res = 0;
int16_t motor_temp = 25;
esp_err_t pl_res = ESP_OK;
bool parking_lock = true;
esp_err_t vbatt_res = ESP_OK;
uint16_t vbatt = 12000; // 12.0V
esp_err_t tft_res = ESP_OK;
int16_t tft = 25; // 25.0C

uint8_t sensor_counter = 0;
int adc_voltage;

static bool IRAM_ATTR on_rpm_timer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    int pulses = 0;
    // N2 Sensor
    read_and_reset_pcnt(PCNT_HANDLE_N2, &pulses);
    n2_avg_buffer->add_sample(pulses*50);

    // N3 Sensor
    read_and_reset_pcnt(PCNT_HANDLE_N3, &pulses);
    n3_avg_buffer->add_sample(pulses*50);

    if (output_rpm_ok) {
        // Output Sensor
        read_and_reset_pcnt(PCNT_HANDLE_OUTPUT, &pulses);
        out_avg_buffer->add_sample(pulses*50);
    }

    adc_oneshot_read_isr(adc2_handle, pcb_gpio_matrix->sensor_data.adc_atf, &tft_adc_res);
    adc_oneshot_read_isr(adc2_handle, pcb_gpio_matrix->sensor_data.adc_batt, &batt_adc_res);

    adc_cali_raw_to_voltage(adc2_cal, batt_adc_res, &adc_voltage);
    // Vin = Vout(R1+R2)/R2
    adc_voltage *= 5.54; // 5.54 = (100+22)/22
    batt_filter->add_sample(adc_voltage);
    vbatt = batt_filter->get_average();

    if (tft_adc_res > 3000) {
        parking_lock = true;
        tft_res = ESP_ERR_INVALID_STATE;
    } else {
        parking_lock = false;
        tft_res = ESP_OK;
        adc_cali_raw_to_voltage(adc2_cal, tft_adc_res, &adc_voltage);
        const temp_reading_t *atf_temp_lookup = pcb_gpio_matrix->sensor_data.atf_calibration_curve;
        int atf_calc_c = 0;
        if (adc_voltage <= atf_temp_lookup[0].v)
        {
            atf_calc_c = (int16_t)(atf_temp_lookup[0].temp);
        }
        else if (adc_voltage >= atf_temp_lookup[NUM_TEMP_POINTS - 1].v)
        {
            atf_calc_c = (int16_t)((atf_temp_lookup[NUM_TEMP_POINTS - 1].temp) / 10);
        }
        else
        {
            for (uint8_t i = 0; i < NUM_TEMP_POINTS - 1; i++)
            {
                // Found! Interpolate linearly to get a better estimate of ATF Temp
                if (atf_temp_lookup[i].v <= adc_voltage && atf_temp_lookup[i + 1].v >= adc_voltage)
                {
                    atf_calc_c = interpolate_int(
                        adc_voltage, // Read voltage
                        atf_temp_lookup[i].temp, // Min temp for this range
                        atf_temp_lookup[i+1].temp, // Max temp for this range
                        atf_temp_lookup[i].v, // Min voltage for this boundary
                        atf_temp_lookup[i+1].v // Max voltage for this boundary
                    );
                    break;
                }
            }
        }
        tft_filter->add_sample(atf_calc_c);
        tft = tft_filter->get_average();
    }
    return true;
}

const pcnt_glitch_filter_config_t glitch_filter = {
    // for input RPM - 1RPM = 1/Sec
    // ==> 10000RPM = 10000RPS
    // => 
    .max_glitch_ns = 1000
};

esp_err_t configure_pcnt(const char* name, gpio_num_t gpio, pcnt_unit_handle_t* UNIT_HANDLE, pcnt_channel_handle_t* CHANNEL_HANDLE, FirstOrderAverage** buffer) {
    *buffer = new FirstOrderAverage(5);
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
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(*UNIT_HANDLE), "SENSORS", "Failed to enable PCNT unit %s", name);
    ESP_RETURN_ON_ERROR(pcnt_unit_start(*UNIT_HANDLE), "SENSORS", "Failed to start PCNT unit %s", name);
    return ESP_OK;
}

esp_err_t Sensors::init_sensors(void){
    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->vsense_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_VBATT to Input!");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->atf_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_ATF to Input!");

    // Set moving average buffers
    // TFT and Battery is averaged over 1 second
    tft_filter = new FirstOrderAverage(500/SENSOR_TIMER_INTERVAL_MS);
    batt_filter = new FirstOrderAverage(500/SENSOR_TIMER_INTERVAL_MS, 12000);

    // Configure ADC2 for analog readings
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_adc2, &adc2_handle), "SENSORS", "Failed to init oneshot ADC2 driver");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc2_handle, pcb_gpio_matrix->sensor_data.adc_atf, &adc2_chan_config), "SENSORS", "Failed to setup oneshot config for ATF channel");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc2_handle, pcb_gpio_matrix->sensor_data.adc_batt, &adc2_chan_config), "SENSORS", "Failed to setup oneshot config for VBATT channel");
    // Characterise ADC2       
    const adc_cali_line_fitting_config_t cali = {
        .unit_id = ADC_UNIT_2,
        .atten = adc_atten_t::ADC_ATTEN_DB_11,
        .bitwidth = adc_bitwidth_t::ADC_BITWIDTH_12,
        .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali, &adc2_cal), "SENSORS", "Failed to create line fitting ADC2 scheme");
    
    // Now configure PCNT to begin counting!
    ESP_RETURN_ON_ERROR(configure_pcnt("N2", pcb_gpio_matrix->n2_pin, &PCNT_HANDLE_N2, &PCNT_C_HANDLE_N2, &n2_avg_buffer), "SENSORS", "N2 PCNT Setup failed");
    ESP_RETURN_ON_ERROR(configure_pcnt("N3", pcb_gpio_matrix->n3_pin, &PCNT_HANDLE_N3, &PCNT_C_HANDLE_N3, &n3_avg_buffer), "SENSORS", "N3 PCNT Setup failed");

    // Enable output RPM reading if needed
    if (VEHICLE_CONFIG.io_0_usage == 1) {
        if (VEHICLE_CONFIG.input_sensor_pulses_per_rev == 0) {
            ESP_LOGE("SENSORS", "Cannot init output sensor with 0 pulses/rev specified");
            return ESP_ERR_INVALID_ARG;
        } else {
            ESP_LOGI("SENSORS", "Will init OUTPUT RPM sensor");
            ESP_RETURN_ON_ERROR(configure_pcnt("OUT", pcb_gpio_matrix->io_pin, &PCNT_HANDLE_OUTPUT, &PCNT_C_HANDLE_OUTPUT, &out_avg_buffer), "SENSORS", "OUTPUT PCNT Setup failed");
            output_rpm_ok = true;
        }
    }

    gptimer_handle_t gptimer_pcnt = NULL;

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000u,
        .flags = {
            .intr_shared = 1
        }
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &gptimer_pcnt), "SENSORS", "Failed to create PCNT read timer");

    const gptimer_alarm_config_t alarm_config_pcnt = {
        .alarm_count = (20 * 1000), // Every 20ms
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        }
    };
    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(gptimer_pcnt, &alarm_config_pcnt), "SENSORS", "Failed to set PCNT Timer Alarm action");

    const gptimer_event_callbacks_t cbs_pcnt = {
        .on_alarm = on_rpm_timer
    };

    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(gptimer_pcnt, &cbs_pcnt, nullptr), "SENSORS", "Failed to register PCNT timer callback");

    ESP_RETURN_ON_ERROR(gptimer_enable(gptimer_pcnt), "SENSORS", "Failed to enable PCNT GPTIMER");

    ESP_RETURN_ON_ERROR(gptimer_start(gptimer_pcnt), "SENSORS", "Failed to start PCNT GPTIMER");
    ESP_LOGI("SENSORS", "Init complete");
    return ESP_OK;
}

// 1rpm = 60 pulse/sec -> 1 pulse every 20ms at MINIMUM
esp_err_t Sensors::read_input_rpm(RpmReading *dest, bool check_sanity)
{
    esp_err_t res = ESP_OK;
    if (nullptr == n2_avg_buffer || nullptr == n3_avg_buffer) {
        res = ESP_ERR_NOT_SUPPORTED;
    } else {
        float f2 = ((float)n2_avg_buffer->get_average() * 60.0) / N_SENSOR_PULSES_PER_REV;
        float f3 = ((float)n3_avg_buffer->get_average() * 60.0) / N_SENSOR_PULSES_PER_REV;
        float c = (f2 * RATIO_2_1) + (f3 - (RATIO_2_1*f3));
        if (c < 0) {
            c = 0;
        }
        dest->n2_raw = f2;
        dest->n3_raw = f3;
        dest->calc_rpm = c;

        // If we need to check sanity, check it, in gears 2,3 and 4, RPM readings should be the same,
        // otherwise we have a faulty conductor place sensor!
        if (check_sanity && abs((int)dest->n2_raw - (int)dest->n3_raw) > 100) {
            res = ESP_ERR_INVALID_STATE;
        }
    }
    return res;
}

void Sensors::set_ratio_2_1(float r) {
    RATIO_2_1 = r;
}

float Sensors::get_ratio_2_1() {
    return RATIO_2_1;
}

esp_err_t Sensors::read_output_rpm(uint16_t* dest) {
    esp_err_t res = ESP_OK;
    if (output_rpm_ok == false) {
        res = ESP_ERR_INVALID_STATE;
    } else {
        // Pulses/sec
        float f_out = (float)out_avg_buffer->get_average();

        float rpm = (f_out * 60) / (float)VEHICLE_CONFIG.input_sensor_pulses_per_rev;
        *dest = (uint16_t)rpm;
    }
    return res;
}

esp_err_t Sensors::read_vbatt(uint16_t *dest){
    if (vbatt_res == ESP_OK) {
        *dest = vbatt;
    }
    return vbatt_res;
}

// Returns ATF temp in *C
esp_err_t Sensors::read_atf_temp(int16_t *dest)
{
    if (tft_res == ESP_OK) {
        *dest = tft/10;
    }
    return tft_res;
}

esp_err_t Sensors::read_atf_temp_fine(int16_t *dest)
{
    if (tft_res == ESP_OK) {
        *dest = tft;
    }
    return tft_res;
}

esp_err_t Sensors::parking_lock_engaged(bool *dest)
{
    if (pl_res == ESP_OK) {
        *dest = parking_lock;
    }
    return ESP_OK;
}

void Sensors::set_motor_temperature(int16_t celcius) {
    motor_temp = celcius;
}