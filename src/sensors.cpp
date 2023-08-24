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
#include "moving_average.h"

#define PULSES_PER_REV 60 // N2 and N3 are 60 pulses per revolution
#define MAX_RPM_PCNT 10000

const pcnt_unit_config_t RPM_UNIT_CFG = {
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
    .ulp_mode = adc_ulp_mode_t::ADC_ULP_MODE_DISABLE
};
adc_oneshot_chan_cfg_t adc2_chan_config = {
    .atten = ADC_ATTEN_DB_11,
    .bitwidth = ADC_BITWIDTH_12,
};
adc_cali_handle_t adc2_cal = nullptr;

// For RPM sensor debouncing / smoothing

#define RPM_CHANGE_MAX 1000
#define RPM_TIMER_INTERVAL_MS 20

MovingUnsignedAverage* n2_avg_buffer = nullptr;
MovingUnsignedAverage* n3_avg_buffer = nullptr;
MovingUnsignedAverage* out_avg_buffer = nullptr;

uint8_t avg_n2_idx = 0;
uint8_t avg_n3_idx = 0;
uint8_t avg_out_idx = 0;
bool output_rpm_ok = false;

// Good enough for both boxes, but will be corrected as soon as the gearbox code boots up
// to a more accurate value
float RATIO_2_1 = 1.61f;


inline static void read_and_reset_pcnt(pcnt_unit_handle_t unit, int* dest) {
    pcnt_unit_get_count(unit, dest);
    pcnt_unit_clear_count(unit);
}

static bool IRAM_ATTR on_rpm_timer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    int pulses = 0;
    // N2 Sensor
    read_and_reset_pcnt(PCNT_HANDLE_N2, &pulses);
    n2_avg_buffer->add_sample(pulses*50);

    // N3 Sensor
    read_and_reset_pcnt(PCNT_HANDLE_N3, &pulses);
    n3_avg_buffer->add_sample(pulses*50);
    
    // Output Sensor (If present)
    if (output_rpm_ok) {
        read_and_reset_pcnt(PCNT_HANDLE_OUTPUT, &pulses);
        out_avg_buffer->add_sample(pulses*50);
    }
    return true;
}

const pcnt_glitch_filter_config_t glitch_filter = {
    // for input RPM - 1RPM = 1/Sec
    // ==> 10000RPM = 10000RPS
    // => 
    .max_glitch_ns = 1000
};

esp_err_t configure_pcnt(const char* name, gpio_num_t gpio, pcnt_unit_handle_t* UNIT_HANDLE, pcnt_channel_handle_t* CHANNEL_HANDLE, MovingUnsignedAverage** buffer) {
    *buffer = new MovingUnsignedAverage((1000/RPM_TIMER_INTERVAL_MS)/4, true); // 250ms moving average
    if (!(*buffer)->init_ok()) {
        ESP_LOGE("SENSORS", "Failed to allocate moving average buffer for %s", name);
        return ESP_ERR_NO_MEM;
    }
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
        pcnt_channel_set_edge_action(*CHANNEL_HANDLE, pcnt_channel_edge_action_t::PCNT_CHANNEL_EDGE_ACTION_INCREASE, pcnt_channel_edge_action_t::PCNT_CHANNEL_EDGE_ACTION_INCREASE),
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
    if (VEHICLE_CONFIG.io_0_usage == 1 && VEHICLE_CONFIG.input_sensor_pulses_per_rev != 0) {
        ESP_LOGI("SENSORS", "Will init OUTPUT RPM sensor");
        if (ESP_OK == configure_pcnt("OUTPUT", pcb_gpio_matrix->io_pin, &PCNT_HANDLE_OUTPUT, &PCNT_C_HANDLE_OUTPUT, &out_avg_buffer)) {
            output_rpm_ok = true;
        }
    }

    gptimer_handle_t gptimer_pcnt = NULL;

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = (1 * 1000 * 1000),
        .flags = {
            .intr_shared = 1
        }
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &gptimer_pcnt), "SENSORS", "Failed to create PCNT read timer");

    const gptimer_alarm_config_t alarm_config_pcnt = {
        .alarm_count = (20 * 1000),
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

    return ESP_OK;
}

// 1rpm = 60 pulse/sec -> 1 pulse every 20ms at MINIMUM
esp_err_t Sensors::read_input_rpm(RpmReading *dest, bool check_sanity)
{
    esp_err_t res = ESP_OK;

    dest->n2_raw = n2_avg_buffer->get_average()/2; // /2 as we are counting both high and low edge, so we get 2x the number of pulses as teeth
    dest->n3_raw = n3_avg_buffer->get_average()/2;
    
    float f2 = (float)dest->n2_raw;
    float f3 = (float)dest->n3_raw;
    float c = (f2 * RATIO_2_1) + (f3 - (RATIO_2_1*f3));
    if (c < 0) {
        c = 0;
    }
    dest->calc_rpm = c;

    // If we need to check sanity, check it, in gears 2,3 and 4, RPM readings should be the same,
    // otherwise we have a faulty conductor place sensor!
    if (check_sanity && abs((int)dest->n2_raw - (int)dest->n3_raw) > 150) {
        res = ESP_ERR_INVALID_STATE;
    }
    return res;
}

void Sensors::set_ratio_2_1(float r) {
    RATIO_2_1 = r;
}

esp_err_t Sensors::read_output_rpm(uint16_t* dest) {
    esp_err_t res = ESP_OK;
    if (output_rpm_ok == false) {
        res = ESP_ERR_INVALID_STATE;
    } else {
        uint16_t rpm = 0;
        //uint64_t pulses_per_min = 60*(output_total*(50/2))/RPM_SAMPLES_DEBOUNCE;
        //rpm = pulses_per_min / VEHICLE_CONFIG.input_sensor_pulses_per_rev;
        *dest = rpm;
    }
    return res;
}

esp_err_t Sensors::read_vbatt(uint16_t *dest){
    int v = 0;
    int read = 0;
    esp_err_t res = adc_oneshot_read(adc2_handle, pcb_gpio_matrix->sensor_data.adc_batt, &read);
    res = adc_cali_raw_to_voltage(adc2_cal, read, &v);
    if (res == ESP_OK) {
        // Vin = Vout(R1+R2)/R2
        *dest = v * 5.54; // 5.54 = (100+22)/22
    }
    return res;
}

// Returns ATF temp in *C
esp_err_t Sensors::read_atf_temp(int16_t *dest)
{
    esp_err_t res;
    int read = 0;
    int voltage = 0;
    res = adc_oneshot_read(adc2_handle, pcb_gpio_matrix->sensor_data.adc_atf, &read);
    res = adc_cali_raw_to_voltage(adc2_cal, read, &voltage);
    if (voltage >= 3000)
    {
        res = ESP_ERR_INVALID_STATE;
    } else {
        res = ESP_OK;
        int atf_calc_c = 0;
        const temp_reading_t *atf_temp_lookup = pcb_gpio_matrix->sensor_data.atf_calibration_curve;
        if (voltage <= atf_temp_lookup[0].v)
        {
            atf_calc_c = (int16_t)(atf_temp_lookup[0].temp);
        }
        else if (voltage >= atf_temp_lookup[NUM_TEMP_POINTS - 1].v)
        {
            atf_calc_c = (int16_t)((atf_temp_lookup[NUM_TEMP_POINTS - 1].temp) / 10);
        }
        else
        {
            for (uint8_t i = 0; i < NUM_TEMP_POINTS - 1; i++)
            {
                // Found! Interpolate linearly to get a better estimate of ATF Temp
                if (atf_temp_lookup[i].v <= voltage && atf_temp_lookup[i + 1].v >= voltage)
                {
                    atf_calc_c = scale_number_int(
                        voltage, // Read voltage
                        atf_temp_lookup[i].temp, // Min temp for this range
                        atf_temp_lookup[i+1].temp, // Max temp for this range
                        atf_temp_lookup[i].v, // Min voltage for this boundary
                        atf_temp_lookup[i+1].v // Max voltage for this boundary
                    )/10; // Convert from 1/10th C to C
                    break;
                }
            }
        }
        *dest = atf_calc_c;
    }
    return res;
}

esp_err_t Sensors::parking_lock_engaged(bool *dest)
{
    int raw;
    esp_err_t res = adc_oneshot_read(adc2_handle, pcb_gpio_matrix->sensor_data.adc_atf, &raw);
    if (res == ESP_OK)
    {
        *dest = raw >= 3000;
    }
    return res;
}

void Sensors::set_motor_temperature(int16_t celcius) {
    
}