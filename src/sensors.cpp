#include "sensors.h"
#include "driver/pcnt.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "esp_check.h"

#define PULSES_PER_REV 60 // N2 and N3 are 60 pulses per revolution
#define PCNT_H_LIM 1

#define LOG_TAG "SENSORS"

const pcnt_unit_t PCNT_N2_RPM = PCNT_UNIT_0;
const pcnt_unit_t PCNT_N3_RPM = PCNT_UNIT_1;
const pcnt_unit_t PCNT_OUT_RPM = PCNT_UNIT_2;

#define ADC2_ATTEN ADC_ATTEN_11db
#define ADC2_WIDTH ADC_WIDTH_12Bit

esp_adc_cal_characteristics_t adc2_cal = {};

portMUX_TYPE n2_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE n3_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE output_mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint64_t n3_intr_times[2] = {0, 0};
volatile uint64_t n2_intr_times[2] = {0, 0};
volatile uint64_t output_intr_times[2] = {0,0};

// For RPM sensor debouncing / smoothing
#define RPM_CHANGE_MAX 1000
#define RPM_SAMPLES_DEBOUNCE 10
#define RPM_TIMER_INTERVAL_MS 20
volatile uint64_t n3_avgs[RPM_SAMPLES_DEBOUNCE] = {0,0,0,0,0,0,0,0,0,0};
volatile uint64_t n3_total = 0;
volatile uint64_t n2_avgs[RPM_SAMPLES_DEBOUNCE] = {0,0,0,0,0,0,0,0,0,0};
volatile uint64_t n2_total = 0;
volatile uint64_t output_avgs[RPM_SAMPLES_DEBOUNCE] = {0,0,0,0,0,0,0,0,0,0};
volatile uint64_t output_total = 0;
uint8_t avg_n2_idx = 0;
uint8_t avg_n3_idx = 0;
uint8_t avg_out_idx = 0;


uint64_t t_n2 = 0;
uint64_t t_n3 = 0;
uint64_t t_output = 0;
bool output_rpm_ok = false;

static void IRAM_ATTR on_pcnt_overflow_n2(void *args)
{
    t_n2 = esp_timer_get_time();
    if (t_n2 - n2_intr_times[1] > 10)
    {
        portENTER_CRITICAL_ISR(&n2_mux);
        n2_intr_times[0] = n2_intr_times[1];
        n2_intr_times[1] = t_n2;
        portEXIT_CRITICAL_ISR(&n2_mux);
    }
    
}

static void IRAM_ATTR on_pcnt_overflow_n3(void *args)
{
    t_n3 = esp_timer_get_time();
    if (t_n3 - n3_intr_times[1] > 10)
    {
        portENTER_CRITICAL_ISR(&n3_mux);
        n3_intr_times[0] = n3_intr_times[1];
        n3_intr_times[1] = t_n3;
        portEXIT_CRITICAL_ISR(&n3_mux);
    }
}

static void IRAM_ATTR on_pcnt_overflow_output(void* args) {
    t_output = esp_timer_get_time();
    if (t_output - output_intr_times[1] > 10) {
        portENTER_CRITICAL_ISR(&output_mux);
        output_intr_times[0] = output_intr_times[1];
        output_intr_times[1] = t_output;
        portEXIT_CRITICAL_ISR(&output_mux);
    }
}

static intr_handle_t rpm_timer_handle;
static void IRAM_ATTR on_rpm_timer(void* args) {
    // Clear timer interrupt
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;
    uint64_t time_delta_n2 = 0;
    uint64_t time_delta_n3 = 0;
    uint64_t time_delta_output = 0;
    portENTER_CRITICAL_ISR(&n2_mux);
    time_delta_n2 = n2_intr_times[1] - n2_intr_times[0];
    portEXIT_CRITICAL_ISR(&n2_mux);
    portENTER_CRITICAL_ISR(&n3_mux);
    time_delta_n3 = n3_intr_times[1] - n3_intr_times[0];
    portEXIT_CRITICAL_ISR(&n3_mux);
    portENTER_CRITICAL_ISR(&output_mux);
    time_delta_output = output_intr_times[1] - output_intr_times[0];
    portEXIT_CRITICAL_ISR(&output_mux);

    if (time_delta_n3 != 0) {
        n3_total -= n3_avgs[avg_n3_idx];
        n3_avgs[avg_n3_idx] = time_delta_n3;
        n3_total += time_delta_n3;
        avg_n3_idx = (avg_n3_idx+1)%RPM_SAMPLES_DEBOUNCE;
    }

    if (time_delta_n2 != 0) {
        n2_total -= n2_avgs[avg_n2_idx];
        n2_avgs[avg_n2_idx] = time_delta_n2;
        n2_total += time_delta_n2;
        avg_n2_idx = (avg_n2_idx+1)%RPM_SAMPLES_DEBOUNCE;
    }

    if (time_delta_output != 0) {
        output_total -= output_avgs[avg_out_idx];
        output_avgs[avg_out_idx] = time_delta_output;
        output_total += time_delta_output;
        avg_out_idx = (avg_out_idx+1)%RPM_SAMPLES_DEBOUNCE;
    }
}

esp_err_t Sensors::init_sensors(){
    const pcnt_config_t pcnt_cfg_n2{
        .pulse_gpio_num = pcb_gpio_matrix->n2_pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = PCNT_H_LIM,
        .counter_l_lim = 0,
        .unit = PCNT_N2_RPM,
        .channel = PCNT_CHANNEL_0};

    const pcnt_config_t pcnt_cfg_n3{
        .pulse_gpio_num = pcb_gpio_matrix->n3_pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = PCNT_H_LIM,
        .counter_l_lim = 0,
        .unit = PCNT_N3_RPM,
        .channel = PCNT_CHANNEL_0};


    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->vsense_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_VBATT to Input!");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->atf_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_ATF to Input!");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->n2_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_N2 to Input!");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pcb_gpio_matrix->n3_pin, GPIO_MODE_INPUT), "SENSORS", "Failed to set PIN_N3 to Input!");
    // Set RPM pins to pullup
    ESP_RETURN_ON_ERROR(gpio_set_pull_mode(pcb_gpio_matrix->n2_pin, GPIO_PULLUP_ONLY), "SENSORS", "Failed to set PIN_N2 to Input!");
    ESP_RETURN_ON_ERROR(gpio_set_pull_mode(pcb_gpio_matrix->n3_pin, GPIO_PULLUP_ONLY), "SENSORS", "Failed to set PIN_N3 to Input!");

    // Configure ADC2 for analog readings
    ESP_RETURN_ON_ERROR(adc2_config_channel_atten(pcb_gpio_matrix->sensor_data.atf_channel, ADC_ATTEN_11db), "SENSORS", "Failed to set ADC attenuation for PIN_ATF!");
    ESP_RETURN_ON_ERROR(adc2_config_channel_atten(pcb_gpio_matrix->sensor_data.batt_channel, ADC_ATTEN_11db), "SENSORS", "Failed to set ADC attenuation for PIN_VBATT!");
    // Characterise ADC2       
    esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_2, adc_atten_t::ADC_ATTEN_DB_11, ADC2_WIDTH, 0, &adc2_cal);
    
    // Now configure PCNT to begin counting!
    ESP_RETURN_ON_ERROR(pcnt_unit_config(&pcnt_cfg_n2), "SENSORS", "Failed to configure PCNT for N2 RPM reading!");
    ESP_RETURN_ON_ERROR(pcnt_unit_config(&pcnt_cfg_n3), "SENSORS", "Failed to configure PCNT for N3 RPM reading!");

    // Pause PCNTs unit configuration is complete
    ESP_RETURN_ON_ERROR(pcnt_counter_pause(PCNT_N2_RPM), "SENSORS", "Failed to pause PCNT N2 RPM!");
    ESP_RETURN_ON_ERROR(pcnt_counter_pause(PCNT_N3_RPM), "SENSORS", "Failed to pause PCNT N3 RPM!");
                    
                        // Clear their stored values (If present)
    ESP_RETURN_ON_ERROR(pcnt_counter_clear(PCNT_N2_RPM), "SENSORS", "Failed to clear PCNT N2 RPM!");
    ESP_RETURN_ON_ERROR(pcnt_counter_clear(PCNT_N3_RPM), "SENSORS", "Failed to clear PCNT N3 RPM!");

    // Setup filter to ignore ultra short pulses (possibly noise)
    // Using a value of 40 at 80Mhz APB_CLOCK = this will correctly filter noise up to 30,000RPM
    ESP_RETURN_ON_ERROR(pcnt_set_filter_value(PCNT_N2_RPM, 1000), "SENSORS", "Failed to set filter for PCNT N2!");
    ESP_RETURN_ON_ERROR(pcnt_set_filter_value(PCNT_N3_RPM, 1000), "SENSORS", "Failed to set filter for PCNT N3!");
    ESP_RETURN_ON_ERROR(pcnt_filter_enable(PCNT_N2_RPM), "SENSORS", "Failed to enable filter for PCNT N2!");
    ESP_RETURN_ON_ERROR(pcnt_filter_enable(PCNT_N3_RPM), "SENSORS", "Failed to enable filter for PCNT N3!");

    // Now install and register ISR interrupts
    ESP_RETURN_ON_ERROR(pcnt_isr_service_install(0), "SENSORS", "Failed to install ISR service for PCNT!");
    ESP_RETURN_ON_ERROR(pcnt_isr_handler_add(PCNT_N2_RPM, &on_pcnt_overflow_n2, nullptr), "SENSORS", "Failed to add PCNT N2 to ISR handler!");
    ESP_RETURN_ON_ERROR(pcnt_isr_handler_add(PCNT_N3_RPM, &on_pcnt_overflow_n3, nullptr), "SENSORS", "Failed to add PCNT N3 to ISR handler!");

    // Enable interrupts on hitting hlim on PCNTs
    ESP_RETURN_ON_ERROR(pcnt_event_enable(PCNT_N2_RPM, pcnt_evt_type_t::PCNT_EVT_H_LIM), "SENSORS", "Failed to register event for PCNT N2!");
    ESP_RETURN_ON_ERROR(pcnt_event_enable(PCNT_N3_RPM, pcnt_evt_type_t::PCNT_EVT_H_LIM), "SENSORS", "Failed to register event for PCNT N3!");

    // Resume counting
    ESP_RETURN_ON_ERROR(pcnt_counter_resume(PCNT_N2_RPM), "SENSORS", "Failed to resume PCNT N2 RPM!");
    ESP_RETURN_ON_ERROR(pcnt_counter_resume(PCNT_N3_RPM), "SENSORS", "Failed to resume PCNT N3 RPM!");
    
    // Enable output RPM reading if needed
    if (VEHICLE_CONFIG.io_0_usage == 1 && VEHICLE_CONFIG.input_sensor_pulses_per_rev != 0) {
        ESP_LOGI("SENSORS", "Will init OUTPUT RPM sensor");
        const pcnt_config_t pcnt_cfg_output {
            .pulse_gpio_num = pcb_gpio_matrix->io_pin,
            .ctrl_gpio_num = PCNT_PIN_NOT_USED,
            .lctrl_mode = PCNT_MODE_KEEP,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = PCNT_COUNT_INC,
            .neg_mode = PCNT_COUNT_DIS,
            .counter_h_lim = PCNT_H_LIM,
            .counter_l_lim = 0,
            .unit = PCNT_OUT_RPM,
            .channel = PCNT_CHANNEL_0
        };
        ESP_RETURN_ON_ERROR(gpio_set_pull_mode(pcb_gpio_matrix->io_pin, GPIO_PULLUP_ONLY), "SENSORS", "Failed to set PIN_IO to Input!");
        ESP_RETURN_ON_ERROR(pcnt_unit_config(&pcnt_cfg_output), "SENSORS", "Failed to configure PCNT for OUTPUT RPM reading!");
        ESP_RETURN_ON_ERROR(pcnt_counter_clear(PCNT_OUT_RPM), "SENSORS", "Failed to clear PCNT OUTPUT RPM!");
        ESP_RETURN_ON_ERROR(pcnt_set_filter_value(PCNT_OUT_RPM, 1000), "SENSORS", "Failed to set filter for PCNT OUTPUT!");
        ESP_RETURN_ON_ERROR(pcnt_filter_enable(PCNT_OUT_RPM), "SENSORS", "Failed to enable filter for PCNT OUTPUT!");
        ESP_RETURN_ON_ERROR(pcnt_isr_handler_add(PCNT_OUT_RPM, &on_pcnt_overflow_output, nullptr), "SENSORS", "Failed to add PCNT OUTPUT to ISR handler!");
        ESP_RETURN_ON_ERROR(pcnt_event_enable(PCNT_OUT_RPM, pcnt_evt_type_t::PCNT_EVT_H_LIM), "SENSORS", "Failed to register event for PCNT OUTPUT!");
        ESP_RETURN_ON_ERROR(pcnt_counter_resume(PCNT_OUT_RPM), "SENSORS", "Failed to resume PCNT OUTPUT RPM!");
        output_rpm_ok = true;
    }

    timer_config_t config = {
            .alarm_en = timer_alarm_t::TIMER_ALARM_EN,
            .counter_en = timer_start_t::TIMER_START,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN,
            .divider = 80   /* 1 us per tick */
    };


    ESP_RETURN_ON_ERROR(timer_init(TIMER_GROUP_0, TIMER_0, &config), "SENSORS", "Failed to init Timer");
    ESP_RETURN_ON_ERROR(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0), "SENSORS", "Failed to set counter value");
    ESP_RETURN_ON_ERROR(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, RPM_TIMER_INTERVAL_MS*1000), "SENSORS", "Failed to set alarm value"); // 20ms intervals
    ESP_RETURN_ON_ERROR(timer_enable_intr(TIMER_GROUP_0, TIMER_0), "SENSORS", "Failed to enable timer interrupt");
    ESP_RETURN_ON_ERROR(timer_isr_register(TIMER_GROUP_0, TIMER_0, &on_rpm_timer, NULL, 0, &rpm_timer_handle), "SENSORS", "Failed to register timer ISR");
    ESP_RETURN_ON_ERROR(timer_start(TIMER_GROUP_0, TIMER_0), "SENSORS", "Failed to start timer");
    ESP_LOG_LEVEL(ESP_LOG_INFO, LOG_TAG, "Sensors INIT OK!");
    return ESP_OK;
}

// 1rpm = 60 pulse/sec -> 1 pulse every 20ms at MINIMUM
esp_err_t Sensors::read_input_rpm(RpmReading *dest, bool check_sanity)
{
    uint64_t d_n2 = n2_total/RPM_SAMPLES_DEBOUNCE;
    uint64_t d_n3 = n3_total/RPM_SAMPLES_DEBOUNCE;
    uint64_t now = esp_timer_get_time();
    esp_err_t res = ESP_OK;

    dest->n2_raw = (now - n2_intr_times[1] > RPM_TIMER_INTERVAL_MS*5000 || d_n2 == 0) ? 0 : 1000000 / d_n2;
    dest->n3_raw = (now - n3_intr_times[1] > RPM_TIMER_INTERVAL_MS*5000 || d_n3 == 0) ? 0 : 1000000 / d_n3;
    
    if (dest->n2_raw < 60 && dest->n3_raw < 60)
    { // Stationary ( < 1rpm ), break here to avoid divideBy0Ex, and also noise
        dest->calc_rpm = 0;
    }
    else if (dest->n2_raw == 0)
    { // In gears R1 or R2 (as N2 is 0)
        dest->calc_rpm = dest->n3_raw;
    }
    else
    {
        if (abs((int)dest->n2_raw - (int)dest->n3_raw) < 10)
        {
            dest->calc_rpm = (dest->n2_raw + dest->n3_raw) / 2;
        } else {
            // More difficult calculation for all forward gears
            // This calculation works when both RPM sensors are the same (Gears 2,3,4)
            // Or when N3 is 0 and N2 is reporting ~0.61x real Rpm (Gears 1 and 5)
            // Also nicely handles transitionary phases between RPM readings, making gear shift RPM readings
            // a lot more accurate for the rest of the TCM code

            float f2 = (float)dest->n2_raw;
            float f3 = (float)dest->n3_raw;
            float ratio = f3 / f2;

            dest->calc_rpm = ((f2 * 1.64f) * (1.0f - ratio)) + (f3 * ratio);

            // If we need to check sanity, check it, in gears 2,3 and 4, RPM readings should be the same,
            // otherwise we have a faulty conductor place sensor!
            if (check_sanity && abs((int)dest->n2_raw - (int)dest->n3_raw) > 150) {
                res = ESP_ERR_INVALID_STATE;
            }
        }
    }
    return res;
}

esp_err_t Sensors::read_output_rpm(uint16_t* dest) {
    esp_err_t res = ESP_OK;
    if (output_rpm_ok == false) {
        res = ESP_ERR_INVALID_STATE;
    } else {
        uint16_t rpm = 0;
        uint64_t d_output = output_total/RPM_SAMPLES_DEBOUNCE;
        uint64_t now = esp_timer_get_time();
        rpm = (now - output_intr_times[1] > RPM_TIMER_INTERVAL_MS*10000) ? 0 : 1000000 / d_output;
        rpm /= VEHICLE_CONFIG.input_sensor_pulses_per_rev;
        *dest = rpm;
    }
    return res;
}

esp_err_t Sensors::read_vbatt(uint16_t *dest){
    uint32_t v;
    esp_err_t res = esp_adc_cal_get_voltage(pcb_gpio_matrix->sensor_data.adc_batt, &adc2_cal, &v);
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
    uint32_t avg = 0;
    float ATF_TEMP_CORR = BOARD_CONFIG.board_ver >= 2 ? 1.0 : 0.8;
    if (BOARD_CONFIG.board_ver >= 2)
    {
        res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_7, &adc2_cal, &avg);
    }
    else
    {
        res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_9, &adc2_cal, &avg);
    }

    if (avg >= 3000)
    {
        res = ESP_ERR_INVALID_STATE; // Parking lock engaged, cannot read.
    }
    if (res == ESP_OK) {
        const temp_reading_t *atf_temp_lookup = pcb_gpio_matrix->sensor_data.atf_calibration_curve;
        if (avg <= atf_temp_lookup[0].v)
        {
            *dest = (int16_t)((float)atf_temp_lookup[0].temp * ATF_TEMP_CORR);
        }
        else if (avg >= atf_temp_lookup[NUM_TEMP_POINTS - 1].v)
        {
            *dest = (int16_t)(ATF_TEMP_CORR * (float)(atf_temp_lookup[NUM_TEMP_POINTS - 1].temp) / 10.0);
        }
        else
        {
            for (uint8_t i = 0; i < NUM_TEMP_POINTS - 1; i++)
            {
                // Found! Interpolate linearly to get a better estimate of ATF Temp
                if (atf_temp_lookup[i].v <= avg && atf_temp_lookup[i + 1].v >= avg)
                {
                    float dx = avg - atf_temp_lookup[i].v;
                    float dy = atf_temp_lookup[i + 1].v - atf_temp_lookup[i].v;
                    *dest = (int16_t)(ATF_TEMP_CORR * (atf_temp_lookup[i].temp + (atf_temp_lookup[i + 1].temp - atf_temp_lookup[i].temp) * ((dx) / dy)) / 10.0);
                    break;
                }
            }
        }
    }
    return res;
}

esp_err_t Sensors::parking_lock_engaged(bool *dest)
{
    uint32_t raw;
    esp_err_t res = esp_adc_cal_get_voltage(pcb_gpio_matrix->sensor_data.adc_atf, &adc2_cal, &raw);
    if (res == ESP_OK)
    {
        *dest = raw >= 3000;
    }
    return res;
}