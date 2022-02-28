#include "sensors.h"
#include "driver/pcnt.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "pins.h"

#define PULSES_PER_REV 60*2 // N2 and N3 are 60 pulses per revolution (NEW: Count on BOTH Rise and fall)
#define SAMPLES_PER_REVOLUTION 20
#define RPM_AVERAGE_SAMPLES 5

#define LOG_TAG "SENSORS"

#define CHECK_ESP_FUNC(x, msg, ...) \
res = x; \
if (res != ESP_OK) { \
    ESP_LOGE(LOG_TAG, msg, ##__VA_ARGS__); \
    return false; \
}   \

const pcnt_unit_t PCNT_N2_RPM = PCNT_UNIT_0;
const pcnt_unit_t PCNT_N3_RPM = PCNT_UNIT_1;

// PCNT configurations
const pcnt_config_t pcnt_cfg_n2 {
    .pulse_gpio_num = PIN_N2,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = PULSES_PER_REV / SAMPLES_PER_REVOLUTION,
    .counter_l_lim = 0,
    .unit = PCNT_N2_RPM,
    .channel = PCNT_CHANNEL_0
};

const pcnt_config_t pcnt_cfg_n3 {
    .pulse_gpio_num = PIN_N3,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = PULSES_PER_REV / SAMPLES_PER_REVOLUTION,
    .counter_l_lim = 0,
    .unit = PCNT_N3_RPM,
    .channel = PCNT_CHANNEL_0
};

struct RpmSampleData {
    uint64_t readings[RPM_AVERAGE_SAMPLES];
    uint8_t sample_id;
    uint64_t sum;
};

void IRAM_ATTR add_value_to_rpm(volatile RpmSampleData* sample, uint64_t reading) {
    sample->sum -= sample->readings[sample->sample_id];
    sample->readings[sample->sample_id] = reading;
    sample->sum += reading;
    sample->sample_id = (sample->sample_id+1) % RPM_AVERAGE_SAMPLES;
}

typedef struct {
    // Voltage in mV
    uint16_t v; 
    // ATF Temp in degrees C * 10
    int temp; 
} temp_reading_t;

#define NUM_TEMP_POINTS 22
const static temp_reading_t atf_temp_lookup[NUM_TEMP_POINTS] = {
//    mV, Temp(x10)
    {446, -400},
    {461, -300},
    {476, -200},
    {491, -100},
    {507, 0},
    {523, 100},
    {540, 200},
    {557, 300},
    {574, 400},
    {592, 500},
    {611, 600},
    {618, 700},
    {649, 800},
    {669, 900},
    {690, 1000},
    {711, 1100},
    {732, 1200},
    {755, 1300},
    {778, 1400},
    {802, 1500},
    {814, 1600},
    {851, 1700}
};

#define ADC_CHANNEL_VBATT adc2_channel_t::ADC2_CHANNEL_8
#define ADC_CHANNEL_ATF adc2_channel_t::ADC2_CHANNEL_9
#define ADC2_ATTEN ADC_ATTEN_11db
#define ADC2_WIDTH ADC_WIDTH_12Bit

#define TIMER_INTERVAL_MS 20 // Every 20ms we poll RPM (Same as other ECUs)
#define PULSE_MULTIPLIER 1000/TIMER_INTERVAL_MS

esp_adc_cal_characteristics_t adc2_cal_atf = {};
esp_adc_cal_characteristics_t adc2_cal_batt = {};

portMUX_TYPE n2_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE n3_mux = portMUX_INITIALIZER_UNLOCKED;

uint64_t n2_pulses;
uint64_t n3_pulses;

volatile RpmSampleData n2_rpm = {
    .readings = {0},
    .sample_id = 0,
    .sum = 0
};
volatile RpmSampleData n3_rpm = {
    .readings = {0},
    .sample_id = 0,
    .sum = 0
};

static intr_handle_t rpm_timer_handle;
static void IRAM_ATTR RPM_TIMER_ISR(void* arg) {
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    portENTER_CRITICAL_ISR(&n3_mux);
    add_value_to_rpm(&n3_rpm, n3_pulses);
    n3_pulses = 0;
    portEXIT_CRITICAL_ISR(&n3_mux);

    portENTER_CRITICAL_ISR(&n2_mux);
    add_value_to_rpm(&n2_rpm, n2_pulses);
    n2_pulses = 0;
    portEXIT_CRITICAL_ISR(&n2_mux);
}

static void IRAM_ATTR on_pcnt_overflow_n2(void* args) {
    portENTER_CRITICAL_ISR(&n2_mux);
    n2_pulses += PULSES_PER_REV/SAMPLES_PER_REVOLUTION;
    portEXIT_CRITICAL_ISR(&n2_mux);
}

static void IRAM_ATTR on_pcnt_overflow_n3(void* args) {
    portENTER_CRITICAL_ISR(&n3_mux);
    n3_pulses += PULSES_PER_REV/SAMPLES_PER_REVOLUTION;
    portEXIT_CRITICAL_ISR(&n3_mux);
}

bool Sensors::init_sensors(){
    esp_err_t res;
    CHECK_ESP_FUNC(gpio_set_direction(PIN_VBATT, GPIO_MODE_INPUT), "Failed to set PIN_VBATT to Input! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(gpio_set_direction(PIN_ATF, GPIO_MODE_INPUT), "Failed to set PIN_ATF to Input! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(gpio_set_direction(PIN_N2, GPIO_MODE_INPUT), "Failed to set PIN_N2 to Input! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(gpio_set_direction(PIN_N3, GPIO_MODE_INPUT), "Failed to set PIN_N3 to Input! %s", esp_err_to_name(res))

    // Set RPM pins to pullup
    CHECK_ESP_FUNC(gpio_set_pull_mode(PIN_N2, GPIO_PULLUP_ONLY), "Failed to set PIN_N2 to Input! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(gpio_set_pull_mode(PIN_N3, GPIO_PULLUP_ONLY), "Failed to set PIN_N3 to Input! %s", esp_err_to_name(res))

    // Configure ADC2 for analog readings
    CHECK_ESP_FUNC(adc2_config_channel_atten(ADC_CHANNEL_VBATT, ADC_ATTEN_11db), "Failed to set ADC attenuation for PIN_ATF! %s", esp_err_to_name(res))
    // Voltage on pin for ATF only goes from 0-838mV(for 170C), so 1100mV Max is OK
    CHECK_ESP_FUNC(adc2_config_channel_atten(ADC_CHANNEL_ATF, ADC_ATTEN_0db), "Failed to set ADC attenuation for PIN_VBATT! %s", esp_err_to_name(res))

    // Characterise ADC2
    esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_2, adc_atten_t::ADC_ATTEN_MAX, ADC2_WIDTH, 0, &adc2_cal_batt);
    esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_2, adc_atten_t::ADC_ATTEN_DB_6, ADC2_WIDTH, 0, &adc2_cal_atf);

    // Now configure PCNT to begin counting!
    CHECK_ESP_FUNC(pcnt_unit_config(&pcnt_cfg_n2), "Failed to configure PCNT for N2 RPM reading! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_unit_config(&pcnt_cfg_n3), "Failed to configure PCNT for N3 RPM reading! %s", esp_err_to_name(res))

    // Pause PCNTs unit configuration is complete
    CHECK_ESP_FUNC(pcnt_counter_pause(PCNT_N2_RPM), "Failed to pause PCNT N2 RPM! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_counter_pause(PCNT_N3_RPM), "Failed to pause PCNT N3 RPM! %s", esp_err_to_name(res))

    // Clear their stored values (If present)
    CHECK_ESP_FUNC(pcnt_counter_clear(PCNT_N2_RPM), "Failed to clear PCNT N2 RPM! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_counter_clear(PCNT_N3_RPM), "Failed to clear PCNT N3 RPM! %s", esp_err_to_name(res))

    // Setup filter to ignore ultra short pulses (possibly noise)
    // Using a value of 40 at 80Mhz APB_CLOCK = this will correctly filter noise up to 30,000RPM 
    CHECK_ESP_FUNC(pcnt_set_filter_value(PCNT_N2_RPM, 1000), "Failed to set filter for PCNT N2! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_set_filter_value(PCNT_N3_RPM, 1000), "Failed to set filter for PCNT N3! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_filter_enable(PCNT_N2_RPM), "Failed to enable filter for PCNT N2! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_filter_enable(PCNT_N3_RPM), "Failed to enable filter for PCNT N3! %s", esp_err_to_name(res))

    // Now install and register ISR interrupts
    CHECK_ESP_FUNC(pcnt_isr_service_install(0), "Failed to install ISR service for PCNT! %s", esp_err_to_name(res))

    CHECK_ESP_FUNC(pcnt_isr_handler_add(PCNT_N2_RPM, &on_pcnt_overflow_n2, nullptr), "Failed to add PCNT N2 to ISR handler! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_isr_handler_add(PCNT_N3_RPM, &on_pcnt_overflow_n3, nullptr), "Failed to add PCNT N3 to ISR handler! %s", esp_err_to_name(res))

    // Enable interrupts on hitting hlim on PCNTs
    CHECK_ESP_FUNC(pcnt_event_enable(PCNT_N2_RPM, pcnt_evt_type_t::PCNT_EVT_H_LIM), "Failed to register event for PCNT N2! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_event_enable(PCNT_N3_RPM, pcnt_evt_type_t::PCNT_EVT_H_LIM), "Failed to register event for PCNT N3! %s", esp_err_to_name(res))

    // Resume counting
    CHECK_ESP_FUNC(pcnt_counter_resume(PCNT_N2_RPM), "Failed to resume PCNT N2 RPM! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(pcnt_counter_resume(PCNT_N3_RPM), "Failed to resume PCNT N3 RPM! %s", esp_err_to_name(res))

    timer_config_t config = {
            .alarm_en = timer_alarm_t::TIMER_ALARM_EN,
            .counter_en = timer_start_t::TIMER_START,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN,
            .divider = 80   /* 1 us per tick */
    };
    CHECK_ESP_FUNC(timer_init(TIMER_GROUP_0, TIMER_0, &config), "Failed to init timer 0 for PCNT measuring! %s", esp_err_to_name(res))
    CHECK_ESP_FUNC(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0), "%s", esp_err_to_name(res))
    CHECK_ESP_FUNC(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_MS*1000), "%s", esp_err_to_name(res))
    CHECK_ESP_FUNC(timer_enable_intr(TIMER_GROUP_0, TIMER_0), "%s", esp_err_to_name(res))
    CHECK_ESP_FUNC(timer_isr_register(TIMER_GROUP_0, TIMER_0, &RPM_TIMER_ISR, NULL, 0, &rpm_timer_handle), "%s", esp_err_to_name(res))
    CHECK_ESP_FUNC(timer_start(TIMER_GROUP_0, TIMER_0), "%s", esp_err_to_name(res))


    ESP_LOGI(LOG_TAG, "Sensors INIT OK!");
    return true;
}

bool Sensors::read_input_rpm(RpmReading* dest, bool check_sanity) {
    dest->n2_raw = (((float)n2_rpm.sum/2.0f) * (float)PULSE_MULTIPLIER / (float)RPM_AVERAGE_SAMPLES);
    dest->n3_raw = (((float)n3_rpm.sum/2.0f) * (float)PULSE_MULTIPLIER / (float)RPM_AVERAGE_SAMPLES);

    if (dest->n2_raw == 0 && dest->n3_raw == 0) { // Stationary, break here to avoid divideBy0Ex
        dest->calc_rpm = 0;
        return true;
    } else if (dest->n2_raw == 0) { // In gears R1 or R2 (as N2 is 0)
        dest->calc_rpm = dest->n3_raw;
        return true;
    } else {
        if (abs((int)dest->n2_raw - (int)dest->n3_raw) < 50) {
            dest->calc_rpm = (dest->n2_raw+dest->n3_raw)/2;
            return true;
        }
        // More difficult calculation for all forward gears
        // This calculation works when both RPM sensors are the same (Gears 2,3,4)
        // Or when N3 is 0 and N2 is reporting ~0.61x real Rpm (Gears 1 and 5)
        // Also nicely handles transitionary phases between RPM readings, making gear shift RPM readings
        // a lot more accurate for the rest of the TCM code
        
        float ratio = (float)dest->n3_raw/(float)dest->n2_raw;
        float f2 = (float)dest->n2_raw;
        float f3 = (float)dest->n3_raw;

        dest->calc_rpm = ((f2*1.64f)*(1.0f-ratio))+(f3*ratio);

        // If we need to check sanity, check it, in gears 2,3 and 4, RPM readings should be the same,
        // otherwise we have a faulty conductor place sensor!
        return check_sanity ? abs((int)dest->n2_raw - (int)dest->n3_raw) < 150 : true;
    }
}

bool Sensors::read_vbatt(uint16_t *dest){
    uint32_t v;
    esp_err_t res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_8, &adc2_cal_batt, &v);
    if (res != ESP_OK) {
        ESP_LOGW("READ_VBATT", "Failed to query VBATT. %s", esp_err_to_name(res));
        return false;
    } else {
        // Vin = Vout(R1+R2)/R2
        *dest = v*5.54; // 5.54 = (100+22)/22
        return true;
    }
}

// Returns ATF temp in *C
bool Sensors::read_atf_temp(int* dest){
    #define NUM_ATF_SAMPLES 5
    uint32_t raw = 0;
    uint32_t avg = 0;
    for (uint8_t i = 0; i < NUM_ATF_SAMPLES; i++) {
        esp_err_t res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_9, &adc2_cal_atf, &raw);
        if (res != ESP_OK) {
            ESP_LOGW("READ_ATF", "Failed to query ATF temp. %s", esp_err_to_name(res));
            return false;
        }
        if (raw >= 1000) {
            return false; // Parking lock engaged, cannot read.
        }
        avg += raw;
    }
    avg /= NUM_ATF_SAMPLES;
    if (avg < atf_temp_lookup[0].v) {
        *dest = atf_temp_lookup[0].temp;
        return true;
    } else if (avg > atf_temp_lookup[NUM_TEMP_POINTS-1].v) {
        *dest = (atf_temp_lookup[NUM_TEMP_POINTS-1].temp) / 10;
        return true;
    } else {
        for (uint8_t i = 0; i < NUM_TEMP_POINTS-1; i++) {
            // Found! Interpolate linearly to get a better estimate of ATF Temp
            if (atf_temp_lookup[i].v <= avg && atf_temp_lookup[i+1].v >= avg) {
                float dx = avg - atf_temp_lookup[i].v;
                float dy = atf_temp_lookup[i+1].v - atf_temp_lookup[i].v;
                *dest = (atf_temp_lookup[i].temp + (atf_temp_lookup[i+1].temp-atf_temp_lookup[i].temp) * ((dx)/dy)) / 10;
                return true;
            }
        }
        return true;
    }
}

bool Sensors::parking_lock_engaged(bool* dest){
    int raw;
    esp_err_t res = adc2_get_raw(ADC_CHANNEL_ATF, ADC2_WIDTH, &raw);
    if (res != ESP_OK) {
        ESP_LOGW("READ_VBATT", "Failed to query parking lock. %s", esp_err_to_name(res));
        return false;
    } else {
        *dest = raw >= 3900;
        return true;
    }
}