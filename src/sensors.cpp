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
#include "macros.h"

#define PULSES_PER_REV 60 // N2 and N3 are 60 pulses per revolution
#define PCNT_H_LIM 1

#define LOG_TAG "SENSORS"

const pcnt_unit_t PCNT_N2_RPM = PCNT_UNIT_0;
const pcnt_unit_t PCNT_N3_RPM = PCNT_UNIT_1;

// PCNT configurations
const pcnt_config_t pcnt_cfg_n2 {
    .pulse_gpio_num = PIN_N2,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = PCNT_H_LIM,
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
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = PCNT_H_LIM,
    .counter_l_lim = 0,
    .unit = PCNT_N3_RPM,
    .channel = PCNT_CHANNEL_0
};

typedef struct {
    // Voltage in mV
    uint16_t v; 
    // ATF Temp in degrees C * 10
    int temp; 
} temp_reading_t;

#define NUM_TEMP_POINTS 22

#ifdef BOARD_V2

const static temp_reading_t atf_temp_lookup[NUM_TEMP_POINTS] = {
//    mV, Temp(x10)
// mV Values are calibrated on 3.45V rail
// as that is how much the ATF sensor power gets
    {725, -400},
    {784, -300},
    {842, -200},
    {900, -100},
    {957, 0},
    {1013, 100},
    {1068, 200},
    {1123, 300},
    {1177, 400},
    {1230, 500},
    {1281, 600},
    {1332, 700},
    {1384, 800},
    {1438, 900},
    {1488, 1000},
    {1538, 1100},
    {1587, 1200},
    {1636, 1300},
    {1685, 1400},
    {1732, 1500},
    {1779, 1600},
    {1826, 1700}
};

#define ADC_CHANNEL_VBATT adc2_channel_t::ADC2_CHANNEL_8
#define ADC_CHANNEL_ATF adc2_channel_t::ADC2_CHANNEL_7

#else 

const static temp_reading_t atf_temp_lookup[NUM_TEMP_POINTS] = {
//    mV, Temp(x10)
// mV Values are calibrated on 3.45V rail
// as that is how much the ATF sensor power gets
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
    {610, 600},
    {629, 700},
    {648, 800},
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

#endif

#define ADC2_ATTEN ADC_ATTEN_11db
#define ADC2_WIDTH ADC_WIDTH_12Bit

esp_adc_cal_characteristics_t adc2_cal_vbatt = {};
esp_adc_cal_characteristics_t adc2_cal_atf = {};

portMUX_TYPE n2_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE n3_mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint64_t n3_intr_times[2] = {0,0};
volatile uint64_t n2_intr_times[2] = {0,0};

uint64_t t_n2 = 0;
uint64_t t_n3 = 0;

static void IRAM_ATTR on_pcnt_overflow_n2(void* args) {
    t_n2 = esp_timer_get_time();
    if (t_n2 - n2_intr_times[1] < 10) {
        return;
    }
    portENTER_CRITICAL_ISR(&n2_mux);
    n2_intr_times[0] = n2_intr_times[1];
    n2_intr_times[1] = t_n2;
    portEXIT_CRITICAL_ISR(&n2_mux);
}

static void IRAM_ATTR on_pcnt_overflow_n3(void* args) {
    t_n3 = esp_timer_get_time();
    if (t_n3 - n3_intr_times[1] < 10) {
        return;
    }
    portENTER_CRITICAL_ISR(&n3_mux);
    n3_intr_times[0] = n3_intr_times[1];
    n3_intr_times[1] = t_n3;
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
    CHECK_ESP_FUNC(adc2_config_channel_atten(ADC_CHANNEL_ATF, ADC_ATTEN_11db), "Failed to set ADC attenuation for PIN_VBATT! %s", esp_err_to_name(res))

    // Characterise ADC2
    esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_2, adc_atten_t::ADC_ATTEN_DB_11, ADC2_WIDTH, 0, &adc2_cal_atf);
    esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_2, adc_atten_t::ADC_ATTEN_DB_11, ADC2_WIDTH, 0, &adc2_cal_vbatt);

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

    ESP_LOG_LEVEL(ESP_LOG_INFO, LOG_TAG, "Sensors INIT OK!");
    return true;
}

#define US_PER_SECOND 1000000 // Microseconds per pulse per RPM

// 1rpm = 60 pulse/sec -> 1 pulse every 20ms at MINIMUM
bool Sensors::read_input_rpm(RpmReading* dest, bool check_sanity) {
    uint64_t n2[2];
    uint64_t n3[2];
    portENTER_CRITICAL(&n2_mux);
    n2[0] = n2_intr_times[0];
    n2[1] = n2_intr_times[1];
    portEXIT_CRITICAL(&n2_mux);
    portENTER_CRITICAL(&n3_mux);
    n3[0] = n3_intr_times[0];
    n3[1] = n3_intr_times[1];
    portEXIT_CRITICAL(&n3_mux);

    uint64_t d_n2 = n2[1] - n2[0];
    uint64_t d_n3 = n3[1] - n3[0];
    uint64_t now = esp_timer_get_time();
    if (d_n2 == 0 || now - n2_intr_times[1] > 20000) {
        dest->n2_raw = 0;
    } else {
        dest->n2_raw = 1000000 / d_n2;
    }
    if (d_n3 == 0 || now - n3_intr_times[1] > 20000) {
        dest->n3_raw = 0;
    } else {
        dest->n3_raw = 1000000 / d_n3;
    }
    if (dest->n2_raw < 60 && dest->n3_raw < 60) { // Stationary ( < 1rpm ), break here to avoid divideBy0Ex, and also noise
        dest->calc_rpm = 0;
        return true;
    } else if (dest->n2_raw == 0) { // In gears R1 or R2 (as N2 is 0)
        dest->calc_rpm = dest->n3_raw;
        return true;
    } else {
        if (abs((int)dest->n2_raw - (int)dest->n3_raw) < 10) {
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
    if (esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_8, &adc2_cal_vbatt, &v) != ESP_OK) {
        return false;
    } else {
        // Vin = Vout(R1+R2)/R2
        *dest = v*5.54; // 5.54 = (100+22)/22
        return true;
    }
}

// Returns ATF temp in *C
bool Sensors::read_atf_temp(int16_t* dest) {
uint32_t avg = 0;

#ifdef BOARD_V2
    static const float ATF_TEMP_CORR = 1.0;
#else
    uint32_t raw = 0;
    static const float ATF_TEMP_CORR = 0.8;
#endif

#ifdef BOARD_V2
    esp_err_t res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_7, &adc2_cal_atf, &avg);
    if (res != ESP_OK) {
        return false;
    }
#else
    #define NUM_ATF_SAMPLES 5
    for (uint8_t i = 0; i < NUM_ATF_SAMPLES; i++) {
        esp_err_t res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_9, &adc2_cal_atf, &raw);
        if (res != ESP_OK) {
            //ESP_LOG_LEVEL(ESP_LOG_WARN, "READ_ATF", "Failed to query ATF temp. %s", esp_err_to_name(res));
            return false;
        }
        avg += raw;
    }
    avg /= NUM_ATF_SAMPLES;
#endif
    if (avg >= 3000) {
        return false; // Parking lock engaged, cannot read.
    }
    if (avg <= atf_temp_lookup[0].v) {
        *dest = (int16_t)((float)atf_temp_lookup[0].temp * ATF_TEMP_CORR);
        return true;
    } else if (avg >= atf_temp_lookup[NUM_TEMP_POINTS-1].v) {
        *dest = (int16_t)(ATF_TEMP_CORR * (float)(atf_temp_lookup[NUM_TEMP_POINTS-1].temp) / 10.0);
        return true;
    } else {
        for (uint8_t i = 0; i < NUM_TEMP_POINTS-1; i++) {
            // Found! Interpolate linearly to get a better estimate of ATF Temp
            if (atf_temp_lookup[i].v <= avg && atf_temp_lookup[i+1].v >= avg) {
                float dx = avg - atf_temp_lookup[i].v;
                float dy = atf_temp_lookup[i+1].v - atf_temp_lookup[i].v;
                *dest = (int16_t)(ATF_TEMP_CORR * (atf_temp_lookup[i].temp + (atf_temp_lookup[i+1].temp-atf_temp_lookup[i].temp) * ((dx)/dy)) / 10.0);
                return true;
            }
        }
        return true;
    }
}

bool Sensors::parking_lock_engaged(bool* dest){
    uint32_t raw;
#ifdef BOARD_V2
    esp_err_t res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_7, &adc2_cal_atf, &raw);
#else
    esp_err_t res = esp_adc_cal_get_voltage(adc_channel_t::ADC_CHANNEL_9, &adc2_cal_atf, &raw);
#endif
    if (res != ESP_OK) {
        return false;
    } else {
        *dest = raw >= 3000;
        return true;
    }
}