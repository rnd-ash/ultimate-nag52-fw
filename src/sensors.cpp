#include <sensors.h>
#include "pwm_channels/channels.h"
#include "driver/pcnt.h"
#include "driver/i2s.h"

#define SAMPLES_PER_REV 8 // measure 8 times per revolution...
#define AVG_SAMPLES 8 // ...and average over 8 samples

const pcnt_unit_t PCNT_N2_RPM = PCNT_UNIT_0; // N2 RPM uses PCNT unit 0
const pcnt_unit_t PCNT_N3_RPM = PCNT_UNIT_1; // N3 RPM uses PCNT unit 1

bool n2_ok = true; // If false DTC is thrown
bool n3_ok = true; // If false DTC is thrown

static uint32_t base_solenoid_readings[6] = {0,0,0,0,0,0}; // Offsets from ADC

uint64_t last_n2_time = 0; // Last time interrupt was called
uint64_t last_n3_time = 0; // Last time interrupt was called

/**
 * To prevent random spikes in RPM, we smooth out the readings
 * for N2 and N3 RPM sensor
 * 
 * This is done by keeping track of the past AVG_SAMPLES RPM measurements
 * in a moving average fashion
 */

uint64_t n2_deltas[AVG_SAMPLES];
uint8_t n2_sample_id = 0;
uint64_t n2_total = 0;

uint64_t n3_deltas[AVG_SAMPLES];
uint8_t n3_sample_id = 0;
uint64_t n3_total = 0;

// Muxes for interrupts
portMUX_TYPE n2_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE n3_mux = portMUX_INITIALIZER_UNLOCKED;

esp_adc_cal_characteristics_t adc1_cal = esp_adc_cal_characteristics_t{};
esp_adc_cal_characteristics_t adc2_cal = esp_adc_cal_characteristics_t{};

uint16_t read_mv = 0;

uint32_t read_pin_mv(ADC_Reading req_pin) {
    uint32_t raw = 0;
    switch (req_pin) {
        case ADC_Reading::Y3:
            raw = adc1_get_raw(ADC_CHANNEL_Y3);
            return esp_adc_cal_raw_to_voltage(raw, &adc1_cal);
        case ADC_Reading::Y4:
            raw = adc1_get_raw(ADC_CHANNEL_Y4);
            return esp_adc_cal_raw_to_voltage(raw, &adc1_cal);
        case ADC_Reading::Y5:
            raw = adc1_get_raw(ADC_CHANNEL_Y5);
            return esp_adc_cal_raw_to_voltage(raw, &adc1_cal);
        case ADC_Reading::MPC:
            raw = adc1_get_raw(ADC_CHANNEL_Y4);
            return esp_adc_cal_raw_to_voltage(raw, &adc2_cal);
        case ADC_Reading::SPC:
            raw = adc1_get_raw(ADC_CHANNEL_Y4);
            return esp_adc_cal_raw_to_voltage(raw, &adc1_cal);
        case ADC_Reading::TCC:
            raw = adc1_get_raw(ADC_CHANNEL_Y4);
            return esp_adc_cal_raw_to_voltage(raw, &adc1_cal);
        case ADC_Reading::ATF:
            adc2_get_raw(ADC_CHANNEL_ATF, adc_bits_width_t::ADC_WIDTH_BIT_12, (int*)&raw);
            return esp_adc_cal_raw_to_voltage(raw, &adc2_cal);
        case ADC_Reading::V_SENSE:
            adc2_get_raw(ADC_CHANNEL_VSENSE, adc_bits_width_t::ADC_WIDTH_BIT_12, (int*)&raw);
            return esp_adc_cal_raw_to_voltage(raw, &adc2_cal);
        default:
            return 0;
    }
}


static void IRAM_ATTR onOverflow(void *args) {
    int unit = (int)args; // PCNT unit we are working with
    uint64_t now = esp_timer_get_time();
    if (unit == PCNT_N2_RPM) { // N2
        portENTER_CRITICAL(&n2_mux);
        n2_total = n2_total - n2_deltas[n2_sample_id];
        uint64_t delta = (now - last_n2_time);
        last_n2_time = now;
        n2_deltas[n2_sample_id] = delta;
        n2_total += delta;
        n2_sample_id = (n2_sample_id+1) % AVG_SAMPLES;
        portEXIT_CRITICAL(&n2_mux);
    } else if (unit == PCNT_N3_RPM) { // N3
        portENTER_CRITICAL(&n3_mux);
        n3_total -= n3_deltas[n3_sample_id];
        uint64_t delta = (now - last_n3_time);
        last_n3_time = now;
        n3_deltas[n3_sample_id] = delta;
        n3_total += delta;
        n3_sample_id = (n3_sample_id+1) % AVG_SAMPLES;
        portEXIT_CRITICAL(&n3_mux);
    }
}

void Sensors::configure_sensor_pins() {
    // Gearbox sensors

    gpio_set_direction(PIN_V_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_ATF_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_N3_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_N2_SENSE, GPIO_MODE_INPUT);

    gpio_set_pull_mode(PIN_N3_SENSE, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_N2_SENSE, GPIO_PULLUP_ONLY);

    // Solenoid current inputs
    gpio_set_direction(PIN_Y3_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_Y4_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_Y5_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_SPC_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_MPC_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_TCC_SENSE, GPIO_MODE_INPUT);



    // Configure ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC_CHANNEL_Y3, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC_CHANNEL_Y4, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC_CHANNEL_Y5, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC_CHANNEL_MPC, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC_CHANNEL_SPC, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC_CHANNEL_TCC, ADC_ATTEN_11db);

    adc2_config_channel_atten(ADC_CHANNEL_VSENSE, ADC_ATTEN_11db);
    adc2_config_channel_atten(ADC_CHANNEL_ATF, ADC_ATTEN_11db);

    /*
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = 20000,
        .bits_per_sample = i2s_bits_per_sample_t::I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 2,
        .dma_buf_len = 1024,
        .use_apll = true,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
    };
    */

    // Read ADC calibration
    esp_adc_cal_value_t type = esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 0, &adc1_cal);
    type = esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 0, &adc2_cal);

    adc1_get_raw(adc1_channel_t::ADC1_CHANNEL_0);

    // Read all solenoids now to get resting current (PWM hasn't started yet!)
    // This is our base measurement
    base_solenoid_readings[(uint8_t)Solenoid::Y3] = read_pin_mv(ADC_Reading::Y3);
    base_solenoid_readings[(uint8_t)Solenoid::Y4] = read_pin_mv(ADC_Reading::Y4);
    base_solenoid_readings[(uint8_t)Solenoid::Y5] = read_pin_mv(ADC_Reading::Y5);
    base_solenoid_readings[(uint8_t)Solenoid::MPC] = read_pin_mv(ADC_Reading::MPC);
    base_solenoid_readings[(uint8_t)Solenoid::SPC] = read_pin_mv(ADC_Reading::SPC);
    base_solenoid_readings[(uint8_t)Solenoid::TCC] = read_pin_mv(ADC_Reading::TCC);

    // Now apply 40% to MPC and SPC
    

    // Now check. According to ESP and testing, we shouldn't see more than 300mV at resting current.
    // If we do, we can assume there is a short somewhere and the MOSFET no longer has control
    // over that solenoid

    const pcnt_config_t pcnt_config_n2 {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PIN_N2_SENSE, // N2 RPM
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No control
        .lctrl_mode = PCNT_MODE_KEEP, // Ignore control
        .hctrl_mode = PCNT_MODE_KEEP,    // Ignore control
        .pos_mode = PCNT_COUNT_DIS,   // Don't count on positive edge
        .neg_mode = PCNT_COUNT_INC,   // Count on calling edge
        .counter_h_lim = N2_PULSES_PER_REV/SAMPLES_PER_REV, // HLim is pulses per rev (60 for N2) / number of samples per rev
        .counter_l_lim = 0,
        .unit = PCNT_N2_RPM,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config_n2);

    const pcnt_config_t pcnt_config_n3 {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PIN_N3_SENSE, // N3 RPM
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No control
        .lctrl_mode = PCNT_MODE_KEEP, // Ignore control
        .hctrl_mode = PCNT_MODE_KEEP, // Ignore control
        .pos_mode = PCNT_COUNT_DIS,   // Don't count on positive edge
        .neg_mode = PCNT_COUNT_INC,   // Count on calling edge
        .counter_h_lim = N3_PULSES_PER_REV/SAMPLES_PER_REV,  // HLim is pulses per rev (60 for N2) / number of samples per rev
        .counter_l_lim = 0,
        .unit = PCNT_N3_RPM,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config_n3);

    // Stop counting and clear. We haven't setup ISR or filter yet
    pcnt_counter_pause(PCNT_N2_RPM);
    pcnt_counter_pause(PCNT_N3_RPM);

    pcnt_counter_clear(PCNT_N2_RPM);
    pcnt_counter_clear(PCNT_N3_RPM);

    // Setup filter and ISR
    pcnt_set_filter_value(PCNT_N2_RPM, 40); // 40us or more counts as a pulse (Works upto 10,000 RPM)
    pcnt_set_filter_value(PCNT_N3_RPM, 40);

    pcnt_filter_enable(PCNT_N2_RPM);
    pcnt_filter_enable(PCNT_N3_RPM);

    pcnt_isr_service_install(0);

    pcnt_isr_handler_add(PCNT_N2_RPM, &onOverflow, (void*)PCNT_N2_RPM);
    pcnt_isr_handler_add(PCNT_N3_RPM, &onOverflow, (void*)PCNT_N3_RPM);

    // Enable interrupt when we hit H Lim of the counters
    pcnt_event_enable(PCNT_N2_RPM, pcnt_evt_type_t::PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_N3_RPM, pcnt_evt_type_t::PCNT_EVT_H_LIM);

    // Resume counting!
    pcnt_counter_resume(PCNT_N2_RPM);
    pcnt_counter_resume(PCNT_N3_RPM);

}


uint16_t Sensors::read_vbatt() {
    return (uint16_t)(read_pin_mv(ADC_Reading::V_SENSE) * 0.5) + 100;
}

uint32_t Sensors::read_n2_rpm() {
    uint64_t now = esp_timer_get_time();

    portENTER_CRITICAL(&n2_mux);
    if (now - last_n2_time > 100000) { // 100ms timeout
        portEXIT_CRITICAL(&n2_mux);
        return 0;
    }

    uint32_t avg = n2_total / AVG_SAMPLES;
    float freq = (1000000 * (N2_PULSES_PER_REV / SAMPLES_PER_REV)) / avg;
    portEXIT_CRITICAL(&n2_mux);
    return freq;
}

uint32_t Sensors::read_n3_rpm() {
    uint64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&n3_mux);
    if (now - last_n3_time > 100000) { // 100ms timeout
        portEXIT_CRITICAL(&n3_mux);
        return 0;
    }

    uint32_t avg = n3_total / AVG_SAMPLES;
    float freq = (1000000 * (N3_PULSES_PER_REV / SAMPLES_PER_REV)) / avg;
    portEXIT_CRITICAL(&n3_mux);
    return freq;
}

uint32_t Sensors::read_solenoid_current(Solenoid sol) {
    uint32_t raw = 0;
    switch(sol) {
        case Solenoid::Y3:
            raw = read_pin_mv(ADC_Reading::Y3);
            break;
        case Solenoid::Y4:
            raw = read_pin_mv(ADC_Reading::Y4);
            break;
        case Solenoid::Y5:
            raw = read_pin_mv(ADC_Reading::Y5);
            break;
        case Solenoid::MPC:
            raw = read_pin_mv(ADC_Reading::MPC);
            break;
        case Solenoid::SPC:
            raw = read_pin_mv(ADC_Reading::SPC);
            break;
        case Solenoid::TCC:
            return 0;
            //return delta_us;
            //return read_mv;
            //break;
        default:
            break;
    }
    return raw;
    // 0-3V = 0-6A with 0.05Ohm resistor, so easy calculation!
    return (raw - base_solenoid_readings[(uint8_t)sol]) * 2;
}

int16_t Sensors::read_atf_temp() {
    return read_pin_mv(ADC_Reading::ATF);
}

bool Sensors::read_park_lock() {
    return read_pin_mv(ADC_Reading::ATF) > 3000;
}

uint32_t Sensors::isr_count = 0;