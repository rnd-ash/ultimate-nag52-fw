#include <sensors.h>
#include "pwm_channels/channels.h"
#include "driver/pcnt.h"

#define SAMPLES_PER_REV 8 // measure 8 times per revolution...
#define AVG_SAMPLES 8 // ...and average over 8 samples

bool n2_ok = true; // If false DTC is thrown
bool n3_ok = true; // If false DTC is thrown

static uint32_t base_solenoid_readings[6] = {0,0,0,0,0,0}; // Offsets from ADC

uint64_t last_n2_time = 0; // Last time interrupt was called
uint64_t last_n3_time = 0; // Last time interrupt was called

uint64_t n2_deltas[AVG_SAMPLES];
uint8_t n2_sample_id = 0;
uint64_t n2_total = 0;

uint64_t n3_deltas[AVG_SAMPLES];
uint8_t n3_sample_id = 0;
uint64_t n3_total = 0;

portMUX_TYPE n2_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE n3_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR onOverflow(void *args) {
    int unit = (int)args;
    uint64_t now = micros();
    if (unit == PCNT_UNIT_0) { // N2
        portENTER_CRITICAL(&n2_mux);
        n2_total = n2_total - n2_deltas[n2_sample_id];
        uint64_t delta = (now - last_n2_time);
        last_n2_time = now;
        n2_deltas[n2_sample_id] = delta;
        n2_total += delta;
        n2_sample_id = (n2_sample_id+1) % AVG_SAMPLES;
        portEXIT_CRITICAL(&n2_mux);
    } else if (unit == PCNT_UNIT_1) { // N3
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
    pinMode(PIN_V_SENSE, INPUT);
    pinMode(PIN_ATF_SENSE, INPUT);
    pinMode(PIN_N3_SENSE, INPUT_PULLUP);
    pinMode(PIN_N2_SENSE, INPUT_PULLUP);

    // Solenoid current inputs
    pinMode(PIN_Y3_SENSE, INPUT);
    pinMode(PIN_Y4_SENSE, INPUT);
    pinMode(PIN_Y5_SENSE, INPUT);
    pinMode(PIN_SPC_SENSE, INPUT);
    pinMode(PIN_MPC_SENSE, INPUT);
    pinMode(PIN_TCC_SENSE, INPUT);

    // Read all solenoids now to get resting current (PWM hasn't started yet!)
    // This is our base measurement
    base_solenoid_readings[(uint8_t)Solenoid::Y3] = analogReadMilliVolts(PIN_Y3_SENSE);
    base_solenoid_readings[(uint8_t)Solenoid::Y4] = analogReadMilliVolts(PIN_Y4_SENSE);
    base_solenoid_readings[(uint8_t)Solenoid::Y5] = analogReadMilliVolts(PIN_Y5_SENSE);
    base_solenoid_readings[(uint8_t)Solenoid::MPC] = analogReadMilliVolts(PIN_MPC_SENSE);
    base_solenoid_readings[(uint8_t)Solenoid::SPC] = analogReadMilliVolts(PIN_SPC_SENSE);
    base_solenoid_readings[(uint8_t)Solenoid::TCC] = analogReadMilliVolts(PIN_TCC_SENSE);

    // Now apply 40% to MPC and SPC
    

    // Now check. According to ESP and testing, we shouldn't see more than 300mV at resting current.
    // If we do, we can assume there is a short somewhere and the MOSFET no longer has control
    // over that solenoid
    //mpc_pwm.write_pwm(128);
    //spc_pwm.write_pwm(102);

    const pcnt_config_t pcnt_config_n2 {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PIN_N2_SENSE, // N2 RPM
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No control
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        .pos_mode = PCNT_COUNT_DIS,   // Don't count on positive edge
        .neg_mode = PCNT_COUNT_INC,   // Count on calling edge
        .counter_h_lim = N2_PULSES_PER_REV/SAMPLES_PER_REV,
        .counter_l_lim = 0,
        .unit = PCNT_UNIT_0, // N2 is unit 0
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config_n2);

    const pcnt_config_t pcnt_config_n3 {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PIN_N3_SENSE, // N2 RPM
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No control
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        .pos_mode = PCNT_COUNT_DIS,   // Don't count on positive edge
        .neg_mode = PCNT_COUNT_INC,   // Count on calling edge
        .counter_h_lim = N3_PULSES_PER_REV/SAMPLES_PER_REV,
        .counter_l_lim = 0,
        .unit = PCNT_UNIT_1, // N3 is unit 1
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config_n3);

    // Stop counting and clear. We haven't setup ISR or filter yet
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    // Setup filter and ISR
    pcnt_set_filter_value(PCNT_UNIT_0, 40); // 40us or more counts as a pulse (Works upto 10,000 RPM)
    pcnt_set_filter_value(PCNT_UNIT_1, 40);

    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_filter_enable(PCNT_UNIT_1);

    pcnt_isr_service_install(0);

    pcnt_isr_handler_add(PCNT_UNIT_0, &onOverflow, (void*)PCNT_UNIT_0);
    pcnt_isr_handler_add(PCNT_UNIT_1, &onOverflow, (void*)PCNT_UNIT_1);


    pcnt_event_enable(PCNT_UNIT_0, pcnt_evt_type_t::PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_1, pcnt_evt_type_t::PCNT_EVT_H_LIM);

    // Resume counting!
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);

}


uint16_t Sensors::read_vbatt() {
    return (uint16_t)(analogRead(PIN_V_SENSE) * 0.4) + 200;
}

uint32_t Sensors::read_n2_rpm() {
    uint64_t now = micros();

    portENTER_CRITICAL(&n2_mux);
    if (now - last_n2_time > 100000) {
        portEXIT_CRITICAL(&n2_mux);
        return 0;
    }

    uint32_t avg = n2_total / AVG_SAMPLES;
    float freq = (1000000 * (N2_PULSES_PER_REV / SAMPLES_PER_REV)) / avg;
    portEXIT_CRITICAL(&n2_mux);
    return freq;
}

uint32_t Sensors::read_n3_rpm() {
    uint64_t now = micros();
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
            raw = analogReadMilliVolts(PIN_Y3_SENSE);
            break;
        case Solenoid::Y4:
            raw = analogReadMilliVolts(PIN_Y4_SENSE);
            break;
        case Solenoid::Y5:
            raw = analogReadMilliVolts(PIN_Y5_SENSE);
            break;
        case Solenoid::MPC:
            raw = analogReadMilliVolts(PIN_MPC_SENSE);
            break;
        case Solenoid::SPC:
            raw = analogReadMilliVolts(PIN_SPC_SENSE);
            break;
        case Solenoid::TCC:
            raw = analogReadMilliVolts(PIN_TCC_SENSE);
            break;
        default:
            break;
    }
    return raw;
    // 0-3V = 0-6A with 0.05Ohm resistor, so easy calculation!
    return (raw - base_solenoid_readings[(uint8_t)sol]) * 2;
}

uint16_t Sensors::read_atf_temp() {
    return analogReadMilliVolts(PIN_ATF_SENSE);
}