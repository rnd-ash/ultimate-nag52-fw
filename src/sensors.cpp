#include <sensors.h>
#include "pwm_channels/channels.h"
#include "driver/pcnt.h"

#define TURN_DIVIDER 4

volatile uint16_t n2_rpm = 0;
volatile uint16_t n3_rpm = 0;

volatile uint16_t n2_pulses = 0;
volatile uint16_t n3_pulses = 0;

volatile uint64_t last_quater_revolution_n2 = micros();
volatile uint64_t last_quater_revolution_n3 = micros();


bool n2_ok = true; // If false DTC is thrown
bool n3_ok = true; // If false DTC is thrown

uint32_t __read_n3(); // Forward decleration
uint32_t __read_n2();

static uint32_t base_solenoid_readings[6] = {0,0,0,0,0,0}; // Offsets from ADC

static void IRAM_ATTR rpm_interrupt_n2(void *arg) {
    n2_pulses += 1;
    last_quater_revolution_n2 = micros();
}

static void IRAM_ATTR rpm_interrupt_n3(void *arg) {
    n3_pulses += 1;
    last_quater_revolution_n3 = micros();
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
        .counter_h_lim = N2_PULSES_PER_REV/10, // interrupt will fire on 1/10'th rev
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
        .counter_h_lim = N2_PULSES_PER_REV/10, // interrupt will fire on 1/2 revolution
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

    pcnt_event_enable(PCNT_UNIT_0, pcnt_evt_type_t::PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_1, pcnt_evt_type_t::PCNT_EVT_H_LIM);

    pcnt_isr_service_install(0);

    pcnt_isr_handler_add(PCNT_UNIT_0, &rpm_interrupt_n2, nullptr);
    pcnt_isr_handler_add(PCNT_UNIT_1, &rpm_interrupt_n3, nullptr);

    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_intr_enable(PCNT_UNIT_1);
    // Resume counting!
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);

}


uint16_t Sensors::read_vbatt() {
    return (uint16_t)(analogRead(PIN_V_SENSE) * 0.4) + 200;
}

uint64_t last_measure_n2 = micros();
uint16_t Sensors::read_n2_rpm() {
    uint64_t gap = micros() - last_measure_n2;
    uint64_t gap_signal = micros() - last_quater_revolution_n2;
    if (gap_signal > 500000) {
        n2_rpm = 0;
        return 0;
    }

    if (gap < 20000) {
        return n2_rpm; // Too fast
    }

    // How much of 1 second has it been
    float gap_as_second = 1000000.f / (float)gap;
    uint16_t rpm_est = n2_pulses*(N3_PULSES_PER_REV/10)*gap_as_second;
    last_measure_n2 = micros();
    n2_pulses = 0;
    n2_rpm = rpm_est;
    return n2_rpm;
}


uint64_t last_measure_n3 = micros();
uint16_t Sensors::read_n3_rpm() {
    uint64_t gap = micros() - last_measure_n3;
    uint64_t gap_signal = micros() - last_quater_revolution_n3;
    if (gap_signal > 500000) {
        n3_rpm = 0;
        return 0;
    }

    if (gap < 20000) {
        return n3_rpm; // Too fast
    }

    // How much of 1 second has it been
    float gap_as_second = 1000000.f / (float)gap;
    uint16_t rpm_est = n3_pulses*(N2_PULSES_PER_REV/10)*gap_as_second;
    last_measure_n3 = micros();
    n3_pulses = 0;
    n3_rpm = rpm_est;
    return n3_rpm;
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

uint32_t __read_n2() {
    uint32_t tmp = n2_rpm;
    n2_rpm = 0;
    return tmp;
}

uint32_t __read_n3() {
    uint32_t tmp = n3_rpm;
    n3_rpm = 0;
    return tmp;
}