#include <sensors.h>


volatile uint64_t n2_times[2] = {0, 0}; // {First interrupt, Period Count}
volatile uint64_t n3_times[2] = {0, 0}; // {First interrupt, Period Count}

hw_timer_t* n2_timer = NULL;
hw_timer_t* n3_timer = NULL;

portMUX_TYPE n2_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE n3_mux = portMUX_INITIALIZER_UNLOCKED;

bool n2_ok = true; // If false DTC is thrown
bool n3_ok = true; // If false DTC is thrown

static uint32_t base_solenoid_readings[6] = {0,0,0,0,0,0}; // Offsets from ADC

void IRAM_ATTR n3_interrupt() {
    portENTER_CRITICAL_ISR(&n3_mux);
    uint64_t tmp = timerRead(n3_timer);
    n3_times[1] = tmp - n3_times[0];
    n3_times[0] = tmp;
    portEXIT_CRITICAL_ISR(&n3_mux);
}


void IRAM_ATTR n2_interrupt() {
    portENTER_CRITICAL_ISR(&n2_mux);
    uint64_t tmp = timerRead(n2_timer);
    n2_times[1] = tmp - n2_times[0];
    n2_times[0] = tmp;
    portEXIT_CRITICAL_ISR(&n2_mux);
}


void Sensors::configure_sensor_pins() {
    // Gearbox sensors
    pinMode(PIN_V_SENSE, INPUT);
    pinMode(PIN_ATF_SENSE, INPUT);
    pinMode(PIN_N2_SENSE, INPUT);
    pinMode(PIN_N3_SENSE, INPUT);

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

    // Now check. According to ESP and testing, we shouldn't see more than 300mV at resting current.
    // If we do, we can assume there is a short somewhere and the MOSFET no longer has control
    // over that solenoid



    // Init N2 and N3 HW timers
    n2_timer = timerBegin(0, 80, true);
    n3_timer = timerBegin(1, 80, true);
    timerStart(n2_timer);
    timerStart(n3_timer);

    // Interrupts for N2/3
    attachInterrupt(PIN_N2_SENSE, &n2_interrupt, FALLING);
    attachInterrupt(PIN_N3_SENSE, &n3_interrupt, FALLING);
}


uint16_t Sensors::read_vbatt() {
    return (uint16_t)(analogRead(PIN_V_SENSE) * 0.4) + 200;
}



uint16_t Sensors::read_n2_rpm() {
    if (n2_times[1] == 0) {
        return 0;
    }
    portENTER_CRITICAL(&n2_mux);
    uint16_t freq = 1000000 / n2_times[1];
    n2_times[1] = 0;
    portEXIT_CRITICAL(&n2_mux);
    return (uint16_t)freq;
}

uint16_t Sensors::read_n3_rpm() {
    if (n3_times[1] == 0) {
        return 0;
    }
    portENTER_CRITICAL(&n3_mux);
    uint16_t freq = 1000000 / n3_times[1];
    n3_times[1] = 0;
    portEXIT_CRITICAL(&n3_mux);
    return (uint16_t)freq;
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