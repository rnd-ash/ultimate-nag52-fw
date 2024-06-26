
#ifndef SPEAKER_H
#define SPEAKER_H

#include "driver/gpio.h"
#include "dtcs.h"
#include "esp_log.h"

enum class SPEAKER_POST_CODE {
    INIT_OK, // short short
    EEPROM_FAIL,            // long long long long
    CAN_FAIL,               // long long short long
    SOLENOID_FAIL,          // long short long long
    SENSOR_FAIL,            // long short short short
    CONTROLLER_FAIL,        // long short short long
    EFUSE_NOT_SET,          // Long long long short
    CONFIGURATION_MISMATCH, // long short long short
    CALIBRATION_FAIL,       // short shot long long
};

enum class ToneLength {
    Short,
    Long
};

class Speaker {
    public:
        explicit Speaker(gpio_num_t pin);
        void send_note(uint32_t freq, ToneLength tone);
        void post(SPEAKER_POST_CODE code);
    private:
        void set_freq(uint32_t freq);
};

extern Speaker* spkr;

#endif // SPEAKER_H