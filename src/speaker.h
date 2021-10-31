
#ifndef __SPEAKER_H_
#define __SPEAKER_H_


#include "driver/gpio.h"
#include "dtcs.h"
#include "esp_log.h"

enum class SPEAKER_POST_CODE {
    INIT_OK, // short short
    EEPROM_FAIL, // long long long long
    CAN_FAIL, // long long short long
    SOLENOID_FAIL, // long short long long
    SENSOR_FAIL, // long short short short
    CONTROLLER_FAIL // long short short long
};

class Speaker {
    public:
        explicit Speaker(gpio_num_t pin);
        void send_note(uint32_t freq, uint32_t play_time_ms, uint32_t total_time_ms);
        void post(SPEAKER_POST_CODE code);
    private:
        void set_freq(uint32_t freq);
};

extern Speaker spkr;

#endif // __SPEAKER_H_