
#ifndef __SPEAKER_H_
#define __SPEAKER_H_


#include "driver/gpio.h"

class Speaker {
    public:
        explicit Speaker(gpio_num_t pin);
        void send_note(uint32_t freq, uint32_t play_time_ms, uint32_t total_time_ms);
    private:
        void set_freq(uint32_t freq);
        
};

extern Speaker spkr;

#endif // __SPEAKER_H_