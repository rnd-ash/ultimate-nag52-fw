
#ifndef __SPEAKER_H_
#define __SPEAKER_H_


#include "driver/gpio.h"

class Speaker {
    public:
        Speaker(gpio_num_t pin);
        void set_freq(uint32_t freq);
        
};

extern Speaker spkr;

#endif // __SPEAKER_H_