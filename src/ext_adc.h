#ifndef EXT_ADC_H
#define EXT_ADC_H

#include <stdint.h>
#include "esp_err.h"
#include <driver/gpio.h>

class ExternalADC {
public:
	ExternalADC(gpio_num_t sda, gpio_num_t scl);
private:
	esp_err_t init_status;
};

extern ExternalADC* ext_adc;
#endif