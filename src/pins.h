#ifndef __PINS_H_
#define __PINS_H_

#include <driver/gpio.h>

class BoardGpioMatrix {
public:
    BoardGpioMatrix();
    gpio_num_t can_tx_pin;
    gpio_num_t can_rx_pin;

    gpio_num_t spkr_pin;

    gpio_num_t vsense_pin;
    gpio_num_t atf_pin;
    gpio_num_t n3_pin;
    gpio_num_t n2_pin;
    
    gpio_num_t io_pin; // Only on 1.3 and newer

    gpio_num_t y3_sense;
    gpio_num_t y3_pwm;

    gpio_num_t y4_sense;
    gpio_num_t y4_pwm;

    gpio_num_t y5_sense;
    gpio_num_t y5_pwm;

    gpio_num_t mpc_sense;
    gpio_num_t mpc_pwm;

    gpio_num_t spc_sense;
    gpio_num_t spc_pwm;

    gpio_num_t tcc_sense;
    gpio_num_t tcc_pwm;

    gpio_num_t i2c_sda; // Only on 1.2 and newer
    gpio_num_t i2c_scl; // Only on 1.2 and newer
};

/**
 * @brief GPIO Matrix for the Red beta PCB (V1.1 12/12/21)
 * 
 * NOTE: This will be removed from the code base once all beta boards have been replaced
 * 
 */
class BoardV11GpioMatrix: public BoardGpioMatrix {
public:
    BoardV11GpioMatrix();
};

/**
 * @brief GPIO Matrix for the Gen 1 production PCB (V1.2 07/07/22)
 * 
 */
class BoardV12GpioMatrix: public BoardGpioMatrix {
public:
    BoardV12GpioMatrix();
};

/**
 * @brief GPIO Matrix for the Gen 2 production PCB (V1.3 29/11/22)
 * NOTE: This PCB is in development, so its matrix can change
 */
class BoardV13GpioMatrix: public BoardGpioMatrix {
public:
    BoardV13GpioMatrix();
};


extern BoardGpioMatrix* pcb_gpio_matrix;

#endif