#ifndef __PINS_H_
#define __PINS_H_

#include "gearbox_config.h"

/**
 * Pin configuration
 * 
 * Current board. V1.0
 */

#ifdef BOARD_V2
    #define PIN_CAN_TX gpio_num_t::GPIO_NUM_5 // CAN TWAI Tx
    #define PIN_CAN_RX gpio_num_t::GPIO_NUM_18 // CAN TWAI Rx

    #define PIN_SPKR gpio_num_t::GPIO_NUM_4 // Piezo speaker

    #define PIN_VBATT gpio_num_t::GPIO_NUM_25 // Battery voltage feedback
    #define PIN_ATF gpio_num_t::GPIO_NUM_27 // ATF temp sensor and lockout
    #define PIN_N3 gpio_num_t::GPIO_NUM_14 // N3 speed sensor
    #define PIN_N2 gpio_num_t::GPIO_NUM_26 // N2 speed sensor

    #define PIN_Y3_SENSE gpio_num_t::GPIO_NUM_36 // Y3 (1-2/4-5) shift solenoid (Current feedback)
    #define PIN_Y3_PWM gpio_num_t::GPIO_NUM_23 // Y3 (1-2/4-5) shift solenoid (PWM output)

    #define PIN_Y4_SENSE gpio_num_t::GPIO_NUM_39 // Y4 (3-4) shift solenoid (Current feedback)
    #define PIN_Y4_PWM gpio_num_t::GPIO_NUM_22 // Y4 (3-4) shift solenoid (PWM output)

    #define PIN_Y5_SENSE gpio_num_t::GPIO_NUM_35 // Y5 (2-3) shift solenoid (Current feedback)
    #define PIN_Y5_PWM gpio_num_t::GPIO_NUM_19 // Y5 (2-3) shift solenoid (PWM output)

    #define PIN_MPC_SENSE gpio_num_t::GPIO_NUM_34 // Modulating pressure solenoid (Current feedback)
    #define PIN_MPC_PWM gpio_num_t::GPIO_NUM_21 // Modulating pressure solenoid (PWM output)

    #define PIN_SPC_SENSE gpio_num_t::GPIO_NUM_32 // Shift pressure solenoid (Current feedback)
    #define PIN_SPC_PWM gpio_num_t::GPIO_NUM_12 // Shift pressure solenoid (PWM output)

    #define PIN_TCC_SENSE gpio_num_t::GPIO_NUM_33 // Torque converter solenoid(Current feedback)
    #define PIN_TCC_PWM gpio_num_t::GPIO_NUM_13 // Torque converter solenoid (PWM output)

    #define PIN_I2C_SDA gpio_num_t::GPIO_NUM_15 // I2C clock
    #define PIN_I2C_SCL gpio_num_t::GPIO_NUM_2 // I2C data 

#else // Legacy red board

    #define PIN_CAN_TX gpio_num_t::GPIO_NUM_5 // CAN TWAI Tx
    #define PIN_CAN_RX gpio_num_t::GPIO_NUM_4 // CAN TWAI Rx

    #define PIN_SPKR gpio_num_t::GPIO_NUM_15 // Piezo speaker
    #define PIN_5V_EN gpio_num_t::GPIO_NUM_2 // 5V enable circuit for sensors

    #define PIN_VBATT gpio_num_t::GPIO_NUM_25 // Battery voltage feedback
    #define PIN_ATF gpio_num_t::GPIO_NUM_26 // ATF temp sensor and lockout
    #define PIN_N3 gpio_num_t::GPIO_NUM_27 // N3 speed sensor
    #define PIN_N2 gpio_num_t::GPIO_NUM_14 // N2 speed sensor

    #define PIN_Y3_SENSE gpio_num_t::GPIO_NUM_36 // Y3 (1-2/4-5) shift solenoid (Current feedback)
    #define PIN_Y3_PWM gpio_num_t::GPIO_NUM_23 // Y3 (1-2/4-5) shift solenoid (PWM output)

    #define PIN_Y4_SENSE gpio_num_t::GPIO_NUM_39 // Y4 (3-4) shift solenoid (Current feedback)
    #define PIN_Y4_PWM gpio_num_t::GPIO_NUM_22 // Y4 (3-4) shift solenoid (PWM output)

    #define PIN_Y5_SENSE gpio_num_t::GPIO_NUM_35 // Y5 (2-3) shift solenoid (Current feedback)
    #define PIN_Y5_PWM gpio_num_t::GPIO_NUM_19 // Y5 (2-3) shift solenoid (PWM output)

    #define PIN_MPC_SENSE gpio_num_t::GPIO_NUM_34 // Modulating pressure solenoid (Current feedback)
    #define PIN_MPC_PWM gpio_num_t::GPIO_NUM_21 // Modulating pressure solenoid (PWM output)

    #define PIN_SPC_SENSE gpio_num_t::GPIO_NUM_32 // Shift pressure solenoid (Current feedback)
    #define PIN_SPC_PWM gpio_num_t::GPIO_NUM_12 // Shift pressure solenoid (PWM output)

    #define PIN_TCC_SENSE gpio_num_t::GPIO_NUM_33 // Torque converter solenoid(Current feedback)
    #define PIN_TCC_PWM gpio_num_t::GPIO_NUM_13 // Torque converter solenoid (PWM output)

#endif


#endif // __PINS_H_