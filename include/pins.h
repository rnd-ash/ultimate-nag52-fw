#ifndef PINS_H_
#define PINS_H_

#define PIN_CAN_TX gpio_num_t::GPIO_NUM_5
#define PIN_CAN_RX gpio_num_t::GPIO_NUM_4

#define PIN_SPKR gpio_num_t::GPIO_NUM_15
#define PIN_5V_ENABLE gpio_num_t::GPIO_NUM_2

// Sensors
#define PIN_V_SENSE gpio_num_t::GPIO_NUM_25 // Voltage monitor
#define PIN_ATF_SENSE gpio_num_t::GPIO_NUM_26 // ATF / lockout sensor
#define PIN_N3_SENSE gpio_num_t::GPIO_NUM_27 // N3 input RPM
#define PIN_N2_SENSE gpio_num_t::GPIO_NUM_14 // N2 input RPM

// Solenoid current feedback
#define PIN_Y3_SENSE gpio_num_t::GPIO_NUM_36
#define PIN_Y4_SENSE gpio_num_t::GPIO_NUM_39
#define PIN_Y5_SENSE gpio_num_t::GPIO_NUM_35
#define PIN_MPC_SENSE gpio_num_t::GPIO_NUM_34
#define PIN_SPC_SENSE gpio_num_t::GPIO_NUM_32
#define PIN_TCC_SENSE gpio_num_t::GPIO_NUM_33

// Solenoid PWM outputs
#define PIN_Y3_PWM gpio_num_t::GPIO_NUM_23  
#define PIN_Y4_PWM gpio_num_t::GPIO_NUM_22
#define PIN_Y5_PWM gpio_num_t::GPIO_NUM_19
#define PIN_MPC_PWM gpio_num_t::GPIO_NUM_21
#define PIN_SPC_PWM gpio_num_t::GPIO_NUM_12
#define PIN_TCC_PWM gpio_num_t::GPIO_NUM_13


#endif // PINS_H_