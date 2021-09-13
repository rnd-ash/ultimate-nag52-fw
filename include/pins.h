#ifndef PINS_H_
#define PINS_H_

#include <pins_arduino.h>

#define PIN_CAN_TX GPIO_NUM_5
#define PIN_CAN_RX GPIO_NUM_4

#define PIN_SPKR GPIO_NUM_15
#define PIN_5V_ENABLE GPIO_NUM_2

// Sensors
#define PIN_V_SENSE GPIO_NUM_36 // Voltage monitor
#define PIN_ATF_SENSE GPIO_NUM_39 // ATF / lockout sensor
#define PIN_N3_SENSE GPIO_NUM_34 // N3 input RPM
#define PIN_N2_SENSE GPIO_NUM_35 // N2 input RPM

// Solenoid current feedback
#define PIN_Y3_SENSE GPIO_NUM_32
#define PIN_Y4_SENSE GPIO_NUM_33
#define PIN_Y5_SENSE GPIO_NUM_26
#define PIN_MPC_SENSE GPIO_NUM_25
#define PIN_SPC_SENSE GPIO_NUM_27
#define PIN_TCC_SENSE GPIO_NUM_14

// Solenoid PWM outputs
#define PIN_Y3_PWM GPIO_NUM_23
#define PIN_Y4_PWM GPIO_NUM_22
#define PIN_Y5_PWM GPIO_NUM_19
#define PIN_MPC_PWM GPIO_NUM_21
#define PIN_SPC_PWM GPIO_NUM_12
#define PIN_TCC_PWM GPIO_NUM_13


#endif // PINS_H_