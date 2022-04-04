#ifndef __EGS_EMULATION_H_
#define __EGS_EMULATION_H_

/// Data structures for EGS52 emulation in DAS

#include <stdint.h>
#include "canbus/can_hal.h"

#define RLI_30 0x30
#define RLI_31 0x31 // 7
#define RLI_35 0x35 // 16

typedef struct {
    uint16_t n2_pulse_count;
    uint16_t n3_pulse_count;
    uint16_t input_rpm;
    uint16_t engine_speed;
    uint16_t front_left_wheel_speed;
    uint16_t front_right_wheel_speed;
    uint16_t rear_left_wheel_speed;
    uint16_t rear_right_wheel_speed;
    uint16_t vehicle_speed_rear_wheels; //km/h
    uint16_t vehicle_speed_front_wheels;
} __attribute__ ((packed)) RLI_31_DATA;

typedef struct {
    uint8_t shifter_position;
    uint8_t id_608_tc_status;
} __attribute__ ((packed)) RLI_33_DATA;


RLI_31_DATA get_rli_31(AbstractCan* can_layer);

#endif

