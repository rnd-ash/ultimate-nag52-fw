//
// Created by ashcon on 9/13/21.
//

#include <config.h>
#include <Arduino.h>
#include "pwm_channels/channels.h"

#ifndef ULTIMATE_NAG52_FW_GEARBOX_H
#define ULTIMATE_NAG52_FW_GEARBOX_H

enum class Profile {
    Agility,
    Winter,
    Manual,
    Standard,
    Comfort,
    Fail
};

class gearbox {
public:
    gearbox();
    ~gearbox();
    uint8_t get_atf_temp();

private:
    xTaskHandle* updater;
    uint32_t n2_rpm;
    uint32_t n3_rpm;
    bool safe_to_start;
    uint8_t atf_temp;
};

#endif //ULTIMATE_NAG52_FW_GEARBOX_H
