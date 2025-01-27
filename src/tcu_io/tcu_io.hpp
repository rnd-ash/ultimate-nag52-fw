//
// Created by ashcon on 9/2/24.
//

#ifndef __TCU_IO_H
#define __TCU_IO_H

#include <stdint.h>
#include "esp_err.h"
#include "canbus/can_hal.h"
#include "firstorder_average.h"

namespace TCUIO {

    struct SmoothedSensor {
        uint8_t e_counter;
        FirstOrderAverage* buffer;
    };

    template <typename T>
    struct OnePollSensor {
        uint8_t e_counter;
        T current_value;
    };


    esp_err_t setup_io_layer();
    void update_io_layer();

    void set_2_1_ratio(float ratio);
    void set_input_rpm_perform_sanity_check(bool conduct);

    uint16_t calc_turbine_rpm(const uint16_t n2, const uint16_t n3);

    uint8_t parking_lock();
    int16_t atf_temperature();
    uint16_t battery_mv();
    uint16_t n2_rpm();
    uint16_t n3_rpm();
    uint16_t output_rpm();

    uint16_t wheel_rr_2x_rpm();
    uint16_t wheel_rl_2x_rpm();
    uint16_t wheel_fr_2x_rpm();
    uint16_t wheel_fl_2x_rpm();

    int16_t motor_temperature();
    int16_t motor_oil_temperature();

};  


#endif //__TCU_IO_H

