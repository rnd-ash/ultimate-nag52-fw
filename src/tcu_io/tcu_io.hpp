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

    struct TorqueInfo {
        float v;
        bool valid;
    };

    struct Torques {
        /**
         * @brief Minimum engine torque to prevent the motor from dying
         */
        TorqueInfo m_min;
        /**
         * @brief Maximum engine torque in the current operation state
         */
        TorqueInfo m_max;
        /**
         * @brief Total engine torque
         */
        TorqueInfo m_sta;
        /**
         * @brief The target torque (Combination driver inputs, ESP, DTR)
         */
        TorqueInfo m_esp;
        /**
         * @brief Calculated input torque
         */
        TorqueInfo m_inp;
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

    // Motor torque values
    int16_t get_static_motor_torque();
    int16_t get_min_motor_torque();
    int16_t get_max_motor_torque();
    int16_t get_esp_ind_torque();

};  


#endif //__TCU_IO_H

