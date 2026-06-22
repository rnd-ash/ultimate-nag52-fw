#include "input_torque.hpp"
#include "tcu_maths.h"
#include "egs_calibration/calibration_structs.h"
#include <tcu_maths_impl.h>

int16_t InputTorqueModel::get_input_torque(uint16_t engine_rpm, uint16_t input_rpm, int16_t static_torque) {
    int16_t ret = 0;
    if (static_torque == INT16_MAX) {
        ret = INT16_MAX;
    } else if (engine_rpm == 0 || engine_rpm == INT16_MAX) {
        ret = static_torque;
    } else {
        float multi = InputTorqueModel::get_input_torque_factor(engine_rpm, input_rpm);
        ret = static_torque * multi;
    }
    return ret;
}

float InputTorqueModel::get_input_torque_factor(uint16_t engine, uint16_t input) {
    if (0 == engine) {
        return 1.0;
    }
    float rpm_multi = (input*1000) / engine;

    //Interpolate map, but faster, since its just 4 values, no need to allocate a whole map for this
    float x1 = (float)(TCC_CFG_PTR->multiplier_map_x[0]);
    float x2 = (float)(TCC_CFG_PTR->multiplier_map_x[1]);
    float z1 = (float)(TCC_CFG_PTR->multiplier_map_z[0]) / 100.0;
    float z2 = (float)(TCC_CFG_PTR->multiplier_map_z[1]) / 100.0;
    return interpolate_float(rpm_multi, z1, z2, x1, x2, InterpType::Linear);
}

int16_t InputTorqueModel::get_pump_torque(uint16_t engine, uint16_t input) {
    if (engine == 0 || engine == INT16_MAX) {
        return INT16_MAX;
    } else {
        int engine_pow_2 = ((int)engine*(int)engine)/1000;

        uint16_t rpm_multi_x1000 = ((int)input*1000) / ((int)engine);

        int lambda = ((float)interpolate_linear_array(rpm_multi_x1000, 11, TCC_CFG_PTR->pump_map_x, TCC_CFG_PTR->pump_map_z));

        int output_torque = (lambda * engine_pow_2)/100000;
        // Clamp output to 10x drag torque
        if (output_torque > VEHICLE_CONFIG.engine_drag_torque) {
            output_torque = VEHICLE_CONFIG.engine_drag_torque;
        }
        return (int16_t)output_torque;
    }

}