#include "input_torque.hpp"
#include "input_torque_logic.h"
#include "tcu_maths.h"
#include "egs_calibration/calibration_structs.h"
#include <tcu_maths_impl.h>
#include <string.h>

int16_t InputTorqueModel::get_input_torque(uint16_t engine_rpm, uint16_t input_rpm, int16_t static_torque) {
    int16_t ret = 0;
    if (static_torque == INT16_MAX) {
        ret = INT16_MAX;
    // engine_rpm is unsigned, so its 'no reading' sentinel is UINT16_MAX.
    // Comparing against INT16_MAX never matched the real invalid value.
    } else if (engine_rpm == 0 || engine_rpm == UINT16_MAX) {
        ret = static_torque;
    } else {
        float multi = InputTorqueModel::get_input_torque_factor(engine_rpm, input_rpm);
        ret = input_torque_scaled_clamped(static_torque, multi);
    }
    return ret;
}

float InputTorqueModel::get_input_torque_factor(uint16_t engine, uint16_t input) {
    if (engine == 0u || engine == UINT16_MAX) {
        return 1.0f;
    }
    float rpm_multi = ((float)input) / ((float)engine);

    //Interpolate map, but faster, since its just 4 values, no need to allocate a whole map for this
    float x1 = (float)(TCC_CFG_PTR->multiplier_map_x[0]) / 1000.0f;
    float x2 = (float)(TCC_CFG_PTR->multiplier_map_x[1]) / 1000.0f;
    float z1 = (float)(TCC_CFG_PTR->multiplier_map_z[0]) / 100.0f;
    float z2 = (float)(TCC_CFG_PTR->multiplier_map_z[1]) / 100.0f;
    float factor = interpolate_float(rpm_multi, z1, z2, x1, x2, InterpType::Linear);
    if (!(factor == factor) || factor <= 0.0f) {
        return 1.0f;
    }
    return factor;
}

int16_t InputTorqueModel::get_pump_torque(uint16_t engine_rpm, uint16_t input_rpm) {
    // engine_rpm is unsigned - UINT16_MAX is the 'no reading' sentinel
    if (engine_rpm == 0 || engine_rpm == UINT16_MAX) {
        return INT16_MAX;
    } else {
        int32_t engine_pow_2 = input_torque_engine_pow2_div1000(engine_rpm);

        uint16_t rpm_multi_x1000 = input_torque_ratio_x1000(input_rpm, engine_rpm);

        // Copy the axes out before taking pointers to them.
        // TorqueConverterCalibration is packed and sits at an odd offset inside
        // CalibrationInfo, so a bare uint16_t* into it would be dereferenced as
        // if 2 byte aligned. memcpy is byte-wise, so reading out is always safe.
        uint16_t pump_map_x[11];
        uint16_t pump_map_z[11];
        memcpy(pump_map_x, TCC_CFG_PTR->pump_map_x, sizeof(pump_map_x));
        memcpy(pump_map_z, TCC_CFG_PTR->pump_map_z, sizeof(pump_map_z));

        int lambda = ((float)interpolate_linear_array(rpm_multi_x1000, 11, pump_map_x, pump_map_z));

        int output_torque = (lambda * engine_pow_2)/100000;
        // Clamp output to 2x drag torque
        if (output_torque > VEHICLE_CONFIG.engine_drag_torque*2) {
            output_torque = VEHICLE_CONFIG.engine_drag_torque*2;
        }
        return (int16_t)output_torque;
    }

}