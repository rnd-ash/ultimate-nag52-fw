#include "input_torque.hpp"
#include "egs51_converter_torque.h"
#include "egs51_pump_torque.h"
#include "egs_calibration/calibration_structs.h"
#include "nvs/eeprom_config.h"

static uint16_t converter_percentage(uint16_t engine, uint16_t input) {
    const uint32_t ratio = uint32_t(input) * 1000 / engine;
    return Egs51ConverterTorque::percentage(ratio,
        TCC_CFG_PTR->multiplier_map_x[0], TCC_CFG_PTR->multiplier_map_x[1],
        TCC_CFG_PTR->multiplier_map_z[0], TCC_CFG_PTR->multiplier_map_z[1]);
}

int16_t InputTorqueModel::get_input_torque(uint16_t engine_rpm, uint16_t input_rpm, int16_t static_torque) {
    if (static_torque == INT16_MAX) return INT16_MAX;
    if (engine_rpm == 0 || Egs51ConverterTorque::unavailable_speed(engine_rpm) ||
        Egs51ConverterTorque::unavailable_speed(input_rpm)) return static_torque;
    return Egs51ConverterTorque::scale(static_torque, converter_percentage(engine_rpm, input_rpm));
}

float InputTorqueModel::get_input_torque_factor(uint16_t engine, uint16_t input) {
    if (engine == 0 || Egs51ConverterTorque::unavailable_speed(engine) ||
        Egs51ConverterTorque::unavailable_speed(input)) return 1.0f;
    return converter_percentage(engine, input) / 100.0f;
}

int16_t InputTorqueModel::get_pump_torque(uint16_t engine_rpm, uint16_t input_rpm) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
    // Safe: the packed calibration's word arrays are asserted even-aligned.
    const int16_t torque = Egs51PumpTorque::calculate(engine_rpm, input_rpm,
        TCC_CFG_PTR->pump_map_x, TCC_CFG_PTR->pump_map_z, 11,
        VEHICLE_CONFIG.engine_drag_torque);
#pragma GCC diagnostic pop
    return torque;
}
