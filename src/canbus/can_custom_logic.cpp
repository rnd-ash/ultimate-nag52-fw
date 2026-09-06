#include "can_custom_logic.h"

#include <limits.h>

int16_t customcan_decode_engine_coolant(const ENGINE_100_CUSTOMCAN& frame) {
    if (frame.T_COOLANT == UINT8_MAX) {
        return Temp::celsius_i16(Temp::INVALID);
    }
    return Temp::celsius_i16(Temp::from_can_u8_offset40(frame.T_COOLANT));
}

bool customcan_decode_kickdown(const ENGINE_100_CUSTOMCAN& frame) {
    return frame.KD;
}

uint16_t customcan_encode_torque_request_nm(float amount_nm) {
    // CustomCAN carries the request in a full 16 bit field, unlike the 13 bit
    // fields on GS218 and ENG_RQ1_TCM.
    return Torque::to_can_raw(amount_nm, UINT16_MAX);
}

CustomCanTorqueRequestFields customcan_build_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    CustomCanTorqueRequestFields fields = {
        .ctrl0 = false,
        .ctrl1 = false,
        .min = false,
        .max = false,
        .raw_torque = 0,
    };

    switch (control_type) {
        case TorqueRequestControlType::FastAsPossible:
            fields.ctrl0 = false;
            fields.ctrl1 = true;
            break;
        case TorqueRequestControlType::BackToDemandTorque:
            fields.ctrl0 = true;
            fields.ctrl1 = true;
            break;
        case TorqueRequestControlType::NormalSpeed:
            fields.ctrl0 = true;
            fields.ctrl1 = false;
            break;
        case TorqueRequestControlType::None:
        default:
            fields.ctrl0 = false;
            fields.ctrl1 = false;
            break;
    }

    if (control_type != TorqueRequestControlType::None) {
        fields.raw_torque = customcan_encode_torque_request_nm(amount_nm);
        if (limit_type == TorqueRequestBounds::LessThan) {
            fields.min = true;
            fields.max = false;
        } else if (limit_type == TorqueRequestBounds::MoreThan) {
            fields.min = false;
            fields.max = true;
        } else {
            fields.min = true;
            fields.max = true;
        }
    }

    return fields;
}