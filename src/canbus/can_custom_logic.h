#ifndef CAN_CUSTOM_LOGIC_H
#define CAN_CUSTOM_LOGIC_H

#include <stdint.h>
#include "../../lib/customcan_ecus/src/ENGINE.h"
#include "can_defines.h"

struct CustomCanTorqueRequestFields {
	bool ctrl0;
	bool ctrl1;
	bool min;
	bool max;
	uint16_t raw_torque;
};

int16_t customcan_decode_engine_coolant(const ENGINE_100_CUSTOMCAN& frame);
bool customcan_decode_kickdown(const ENGINE_100_CUSTOMCAN& frame);
uint16_t customcan_encode_torque_request_nm(float amount_nm);
CustomCanTorqueRequestFields customcan_build_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm);

#endif