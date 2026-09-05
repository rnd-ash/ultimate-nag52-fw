#ifndef CAN_EGS51_LOGIC_H
#define CAN_EGS51_LOGIC_H

#include <stdint.h>

struct Egs51FreezeResult {
    int16_t driver_converted;
    int16_t req_static_torque_delta;
};

uint8_t egs51_torque_request_to_raw(float amount_nm);
Egs51FreezeResult egs51_apply_freeze_logic(bool freeze, int16_t driver_converted, int16_t static_converted, int16_t req_static_torque_delta);
uint8_t egs51_tcc_multiplier_to_raw(float multi);
uint16_t egs51_decode_wheel_speed_or_sna(uint16_t wheel_raw);
bool egs51_infer_engine_limp(bool temp_kl, bool uehitz, bool diag_kl);
int16_t egs51_apply_max_torque_factor(int16_t base_max_nm, uint8_t max_trq_factor_raw);

#endif // CAN_EGS51_LOGIC_H
