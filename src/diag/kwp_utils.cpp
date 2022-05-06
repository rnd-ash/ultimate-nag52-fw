#include "kwp2000_defines.h"
#include "kwp_utils.h"

// Couple of helpful functions
void global_make_diag_neg_msg(DiagMessage *dest, uint8_t sid, uint8_t nrc) {
    dest->id = KWP_ECU_TX_ID;
    dest->data_size = 3;
    dest->data[0] = 0x7F;
    dest->data[1] = sid;
    dest->data[2] = nrc;
}

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, const uint8_t* resp, uint16_t len) {
    if (len + 2 > DIAG_CAN_MAX_SIZE) {
        global_make_diag_neg_msg(dest, sid, NRC_GENERAL_REJECT);
        return;
    }
    dest->id = KWP_ECU_TX_ID;
    dest->data_size = len+1;
    dest->data[0] = sid+0x40;
    memcpy(&dest->data[1], resp, len);
}

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len) {
    if (len + 3 > DIAG_CAN_MAX_SIZE) {
        global_make_diag_neg_msg(dest, sid, NRC_GENERAL_REJECT);
        return;
    }
    dest->id = KWP_ECU_TX_ID;
    dest->data_size = len+2;
    dest->data[0] = sid+0x40;
    dest->data[1] = pid;
    memcpy(&dest->data[2], resp, len);
}

bool is_engine_off(AbstractCan* can) {
    // Engine MUST be off (Ignition state)
    int rpm = egs_can_hal->get_engine_rpm(esp_timer_get_time()/1000, 250);
    if (rpm != 0 && rpm != UINT16_MAX) { // 0 = 0RPM, MAX = SNV (Engine ECU is offline)
        return false;
    }
    return true;
}

bool is_shifter_passive(AbstractCan* can) {
    // Shifter must be Offline (SNV) or P or N
    ShifterPosition pos = can->get_shifter_position_ewm(esp_timer_get_time()/1000, 250);
    if (
        pos == ShifterPosition::D || pos == ShifterPosition::MINUS || pos == ShifterPosition::PLUS || pos == ShifterPosition::R || // Stationary positions
        pos == ShifterPosition::N_D || pos == ShifterPosition::P_R || pos == ShifterPosition::R_N // Intermediate positions
        ) {
            return false;
    }
    return true;
}