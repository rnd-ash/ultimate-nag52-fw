#include "kwp2000_defines.h"
#include "kwp_utils.h"
#include "clock.hpp"

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
    } else {
        dest->id = KWP_ECU_TX_ID;
        dest->data_size = len+1;
        dest->data[0] = sid+0x40;
        memcpy(&dest->data[1], resp, len);
    }
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

bool is_engine_off(EgsBaseCan* can) {
    // Engine MUST be off (Ignition state)
    int rpm = egs_can_hal->get_engine_rpm(250);
    return (rpm == 0 || rpm == UINT16_MAX); // 0 = 0RPM, MAX = SNV (Engine ECU is offline)
}

bool is_shifter_passive(EgsBaseCan* can) {
    // Shifter must be Offline (SNV) or P or N
    ShifterPosition pos = can->get_shifter_position(250);
    return (pos == ShifterPosition::N || pos == ShifterPosition::P || pos == ShifterPosition::SignalNotAvailable);
}






// Error lookup
#define ERR_TABLE_ADD(err) { err, #err }

typedef struct {
    kwp_result_t code;
    const char* name;
} kwp_err_desc;

const char* kwp_nrc_unknown  = "NRC_UNKNOWN";
static const kwp_err_desc kwp_err_table[] = {
    ERR_TABLE_ADD(NRC_OK),
    ERR_TABLE_ADD(NRC_GENERAL_REJECT),
    ERR_TABLE_ADD(NRC_SERVICE_NOT_SUPPORTED),
    ERR_TABLE_ADD(NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT),
    ERR_TABLE_ADD(NRC_BUSY_REPEAT_REQUEST),
    ERR_TABLE_ADD(NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR),
    ERR_TABLE_ADD(NRC_ROUTINE_NOT_COMPLETE),
    ERR_TABLE_ADD(NRC_REQUEST_OUT_OF_RANGE),
    ERR_TABLE_ADD(NRC_SECURITY_ACCESS_DENIED),
    ERR_TABLE_ADD(NRC_INVALID_KEY),
    ERR_TABLE_ADD(NRC_EXCEED_NUMBER_OF_ATTEMPTS),
    ERR_TABLE_ADD(NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED),
    ERR_TABLE_ADD(NRC_DOWNLOAD_NOT_ACCEPTED),
    ERR_TABLE_ADD(NRC_UPLOAD_NOT_ACCEPTED),
    ERR_TABLE_ADD(NRC_TRANSFER_SUSPENDED),
    ERR_TABLE_ADD(NRC_RESPONSE_PENDING),
    ERR_TABLE_ADD(NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_DIAG_SESSION),
    ERR_TABLE_ADD(NRC_DATA_DECOMPRESSION_FAILED),
    ERR_TABLE_ADD(NRC_DATA_DECRYPTION_FAILED),
    ERR_TABLE_ADD(NRC_ECU_NOT_RESPONDING),

    ERR_TABLE_ADD(NRC_UN52_NO_MEM),
    ERR_TABLE_ADD(NRC_UN52_ENGINE_ON),
    ERR_TABLE_ADD(NRC_UN52_ENGINE_OFF),
    ERR_TABLE_ADD(NRC_UN52_SHIFTER_PASSIVE),
    ERR_TABLE_ADD(NRC_UN52_SHIFTER_ACTIVE),
    ERR_TABLE_ADD(NRC_UN52_NULL_PARTITION),
    ERR_TABLE_ADD(NRC_UN52_OTA_BEGIN_FAIL),
    ERR_TABLE_ADD(NRC_UN52_OTA_WRITE_FAIL),
    ERR_TABLE_ADD(NRC_UN52_OTA_INVALID_TY),
};


const char* kwp_nrc_to_name(kwp_result_t res) {
    const char* def = nullptr;
    for (int i = 0; i < sizeof(kwp_err_table) / sizeof(kwp_err_desc); i++) {
        if (kwp_err_table[i].code == res) {
            def = kwp_err_table->name;
            break;
        }
    }
    if (def == nullptr) {
        def = kwp_nrc_unknown;
    }
    return def;
}