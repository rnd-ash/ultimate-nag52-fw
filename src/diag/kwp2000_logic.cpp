#include "kwp2000_logic.h"

#include "kwp2000_defines.h"

bool kwp_has_arg0(const uint8_t* args, uint16_t arg_len) {
    return args != nullptr && arg_len > 0u;
}

uint8_t decToBcd(uint8_t val) {
    return (uint8_t)(((val / 10u) * 16u) + (val % 10u));
}

uint8_t bcd_to_hex(char c) {
    if (c >= '0' && c <= '9') {
        return (uint8_t)(c - '0');
    }
    if (c >= 'A' && c <= 'F') {
        return (uint8_t)(c - 'A' + 10);
    }
    if (c >= 'a' && c <= 'f') {
        return (uint8_t)(c - 'a' + 10);
    }
    return 0x0Fu;
}

bool kwp_is_valid_ecu_reset_subfn(const uint8_t* args, uint16_t arg_len) {
    if (!kwp_has_arg0(args, arg_len) || arg_len != 1u) {
        return false;
    }
    return args[0] == ECU_RESET_POWER_ON || args[0] == ECU_RESET_NON_VOLATILE;
}

KwpTesterPresentSubfn kwp_parse_tester_present_subfn(const uint8_t* args, uint16_t arg_len) {
    if (!kwp_has_arg0(args, arg_len) || arg_len != 1u) {
        return KwpTesterPresentSubfn::Invalid;
    }

    if (args[0] == KWP_CMD_RESPONSE_REQUIRED) {
        return KwpTesterPresentSubfn::ResponseRequired;
    }
    if (args[0] == KWP_CMD_NO_RESPONSE_REQUIRED) {
        return KwpTesterPresentSubfn::NoResponseRequired;
    }
    return KwpTesterPresentSubfn::Invalid;
}

bool kwp_has_valid_read_data_local_ident_header(const uint8_t* args, uint16_t arg_len) {
    if (!kwp_has_arg0(args, arg_len)) {
        return false;
    }

    if (args[0] == KWP_RLI_MAP_EDITOR || args[0] == KWP_RLI_SETTINGS_EDIT) {
        return true;
    }
    return arg_len == 1u;
}

uint32_t kwp_read_u24_be(const uint8_t* args) {
    if (args == nullptr) {
        return 0u;
    }
    return ((uint32_t)args[0] << 16) | ((uint32_t)args[1] << 8) | (uint32_t)args[2];
}

uint32_t kwp_read_u32_be(const uint8_t* args) {
    if (args == nullptr) {
        return 0u;
    }
    return ((uint32_t)args[0] << 24) | ((uint32_t)args[1] << 16) | ((uint32_t)args[2] << 8) | (uint32_t)args[3];
}