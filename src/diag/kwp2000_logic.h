#ifndef KWP2000_LOGIC_H
#define KWP2000_LOGIC_H

#include <stdint.h>

enum class KwpTesterPresentSubfn : uint8_t {
    Invalid = 0,
    ResponseRequired = 1,
    NoResponseRequired = 2,
};

constexpr uint8_t KWP_RLI_MAP_EDITOR = 0x19;
constexpr uint8_t KWP_RLI_SETTINGS_EDIT = 0xFC;

bool kwp_has_arg0(const uint8_t* args, uint16_t arg_len);
bool kwp_is_valid_ecu_reset_subfn(const uint8_t* args, uint16_t arg_len);
KwpTesterPresentSubfn kwp_parse_tester_present_subfn(const uint8_t* args, uint16_t arg_len);
bool kwp_has_valid_read_data_local_ident_header(const uint8_t* args, uint16_t arg_len);
uint32_t kwp_read_u24_be(const uint8_t* args);
uint32_t kwp_read_u32_be(const uint8_t* args);

#endif