#ifndef DIAG_DATA_LOGIC_H
#define DIAG_DATA_LOGIC_H

#include <stdint.h>
#include "kwp2000_defines.h"

enum class DiagModuleSettingsWriteAction : uint8_t {
    Invalid = 0u,
    Reset,
    Write
};

kwp_result_t diag_validate_module_settings_read_args(const uint16_t* buffer_len, uint8_t* const* buffer);
DiagModuleSettingsWriteAction diag_get_module_settings_write_action(uint16_t buffer_len, const uint8_t* buffer);

bool diag_has_tcc_program_sources(const void* gearbox_ptr, const void* tcc_ptr);
bool diag_has_rx_can_sources(const void* can_layer_ptr, const void* shifter_ptr, const void* gearbox_ptr, const void* can_global_ptr);
bool diag_has_shift_live_sources(
    const void* can_layer_ptr,
    const void* gearbox_ptr,
    const void* pressure_mgr_ptr,
    const void* sol_y3_ptr,
    const void* sol_y4_ptr,
    const void* sol_y5_ptr
);

#endif