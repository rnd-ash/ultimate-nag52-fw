#include "diag_data_logic.h"

kwp_result_t diag_validate_module_settings_read_args(const uint16_t* buffer_len, uint8_t* const* buffer) {
    if (buffer_len == nullptr || buffer == nullptr) {
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT;
    }
    return NRC_OK;
}

DiagModuleSettingsWriteAction diag_get_module_settings_write_action(uint16_t buffer_len, const uint8_t* buffer) {
    if (buffer == nullptr) {
        return DiagModuleSettingsWriteAction::Invalid;
    }
    if (buffer_len == 1u && buffer[0] == 0x00u) {
        return DiagModuleSettingsWriteAction::Reset;
    }
    return DiagModuleSettingsWriteAction::Write;
}

bool diag_has_tcc_program_sources(const void* gearbox_ptr, const void* tcc_ptr) {
    return gearbox_ptr != nullptr && tcc_ptr != nullptr;
}

bool diag_has_rx_can_sources(const void* can_layer_ptr, const void* shifter_ptr, const void* gearbox_ptr, const void* can_global_ptr) {
    return can_layer_ptr != nullptr && shifter_ptr != nullptr && gearbox_ptr != nullptr && can_global_ptr != nullptr;
}

bool diag_has_shift_live_sources(
    const void* can_layer_ptr,
    const void* gearbox_ptr,
    const void* pressure_mgr_ptr,
    const void* sol_y3_ptr,
    const void* sol_y4_ptr,
    const void* sol_y5_ptr
) {
    return can_layer_ptr != nullptr &&
           gearbox_ptr != nullptr &&
           pressure_mgr_ptr != nullptr &&
           sol_y3_ptr != nullptr &&
           sol_y4_ptr != nullptr &&
           sol_y5_ptr != nullptr;
}