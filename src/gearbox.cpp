#include "gearbox.h"



Gearbox::Gearbox() {
    this->current_profile = nullptr;
    egs_can_hal->set_drive_profile(GearboxProfile::Underscore); // Uninitialized
    this->profile_mutex = portMUX_INITIALIZER_UNLOCKED;
}

void Gearbox::set_profile(AbstractProfile* prof) {
    if (prof != nullptr) { // Only change if not nullptr!
        portENTER_CRITICAL(&this->profile_mutex);
        this->current_profile = prof;
        // Set CAN display
        egs_can_hal->set_drive_profile(this->current_profile->get_profile());
        portEXIT_CRITICAL(&this->profile_mutex);
    }
}

bool Gearbox::start_controller() {
    xTaskCreatePinnedToCore(Gearbox::start_controller_internal, "GEARBOX", 32768, (void*)this, 10, nullptr, 1);
    return true;
}

void Gearbox::controller_loop() {
    bool lock_state = false;
    int atf_temp = 0;
    while(1) {
        if (Sensors::parking_lock_engaged(&lock_state)) {
            egs_can_hal->set_safe_start(lock_state);
        }
        if (!Sensors::read_atf_temp(&atf_temp)) {
            // Default to engine coolant
            atf_temp = (egs_can_hal->get_engine_coolant_temp())*10;
        }
        egs_can_hal->set_gearbox_temperature(atf_temp/10);
        egs_can_hal->set_shifter_position(egs_can_hal->get_shifter_position_ewm());
        vTaskDelay(10); // 100 updates/sec!
    }
}