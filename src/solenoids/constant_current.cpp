#include "constant_current.h"
#include "esp_log.h"

#define REFERENCE_RESISTANCE 6.0 // Ohms

ConstantCurrentDriver::ConstantCurrentDriver(Solenoid* target, const char *name) {
    this->solenoid = target;
    this->is_cc_mode = true;
    this->pwm_adjustment_percent = 1.0;
    this->current_target = 0;
    this->last_off_time = esp_timer_get_time()/1000;
    this->last_change_time = esp_timer_get_time()/1000;
    this->name = name;
}

void ConstantCurrentDriver::set_target_current(uint16_t current) {
    if (current != this->current_target) {
        this->last_change_time = esp_timer_get_time()/1000;
    }
    this->current_target = current;
}

void ConstantCurrentDriver::toggle_state(bool enable) {
    this->is_cc_mode = enable;
}

float ConstantCurrentDriver::get_adjustment() {
    return this->pwm_adjustment_percent;
}

void ConstantCurrentDriver::update() {
    if (!this->is_cc_mode) {
        this->last_off_time = esp_timer_get_time()/1000;
        return;
    }
    uint16_t pwm;
    uint64_t now = esp_timer_get_time()/1000;
    if (this->current_target == 0) {
        pwm = 0;
        this->last_off_time = now;
    } else {
        uint16_t actual_current = this->solenoid->get_current_on();
        float error; // Current error (reading vs target)
         // Solenoid was commanded on but hasn't activated yet, or req current is too small to measure
        if (actual_current < 400 || this->current_target == 0 || now-this->last_change_time < 100) {
            error = 0;
        } else {
            error = (float)(this->current_target-actual_current)/(float)this->current_target;
            ESP_LOGI("CC", "%s ERROR IS %.3f", this->name, error*100);
        }
        if (error > 0.01) {
            error = 0.01;
        } else if (error < -0.01) {
            error = -0.01;
        }
        this->pwm_adjustment_percent += (error);
        // Assume 12.0V and target resistance, errors will be compensated by solenoid API and this error
        float req_pwm = (4096.0*((float)this->current_target/((float)Solenoids::get_solenoid_voltage()/REFERENCE_RESISTANCE)))*this->pwm_adjustment_percent;
        pwm = req_pwm;
        //ESP_LOGI("CC","%s: ADJ %.2f, REQ: %d, READ %d, PWM %d", this->name, this->pwm_adjustment_percent, this->current_target, actual_current, pwm);
    }
    this->solenoid->write_pwm_12_bit(pwm, false);
}

void constant_current_driver_thread(void*) {
    while(true) {
        mpc_cc->update();
        spc_cc->update();
        if (Solenoids::is_monitoring_all_solenoids()) {
            vTaskDelay(I2S_LOOP_INVERVAL_ALL);
        } else {
            vTaskDelay(I2S_LOOP_INTERVAL_CC_ONLY);
        }
    }
}

namespace CurrentDriver {
    bool init_current_driver() {
        mpc_cc = new ConstantCurrentDriver(sol_mpc, "MPC");
        spc_cc = new ConstantCurrentDriver(sol_spc, "SPC");
        mpc_cc->set_target_current(0);
        spc_cc->set_target_current(0);
        xTaskCreate(constant_current_driver_thread, "CCDriver", 4096, nullptr, 3, nullptr);
        return true;
    }
}


ConstantCurrentDriver* mpc_cc;
ConstantCurrentDriver* spc_cc;