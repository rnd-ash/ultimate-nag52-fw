#include "constant_current.h"
#include "esp_log.h"
#include "math.h"
#include "gearbox.h"

#define REFERENCE_RESISTANCE 5.3 // 
#define REFERENCE_TEMP 25 // Celcius

ConstantCurrentDriver::ConstantCurrentDriver(Solenoid* target, const char *name) {
    this->solenoid = target;
    this->is_cc_mode = true;
    this->pwm_adjustment_percent = 0;
    this->current_target = 0;
    this->last_off_time = 0;
    this->last_change_time = 0;
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

        float calc_resistance = REFERENCE_RESISTANCE;
        if (gearbox != nullptr) {
            calc_resistance = (float)REFERENCE_RESISTANCE+(((float)REFERENCE_RESISTANCE*(gearbox->sensor_data.atf_temp-REFERENCE_TEMP)*0.393)/100.0);
        }
        // Assume 12.0V and target resistance, errors will be compensated by solenoid API and this error
        float req_pwm = (4096.0*((float)this->current_target/((float)12000.0/calc_resistance)))*(1.0+this->pwm_adjustment_percent);
        pwm = req_pwm;
        // Calc PWM before we look at previous error

        uint16_t actual_current = this->solenoid->get_current_avg();
        float error = 0; // Current error (reading vs target)
         // Solenoid was commanded on but hasn't activated yet, or req current is too small to measure
        if (this->current_target == 0 || actual_current == 0){ //(now-this->last_change_time) < 50) {
            error = 0;
        } else {
            error = (float)(this->current_target-actual_current)/((float)12000.0/calc_resistance);
            error *= this->current_target/((float)12000.0/calc_resistance);
            //if (error > 0.05) {
            //    error = 0.05;
            //} else if (error < -0.05) {
            //    error = -0.05;
            //}
        }
        //ESP_LOGI("CC", "SOL %s, E %.2f, ADJ %.2f", this->name, error, this->pwm_adjustment_percent);
        if (abs(this->current_target-actual_current) > 10 && error != 0) {
            this->pwm_adjustment_percent += (error);
            this->last_change_time = now;
        }
        if(this->pwm_adjustment_percent > 2) {
            this->pwm_adjustment_percent = 2;
        } else if (this->pwm_adjustment_percent < -2) {
            this->pwm_adjustment_percent = -2;
        }
        //ESP_LOGI("CC","%s: ADJ %.2f, REQ: %d, READ %d, PWM %d", this->name, this->pwm_adjustment_percent, this->current_target, actual_current, pwm);
    }
    this->solenoid->write_pwm_12_bit(pwm);
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