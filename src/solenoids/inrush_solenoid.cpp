#include "inrush_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"

const uint16_t INRUSH_START_PWM = 224; // Any PWM below this will just write 0 to solenoid (Not enough open time for arm to move)
const uint16_t INRUSH_SKIP_PWM = 3220; // Any PWM above this will skip inrush and just go to hold as there is enough current
const uint16_t INRUSH_TIME_US = 2500;
const uint16_t INRUSH_PWM = 4096;
const uint16_t HOLD_PWM = 1300;



static bool IRAM_ATTR inrush_solenoid_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    InrushControlSolenoid* solenoid = (InrushControlSolenoid*)user_data;
    uint32_t next_alarm_in = solenoid->on_timer_interrupt();
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + next_alarm_in,
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    return true;
}

InrushControlSolenoid::InrushControlSolenoid(const char *name, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint8_t current_samples)
: PwmSolenoid(name, pwm_pin, channel, read_channel, current_samples) {
    this->target_hold_current = target_hold_current_ma;
    this->period_duration_us = (1000 * 1000) / period_hz;
    if (ESP_OK != this->ready) {
        return; // Error trying to init base class, so skip
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = (1 * 1000 * 1000),
        .flags = {
            .intr_shared = 0
        }
    };

    this->ready = gptimer_new_timer(&timer_config, &this->timer);
    if (ESP_OK == ready) {
        const gptimer_alarm_config_t alarm_config = {
            .alarm_count = 0,
        };
        this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
        if (ESP_OK == ready) {
            gptimer_event_callbacks_t cbs = {
                .on_alarm = inrush_solenoid_timer_isr
            };
            this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
            if (ESP_OK == ready) {
                this->ready = gptimer_register_event_callbacks(this->timer, &cbs, (void*)this);
                if (ESP_OK == ready) {
                    this->ready = gptimer_enable(this->timer);
                    if (ESP_OK == ready) {
                        this->ready = gptimer_start(this->timer);
                        ESP_LOGI("ICSolenoid", "ICSolenoid %s init OK!", this->name);
                    } else {
                        ESP_LOGE("ICSolenoid", "ICSolenoid %s gptimer_start failed: %s", this->name, esp_err_to_name(this->ready));
                    }
                } else {
                    ESP_LOGE("ICSolenoid", "ICSolenoid %s gptimer_enable failed: %s", this->name, esp_err_to_name(this->ready));
                }
            }
        }
    }
}

uint32_t InrushControlSolenoid::on_timer_interrupt() {
    uint32_t ret;
    uint16_t write_pwm;
    uint16_t req_pwm = this->pwm_raw;
    if (this->phase_id == 2) { // Off -> Inrush
        // Reset
        if (req_pwm <= INRUSH_START_PWM) { // Too low to activate solenoid
            this->phase_id = 2;
            write_pwm = 0;
            ret = this->period_duration_us;
        } else if (req_pwm >= INRUSH_SKIP_PWM) { // High enough to skip inrush phase (Valve sticks open)
            this->period_on_time = (this->period_duration_us*req_pwm)/4096;
            this->period_off_time = this->period_duration_us-this->period_on_time;
            write_pwm = this->calc_hold_pwm;
            ret = this->period_on_time;
            this->phase_id = 1;
        } else { // Inrush + On time
            this->inrush_time = 2500;
            this->period_on_time = (this->period_duration_us*req_pwm)/4096;
            this->period_off_time = this->period_duration_us-this->period_on_time;
            if (this->inrush_time > this->period_on_time) {
                this->inrush_time = this->period_on_time;
                this->period_on_time = 0;
            } else {
                this->period_on_time -= this->inrush_time;
            }
            
            write_pwm = INRUSH_PWM;
            ret = this->inrush_time;
            this->phase_id = 0;
        }
        if (req_pwm > INRUSH_SKIP_PWM) {
            write_pwm = this->calc_hold_pwm;
        }
    } else if (this->phase_id == 0) { // Inrush - On
        if (this->period_on_time == 0) { // Inrush - Off
            ret = this->period_off_time;
            write_pwm = 0;
            this->phase_id = 2;
        } else {
            ret = this->period_on_time;
            write_pwm = this->calc_hold_pwm;
            this->phase_id = 1;
        }
    } else { // On -> Off
        ret = this->period_off_time;
        write_pwm = 0;
        this->phase_id = 2;
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->channel, write_pwm);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->channel);
    return ret;
}

void InrushControlSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    this->calc_hold_pwm = (float)HOLD_PWM * vref_compensation;
}

void InrushControlSolenoid::set_duty(uint16_t duty) {
    this->pwm_raw = duty;
}
