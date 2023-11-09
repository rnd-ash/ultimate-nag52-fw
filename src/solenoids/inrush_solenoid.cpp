#include "inrush_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"

// AT 12.0V
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

InrushControlSolenoid::InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms)
: PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->target_hold_current = target_hold_current_ma;
    this->period_duration_us = (10 * 1000 * 1000) / period_hz;
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, ledc_timer, 10000);
    if (ESP_OK != this->ready) {
        return; // Error trying to init base class, so skip
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = (1 * 1000 * 1000 * 10), // 10Khz
        .flags = {
            .intr_shared = 1
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

void InrushControlSolenoid::pre_current_test() {
    gptimer_stop(this->timer);
}

void InrushControlSolenoid::post_current_test() {
    gptimer_start(this->timer);
}

// 100,000 is 10ms of time
uint32_t InrushControlSolenoid::on_timer_interrupt() {
    uint32_t ret = 0;
    uint16_t write_pwm = 0;
    if (this->phase_id == 0) { // Off -> Inrush
        if (this->period_on_time > 3000) { // 300us
            if (this->period_on_time > this->inrush_time) {
                // Calc hold phase
                // Inrush -> Hold
                this->phase_id = 1;
                write_pwm = 4096;
                ret = this->inrush_time;
                this->hold_time = this->period_on_time - this->inrush_time;
            } else {
                // Inrush -> Off
                this->phase_id = 2;
                write_pwm = 4096;
                ret = this->period_on_time;
                this->hold_time = 0;
            }
        } else {
            // OFF
            // PWM too low to actuate solenoid
            write_pwm = 0;
            this->phase_id = 0; // Come back to off phase at next cycle
            ret = this->period_duration_us; // Next pwm phase
        }
    } else if (this->phase_id == 1 && this->hold_time != 0) {
        // Hold
        write_pwm = this->calc_hold_pwm;
        ret = this->hold_time;
        this->phase_id = 2;
    } else {
        this->phase_id = 0; // Hold -> off
        write_pwm = 0;
        ret = this->period_duration_us - this->period_on_time;
    }

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->channel, write_pwm);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->channel);
    return ret;
}

void InrushControlSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    this->calc_hold_pwm = (float)(HOLD_PWM) * vref_compensation;
    this->inrush_time = 20000.0 * vref_compensation;
}

void InrushControlSolenoid::set_duty(uint16_t duty) {
    this->pwm_raw = duty;
    this->period_on_time = (float)duty / 4096.0 * ((float)this->period_duration_us);
}
