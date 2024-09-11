#include "inrush_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"

// AT 12.0V
const uint16_t INRUSH_START_PWM = 224; // Any PWM below this will just write 0 to solenoid (Not enough open time for arm to move)
const uint16_t INRUSH_SKIP_PWM = 3220; // Any PWM above this will skip inrush and just go to hold as there is enough current
const uint16_t INRUSH_TIME_US = 15000; 
const uint16_t INRUSH_PWM = 4096;
const uint16_t HOLD_PWM = 1300;

const uint32_t TOTAL_PERIOD_TIME_US = 100000; // Timer runs at 10Khz, Hydralic PWM is 100Hz, so 10_000_000/100


static bool IRAM_ATTR inrush_solenoid_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    InrushControlSolenoid* solenoid = reinterpret_cast<InrushControlSolenoid*>(user_data);
    uint32_t next_alarm_in = solenoid->on_timer_interrupt();
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + next_alarm_in,
        .reload_count = 0u,
        .flags = {
            .auto_reload_on_alarm = 0u
        }
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    return true;
}

InrushControlSolenoid::InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms)
: PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->ledc_timer = ledc_timer;
    this->target_hold_current = target_hold_current_ma;
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, ledc_timer, 10000);
    if (ESP_OK != this->ready) {
        return; // Error trying to init base class, so skip
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = (1u * 1000u * 1000u * 10u), // 10Khz
        .flags = {
            .intr_shared = 0
        }
    };

    this->ready = gptimer_new_timer(&timer_config, &this->timer);
    if (ESP_OK == ready) {
        const gptimer_alarm_config_t alarm_config = {
            .alarm_count = 0u,
            .reload_count = 0u,
            .flags = {
                .auto_reload_on_alarm = 0u
            }
        };
        this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
        if (ESP_OK == ready) {
            gptimer_event_callbacks_t cbs = {
                .on_alarm = inrush_solenoid_timer_isr
            };
            this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
            if (ESP_OK == ready) {
                this->ready = gptimer_register_event_callbacks(this->timer, &cbs, reinterpret_cast<void*>(this));
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
    // Special handling for Min/Max PWM
    if (this->pwm_raw < INRUSH_START_PWM) {
        write_pwm = 0;
        ret = TOTAL_PERIOD_TIME_US;
    } else if (this->pwm_raw > INRUSH_SKIP_PWM) {
        write_pwm = this->calc_hold_pwm;
        ret = TOTAL_PERIOD_TIME_US;
    } else {
        if (this->phase_id == 0) { // Off -> Inrush
            write_pwm = 4096;
            // Grab all values now
            this->inrush_time_this_cycle = this->inrush_time;
            this->hold_time_this_cycle = this->hold_time;
            this->off_time_this_cycle = this->off_time;
            ret = this->inrush_time_this_cycle;
            this->phase_id = this->hold_time_this_cycle == 0 ? 2 : 1;
        } else if (this->phase_id == 1) { // Inrush -> hold
            ret = this->hold_time_this_cycle;
            this->phase_id = 2;
            write_pwm = this->calc_hold_pwm;
        } else { // Hold -> Off
            write_pwm = 0;
            this->phase_id = 0;
            ret = this->off_time_this_cycle;
        }
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->channel, write_pwm);
    while (ledc_get_duty(LEDC_HIGH_SPEED_MODE, this->channel) != write_pwm) {
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->channel);
    }
    return ret;
}

void InrushControlSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    this->calc_hold_pwm = (float)(HOLD_PWM) * vref_compensation;
    this->vref = vref_compensation;
}

void InrushControlSolenoid::set_duty(uint16_t duty) {
    this->pwm_raw = duty;
    this->pwm = duty;
    this->period_on_time = ((float)duty / 4096.0) * ((float)TOTAL_PERIOD_TIME_US/2);
    if (this->period_on_time > INRUSH_TIME_US) {
        this->hold_time = this->period_on_time - (INRUSH_TIME_US);
        this->inrush_time = INRUSH_TIME_US;
    } else {
        this->inrush_time = this->period_on_time;
        this->hold_time = 0;
    }
    this->off_time = TOTAL_PERIOD_TIME_US - this->period_on_time;
}
