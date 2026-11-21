#include "inrush_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"
#include "hal/ledc_ll.h"
#include "soc/ledc_struct.h"

#include <cmath>

static bool IRAM_ATTR inrush_solenoid_timer_isr(gptimer_handle_t timer,
        const gptimer_alarm_event_data_t* event, void* user_data) {
    return static_cast<InrushControlSolenoid*>(user_data)->on_alarm(timer, event);
}

InrushControlSolenoid::InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, gpio_num_t zener_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms)
: PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    (void)period_hz;
    (void)target_hold_current_ma;
    this->zener_pin = zener_pin;
    this->pwm_pin = pwm_pin;
    // Old defaults
    int freq = 10000;
    gptimer_alarm_cb_t callback = inrush_solenoid_timer_isr;
    if (GPIO_NUM_NC != this->zener_pin) { // Override! New mechanics
        freq = 1000;
        ledc_stop(LEDC_HIGH_SPEED_MODE, channel, 0);
        gpio_set_direction(pwm_pin, gpio_mode_t::GPIO_MODE_OUTPUT);
        gpio_set_direction(zener_pin, gpio_mode_t::GPIO_MODE_OUTPUT);
        gpio_set_level(pwm_pin, 0);
        gpio_set_level(zener_pin, 0);
    } else {
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, ledc_timer, freq);
    }
    if (ESP_OK != this->ready) {
        return; // Error trying to init base class, so skip
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = (10u * 1000u * 1000u), // 10MHz
        .flags = {
            .intr_shared = 0,
            .allow_pd = 0,
            .backup_before_sleep = 0
        }
    };

    this->ready = gptimer_new_timer(&timer_config, &this->timer);
    if (ESP_OK == ready) {
        const gptimer_alarm_config_t alarm_config = {
            .alarm_count = 1000u,
            .reload_count = 0u,
            .flags = {
                .auto_reload_on_alarm = 0u,
            }
        };
        this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
        if (ESP_OK == ready) {
            gptimer_event_callbacks_t cbs = {
                .on_alarm = callback
            };
            this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
            if (ESP_OK == ready) {
                this->ready = gptimer_register_event_callbacks(this->timer, &cbs, reinterpret_cast<void*>(this));
                if (ESP_OK == ready) {
                    this->ready = gptimer_enable(this->timer);
                } else {
                    ESP_LOGE("ICSolenoid", "ICSolenoid %s gptimer_enable failed: %s", this->name, esp_err_to_name(this->ready));
                }
            }
        }
    }
}

void InrushControlSolenoid::pre_current_test() {
    this->isr_disable();
}

void InrushControlSolenoid::post_current_test() {
    this->isr_enable();
}

// This channel is owned by the TCC ISR, or by a diagnostic test while paused.
// IDF 5.5 ledc_set_duty() is in flash even with LEDC_CTRL_FUNC_IN_IRAM enabled;
// use the channel-local LL operations here, with no blocking fade semaphore.
void IRAM_ATTR InrushControlSolenoid::write_legacy_duty_locked(uint16_t duty) {
    ledc_ll_set_duty_int_part(&LEDC, LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_ll_set_duty_direction(&LEDC, LEDC_HIGH_SPEED_MODE, channel, LEDC_DUTY_DIR_INCREASE);
    ledc_ll_set_duty_num(&LEDC, LEDC_HIGH_SPEED_MODE, channel, 1);
    ledc_ll_set_duty_cycle(&LEDC, LEDC_HIGH_SPEED_MODE, channel, 1);
    ledc_ll_set_duty_scale(&LEDC, LEDC_HIGH_SPEED_MODE, channel, 0);
    ledc_ll_set_sig_out_en(&LEDC, LEDC_HIGH_SPEED_MODE, channel, true);
    ledc_ll_set_duty_start(&LEDC, LEDC_HIGH_SPEED_MODE, channel, true);
}

// Called with output_lock held, including from the ISR error path.
void IRAM_ATTR InrushControlSolenoid::force_off_locked() {
    if (zener_pin != GPIO_NUM_NC) {
        gpio_set_level(zener_pin, 0);
        gpio_set_level(pwm_pin, 0);
    } else {
        // GPIO writes do not override a pin still routed to the LEDC peripheral.
        ledc_stop(LEDC_HIGH_SPEED_MODE, channel, 0);
    }
}

void InrushControlSolenoid::isr_disable() {
    portENTER_CRITICAL(&output_lock);
    isr_disabled = true;
    if (timer) gptimer_stop(timer);
    force_off_locked();
    pulse.reset();
    portEXIT_CRITICAL(&output_lock);
}

void InrushControlSolenoid::isr_enable() {
    esp_err_t res = ESP_OK;
    portENTER_CRITICAL(&output_lock);
    if (isr_disabled) {
        pulse.reset();
        uint64_t count = 0;
        res = timer ? gptimer_get_raw_count(timer, &count) : ESP_ERR_INVALID_STATE;
        gptimer_alarm_config_t alarm = {};
        alarm.alarm_count = count + 1000; // fresh future deadline after a pause
        if (res == ESP_OK) res = gptimer_set_alarm_action(timer, &alarm);
        if (res == ESP_OK) res = gptimer_start(timer);
        isr_disabled = res != ESP_OK;
        if (isr_disabled) force_off_locked();
    }
    portEXIT_CRITICAL(&output_lock);
    if (res != ESP_OK) ESP_LOGE("ICSolenoid", "%s timer resume failed: %s", name, esp_err_to_name(res));
}

bool InrushControlSolenoid::is_disabled() {
    portENTER_CRITICAL(&output_lock);
    bool disabled = isr_disabled;
    portEXIT_CRITICAL(&output_lock);
    return disabled;
}

bool IRAM_ATTR InrushControlSolenoid::on_alarm(gptimer_handle_t handle,
        const gptimer_alarm_event_data_t* event) {
    portENTER_CRITICAL_ISR(&output_lock);
    if (isr_disabled) {
        force_off_locked();
        portEXIT_CRITICAL_ISR(&output_lock);
        return false;
    }
    const auto edge = pulse.next(zener_pin != GPIO_NUM_NC);
    esp_err_t res = ESP_OK;
    if (zener_pin != GPIO_NUM_NC) {
        gpio_set_level(zener_pin, edge.zener);
        gpio_set_level(pwm_pin, edge.duty != 0);
    } else {
        write_legacy_duty_locked(edge.duty);
        // Do not busy-wait inside an ISR for a peripheral update.
    }
    uint64_t now = 0;
    if (res == ESP_OK) res = gptimer_get_raw_count(handle, &now);
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = event->alarm_value + edge.ticks;
    // Leave 100 us to arm the comparator. Never schedule in the past after
    // interrupt latency, a critical section, or an empty phase.
    if (alarm.alarm_count < now + 1000) alarm.alarm_count = now + 1000;
    if (res == ESP_OK) res = gptimer_set_alarm_action(handle, &alarm);
    if (res != ESP_OK) {
        isr_disabled = true;
        force_off_locked();
    }
    portEXIT_CRITICAL_ISR(&output_lock);
    return false; // no task was woken
}

void InrushControlSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    (void)temperature_factor;
    float ratio = 0.35f * vref_compensation;
    if (!std::isfinite(ratio)) ratio = 0.35f;
    // Keep both carrier phases nonzero even with invalid/low supply readings.
    ratio = fmaxf(0.05f, fminf(0.95f, ratio));
    portENTER_CRITICAL(&output_lock);
    pulse.command.on_ticks = pulse.command.inrush == 0 ? 5000 : uint32_t(10000 * ratio);
    pulse.command.off_ticks = 10000 - pulse.command.on_ticks;
    portEXIT_CRITICAL(&output_lock);
}

void InrushControlSolenoid::set_duty(uint16_t duty) {
    if (duty > 4096) duty = 4096;
    portENTER_CRITICAL(&output_lock);
    pwm_raw = pwm = duty;
    auto& c = pulse.command;
    c.duty = duty;
    const uint32_t on = uint32_t(duty) * TccPulse::period / 4096;
    if (zener_pin != GPIO_NUM_NC) {
        if (on < TccPulse::period / 20) {
            c.duty = 0;
            c.inrush = c.hold = 0;
            c.off = TccPulse::period;
        } else if (on > TccPulse::period * 95 / 100) {
            c.inrush = 0;
            c.hold = TccPulse::period;
            c.off = 0;
            c.on_ticks = c.off_ticks = 5000;
        } else {
            c.inrush = on < 25000 ? on : 25000;
            c.hold = on - c.inrush;
            c.off = TccPulse::period - on;
        }
    } else {
        c.inrush = on / 2 < 15000 ? on / 2 : 15000;
        c.hold = on / 2 - c.inrush;
        c.off = TccPulse::period - on / 2;
    }
    // A zero command must not wait for a possibly delayed alarm.
    if (c.duty == 0) { force_off_locked(); pulse.reset(); }
    portEXIT_CRITICAL(&output_lock);
}
