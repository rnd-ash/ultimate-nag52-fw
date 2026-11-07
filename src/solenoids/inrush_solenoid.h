#ifndef __INRUSH_SOLENOID_H_
#define __INRUSH_SOLENOID_H_

#include "pwm_solenoid.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "tcc_pulse.h"
class InrushControlSolenoid : public PwmSolenoid {
public:
    explicit InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, gpio_num_t zener_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms);
    void __write_pwm(float vref_compensation, float temperature_factor);
    bool on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event);
    void set_duty(uint16_t duty);
    void pre_current_test() override;
    void post_current_test() override;
    void isr_disable();
    void isr_enable();
    bool is_disabled();
private:
    void force_off_locked();
    void write_legacy_duty_locked(uint16_t duty);
    portMUX_TYPE output_lock = portMUX_INITIALIZER_UNLOCKED;
    TccPulse pulse;
    gptimer_handle_t timer = nullptr;
    gpio_num_t zener_pin = GPIO_NUM_NC;
    gpio_num_t pwm_pin = GPIO_NUM_NC;
    // Initial hold is released by the gearbox after all initialization is done.
    bool isr_disabled = true;
};

#endif
