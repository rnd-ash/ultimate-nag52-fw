#include "inrush_solenoid.h"
#include "inrush_solenoid_logic.h"
#include "esp_check.h"
#include "tcu_maths.h"
#include "soc/gpio_struct.h"
#include "sdkconfig.h"

// cppcheck-suppress-file noValidConfiguration

// ---------------------------------------------------------------------------
// ISR CACHE-SAFETY CONTRACT - read before touching anything in this file
// ---------------------------------------------------------------------------
// The TCC solenoid is driven from a gptimer alarm interrupt. Any flash
// operation (an NVS write, a map burn, an OTA transfer) disables the CPU cache
// on BOTH cores for its duration. While the cache is off, every flash-backed
// region becomes unreachable: IROM instructions, DROM read-only data, and
// PSRAM. An interrupt that is still serviced in that window and touches any of
// them takes an immediate "Cache disabled but cached memory region accessed"
// panic and the TCU reboots.
//
// This is not theoretical - it is the crash seen when the timer fired during a
// flash read/write. Note that EEPROM::ewm_btn_save_profile() performs an NVS
// write on every shift into Park, so the two overlap in normal driving.
//
// CONFIG_GPTIMER_ISR_IRAM_SAFE is the option that actually makes this safe. It
// allocates the interrupt with ESP_INTR_FLAG_IRAM, forces the driver's whole
// ISR path into IRAM, and keeps the driver object out of PSRAM.
//
// BEWARE: CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM and CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM
// are PERFORMANCE options. They place code in IRAM but do NOT keep the
// interrupt alive with the cache disabled. Having only those two set is exactly
// what produced the original bug - the ISR looked "IRAM safe" but was not.
//
// Consequences for anything reached from on_timer_interrupt*():
//   * must be IRAM_ATTR (or an inlined/constant-folded expression),
//   * must not call into flash-resident driver code (this is why the LEDC
//     writes are deferred to task context - see __write_pwm below),
//   * must not read non-inlined const data, which lands in DROM,
//   * must not use ESP_LOG*, and must only use *FromISR FreeRTOS APIs,
//   * must not dispatch virtually - vtables live in DROM. on_timer_interrupt()
//     and on_timer_interrupt_new() are deliberately NON virtual for this reason.
//
// Currently on that path, and all verified IRAM resident:
//   * InrushControlSolenoid::on_timer_interrupt{,_new}()  (IRAM_ATTR below)
//   * inrush_plan_hold_step()  (INRUSH_LOGIC_IRAM, see inrush_solenoid_logic.h)
//   * gptimer_set_alarm_action()  (CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM)
//   * portENTER/EXIT_CRITICAL_ISR and direct GPIO register writes
#if !defined(CONFIG_GPTIMER_ISR_IRAM_SAFE)
#error "CONFIG_GPTIMER_ISR_IRAM_SAFE must be enabled: the TCC solenoid gptimer ISR runs while flash writes have the cache disabled, and will panic the TCU without it. See the ISR CACHE-SAFETY CONTRACT above."
#endif
#if !defined(CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM)
#error "CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM must be enabled: the TCC solenoid ISR calls gptimer_set_alarm_action(), which would otherwise live in flash. See the ISR CACHE-SAFETY CONTRACT above."
#endif

// AT 12.0V
const uint16_t INRUSH_START_PWM = 224; // Any PWM below this will just write 0 to solenoid (Not enough open time for arm to move)
const uint16_t INRUSH_SKIP_PWM = 3220; // Any PWM above this will skip inrush and just go to hold as there is enough current
const uint16_t INRUSH_TIME_US = 15000; 
const uint16_t INRUSH_PWM = 4096;
const uint16_t HOLD_PWM = 1300;

const uint32_t TOTAL_PERIOD_TIME_US = 100000; // Timer runs at 10MHz, Hydralic PWM is 100Hz, so 10_000_000/100


/**
 * @brief Sets or clears a GPIO straight from the ISR, without going through the
 *        driver (which is not guaranteed to be IRAM resident).
 *
 * The ESP32 splits its GPIO output registers into two banks: out_w1ts/out_w1tc
 * cover GPIO 0-31, and out1_w1ts/out1_w1tc cover GPIO 32-39 using bit (pin-32).
 * The previous code only ever wrote the first bank as `1 << pin`, which for any
 * pin >= 32 is a shift wider than the uint32_t operand - undefined behaviour,
 * and in practice it would set a completely unrelated GPIO.
 *
 * No current board maps the TCC pins that high (tcc_pwm is 13, io_pin is 4), so
 * this is future proofing rather than a live fault - but board_config.cpp is
 * exactly the kind of file that gets a new revision added to it.
 *
 * MUST stay IRAM safe - see the ISR CACHE-SAFETY CONTRACT above.
 */
static inline void IRAM_ATTR isr_gpio_write(gpio_num_t pin, bool level) {
    if (pin < GPIO_NUM_32) {
        const uint32_t mask = (uint32_t)1u << (uint32_t)pin;
        if (level) {
            GPIO.out_w1ts = mask;
        } else {
            GPIO.out_w1tc = mask;
        }
    } else {
        // Bank 2. Note GPIO 34-39 are input only on the ESP32, so only 32/33
        // are actually drivable here.
        const uint32_t mask = (uint32_t)1u << ((uint32_t)pin - 32u);
        if (level) {
            GPIO.out1_w1ts.val = mask;
        } else {
            GPIO.out1_w1tc.val = mask;
        }
    }
}

// MUST stay IRAM safe - see the ISR CACHE-SAFETY CONTRACT at the top of this file.
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
    // The return value means "did this ISR wake a higher priority task", NOT
    // "was the callback handled". This path wakes nothing - it only mutates
    // solenoid state and reprograms the alarm - so returning true forced a
    // needless context switch on every single alarm.
    return false;
}

// MUST stay IRAM safe - see the ISR CACHE-SAFETY CONTRACT at the top of this file.
static bool IRAM_ATTR inrush_solenoid_timer_isr_new(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    InrushControlSolenoid* solenoid = reinterpret_cast<InrushControlSolenoid*>(user_data);
    uint32_t next_alarm_in = solenoid->on_timer_interrupt_new();
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + next_alarm_in,
        .reload_count = 0u,
        .flags = {
            .auto_reload_on_alarm = 0u
        }
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    // No task is woken here (see the note in inrush_solenoid_timer_isr), so no
    // yield is requested.
    return false;
}

InrushControlSolenoid::InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, gpio_num_t zener_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms)
: PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->ledc_timer = ledc_timer;
    this->target_hold_current = target_hold_current_ma;
    this->zener_pin = zener_pin;
    this->pwm_pin = pwm_pin;
    // Old defaults
    gptimer_alarm_cb_t callback = inrush_solenoid_timer_isr;
    if (GPIO_NUM_NC != this->zener_pin) { // Override! New mechanics
        callback = inrush_solenoid_timer_isr_new;
        ledc_stop(LEDC_HIGH_SPEED_MODE, channel, 0);
        gpio_set_direction(pwm_pin, gpio_mode_t::GPIO_MODE_OUTPUT);
        gpio_set_direction(zener_pin, gpio_mode_t::GPIO_MODE_OUTPUT);
        this->inrush_time = 0;
        this->hold_time = 0;
        this->off_time = TOTAL_PERIOD_TIME_US;
    } else {
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, ledc_timer, 10000);
    }
    if (ESP_OK != this->ready) {
        return; // Error trying to init base class, so skip
    }

    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = (10u * 1000u * 1000u); // 10MHz
    timer_config.intr_priority = 0;
    timer_config.flags.intr_shared = 0;

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
                .on_alarm = callback
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

uint32_t IRAM_ATTR InrushControlSolenoid::on_timer_interrupt_new() {
    // Control the zener phase
    uint32_t ret = TOTAL_PERIOD_TIME_US;
    bool pwm_out = false;
    bool zener_out = false;
    portENTER_CRITICAL_ISR(&this->phase_lock);
    if (this->inrush_time != 0 || this->hold_time != 0) {
        if (this->phase_id == 0) { // Off -> Inrush
            pwm_out = true;
            zener_out = false;
            ret = this->inrush_time;
            if (this->hold_time != 0) {
                this->phase_id = 1; // Inrush -> hold
                this->pwm_phase_enable = false;
                this->hold_phase_elapsed = 0;
            } else {
                this->phase_id = 2; // Inrush -> off (No hold)
            }
        } else if (this->phase_id == 1) { // Inrush -> Hold
            zener_out = false;
            pwm_out = this->pwm_phase_enable;
            const bool phase_is_pwm_off = !pwm_out;
            this->pwm_phase_enable = !this->pwm_phase_enable;

            InrushHoldStepPlan step = inrush_plan_hold_step(
                this->pwm_on_time,
                this->pwm_off_time,
                phase_is_pwm_off,
                this->hold_phase_elapsed,
                this->hold_time
            );
            ret = step.step_us;
            this->hold_phase_elapsed = step.next_total_us;
            if (step.hold_completed) {
                if (this->off_time == 0) {
                    // We go back to this phase (Constant hold)
                    this->hold_phase_elapsed = 0;
                } else {
                    this->phase_id = 2; // Done, turn off!
                }
            }
        } else { // Hold -> Off
            zener_out = true;
            pwm_out = false;
            this->phase_id = 0;
            ret = this->off_time;
        }
    } else {
        zener_out = false;
        pwm_out = false;
        this->phase_id = 0; // Off
    }
    this->pwm_phase_on = pwm_out;
    this->zener_phase_on = zener_out;
    portEXIT_CRITICAL_ISR(&this->phase_lock);

    isr_gpio_write(this->pwm_pin, this->pwm_phase_on);
    if (GPIO_NUM_NC != this->zener_pin) {
        isr_gpio_write(this->zener_pin, this->zener_phase_on);
    }
    return ret;
}

// 100,000 is 10ms of time
uint32_t IRAM_ATTR InrushControlSolenoid::on_timer_interrupt() {
    uint32_t ret = 0;
    uint16_t write_pwm = 0;
    portENTER_CRITICAL_ISR(&this->phase_lock);
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
    // Maintain this deferral on purpose. It is REQUIRED, not an optimisation:
    // - CONFIG_LEDC_CTRL_FUNC_IN_IRAM is disabled, so the LEDC control functions
    //   live in flash. This ISR is IRAM safe and therefore still runs while a
    //   flash write has the cache disabled - calling them here would panic.
    //   (See the ISR CACHE-SAFETY CONTRACT at the top of this file.)
    // - ESP-IDF explicitly allows ledc_update_duty() in ISR, but not ledc_set_duty().
    // - Our duty update needs ledc_set_duty() + ledc_update_duty() on the same channel.
    // - These APIs are not thread-safe across tasks for one channel, so we centralize writes.
    this->deferred_ledc_pwm = write_pwm;
    this->deferred_ledc_pwm_pending = true;
    portEXIT_CRITICAL_ISR(&this->phase_lock);
    return ret;
}

const float TOTAL_PERIOD_PWM = (float)TOTAL_PERIOD_TIME_US / 10.0f; // Per cycle

void InrushControlSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    uint16_t deferred_pwm = 0;
    bool apply_deferred_pwm = false;
    portENTER_CRITICAL(&this->phase_lock);
    if (this->hold_time != 0) {
        float on_pwm_ratio = 0.35f * vref_compensation;
        if (inrush_time == 0) {
            on_pwm_ratio = 0.5f;
        }
        this->pwm_on_time = TCU_ROUND_TO_U32_SAT(TOTAL_PERIOD_PWM * on_pwm_ratio);
        this->pwm_off_time = TOTAL_PERIOD_PWM - this->pwm_on_time;
    }
    if (GPIO_NUM_NC == this->zener_pin && this->deferred_ledc_pwm_pending) {
        deferred_pwm = this->deferred_ledc_pwm;
        this->deferred_ledc_pwm_pending = false;
        apply_deferred_pwm = true;
    }
    portEXIT_CRITICAL(&this->phase_lock);

    if (apply_deferred_pwm) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, this->channel, deferred_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, this->channel);
    }
}

void InrushControlSolenoid::set_duty(uint16_t duty) {
    portENTER_CRITICAL(&this->phase_lock);
    this->pwm_raw = duty;
    this->pwm = duty;
    if (GPIO_NUM_NC != this->zener_pin) {
        // NEW! TCC Zener mode
        int total_on_time = (float)TOTAL_PERIOD_TIME_US * ((float)duty / 4096.0f);
        if (total_on_time < TOTAL_PERIOD_TIME_US/20) { // < 5%
            this->inrush_time = 0;
            this->hold_time = 0;
            this->off_time = TOTAL_PERIOD_TIME_US;
        } else if (total_on_time > TOTAL_PERIOD_TIME_US * 0.95f) { // > 95%
            this->inrush_time = 0;
            this->hold_time = TOTAL_PERIOD_TIME_US;
            this->off_time = 0;
        } else {
            this->inrush_time = MIN(total_on_time, 25000); // MIN period, 2.5ms (Inrush phase)
            this->hold_time = 0;
            if (total_on_time > this->inrush_time) {
                this->hold_time = total_on_time - this->inrush_time;
            }
            this->off_time = TOTAL_PERIOD_TIME_US - (this->inrush_time + this->hold_time); 
        }
    } else {
        this->period_on_time = ((float)duty / 4096.0f) * ((float)TOTAL_PERIOD_TIME_US / 2.0f);
        if (this->period_on_time > INRUSH_TIME_US) {
            this->hold_time = this->period_on_time - (INRUSH_TIME_US);
            this->inrush_time = INRUSH_TIME_US;
        } else {
            this->inrush_time = this->period_on_time;
            this->hold_time = 0;
        }
        this->off_time = TOTAL_PERIOD_TIME_US - this->period_on_time;
    }
    // Restart phase bookkeeping whenever command changes to avoid stale-cycle carryover.
    this->hold_phase_elapsed = 0;
    this->pwm_phase_enable = false;
    portEXIT_CRITICAL(&this->phase_lock);
}
