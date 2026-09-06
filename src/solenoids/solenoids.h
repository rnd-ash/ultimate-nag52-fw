#ifndef SOLENOID_H
#define SOLENOID_H

#include <stdint.h>
#include "pwm_solenoid.h"
#include "inrush_solenoid.h"
#include "on_off_solenoid.h"
#include "cc_solenoid.h"

#define I2S_LOOP_INTERVAL_CC_ONLY (10)
#define I2S_LOOP_INVERVAL_ALL (20)

// How long the power on test waits for current sensing to produce its first
// sample before declaring the solenoid subsystem faulty
#define SOLENOID_FIRST_READ_TIMEOUT_MS (2000u)

namespace Solenoids {
    /**
     * @brief Tries to initialize all the solenoids on the transmission (MPC,SPC,TCC,Y3,Y4,Y5)
     * 
     * @return true All solenoids initialized OK
     * @return false A solenoid failed to initialize
     */
    esp_err_t init_all_solenoids(void);
    uint16_t get_solenoid_voltage(void);

    void boot_solenoid_test(void*);

    /**
     * @brief Spawns the power on solenoid test.
     *
     * MUST be called only after the module settings have been loaded. The test
     * compares measured current against
     * SOL_CURRENT_SETTINGS.current_threshold_error, so starting it any earlier
     * silently judges the solenoids against the compiled in default threshold
     * rather than the one the user configured.
     *
     * @return ESP_OK if the test task was created
     */
    esp_err_t start_boot_test(void);

    /**
     * @brief The GPIO driving the TCC zener cutoff MOSFET, or GPIO_NUM_NC if
     *        the TCU is not running in TCC zener mode.
     *
     * Single source of truth - the enable condition used to be duplicated at
     * every call site, which is how the two copies drift apart.
     */
    gpio_num_t get_tcc_zener_pin(void);

    bool init_routine_completed(void);

    /**
     * @brief Result of the power on solenoid test
     *
     * @return true if the test ran and every solenoid was within its current limit
     */
    bool startup_test_ok(void);

    /**
     * @brief Blocks until the power on solenoid test finishes, or the timeout expires
     *
     * @param timeout_ms Maximum time to wait for the test to complete
     * @return true if the test completed within the timeout and passed
     */
    bool wait_for_boot_test(uint32_t timeout_ms);

    void notify_diag_test_start(void);
    void notify_diag_test_end(void);

    void set_calibration_adjusted_resistance(float new_spc, float new_mpc, int16_t temp_c);
    void get_calibration_adjusted_resistance(float* out_spc, float* out_mpc, bool* out_calibrated, int16_t* out_temp_c);
}

extern OnOffSolenoid *sol_y3;
extern OnOffSolenoid *sol_y4;
extern OnOffSolenoid *sol_y5;

extern ConstantCurrentSolenoid *sol_mpc;
extern ConstantCurrentSolenoid *sol_spc;
extern InrushControlSolenoid *sol_tcc;

extern float resistance_spc;
extern float resistance_mpc;

extern bool temp_cal;
extern int16_t temp_at_test;

#endif // SOLENOID_H