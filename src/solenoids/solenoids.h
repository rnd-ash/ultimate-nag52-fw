#ifndef SOLENOID_H
#define SOLENOID_H

#include <stdint.h>
#include "pwm_solenoid.h"
#include "inrush_solenoid.h"
#include "on_off_solenoid.h"
#include "cc_solenoid.h"

#define I2S_LOOP_INTERVAL_CC_ONLY (10)
#define I2S_LOOP_INVERVAL_ALL (20)

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

    bool init_routine_completed(void);

    void notify_diag_test_start(void);
    void notify_diag_test_end(void);

    void set_calibration_adjusted_resistance(float new_spc, float new_mpc, int16_t temp_c);
    void get_calibration_adjusted_resistance(float* out_spc, float* out_mpc, bool* out_calibrated, int16_t* out_temp_c);

    // bool startup_test_ok();
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