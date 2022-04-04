/** @file */

/*
    Ultimate NAG52 advanced options

    MODIFY AT YOUR OWN RISK!

    THESE OPTIONS ARE EXPERIMENTAL
*/

/**
 * @brief Enables auto downshift in manual modes when input RPM falls
 * below 700RPM
 * 
 * Default: Enabled
 */
#define MANUAL_AUTO_DOWNSHIFT 1

/**
 * @brief Possible fix for those with Blue topped high-flow MPC and SPC solenoids (Standard solenoids are black/brown)
 * 
 * Default: Disabled
 */
#define BLUE_SOLENOIDS 0


#define SPC_RAMP_ALGO_LINEAR // Linear pressure ramp from start to end
//#define SPC_RAMP_ALGO_EASE_IN_OUT // Ease in pressure ramp with ease out once shift completes
//#define SPC_RAMP_ALGO_EASE_IN // Ease in pressure ramp with cut off once shift completes

// ----- AUTO BEHAVIOUR SELECTOR (DOES NOT WORK ATM!) ----- //
// #define AUTO_BEHAVIOUR_EGS // Try to mimick stock EGS shift behaviour
// #define AUTO_BEHAVIOUR_PSYCHO // ;)
// #define AUTO_BEHAVIOUR_RELAXED // 