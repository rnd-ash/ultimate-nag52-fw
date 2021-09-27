#ifndef CONFIG_H_
#define CONFIG_H_

/// ---------------------------------------------------------------------------------- ///
///                                 SAFE ZONE                                          ///
///                                                                                    ///
/// These options can be modified safely. If you are looking for the danger zone for   ///
/// experimental options, see the bottom of this file                                  ///
/// ---------------------------------------------------------------------------------- ///


/**
 * If enabled, this will compile for W5A580 722.6 (Found with V6 and larger Engines)
 * Default firmware is compiled for W5A330 722.6.
 * 
 * Default: false (W5A330)
 */

#define LARGE_NAG false

/**
 * If set to true, then the gearbox will not use values from SCN coding,
 * instead use values found in this configuration file!
 */

#define FORCE_HARDCODED_VALUES false

/**
 * Modify to set the diameter of your tyre edge to edge.
 * 
 * !!!! IT IS IMPORTANT TO SET THIS CORRECTLY!!!!
 * Setting this parameter incorrectly will result in output shaft
 * speed calculations to be incorrect, thus causing limp home mode
 * as gearbox cannot accurately determine which gear it is in!
 * 
 * You can use this website to calculate the diameter of your rear tyres
 * https://www.blocklayer.com/tire-size-calculator.aspx
 * 
 * Default is 1972mm (245/40 R17 tyres)
 * 
 * NOTE: This parameter will be overriden by SCN coding unless FORCE_HARDCODED_VALUES
 * is enabled, or SCN coding is corrupt / invalid
 * 
 */

#define TYRE_DIAM_MM 1972 // 1.972m


/**
 * Modify to set the ratio of your rear differential
 * 
 * !!!! IT IS IMPORTANT TO SET THIS CORRECTLY!!!!
 * Setting this parameter incorrectly will result in output shaft
 * speed calculations to be incorrect, thus causing limp home mode
 * as gearbox cannot accurately determine which gear it is in!
 * 
 * Default is 1972mm (245/40 R17 tyres)
 * 
 * NOTE: This parameter will be overriden by SCN coding unless FORCE_HARDCODED_VALUES
 * is enabled, or SCN coding is corrupt / invalid
 */
#define DIFF_RATIO 2870 // 2.870 : 1


/**
 * Modify these to enable / disable different profiles.
 * 
 * A brief rundown on profiles:
 * 
 * - Agility (A) - Also known as Sport+ on newer Mercs, makes the gearbox shift harshly
 *             without any regards for comfort.
 * - Manual (M) - Uses the same profile as 'S' mode, except the car will not automatically shift
 *                and requires the user to either operate the shift paddles, or +/- position on the 
 *                shift lever
 * - Winter (W) - Starts the car in 2nd gear, and very gradual shifting to reduce tyre slipping
 * - Comfort (C) - Starts the car in 1st gear, and tries to create as much seamless shifting as possible,
 *                 no matter how harshly the user is accelerating
 * - Standard (S) - Moderate shifting, with more agressive shifting occuring at higher RPMs, still smoother
 *                  than in 'A' mode.
 */

#define AGILITY_ENABLE true  // 'A' mode (Default: true)
#define MANUAL_ENABLE true   // 'M' mode (Default: true)
#define WINTER_ENABLE true   // 'W' mode (Default: true)
#define COMFORT_ENABLE true  // 'C' mode (Default: true)
#define STANDARD_ENABLE true // 'S' mode (Default: true)


/**
 * If disabled, gearbox will not upshift in manual mode when hitting redline.
 * 
 * This does NOT disable automatic downshifting to prevent stalling.
 * 
 * Default: true
 */
#define MANUAL_SHIFT_OVERRIDE true

/**
 * If enabled, the car will automatically start in Winter mode if outside air temperature
 * is below 4C.
 * 
 * Default: true
 */
#define WINTER_AUTOSTART true

/**
 * If enabled, the last profile used will be written to EEPROM, and restored
 * on next vehicle startup, rather than always defaulting to C/S mode!
 * 
 * Default: true
 */

#define REMEMBER_LAST_PROFILE true


/// ---------------------------------------------------------------------------------- ///
///                                 DANGER ZONE                                        ///
///                                                                                    ///
/// Modifying these settings can cause undefined behaviour as they are experimental!!  ///
/// Modify at your own risk!                                                           ///
///                                                                                    ///
/// Once these settings become stable, they will be moved to the safe zone             ///
/// ---------------------------------------------------------------------------------- ///

/**
 * Enables experimental 6th and 7th gear.
 * 
 * These are completely fake gear. And are just extras to provide gears where
 * the torque converter completely locks up to improve efficiency. See the
 * gearing table below to see how it works!
 * 
 * D1 - 1st
 * D2 - 2nd
 * D3 - 3rd
 * D4 - 4th
 * D5 - 4th + 100% lockup
 * D6 - 5th
 * D7 - 5th + 100% lockup
 * 
 * Default: False
 */

#define ENABLE_100_LOCKUP_GEARS false


/**
 * Changes CAN compatibility from EGS52 to the EGS53's CAN network (Used in 2008 and newer cars),
 * as well as changing the diagnostic layer from KWP2000 to UDS.
 * 
 * Default: false
 */

#define EGS53_MODE false

/**
 * Enables experimental shift AI framework (SAI)
 *
 * Default: false
 */

#define ENABLE_SAI false

/**
 * Makes the Instrument cluster BEEP in manual mode if approaching max engine RPM to prompt an upshift from the user
 *
 * This is done by doing some magic over CANBUS to fake a frame from the Cruise control ECU in order to make
 * the cluster BEEP rapidly.
 * 
 * Default: false
 */

 #define BEEP_MANUAL_UPSHIFT false



 #endif // CONFIG_H_