#ifndef DTCS_H_
#define DTCS_H_

#include <stdint.h>

/**
 * DTCs (Diagnostic trouble codes) for stock EGS52.
 * 
 * Not all DTCs will be utilized by ultimate-nag52
 * 
 * Diagversion 51
 * 
 * ## DTC Ranges
 * * P2000 - P2012 - TCU Internal errors
 * * P2100 - P210A - Gearbox solenoid errors
 * * P2200 - P2226 - Gearbox sensor errors
 * * P2300 - P233B - CAN errors (module communication)
 * * P2400 - P2451 - CAN errors (invalid or missing data)
 * * P2500 - P2564 - Gear errors
 * * P2600 - P260F - Power supply errors
 * * P2700 - P2900 - Ultimate NAG52 specific errors (Will not be recognised with Daimler's own tools)
 */

enum class DtcCode {
    /// Control unit watchdog error
    P2000 = 0x2000,
    /// Control unit clock error 
    P2004 = 0x2004,
    /// Control unit RAM error
    P2005 = 0x2005,
    /// Control unit ROM error
    P2008 = 0x2008, 
    /// EEPROM non functional
    P200A = 0x200A,
    /// Control flow error (Kernel) 
    P200C = 0x200C,
    /// Uncoded EGS module (EEPROM blank - no SCN coding present)
    P2010 = 0x2010, 
    /// Invalid variant identifier
    P2011 = 0x2011, 
    /// SCN Checksum invalid
    P2012 = 0x2012, 

    /// Y3 solenoid error
    P2100 = 0x2100,
    /// Y3 solenoid short circuit 
    P2101 = 0x2101,
    /// Y4 solenoid error
    P2102 = 0x2102, 
    /// Y4 solenoid short circuit
    P2103 = 0x2103, 
    /// Y5 solenoid error
    P2104 = 0x2104, 
    /// Y5 solenoid short circuit
    P2105 = 0x2105, 
    /// TCC solenoid error
    P2106 = 0x2106, 
    /// MPC solenoid error
    P2107 = 0x2107, 
    /// SPC solenoid error
    P2108 = 0x2108, 
    /// R-P Lock solenoid error
    P2109 = 0x2109, 
    /// Starter lock relay error
    P210A = 0x210A, 

    /// N2 speed sensor no signal
    P2200 = 0x2200, 
    /// N3 speed sensor no signal
    P2203 = 0x2203, 
    /// Output speed sensor no signal
    P2206 = 0x2206, 
    /// Output speed sensor consistency error
    P2207 = 0x2207, 
    /// N2 and N3 speed sensors disagree
    P220A = 0x220A, 
    /// N2 or N3 speed sensors overspeed
    P220B = 0x220B, 
    /// Selector lever SCN coding invalid
    P2210 = 0x2210, 
    /// Selector lever position implausible
    P2211 = 0x2211, 
    /// ATF Temp sensor / Starter lockout contact short circuit
    P2220 = 0x2220, 
    /// ATF Temp sensor / Starter lockout contact implausible
    P2221 = 0x2221, 
    /// ATF Temp sensor / Starter lockout contact inconsistent
    P2222 = 0x2222, 
    /// Gearbox overheated
    P2226 = 0x2226, 

    /// CAN Controller error
    P2300 = 0x2300, 
    /// CAN communication with BS disturbed
    P2310 = 0x2310, 
    /// CAN communication with MS disturbed (short term)
    P2311 = 0x2311, 
    /// CAN communication with MS disturbed (long term)
    P2312 = 0x2312, 
    /// CAN communication with EWM disturbed
    P2313 = 0x2313, 
    /// CAN communication with EZS disturbed
    P2314 = 0x2314, 
    /// CAN communication with KOMBI disturbed
    P2315 = 0x2315, 
    /// CAN communication with AAC disturbed
    P2316 = 0x2316, 
    /// CAN communication with VG disturbed
    P2317 = 0x2317, 
    /// CAN Variant data from MS missing
    P2322 = 0x2322, 
    /// CAN message length from BS inconsistent
    P2330 = 0x2330, 
    /// CAN message length from MS inconsistent (short term)
    P2331 = 0x2331, 
    /// CAN message length from MS inconsistent (long term)
    P2332 = 0x2332, 
    /// CAN message length from EWM inconsistent
    P2333 = 0x2333, 
    /// CAN message length from EZS inconsistent
    P2334 = 0x2334, 
    /// CAN message length from KOMBI inconsistent
    P2335 = 0x2335, 
    /// CAN message length from AAC inconsistent
    P2336 = 0x2336, 
    /// CAN message length from VG inconsistent
    P2337 = 0x2337, 
    /// CAN message length for variant type from MS inconsistent
    P233B = 0x233B, 

    /// CAN Wheel speed RR (From BS) not available
    P2400 = 0x2400, 
    /// CAN Wheel speed RL (From BS) not available
    P2401 = 0x2401, 
    /// CAN Wheel speed FR (From BS) not available
    P2402 = 0x2402, 
    /// CAN Wheel speed FL (From BS) not available
    P2403 = 0x2403, 
    /// CAN Brake light switch (From BS) not available
    P2404 = 0x2404, 
    /// CAN Accelerator pedal position (From MS) not available
    P2405 = 0x2405, 
    /// CAN engine static torque (From MS) not available
    P2406 = 0x2406, 
    /// CAN default torque (From BS) not available
    P2407 = 0x2407, 
    /// CAN engine minimal torque (From MS) not available
    P2408 = 0x2408, 
    /// CAN engine maximum torque (From MS) not available
    P2409 = 0x2409, 
    /// CAN engine RPM (From MS) not available
    P240A = 0x240A, 
    /// CAN engine coolant temperature (From MS) not available
    P240B = 0x240B, 
    /// CAN Selector lever position (From EWM) not available
    P240C = 0x240C, 
    /// CAN Transfer case position (From VG) not available
    P240D = 0x240D, 
    /// CAN Steering wheel paddle positions (From MRM) not available
    P2415 = 0x2415, 
    /// CAN Steering wheel control element (From MRM) not available
    P2418 = 0x2418, 
    /// CAN Multiple wheel speeds (from BS) not available
    P2430 = 0x2430, 
    /// CAN Variant ID (From MS) is invalid
    P2450 = 0x2450, 
    /// Variant coding - EGS and MS mismatch!
    P2451 = 0x2451, 

    /// Inadmissible gear ratio
    P2500 = 0x2500,
    /// Engine overspeed 
    P2501 = 0x2501,
    /// Gear implausible or transmission is slipping
    P2502 = 0x2502, 
    /// Gear comparison implausible multiple times!
    P2503 = 0x2503, 
    /// Overspeed N2 RPM sensor
    P2507 = 0x2507, 
    /// Overspeed N3 RPM sensor
    P2508 = 0x2508, 
    /// Torque converter un-commanded lockup!
    P2510 = 0x2510, 
    /// TCC solenoid - Excessive power consumption
    P2511 = 0x2511, 
    /// Torque converter control not possible
    P2512 = 0x2512, 
    /// Gear protection (multiple times) was not received
    P2520 = 0x2520, 
    /// Gear 1 implausible or transmission slipping
    P2560 = 0x2560,
    /// Gear 2 implausible or transmission slipping 
    P2561 = 0x2561,
    /// Gear 3 implausible or transmission slipping
    P2562 = 0x2562, 
    /// Gear 4 implausible or transmission slipping
    P2563 = 0x2563, 
    /// Gear 5 implausible or transmission slipping
    P2564 = 0x2564, 

    /// Undervoltage to entire module
    P2600 = 0x2600, 
    /// Overvoltage to entire power supply
    P2601 = 0x2601, 
    /// Supply voltage for module outside tolerance
    P2602 = 0x2602, 
    /// Supply voltage for sensors outside tolerance
    P2603 = 0x2603, 
    /// Supply voltage for sensors undervoltage
    P260E = 0x260E, 
    /// Supply voltage for sensors overvoltage
    P260F = 0x260F, 

    /// Memory allocation failure in PSRAM in one or more function calls
    P2700 = 0x2700,
    /// Crash detected (Rebooted, coredump saved)
    P2701 = 0x2701, 
    /// Engine failing to acknowledge torque requests (Only when ESP not intervening)
    P2702 = 0x2702, 
    P2703 = 0x2703,
};

#endif