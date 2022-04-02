#ifndef __DIAG_DATA_H__
#define __DIAG_DATA_H__

#include <stdint.h>

// Diagnostic data IDs and data structures
// used by the KWP2000 server on the TCM


// -- Record local identifiers (SID 0x21) -- //
// NOTE
// The following IDs are already taken by the OEM EGS52/53 module
// 0F,30-60,7A,A0,B0,B1,C0-C4,D1

#define RLI_GEARBOX_SENSORS 0x20
#define RLI_SOLENOID_STATUS 0x21

// Gearbox sensor struct
typedef struct {
    uint16_t n2_rpm; // Raw N2 RPM
    uint16_t n3_rpm; // Raw N3 RPM
    uint16_t calculated_rpm; // Calculated input RPM (From N2 and N3)
    uint16_t v_batt; // Battery voltage (mV)
    int atf_temp_c; // ATF Temp (Celcius)
    bool parking_lock; // Parking lock (1 for Engaged, 0 for disengaged)
} __attribute__ ((packed)) DATA_GEARBOX_SENSORS;

// Solenoid command struct
typedef struct {
    uint16_t spc_pwm;
    uint16_t mpc_pwm;
    uint16_t tcc_pwm;
    uint16_t y3_pwm;
    uint16_t y4_pwm;
    uint16_t y5_pwm;
    uint16_t spc_current;
    uint16_t mpc_current;
    uint16_t tcc_current;
    uint16_t y3_current;
    uint16_t y4_current;
    uint16_t y5_current;
}  __attribute__ ((packed)) DATA_SOLENOIDS;

DATA_GEARBOX_SENSORS get_gearbox_sensors();
DATA_SOLENOIDS get_solenoid_data();



#endif // __DIAG_DATA_H__