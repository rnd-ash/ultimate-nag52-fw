#ifndef __DIAG_DATA_H__
#define __DIAG_DATA_H__

#include <stdint.h>
#include "canbus/can_hal.h"
#include "gearbox.h"
#include "nvs/eeprom_config.h"
#include "common_structs.h"
#include "endpoint.h"

// Needed extra bytes for response, SID, RLI, Shift ID
static_assert(sizeof(ShiftReport) < DIAG_CAN_MAX_SIZE-3, "Shift report is too big to fit in Diag message!");
// Diagnostic data IDs and data structures
// used by the KWP2000 server on the TCM


// -- Record local identifiers (SID 0x21) -- //
// NOTE
// The following IDs are already taken by the OEM EGS52/53 module
// 0F,30-60,7A,A0,B0,B1,C0-C4,D1

#define RLI_MAP_EDITOR      0x19 // See map_editor.cpp
#define RLI_GEARBOX_SENSORS 0x20 // Sensor data status
#define RLI_SOLENOID_STATUS 0x21 // Solenoid data status
#define RLI_CAN_DATA_DUMP   0x22 // Gearbox brain logic status
#define RLI_SYS_USAGE       0x23 // Brain usage
#define RLI_COREDUMP_SIZE   0x24 // Coredump size
#define RLI_PRESSURES       0x25
#define RLI_DMA_DUMP        0x26
#define RLI_TCM_CONFIG      0xFE // TCM configuration (AKA SCN)

// Gearbox sensor struct
typedef struct {
    uint16_t n2_rpm; // Raw N2 RPM
    uint16_t n3_rpm; // Raw N3 RPM
    uint16_t calculated_rpm; // Calculated input RPM (From N2 and N3)
    uint16_t calc_ratio;
    uint16_t v_batt; // Battery voltage (mV)
    int atf_temp_c; // ATF Temp (Celcius)
    uint8_t parking_lock; // Parking lock (1 for Engaged, 0 for disengaged)
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
    uint16_t targ_spc_current;
    uint16_t targ_mpc_current;
    uint16_t adjustment_spc;
    uint16_t adjustment_mpc;
    uint16_t y3_current;
    uint16_t y4_current;
    uint16_t y5_current;
}  __attribute__ ((packed)) DATA_SOLENOIDS;

// Pressure command struct
typedef struct {
    uint16_t spc_pwm;
    uint16_t mpc_pwm;
    uint16_t tcc_pwm;
    uint16_t spc_pressure;
    uint16_t mpc_pressure;
    uint16_t tcc_pressure;
}  __attribute__ ((packed)) DATA_PRESSURES;

// Solenoid command struct
typedef struct {
    uint8_t pedal_pos;
    uint16_t min_torque;
    uint16_t max_torque;
    uint16_t static_torque;
    uint16_t left_rear_rpm;
    uint16_t right_rear_rpm;
    uint8_t shift_button_pressed;
    ShifterPosition shifter_position;
    PaddlePosition paddle_position;
    uint16_t engine_rpm;
} __attribute__ ((packed)) DATA_CANBUS_RX;

/// System usage stats 
typedef struct {
    uint16_t core1_usage;
    uint16_t core2_usage;
    uint32_t free_ram;
    uint32_t total_ram;
    uint32_t free_psram;
    uint32_t total_psram;
    uint32_t num_tasks;
} __attribute__ ((packed)) DATA_SYS_USAGE;

typedef struct {
    uint16_t adc_reading;
    uint16_t dma;
} __attribute__ ((packed)) DATA_DMA_BUFFER;

typedef struct {
    uint32_t address;
    uint32_t size;
} __attribute__ ((packed)) COREDUMP_INFO;

DATA_GEARBOX_SENSORS get_gearbox_sensors(Gearbox* g);
DATA_SOLENOIDS get_solenoid_data(Gearbox* gb_ptr);
DATA_PRESSURES get_pressure_data(Gearbox* gb_ptr);
DATA_CANBUS_RX get_rx_can_data(AbstractCan* can_layer);
DATA_SYS_USAGE get_sys_usage();
DATA_DMA_BUFFER dump_i2s_dma();


// Read and write SCN config
TCM_CORE_CONFIG get_tcm_config();
uint8_t set_tcm_config(TCM_CORE_CONFIG cfg);

COREDUMP_INFO get_coredump_info();


#endif // __DIAG_DATA_H__