#ifndef MAP_H_
#define MAP_H_
/*

    This file contains default map data for all profiles

    Note that when these maps are edited in the config app,
    the TCU will use THOSE maps, not the ones specified here.

    Only by resetting the map data in the config app will these maps be applied

*/

#include <stdint.h>
#define SHIFT_MAP_SIZE 44

/** All values for the following maps are in input shaft RPM **/

/*
---------------------------------------------------------------------------------
                    STANDARD/SPORT MODE MAPS
---------------------------------------------------------------------------------
*/

extern const int16_t S_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t S_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t S_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t S_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

/*
---------------------------------------------------------------------------------
                    COMFORT MODE MAPS
---------------------------------------------------------------------------------
*/

extern const int16_t C_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t C_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t C_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t C_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

/*
---------------------------------------------------------------------------------
                    AGILITY MODE MAPS
---------------------------------------------------------------------------------
*/

extern const int16_t A_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t A_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t A_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t A_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

/*
---------------------------------------------------------------------------------
                    MANUAL MODE MAPS - ONLY USED FOR AUTO DOWNSHIFT.
                    BY DEFAULT, MANUAL MODE DOES NOT AUTO UPSHIFT
                    SO ALL UPSHIFT VALUES ARE SET TO 9999.

                    DOWNSHIFT TABLES ARE NEEDED TO PROTECT THE GEARBOX!
                    THIS PREVENTS THE GEARBOX ENTERING A STATE WHERE IT CANNOT
                    CREATE ENOUGH PRESSURE TO KEEP THE CLUTCHES IN PLACE
                    WITH LOADS OF INPUT TORQUE, RESULTING IN SLIP.

                    MANUAL MAPS ARE NOT EDITABLE IN THE CONFIG APP!
---------------------------------------------------------------------------------
*/

extern const int16_t M_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t M_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t M_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t M_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

/*
================================================================================

   CLUTCH DATA MAPS (From EGS52)

   There are 10 maps in each data 'set'. Sets 0-3 are for small NAG, and 4-9 are for large NAG

================================================================================

 */

#define NUM_MECHANICAL_CALIBRATIONS 5
#define NUM_SPRING_CALIBRATIONS 2

extern const int16_t CLUTCH_FRICTION_MAP[NUM_MECHANICAL_CALIBRATIONS][9*6];
extern const int16_t CLUTCH_RELEASE_SPRING_MAP[NUM_SPRING_CALIBRATIONS][6];
extern const uint16_t CENTIFUGAL_PRESSURE_FACTOR_SMALL_NAG[2];
extern const uint16_t CENTIFUGAL_PRESSURE_FACTOR_LARGE_NAG[2];

/* Order is N,1,2,3,4,5,R1,R2 */
extern const uint8_t STRONGEST_LOADED_CLUTCH_SMALL_NAG[8];
/* Order is N,1,2,3,4,5,R1,R2 */
extern const uint8_t STRONGEST_LOADED_CLUTCH_LARGE_NAG[8];

/*
================================================================================

   HYDRALIC DATA MAPS (From EGS52)

   These maps contain data for hydralic variants. There are only 2, and only ever 2 of these

================================================================================
 */

#define NUM_HYDRALIC_CALIBRATIONS 2
extern const int16_t HYDRALIC_PCS_MAP[NUM_HYDRALIC_CALIBRATIONS][8*4];
extern const int16_t HYDRALIC_PCS_MAP_X_HEADER[NUM_HYDRALIC_CALIBRATIONS][8];

/*
================================================================================

   PRESSURE MAPS

================================================================================

 */

// -- FILL TIME --

#define FILL_TIME_MAP_SIZE 5*4 // 4 temp positions, 5 clutch groups

extern const int16_t SMALL_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];
extern const int16_t LARGE_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];

// -- PWM maps --

#define TCC_PWM_MAP_SIZE 7*5 // 5 temp positions, 7 pressure readings

extern const int16_t TCC_PWM_MAP[TCC_PWM_MAP_SIZE];


// -- Hold phase pressures --
#define FILL_PRESSURE_MAP_SIZE 6 // 6 clutches (Including B3 for reverse), 4 load points

extern const int16_t NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE];

#define LOW_FILL_PRESSURE_MAP_SIZE 5 // Only for forward clutches (K1, K2, K3, B1, B2)

extern const int16_t NAG_FILL_LOW_PRESSURE_MAP[LOW_FILL_PRESSURE_MAP_SIZE];

#define SHIFT_VALVE_MAP_SIZE 8
extern const uint16_t SHIFT_P_OVERLAP_AREA_MAP[SHIFT_VALVE_MAP_SIZE];
extern const uint16_t MODULATING_P_OVERLAP_AREA_MAP[SHIFT_VALVE_MAP_SIZE];
extern const int16_t SHIFT_VALVE_SPRING_PRESSURE_MAP[SHIFT_VALVE_MAP_SIZE];

// -- Target Shift time maps -- 

#define SHIFT_TIME_MAP_SIZE 30

extern const int16_t M_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t M_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

extern const int16_t C_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t C_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

extern const int16_t S_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t S_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

extern const int16_t A_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t A_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

extern const int16_t W_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t W_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

extern const int16_t R_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t R_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

/** TCC Slip maps */
// 1000 - 6000RPM (Step of 500 RPM), 0-100% Pedal, step of 10%
#define TCC_RPM_TARGET_MAP_SIZE 11*11
extern const int16_t TCC_RPM_TARGET_MAP[TCC_RPM_TARGET_MAP_SIZE];

/**
 * Adaptation subsystem maps
*/
// 1 load points, 8 shifts
#define PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE 8

extern const int16_t PREFILL_ADAPT_PREFILL_PRESSURE_MAP[PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE];
extern const int16_t PREFILL_ADAPT_PREFILL_TIMING_MAP[PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE];
extern const int16_t PREFILL_ADAPT_PREFILL_MAXTORQUE_MAP[PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE];


/**
 * torque tables
 */

// -- Engine torque table for Hfm-ECUs or custom ECUs without torque models --
#define TORQUE_HEADERS_MAP_NAME_SIZE 18u
extern const char MAP_NAME_ENGINE_TORQUE_MAX[TORQUE_HEADERS_MAP_NAME_SIZE];
#define TORQUE_MAP_SIZE 33u
extern const int16_t ENGINE_TORQUE_MAP[TORQUE_MAP_SIZE];
extern const int16_t ENGINE_TORQUE_HEADERS_MAP[TORQUE_MAP_SIZE];

#endif