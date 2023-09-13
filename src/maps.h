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

   PRESSURE MAPS

================================================================================

 */

// -- MPC working pressure --

#define WORKING_PRESSURE_MAP_SIZE 16*7 // 16 positions (0-150%), 7 gears (P/N, R1/R2, 1, 2, 3, 4, 5)

extern const int16_t NAG_WORKING_MAP[WORKING_PRESSURE_MAP_SIZE];

// -- FILL TIME --

#define FILL_TIME_MAP_SIZE 5*4 // 4 temp positions, 5 clutch groups

extern const int16_t SMALL_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];
extern const int16_t LARGE_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];

// -- PWM maps --

#define TCC_PWM_MAP_SIZE 7*5 // 5 temp positions, 7 pressure readings

extern const int16_t TCC_PWM_MAP[TCC_PWM_MAP_SIZE];

#define PCS_CURRENT_MAP_SIZE 8*4

extern const int16_t BROWN_PCS_CURRENT_MAP[PCS_CURRENT_MAP_SIZE];
extern const int16_t BLUE_PCS_CURRENT_MAP[PCS_CURRENT_MAP_SIZE];


// -- Hold phase pressures --
#define FILL_PRESSURE_MAP_SIZE 5 // 6 clutches (Including B3 for reverse), 4 load points

extern const int16_t NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE];

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

/**
 * Adaptation subsystem maps
*/
// 1 load points, 8 shifts
#define PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE 8

extern const int16_t PREFILL_ADAPT_PREFILL_PRESSURE_MAP[PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE];
extern const int16_t PREFILL_ADAPT_PREFILL_TIMING_MAP[PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE];
extern const int16_t PREFILL_ADAPT_PREFILL_MAXTORQUE_MAP[PREFILL_ADAPT_PREFILL_DATA_MAP_SIZE];

#endif