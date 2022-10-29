#ifndef __MAP_H_
#define __MAP_H_
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

#define MAP_NAME_S_DIESEL_UPSHIFT "S_DIESEL_UP"
extern int16_t S_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_S_DIESEL_DOWNSHIFT "S_DIESEL_DOWN"
extern int16_t S_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_S_PETROL_UPSHIFT "S_PETROL_UP"
extern int16_t S_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_S_PETROL_DOWNSHIFT "S_PETROL_DOWN"
extern int16_t S_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

/*
---------------------------------------------------------------------------------
                    COMFORT MODE MAPS
---------------------------------------------------------------------------------
*/

#define MAP_NAME_C_DIESEL_UPSHIFT "C_DIESEL_UP"
extern int16_t C_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_C_DIESEL_DOWNSHIFT "C_DIESEL_DOWN"
extern int16_t C_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_C_PETROL_UPSHIFT "C_PETROL_UP"
extern int16_t C_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_C_PETROL_DOWNSHIFT "C_PETROL_DOWN"
extern int16_t C_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

/*
---------------------------------------------------------------------------------
                    AGILITY MODE MAPS
---------------------------------------------------------------------------------
*/

#define MAP_NAME_A_DIESEL_UPSHIFT "A_DIESEL_UP"
extern int16_t A_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_A_DIESEL_DOWNSHIFT "A_DIESEL_DOWN"
extern int16_t A_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_A_PETROL_UPSHIFT "A_PETROL_UP"
extern int16_t A_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_A_PETROL_DOWNSHIFT "A_PETROL_DOWN"
extern int16_t A_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

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

#define MAP_NAME_M_DIESEL_UPSHIFT "M_DIESEL_UP"
extern int16_t M_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_M_DIESEL_DOWNSHIFT "M_DIESEL_DOWN"
extern int16_t M_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_M_PETROL_UPSHIFT "M_PETROL_UP"
extern int16_t M_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];

#define MAP_NAME_M_PETROL_DOWNSHIFT "M_PETROL_DOWN"
extern int16_t M_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];


/*
================================================================================

   PRESSURE MAPS

================================================================================

 */

// -- MPC working pressure --

#define WORKING_PRESSURE_MAP_SIZE 11*7 // 11 positions, 7 gears (P/N, R1/R2, 1, 2, 3, 4, 5)

#define MAP_NAME_WORKING_SMALL "WORK_SMALL"
extern int16_t SMALL_NAG_WORKING_MAP[WORKING_PRESSURE_MAP_SIZE];

#define MAP_NAME_WORKING_LARGE "WORK_LARGE"
extern int16_t LARGE_NAG_WORKING_MAP[WORKING_PRESSURE_MAP_SIZE];

// -- FILL TIME --

#define FILL_TIME_MAP_SIZE 5*4 // 4 temp positions, 5 clutch groups

#define MAP_NAME_FILL_TIME_SMALL "FILL_T_SMALL"
extern int16_t SMALL_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];

#define MAP_NAME_FILL_TIME_LARGE "FILL_T_LARGE"
extern int16_t LARGE_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];

// -- PWM maps --

#define TCC_PWM_MAP_SIZE 7*5 // 5 temp positions, 7 pressure readings
#define MAP_NAME_TCC_PWM "TCC_PWM"
extern int16_t TCC_PWM_MAP[TCC_PWM_MAP_SIZE];

#define PCS_CURRENT_MAP_SIZE 8*4
#define MAP_NAME_PCS_BROWN "PCS_BROWN"
extern int16_t BROWN_PCS_CURRENT_MAP[PCS_CURRENT_MAP_SIZE];
#define MAP_NAME_PCS_BLUE "PCS_BLUE"
extern int16_t BLUE_PCS_CURRENT_MAP[PCS_CURRENT_MAP_SIZE];


// -- Hold phase pressures --
#define FILL_PRESSURE_MAP_SIZE 5 // 5 clutches
#define MAP_NAME_FILL_PRESSURE_LARGE "FILL_PRESS_L"
extern int16_t LARGE_NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE];
#define MAP_NAME_FILL_PRESSURE_SMALL "FILL_PRESS_S"
extern int16_t SMALL_NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE];

#endif