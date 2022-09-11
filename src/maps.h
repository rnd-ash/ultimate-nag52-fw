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

#endif