#include "maps.h"

const int16_t S_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    1400, 1550, 1800, 2000, 2200, 2450, 2600, 2850, 3200, 4000, 4500,/* 1 -> 2 */
    1400, 1550, 1700, 1850, 2000, 2200, 2350, 2600, 2900, 3550, 4500,/* 2 -> 3 */
    1400, 1550, 1700, 1850, 1950, 2050, 2200, 2450, 2700, 3450, 4500,/* 3 -> 4 */
    1500, 1550, 1700, 1800, 1950, 2050, 2200, 2450, 2650, 3400, 4500 /* 4 -> 5 */
};


const int16_t S_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
       100, 500, 850, 1050, 1200, 1400, 1500, 1700, 1800, 1900, 2000,/* 2 -> 1 */
       150, 600, 900, 1100, 1250, 1400, 1500, 1700, 1800, 2000, 2000,/* 3 -> 2 */
      700, 850, 1000, 1150, 1300, 1400, 1550, 1700, 1800, 2100, 2000,/* 4 -> 3 */
     900, 1000, 1100, 1200, 1350, 1450, 1500, 1750, 1900, 2200, 2500 /* 5 -> 4 */
    };

const int16_t S_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 1 -> 2 */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 2 -> 3 */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 3 -> 4 */
    1500, 1550, 1600, 1700, 1900, 2000, 2500, 3000, 3500, 4500, 6000 /* 4 -> 5 */
};


const int16_t S_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
      75,  150,  300,  600,  900, 1200, 1400, 1600, 1800, 2000, 2200,/* 2 -> 1 */
     900,  900, 1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000, 2200,/* 3 -> 2 */
     900,  900, 1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000, 2200,/* 4 -> 3 */
    1000, 1050, 1100, 1200, 1300, 1400, 1500, 1600, 2000, 2400, 2800 /* 5 -> 4 */
};

/*
---------------------------------------------------------------------------------
                    COMFORT MODE MAPS
---------------------------------------------------------------------------------
*/

const int16_t C_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    1500, 1750, 1900, 2100, 2200, 2450, 2600, 3000, 3500, 4000, 4500,/* 1 -> 2 */
    1500, 1500, 1400, 1500, 1700, 2000, 2300, 2450, 3200, 3750, 4500,/* 2 -> 3 */
    1450, 1500, 1500, 1600, 1800, 2100, 2400, 2550, 3400, 3800, 4500,/* 3 -> 4 */
    1450, 1500, 1600, 1650, 1900, 2200, 2500, 3000, 3700, 4200, 4500 /* 4 -> 5 */
};

const int16_t C_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
       0,    0,    0,    0,  300,  400,  500,  600,  700,  800,  900,/* 2 -> 1 */
     150,  500,  850, 1050, 1200, 1300, 1400, 1500, 1600, 1700, 1800,/* 3 -> 2 */
     900,  900,  900, 1050, 1200, 1300, 1400, 1500, 1600, 1700, 1800,/* 4 -> 3 */
     900,  900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800 /* 5 -> 4 */
};

const int16_t C_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 1 -> 2 */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 2 -> 3 */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 3 -> 4 */
    1500, 1550, 1600, 1700, 1900, 2000, 2500, 3000, 3500, 4500, 6000 /* 4 -> 5 */
};

const int16_t C_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
       0,    0,    0,    0,  300,  400,  500,  600,  700,  800,  900,/* 2 -> 1 */
     900,  900, 1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000, 2200,/* 3 -> 2 */
     900,  900, 1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000, 2200,/* 4 -> 3 */
    1000, 1050, 1100, 1200, 1300, 1400, 1500, 1600, 2000, 2400, 2800 /* 5 -> 4 */
};

/*
---------------------------------------------------------------------------------
                    AGILITY MODE MAPS
---------------------------------------------------------------------------------
*/

const int16_t A_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    1400, 1550, 1700, 2050, 2300, 2600, 2900, 3150, 3500, 4000, 4500,/* 1 -> 2 */
    1400, 1550, 1750, 1950, 2150, 2400, 2650, 2850, 3250, 3850, 4500,/* 2 -> 3 */
    1400, 1500, 1650, 1850, 2050, 2300, 2550, 2750, 3150, 3750, 4500,/* 3 -> 4 */
    1450, 1550, 1650, 1850, 2050, 2250, 2500, 2700, 3100, 3700, 4500 /* 4 -> 5 */
};

const int16_t A_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
       100, 500, 900, 1100, 1200, 1400, 1600, 1700, 1800, 2000, 2300,/* 2 -> 1 */
      150, 600, 1000, 1050, 1200, 1400, 1600, 1700, 1800, 2000, 2300,/* 3 -> 2 */
      900, 900, 1000, 1100, 1200, 1300, 1500, 1600, 1700, 2000, 2300,/* 4 -> 3 */
      900, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1900, 2200 /* 5 -> 4 */
};

const int16_t A_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 1 -> 2 */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 2 -> 3 */
    1500, 1600, 1700, 1800, 1900, 2200, 2500, 3000, 3500, 4500, 6000,/* 3 -> 4 */
    1500, 1550, 1600, 1700, 1900, 2000, 2500, 3000, 3500, 4500, 6000 /* 4 -> 5 */
};

const int16_t A_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
      75,  150,  300,  600,  900, 1200, 1400, 1600, 1800, 2000, 2200,/* 2 -> 1 */
     900,  900, 1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000, 2200,/* 3 -> 2 */
     900,  900, 1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000, 2200,/* 4 -> 3 */
    1000, 1050, 1100, 1200, 1300, 1400, 1500, 1600, 2000, 2400, 2800 /* 5 -> 4 */
};

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
const int16_t M_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,/* 1 -> 2 */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,/* 2 -> 3 */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,/* 3 -> 4 */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999 /* 4 -> 5 */
};

const int16_t M_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
     300,  350,  400,  450,  500,  550,  600,  650,  700,  750,  800,/* 2 -> 1 */
     400,  450,  500,  550,  600,  650,  700,  750,  800,  850,  900,/* 3 -> 2 */
     500,  550,  600,  650,  700,  750,  800,  850,  900,  950, 1000,/* 4 -> 3 */
     600,  650,  700,  750,  800,  850,  900,  950, 1000, 1050, 1100 /* 5 -> 4 */
};

const int16_t M_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,/* 1 -> 2 */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,/* 2 -> 3 */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,/* 3 -> 4 */
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999 /* 4 -> 5 */
};

const int16_t M_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE] = {
    /*                        Pedal position                                   */
    /*0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%           */
      75,  150,  300,  450,  500,  550,  600,  650,  700,  750,  800,/* 2 -> 1 */
     400,  450,  500,  550,  600,  650,  700,  750,  800,  850,  900,/* 3 -> 2 */
     500,  550,  600,  650,  700,  750,  800,  850,  900,  950, 1000,/* 4 -> 3 */
     600,  650,  700,  750,  800,  850,  900,  950, 1000, 1050, 1100 /* 5 -> 4 */
};

const int16_t SMALL_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE] = {
    /* ATF TEMP             */
    /*  -20C,  5C, 25C, 60C */
         600, 360, 220, 160, // K1 clutch
        1620, 560, 260, 160, // K2 clutch
         860, 500, 160, 160, // K3 clutch
         600, 380, 220, 180, // B1 brake
         820, 680, 260, 120  // B2 brake
};

const int16_t LARGE_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE] = {
    /* ATF TEMP             */
    /*  -20C,  5C, 25C, 60C */
         600, 360, 220, 160, // K1 clutch
        1620, 560, 260, 160, // K2 clutch
         860, 500, 160, 160, // K3 clutch
         600, 380, 220, 180, // B1 brake
         820, 680, 260, 120  // B2 brake
};

const int16_t TCC_PWM_MAP[TCC_PWM_MAP_SIZE] = { // values are in /4096
 /*         TCC PRESSURE (mBar)              */   
 /* 0   2000  4000  5000  7500  10000  15000 */
    0,   480,  960, 1280, 1920,  2560,  4096, // 0C
    0,   560, 1040, 1280, 1920,  2560,  4096, // 30C
    0,   640, 1120, 1280, 1920,  2560,  4096, // 60C
    0,   640, 1120, 1280, 1920,  2560,  4096, // 90C
    0,   640, 1120, 1280, 1920,  2560,  4096, // 120C
};

const int16_t NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE] = {
    /* Clutch                    */
    /* K1    K2    K3    B1    B2    B3 */
     1500, 1500, 1500, 1500, 1500, 1500
};

const int16_t NAG_FILL_LOW_PRESSURE_MAP[LOW_FILL_PRESSURE_MAP_SIZE] = {
    /* Clutch                    */
    /* K1    K2    K3    B1    B2 */
      100,  700,  850,  750,  700
};



const int16_t M_UPSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1500, 1000,  750,  500,  500, 
    /* 25% Redline     */ 1000,  900,  800,  600,  475,  450,
    /* 50% Redline     */  700,  650,  600,  500,  450,  400,
    /* 75% Redline     */  500,  475,  450,  425,  400,  375,
    /* Redline (100%)  */  450,  425,  400,  375,  350,  350
};

const int16_t M_DOWNSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1000,  750,  500, 
    /* 25% Redline     */ 1750, 1500, 1200,  800,  650,  450,
    /* 50% Redline     */ 1500, 1250,  900,  600,  600,  400,
    /* 75% Redline     */ 1000, 1000,  600,  550,  550,  350,
    /* Redline (100%)  */  800,  750,  500,  500,  400,  300
};

const int16_t S_UPSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1500, 1000,  750,  500,  500, 
    /* 25% Redline     */ 1000,  900,  800,  600,  475,  450,
    /* 50% Redline     */  700,  650,  600,  500,  450,  400,
    /* 75% Redline     */  500,  475,  450,  425,  400,  375,
    /* Redline (100%)  */  450,  425,  400,  375,  350,  350
};

const int16_t S_DOWNSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1000,  750,  500, 
    /* 25% Redline     */ 1750, 1500, 1200,  800,  650,  450,
    /* 50% Redline     */ 1500, 1250,  900,  600,  600,  400,
    /* 75% Redline     */ 1000, 1000,  600,  550,  550,  350,
    /* Redline (100%)  */  800,  750,  500,  500,  400,  300
};

const int16_t A_UPSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1500, 1000,  750,  500,  500, 
    /* 25% Redline     */ 1000,  900,  800,  600,  475,  450,
    /* 50% Redline     */  700,  650,  600,  500,  450,  400,
    /* 75% Redline     */  500,  475,  450,  425,  400,  375,
    /* Redline (100%)  */  450,  425,  400,  375,  350,  350
};

const int16_t A_DOWNSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1000,  750,  500, 
    /* 25% Redline     */ 1750, 1500, 1200,  800,  650,  450,
    /* 50% Redline     */ 1500, 1250,  900,  600,  600,  400,
    /* 75% Redline     */ 1000, 1000,  600,  550,  550,  350,
    /* Redline (100%)  */  800,  750,  500,  500,  400,  300
};


const int16_t C_UPSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1250, 1000,  750, 
    /* 25% Redline     */ 1000,  900,  805,  800,  750,  700,
    /* 50% Redline     */  900,  850,  800,  750,  700,  650,
    /* 75% Redline     */  800,  750,  700,  650,  625,  600,
    /* Redline (100%)  */  700,  675,  650,  625,  600,  550
};

const int16_t C_DOWNSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1250, 1000,  800, 
    /* 25% Redline     */ 1800, 1600, 1400, 1250,  950,  750,
    /* 50% Redline     */ 1600, 1500, 1400, 1200,  900,  700,
    /* 75% Redline     */ 1400, 1300, 1200, 1100,  800,  650,
    /* Redline (100%)  */ 1200, 1000,  900,  800,  700,  600
};

const int16_t W_UPSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1250, 1000,  750, 
    /* 25% Redline     */ 1000,  900,  805,  800,  750,  700,
    /* 50% Redline     */  900,  850,  800,  750,  700,  650,
    /* 75% Redline     */  800,  750,  700,  650,  625,  600,
    /* Redline (100%)  */  700,  675,  650,  625,  600,  550
};

const int16_t W_DOWNSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1250, 1000,  800, 
    /* 25% Redline     */ 1800, 1600, 1400, 1250,  950,  750,
    /* 50% Redline     */ 1600, 1500, 1400, 1200,  900,  700,
    /* 75% Redline     */ 1400, 1300, 1200, 1100,  800,  650,
    /* Redline (100%)  */ 1200, 1000,  900,  800,  700,  600
};

const int16_t R_UPSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */  100,  100,  100,  100,  100,  100, 
    /* 25% Redline     */  100,  100,  100,  100,  100,  100,
    /* 50% Redline     */  100,  100,  100,  100,  100,  100,
    /* 75% Redline     */  100,  100,  100,  100,  100,  100,
    /* Redline (100%)  */  100,  100,  100,  100,  100,  100
};

const int16_t R_DOWNSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */  100,  100,  100, 100,  100,  100, 
    /* 25% Redline     */  100,  100,  100, 100,  100,  100,
    /* 50% Redline     */  100,  100,  100, 100,  100,  100,
    /* 75% Redline     */  100,  100,  100, 100,  100,  100,
    /* Redline (100%)  */  100,  100,  100, 100,  100,  100
};

/** TCC Slip maps */

// VALUES (Target TCC slip):
// Values <=10 - Sets TCC is closed
// Values > 100 - Sets TCC is open
// Values between 10 and 100 - Tcc is slipping with desired target slip
const int16_t TCC_RPM_TARGET_MAP[TCC_RPM_TARGET_MAP_SIZE] = {
    /*  0    10   20   30   40   50   60   70   80   90   100 <- Pedal pos (%) */
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, // Input 1000 RPM 
         50,  50,  50, 100, 125, 150, 175, 180, 185, 190, 200, // Input 1500 RPM
         10,  20,  40,  80,  90, 100, 120, 140, 160, 170, 180, // Input 2000 RPM 
          0,  10,  20,  55,  60,  75,  95, 105, 125, 150, 160, // Input 2500 RPM 
          0,   0,  10,  20,  30,  40,  50,  70,  90, 110, 130, // Input 3000 RPM 
          0,   0,   0,  10,  20,  30,  40,  60,  70,  90, 110, // Input 3500 RPM 
          0,   0,   0,   0,  10,  20,  40,  50,  60,  80, 100, // Input 4000 RPM 
          0,   0,   0,   0,   0,  10,  30,  40,  50,  70,  90, // Input 4500 RPM 
          0,   0,   0,   0,   0,   0,  10,  20,  40,  60,  80, // Input 5000 RPM 
          0,   0,   0,   0,   0,   0,   0,  10,  30,  50,  70, // Input 5500 RPM 
          0,   0,   0,   0,   0,   0,   0,   0,  10,  30,  50, // Input 6000 RPM 
};


const int16_t PREFILL_ADAPT_PREFILL_PRESSURE_MAP[] = {
//  1-2, 2-3, 3-4, 4-5, 5-4, 4-3, 3-2, 2-1    
      0,   0,   0,   0,   0,   0,   0,   0
};

const int16_t PREFILL_ADAPT_PREFILL_TIMING_MAP[] = {
//  1-2, 2-3, 3-4, 4-5, 5-4, 4-3, 3-2, 2-1    
    0,   0,   0,   0,   0,   0,   0,   0
};

// Values here are x100 (so 20 would be 0.2)
const int16_t PREFILL_ADAPT_PREFILL_MAXTORQUE_MAP[] = {
//  1-2, 2-3, 3-4, 4-5, 5-4, 4-3, 3-2, 2-1    
     20,  30,  40,  50,  50,  40,  30,  20
};

const char MAP_NAME_ENGINE_TORQUE_MAX[TORQUE_HEADERS_MAP_NAME_SIZE] = "ENGINE_TORQUE_MAX";
const int16_t ENGINE_TORQUE_HEADERS_MAP[] = {
    /* rpm */    0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000, 7250, 7500, 7750, 8000
};
const int16_t ENGINE_TORQUE_MAP[] = { // Value = maximum torque at rpm
    // initial values are taken from E 320 (M104 E32 in 124)
    /*              0 rpm	250 rpm	500 rpm	750 rpm	1000 rpm	1250 rpm	1500 rpm	1750 rpm	2000 rpm	2250 rpm	2500 rpm	2750 rpm	3000 rpm	3250 rpm	3500 rpm	3750 rpm	4000 rpm	4250 rpm	4500 rpm	4750 rpm	5000 rpm	5250 rpm	5500 rpm	5750 rpm	6000 rpm	6250 rpm	6500 rpm	6750 rpm	7000 rpm	7250 rpm	7500 rpm	7750 rpm	8000 rpm */
    /* in Nm */     0,	    61,	    121,	182,	242,	    251,	    259,	    268,	    276,	    280,	    284,	    288,	    292,	    299,	    306,	    310,	    309,    	308,    	304,    	300,    	296,    	290,    	282,    	266,    	250,    	235,    	219,	    205,	    190,	    176,	    161,	    147,	    132
};