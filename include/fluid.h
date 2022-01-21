
/**
 * ATF fluid viscosity readings.
 * 
 * 
 * There are 3 readings that the gearbox uses
 * ATF_KV_40C - ATF kinematic viscosity (ISO 3104) at 40C (mm^2/sec)
 * ATF_KV_100C - ATF kinematic viscosity (ISO 3104) at 100C (mm^2/sec)
 * ATF_VI - ATF viscosity index (ISO2909)
 */

#define MERCEDES_236_10
//#define MERCEDES_236_12
//#define MERCEDES_234_14

#ifdef MERCEDES_236_10
    #define ATF_KV_40C 34.5
    #define ATF_KV_100C 7.4
    #define ATF_VI 189
#endif

#ifdef MERCEDES_236_12
    #define ATF_KV_40C 29
    #define ATF_KV_100C 6.3
    #define ATF_VI 179
#endif

#ifdef MERCEDES_236_14
    #define ATF_KV_40C 29
    #define ATF_KV_100C 6.5
    #define ATF_VI 188
#endif

#ifdef MERCEDES_236_15
    #define ATF_KV_40C 29.6
    #define ATF_KV_100C 6.0
    #define ATF_VI 154
#endif

#if !defined(ATF_KV_40C) || !defined(ATF_KV_100C)
    #error "ATF viscosity undefined!"
#endif
