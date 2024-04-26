#ifndef __NVS_ALL_KEYS_H_
#define __NVS_ALL_KEYS_H_

#define CREATE_NAME_EXTERN(def, str) extern const char* NVS_KEY_##def;

#define FOR_LIST(DO) \
    /* -- MISC CORE KEYS -- */ \
    DO(CORE_SCN , "CORE_SCN")\
    DO(DEV_MODE , "DEV_MODE")\
    DO(LAST_PROFILE, "LAST_PROFILE")\
    DO(LAST_FW, "LAST_FW")\
    /* -- MAP NAMES -- */ \
    DO(MAP_NAME_S_DIESEL_UPSHIFT , "S_DIESEL_UP")\
    DO(MAP_NAME_S_DIESEL_DOWNSHIFT , "S_DIESEL_DOWN")\
    DO(MAP_NAME_S_PETROL_UPSHIFT , "S_PETROL_UP")\
    DO(MAP_NAME_S_PETROL_DOWNSHIFT , "S_PETROL_DOWN")\
    DO(MAP_NAME_C_DIESEL_UPSHIFT , "C_DIESEL_UP")\
    DO(MAP_NAME_C_DIESEL_DOWNSHIFT , "C_DIESEL_DOWN")\
    DO(MAP_NAME_C_PETROL_UPSHIFT , "C_PETROL_UP")\
    DO(MAP_NAME_C_PETROL_DOWNSHIFT , "C_PETROL_DOWN")\
    DO(MAP_NAME_A_DIESEL_UPSHIFT , "A_DIESEL_UP")\
    DO(MAP_NAME_A_DIESEL_DOWNSHIFT , "A_DIESEL_DOWN")\
    DO(MAP_NAME_A_PETROL_UPSHIFT , "A_PETROL_UP")\
    DO(MAP_NAME_A_PETROL_DOWNSHIFT , "A_PETROL_DOWN")\
    DO(MAP_NAME_M_DIESEL_UPSHIFT , "M_DIESEL_UP")\
    DO(MAP_NAME_M_DIESEL_DOWNSHIFT , "M_DIESEL_DOWN")\
    DO(MAP_NAME_M_PETROL_UPSHIFT, "M_PETROL_UP")\
    DO(MAP_NAME_M_PETROL_DOWNSHIFT , "M_PETROL_DOWN")\
    DO(MAP_NAME_FILL_TIME_SMALL, "FILL_T_SMALL")\
    DO(MAP_NAME_FILL_TIME_LARGE, "FILL_T_LARGE")\
    DO(MAP_NAME_TCC_PWM , "TCC_PWM")\
    DO(MAP_NAME_FILL_PRESSURE , "FILL_PRESS_2")\
    DO(MAP_NAME_FILL_LOW_PRESSURE , "FILL_LPRESS_2")\
    DO(MAP_NAME_M_UPSHIFT_TIME , "M_UPSHIFT_TIME")\
    DO(MAP_NAME_M_DOWNSHIFT_TIME , "M_DNSHIFT_TIME")\
    DO(MAP_NAME_C_UPSHIFT_TIME , "C_UPSHIFT_TIME")\
    DO(MAP_NAME_C_DOWNSHIFT_TIME , "C_DNSHIFT_TIME")\
    DO(MAP_NAME_S_UPSHIFT_TIME , "S_UPSHIFT_TIME")\
    DO(MAP_NAME_S_DOWNSHIFT_TIME , "S_DNSHIFT_TIME")\
    DO(MAP_NAME_A_UPSHIFT_TIME , "A_UPSHIFT_TIME")\
    DO(MAP_NAME_A_DOWNSHIFT_TIME , "A_DNSHIFT_TIME")\
    DO(MAP_NAME_W_UPSHIFT_TIME , "W_UPSHIFT_TIME")\
    DO(MAP_NAME_W_DOWNSHIFT_TIME , "W_DNSHIFT_TIME")\
    DO(MAP_NAME_R_UPSHIFT_TIME , "R_UPSHIFT_TIME")\
    DO(MAP_NAME_R_DOWNSHIFT_TIME , "R_DNSHIFT_TIME")\
    DO(TCC_ADAPT_SLIP_MAP,"TCC_S_ADAPT_2")\
    DO(TCC_ADAPT_LOCK_MAP,"TCC_L_ADAPT_2")\
    DO(TCC_SLIP_TARGET_MAP,"TCC_RPM_1")\
    DO(MAP_NAME_PREFILL_ADAPT_PREFILL_PRESSURE , "ADP_P_P3")\
    DO(MAP_NAME_PREFILL_ADAPT_PREFILL_TIMING , "ADP_P_T3")\
    DO(MAP_NAME_PREFILL_ADAPT_PREFILL_TORQUE_LIM , "ADP_MAXT_F0")\
    /* SUBSYSTEM SETTINGS */ \
    DO(TCC_SETTINGS, "TCC_A5")\
    DO(SOL_SETTINGS, "SOL_A0")\
    DO(SBS_SETTINGS, "SBS_A3")\
    DO(PRM_SETTINGS, "PRM_A2")\
    DO(ADP_SETTINGS, "ADP_A1")\
    DO(ETS_SETTINGS, "ETS_A3")\

extern const char** ALL_NVS_KEYS[];

extern int ALL_NVS_KEYS_LEN;

FOR_LIST(CREATE_NAME_EXTERN)

#endif

