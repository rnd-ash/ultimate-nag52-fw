
/**
* AUTOGENERATED BY convert.py
* DO NOT EDIT THIS FILE!
*
* IF MODIFICATIONS NEED TO BE MADE, MODIFY can_data.txt!
*
* CAN Defintiion for ECU 'ESP_SBC'
*/

#ifdef EGS52_MODE

#ifndef __ECU_ESP_SBC_H_
#define __ECU_ESP_SBC_H_

#include <stdint.h>
    
#define BS_200_CAN_ID 0x0200
#define BS_208_CAN_ID 0x0208
#define BS_270_CAN_ID 0x0270
#define BS_300_CAN_ID 0x0300
#define BS_328_CAN_ID 0x0328

/** brake light switch */
enum class BS_200h_BLS {
	BREMSE_NBET = 0, // Brake not actuated
	BREMSE_BET = 1, // brake actuated
	UNKNOWN = 2, // not defined
	SNV = 3, // signal not available
};

/** rotary direction wheel front left */
enum class BS_200h_DRTGVL {
	PASSIVE = 0, // No rotation detection
	FWD = 1, // direction of rotation forward
	REV = 2, // direction of rotation backwards
	SNV = 3, // signal not available
};

/** direction of rotation wheel front right */
enum class BS_200h_DRTGVR {
	PASSIVE = 0, // No rotation detection
	FWD = 1, // direction of rotation forward
	REV = 2, // direction of rotation backwards
	SNV = 3, // signal not available
};

/** Rad Left for Cruise */
enum class BS_200h_DRTGTM {
	PASSIVE = 0, // No rotation detection
	FWD = 1, // direction of rotation forward
	REV = 2, // direction of rotation backwards
	SNV = 3, // signal not available
};

/** Gear, upper limit */
enum class BS_208h_GMAX_ESP {
	PASSIVE = 0, // passive value
	G1 = 1, // Gear, upper limit = 1
	G2 = 2, // Gear, upper limit = 2
	G3 = 3, // Gear, upper limit = 3
	G4 = 4, // Gear, upper limit = 4
	G5 = 5, // Gear, upper limit = 5
	G6 = 6, // Gear, upper limit = 6
	G7 = 7, // Gear, upper limit = 7
};

/** Gear, lower limit */
enum class BS_208h_GMIN_ESP {
	PASSIVE = 0, // passive value
	G1 = 1, // Gear, lower limit = 1
	G2 = 2, // Gear, lower limit = 2
	G3 = 3, // Gear, lower limit = 3
	G4 = 4, // Gear, lower limit = 4
	G5 = 5, // Gear, lower limit = 5
	G6 = 6, // Gear, lower limit = 6
	G7 = 7, // Gear, lower limit = 7
};

/** system condition */
enum class BS_208h_SZS {
	ERR = 0, // system error
	NORM = 1, // normal operation
	DIAG = 2, // Diagnosis
	ABGAS = 3, // exhaust gas test
};

/** Switching Difference ESP */
enum class BS_208h_SLV_ESP {
	SKL0 = 0, // Shift characteristic "0"
	SKL1 = 1, // Shift characteristic "1"
	SKL2 = 2, // Shift characteristic "2"
	SKL3 = 3, // Shift characteristic "3"
	SKL4 = 4, // Shift characteristic "4"
	SKL5 = 5, // Shift characteristic "5"
	SKL6 = 6, // Shift characteristic "6"
	SKL7 = 7, // Shift characteristic "7"
	SKL8 = 8, // Shift characteristic "8"
	SKL9 = 9, // Shift characteristic "9"
	SKL10 = 10, // Shift characteristic "10"
};

/** ESP request: "N" Insert */
enum class BS_208h_ANFN {
	UNKNOWN = 0, // not defined
	ANF_N = 1, // requirement "neutral"
	IDLE = 2, // No requirement
	SNV = 3, // signal not available
};

/** rotary direction wheel rear right */
enum class BS_208h_DRTGHR {
	PASSIVE = 0, // No rotation detection
	FWD = 1, // direction of rotation forward
	REV = 2, // direction of rotation backwards
	SNV = 3, // signal not available
};

/** rotary direction wheel rear left */
enum class BS_208h_DRTGHL {
	PASSIVE = 0, // No rotation detection
	FWD = 1, // direction of rotation forward
	REV = 2, // direction of rotation backwards
	SNV = 3, // signal not available
};

/** Alerts PlatRollwarner */
enum class BS_270h_PRW_WARN {
	OK = 0, // No warning
	WARN_OHNE = 1, // Tire pressure warning without position specification
	PRW_NV = 2, // PRW not available
	PRW_START = 3, // Restart PRW
	WARN_VL = 4, // Tire pressure warning front left
	WARN_VR = 5, // Tire pressure warning front right
	WARN_HL = 6, // Tire pressure warning rear left
	WARN_HR = 7, // Tire pressure warning rear right
	UNKNOWN_1 = 8, // not defined
	UNKNOWN_2 = 14, // not defined
	SNV = 15, // signal not available
};

/** Status flat tyre warner */
enum class BS_270h_PRW_ST {
	EIN = 0, // PRW active, no warning
	WARN = 1, // PRW active, warning is available
	AUS = 2, // PRW inactive or not available
	INIT = 3, // PRW is initialized
	UNKNOWN_1 = 4, // not defined
	UNKNOWN_2 = 5, // not defined
	PRW_NV = 6, // PRW not available
	SNV = 7, // signal not available
};

/** Send cycle time */
enum class BS_300h_T_Z {
	UNKNOWN = 0, // not defined
	T20_0 = 1, // send cycle time 20 ms
	T23_1 = 2, // send cycle time 23.1 ms
	SNV = 3, // signal not available
};

/** driver brakes */
enum class BS_300h_SFB {
	BREMSE_NEIN = 0, // driver does not slower
	BREMSE_JA = 1, // driver brakes
	UNKNOWN = 2, // not defined
	SNV = 3, // signal not available
};

/** ESP display messages */
enum class BS_328h_ESP_DSPL {
	OK = 0, // No error
	ESP_DEF = 1, // ESP defective
	ESP_OFF = 2, // ESP not available
	ESP_OFF_DS = 3, // ESP not available, DifferentialSp. active (only 463/461)
	BAS_DEF = 4, // Bas defective
	ABS_DEF = 5, // ABS defective (only 463/461)
	ABS_OFF = 6, // ABS Not available (463/461 only)
	BKV_DEF_GBV = 7, // BKV defective (463/461 only)
	ALL_OFF_DS = 8, // ABS, BAS u. ESP Not available (463/461 only)
	SBCS_DEF = 9, // Stop & Roll defective
	SBCS_ON = 10, // Stop & Roll
	SBCH_N_AKT = 11, // SBC HOLD not activatable
	ESP_BAS_DEF = 13, // BAS u. ESP defective
	SBCH_OFF = 14, // SBC HOLD OFF
	SBCH = 15, // SBC HOLD
	ALL_DEF = 16, // ABS, BAS u. ESP defective
	ALL_DEF_GBV = 17, // ABS, BAS, ESP u. BKV defective
	ALL_DIAG = 19, // ABS, BAS u. ESP DIAG. Test.
	ALL_DIAG_GBV = 20, // ABS, BAS, ESP u. BKV DIAG. Test
	ALL_OFF = 22, // ABS, BAS u. ESP not available
	ALL_OFF_GBV = 23, // ABS, BAS, ESP u. BKV not available
	SBCS = 24, // SBC stop
	SBCS_OFF = 25, // SBC stop out
	BRAKE = 26, // Brake immediately!
	SBCS_N_AKT = 27, // SBC stop not activatable
	SBCH_DEF = 28, // SBC HOLD defective
	SBCS_DEF2 = 29, // SBC Stop defective
	GWH_P = 30, // selector lever according to P
	SNV = 31, // signal not available
};



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of BS_200 */
	uint32_t get_canid(){ return BS_200_CAN_ID; }
    /** Sets Brake defective control lamp (EBV_KL at 463/461 / NCV2) */
    void set_BRE_KL(bool value){ raw = (raw & 0x7fffffffffffffff) | ((uint64_t)value & 0x1) << 63; }

    /** Gets Brake defective control lamp (EBV_KL at 463/461 / NCV2) */
    bool get_BRE_KL() const { return (bool)(raw >> 63 & 0x1); }
        
    /** Sets Bas defective control lamp */
    void set_BAS_KL(bool value){ raw = (raw & 0xbfffffffffffffff) | ((uint64_t)value & 0x1) << 62; }

    /** Gets Bas defective control lamp */
    bool get_BAS_KL() const { return (bool)(raw >> 62 & 0x1); }
        
    /** Sets ESP Infolramp flashing light */
    void set_ESP_INFO_BL(bool value){ raw = (raw & 0xdfffffffffffffff) | ((uint64_t)value & 0x1) << 61; }

    /** Gets ESP Infolramp flashing light */
    bool get_ESP_INFO_BL() const { return (bool)(raw >> 61 & 0x1); }
        
    /** Sets ESP Info lamp permanent light */
    void set_ESP_INFO_DL(bool value){ raw = (raw & 0xefffffffffffffff) | ((uint64_t)value & 0x1) << 60; }

    /** Gets ESP Info lamp permanent light */
    bool get_ESP_INFO_DL() const { return (bool)(raw >> 60 & 0x1); }
        
    /** Sets ESP defective control lamp */
    void set_ESP_KL(bool value){ raw = (raw & 0xf7ffffffffffffff) | ((uint64_t)value & 0x1) << 59; }

    /** Gets ESP defective control lamp */
    bool get_ESP_KL() const { return (bool)(raw >> 59 & 0x1); }
        
    /** Sets ABS defective control lamp */
    void set_ABS_KL(bool value){ raw = (raw & 0xfbffffffffffffff) | ((uint64_t)value & 0x1) << 58; }

    /** Gets ABS defective control lamp */
    bool get_ABS_KL() const { return (bool)(raw >> 58 & 0x1); }
        
    /** Sets brake pad wear control lamp */
    void set_BBV_KL(bool value){ raw = (raw & 0xfeffffffffffffff) | ((uint64_t)value & 0x1) << 56; }

    /** Gets brake pad wear control lamp */
    bool get_BBV_KL() const { return (bool)(raw >> 56 & 0x1); }
        
    /** Sets Brake light suppression (EBV_KL at 163 / T0 / T1N) */
    void set_BLS_UNT(bool value){ raw = (raw & 0xff7fffffffffffff) | ((uint64_t)value & 0x1) << 55; }

    /** Gets Brake light suppression (EBV_KL at 163 / T0 / T1N) */
    bool get_BLS_UNT() const { return (bool)(raw >> 55 & 0x1); }
        
    /** Sets BLS Parity (straight parity) */
    void set_BLS_PA(bool value){ raw = (raw & 0xffbfffffffffffff) | ((uint64_t)value & 0x1) << 54; }

    /** Gets BLS Parity (straight parity) */
    bool get_BLS_PA() const { return (bool)(raw >> 54 & 0x1); }
        
    /** Sets Message counter. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_BZ200h(uint8_t value){ raw = (raw & 0xffc3ffffffffffff) | ((uint64_t)value & 0xf) << 50; }

    /** Gets Message counter. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_BZ200h() const { return (uint8_t)(raw >> 50 & 0xf); }
        
    /** Sets brake light switch */
    void set_BLS(BS_200h_BLS value){ raw = (raw & 0xfffcffffffffffff) | ((uint64_t)value & 0x3) << 48; }

    /** Gets brake light switch */
    BS_200h_BLS get_BLS() const { return (BS_200h_BLS)(raw >> 48 & 0x3); }
        
    /** Sets rotary direction wheel front left */
    void set_DRTGVL(BS_200h_DRTGVL value){ raw = (raw & 0xffff3fffffffffff) | ((uint64_t)value & 0x3) << 46; }

    /** Gets rotary direction wheel front left */
    BS_200h_DRTGVL get_DRTGVL() const { return (BS_200h_DRTGVL)(raw >> 46 & 0x3); }
        
    /** Sets wheel speed front left. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_DVL(uint16_t value){ raw = (raw & 0xffffc000ffffffff) | ((uint64_t)value & 0x3fff) << 32; }

    /** Gets wheel speed front left. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_DVL() const { return (uint16_t)(raw >> 32 & 0x3fff); }
        
    /** Sets direction of rotation wheel front right */
    void set_DRTGVR(BS_200h_DRTGVR value){ raw = (raw & 0xffffffff3fffffff) | ((uint64_t)value & 0x3) << 30; }

    /** Gets direction of rotation wheel front right */
    BS_200h_DRTGVR get_DRTGVR() const { return (BS_200h_DRTGVR)(raw >> 30 & 0x3); }
        
    /** Sets Right speed front right. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_DVR(uint16_t value){ raw = (raw & 0xffffffffc000ffff) | ((uint64_t)value & 0x3fff) << 16; }

    /** Gets Right speed front right. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_DVR() const { return (uint16_t)(raw >> 16 & 0x3fff); }
        
    /** Sets Rad Left for Cruise */
    void set_DRTGTM(BS_200h_DRTGTM value){ raw = (raw & 0xffffffffffff3fff) | ((uint64_t)value & 0x3) << 14; }

    /** Gets Rad Left for Cruise */
    BS_200h_DRTGTM get_DRTGTM() const { return (BS_200h_DRTGTM)(raw >> 14 & 0x3); }
        
    /** Sets wheel speed links for cruise control. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_TM_DL(uint16_t value){ raw = (raw & 0xffffffffffffc000) | ((uint64_t)value & 0x3fff) << 0; }

    /** Gets wheel speed links for cruise control. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_TM_DL() const { return (uint16_t)(raw >> 0 & 0x3fff); }
        
} BS_200;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of BS_208 */
	uint32_t get_canid(){ return BS_208_CAN_ID; }
    /** Sets ESP / Art-Wish: "Active Retract" */
    void set_AKT_R_ESP(bool value){ raw = (raw & 0x7fffffffffffffff) | ((uint64_t)value & 0x1) << 63; }

    /** Gets ESP / Art-Wish: "Active Retract" */
    bool get_AKT_R_ESP() const { return (bool)(raw >> 63 & 0x1); }
        
    /** Sets Gear requirement of art */
    void set_MINMAX_ART(bool value){ raw = (raw & 0xbfffffffffffffff) | ((uint64_t)value & 0x1) << 62; }

    /** Gets Gear requirement of art */
    bool get_MINMAX_ART() const { return (bool)(raw >> 62 & 0x1); }
        
    /** Sets Gear, upper limit */
    void set_GMAX_ESP(BS_208h_GMAX_ESP value){ raw = (raw & 0xc7ffffffffffffff) | ((uint64_t)value & 0x7) << 59; }

    /** Gets Gear, upper limit */
    BS_208h_GMAX_ESP get_GMAX_ESP() const { return (BS_208h_GMAX_ESP)(raw >> 59 & 0x7); }
        
    /** Sets Gear, lower limit */
    void set_GMIN_ESP(BS_208h_GMIN_ESP value){ raw = (raw & 0xf8ffffffffffffff) | ((uint64_t)value & 0x7) << 56; }

    /** Gets Gear, lower limit */
    BS_208h_GMIN_ESP get_GMIN_ESP() const { return (BS_208h_GMIN_ESP)(raw >> 56 & 0x7); }
        
    /** Sets Suppression Dynamic fully detection */
    void set_DDYN_UNT(bool value){ raw = (raw & 0xff7fffffffffffff) | ((uint64_t)value & 0x1) << 55; }

    /** Gets Suppression Dynamic fully detection */
    bool get_DDYN_UNT() const { return (bool)(raw >> 55 & 0x1); }
        
    /** Sets system condition */
    void set_SZS(BS_208h_SZS value){ raw = (raw & 0xff9fffffffffffff) | ((uint64_t)value & 0x3) << 53; }

    /** Gets system condition */
    BS_208h_SZS get_SZS() const { return (BS_208h_SZS)(raw >> 53 & 0x3); }
        
    /** Sets Tempomat operation */
    void set_TM_AUS(bool value){ raw = (raw & 0xffefffffffffffff) | ((uint64_t)value & 0x1) << 52; }

    /** Gets Tempomat operation */
    bool get_TM_AUS() const { return (bool)(raw >> 52 & 0x1); }
        
    /** Sets Switching Difference ESP */
    void set_SLV_ESP(BS_208h_SLV_ESP value){ raw = (raw & 0xfff0ffffffffffff) | ((uint64_t)value & 0xf) << 48; }

    /** Gets Switching Difference ESP */
    BS_208h_SLV_ESP get_SLV_ESP() const { return (BS_208h_SLV_ESP)(raw >> 48 & 0xf); }
        
    /** Sets ESP brake engagement active */
    void set_BRE_AKT_ESP(bool value){ raw = (raw & 0xffff7fffffffffff) | ((uint64_t)value & 0x1) << 47; }

    /** Gets ESP brake engagement active */
    bool get_BRE_AKT_ESP() const { return (bool)(raw >> 47 & 0x1); }
        
    /** Sets ESP request: "N" Insert */
    void set_ANFN(BS_208h_ANFN value){ raw = (raw & 0xffff9fffffffffff) | ((uint64_t)value & 0x3) << 45; }

    /** Gets ESP request: "N" Insert */
    BS_208h_ANFN get_ANFN() const { return (BS_208h_ANFN)(raw >> 45 & 0x3); }
        
    /** Sets ART brake intervention active */
    void set_BRE_AKT_ART(bool value){ raw = (raw & 0xffffefffffffffff) | ((uint64_t)value & 0x1) << 44; }

    /** Gets ART brake intervention active */
    bool get_BRE_AKT_ART() const { return (bool)(raw >> 44 & 0x1); }
        
    /** Sets set braking torque (BR240 factor 1.8 larger). Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_MBRE_ESP(uint16_t value){ raw = (raw & 0xfffff000ffffffff) | ((uint64_t)value & 0xfff) << 32; }

    /** Gets set braking torque (BR240 factor 1.8 larger). Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_MBRE_ESP() const { return (uint16_t)(raw >> 32 & 0xfff); }
        
    /** Sets rotary direction wheel rear right */
    void set_DRTGHR(BS_208h_DRTGHR value){ raw = (raw & 0xffffffff3fffffff) | ((uint64_t)value & 0x3) << 30; }

    /** Gets rotary direction wheel rear right */
    BS_208h_DRTGHR get_DRTGHR() const { return (BS_208h_DRTGHR)(raw >> 30 & 0x3); }
        
    /** Sets Rear wheel speed. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_DHR(uint16_t value){ raw = (raw & 0xffffffffc000ffff) | ((uint64_t)value & 0x3fff) << 16; }

    /** Gets Rear wheel speed. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_DHR() const { return (uint16_t)(raw >> 16 & 0x3fff); }
        
    /** Sets rotary direction wheel rear left */
    void set_DRTGHL(BS_208h_DRTGHL value){ raw = (raw & 0xffffffffffff3fff) | ((uint64_t)value & 0x3) << 14; }

    /** Gets rotary direction wheel rear left */
    BS_208h_DRTGHL get_DRTGHL() const { return (BS_208h_DRTGHL)(raw >> 14 & 0x3); }
        
    /** Sets Rear wheel speed. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_DHL(uint16_t value){ raw = (raw & 0xffffffffffffc000) | ((uint64_t)value & 0x3fff) << 0; }

    /** Gets Rear wheel speed. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_DHL() const { return (uint16_t)(raw >> 0 & 0x3fff); }
        
} BS_208;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of BS_270 */
	uint32_t get_canid(){ return BS_270_CAN_ID; }
    /** Sets Impulse ring counter wheel rear left (48 per revolution). Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_RIZ_HL(uint8_t value){ raw = (raw & 0x00ffffffffffffff) | ((uint64_t)value & 0xff) << 56; }

    /** Gets Impulse ring counter wheel rear left (48 per revolution). Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_RIZ_HL() const { return (uint8_t)(raw >> 56 & 0xff); }
        
    /** Sets Impulse ring counter wheel rear right (48 per revolution). Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_RIZ_HR(uint8_t value){ raw = (raw & 0xff00ffffffffffff) | ((uint64_t)value & 0xff) << 48; }

    /** Gets Impulse ring counter wheel rear right (48 per revolution). Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_RIZ_HR() const { return (uint8_t)(raw >> 48 & 0xff); }
        
    /** Sets Alerts PlatRollwarner */
    void set_PRW_WARN(BS_270h_PRW_WARN value){ raw = (raw & 0xffff0fffffffffff) | ((uint64_t)value & 0xf) << 44; }

    /** Gets Alerts PlatRollwarner */
    BS_270h_PRW_WARN get_PRW_WARN() const { return (BS_270h_PRW_WARN)(raw >> 44 & 0xf); }
        
    /** Sets Status flat tyre warner */
    void set_PRW_ST(BS_270h_PRW_ST value){ raw = (raw & 0xfffff8ffffffffff) | ((uint64_t)value & 0x7) << 40; }

    /** Gets Status flat tyre warner */
    BS_270h_PRW_ST get_PRW_ST() const { return (BS_270h_PRW_ST)(raw >> 40 & 0x7); }
        
} BS_270;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of BS_300 */
	uint32_t get_canid(){ return BS_300_CAN_ID; }
    /** Sets Engine torque Request Parity (just parity) */
    void set_DMPAR_ART(bool value){ raw = (raw & 0x7fffffffffffffff) | ((uint64_t)value & 0x1) << 63; }

    /** Gets Engine torque Request Parity (just parity) */
    bool get_DMPAR_ART() const { return (bool)(raw >> 63 & 0x1); }
        
    /** Sets Engine torque request dynamic */
    void set_DMDYN_ART(bool value){ raw = (raw & 0xbfffffffffffffff) | ((uint64_t)value & 0x1) << 62; }

    /** Gets Engine torque request dynamic */
    bool get_DMDYN_ART() const { return (bool)(raw >> 62 & 0x1); }
        
    /** Sets Bas-control active */
    void set_BAS_AKT(bool value){ raw = (raw & 0xdfffffffffffffff) | ((uint64_t)value & 0x1) << 61; }

    /** Gets Bas-control active */
    bool get_BAS_AKT() const { return (bool)(raw >> 61 & 0x1); }
        
    /** Sets full braking (ABS regulates all 4 wheels) */
    void set_VOLLBRE(bool value){ raw = (raw & 0xefffffffffffffff) | ((uint64_t)value & 0x1) << 60; }

    /** Gets full braking (ABS regulates all 4 wheels) */
    bool get_VOLLBRE() const { return (bool)(raw >> 60 & 0x1); }
        
    /** Sets Enable Art */
    void set_ART_E(bool value){ raw = (raw & 0xf7ffffffffffffff) | ((uint64_t)value & 0x1) << 59; }

    /** Gets Enable Art */
    bool get_ART_E() const { return (bool)(raw >> 59 & 0x1); }
        
    /** Sets ESP giermom control active */
    void set_ESP_GIER_AKT(bool value){ raw = (raw & 0xfbffffffffffffff) | ((uint64_t)value & 0x1) << 58; }

    /** Gets ESP giermom control active */
    bool get_ESP_GIER_AKT() const { return (bool)(raw >> 58 & 0x1); }
        
    /** Sets Initialization Steering Angle Sensor O.K. */
    void set_LWS_INI_OK(bool value){ raw = (raw & 0xfdffffffffffffff) | ((uint64_t)value & 0x1) << 57; }

    /** Gets Initialization Steering Angle Sensor O.K. */
    bool get_LWS_INI_OK() const { return (bool)(raw >> 57 & 0x1); }
        
    /** Sets Initialization steering angle sensor possible */
    void set_LWS_INI_EIN(bool value){ raw = (raw & 0xfeffffffffffffff) | ((uint64_t)value & 0x1) << 56; }

    /** Gets Initialization steering angle sensor possible */
    bool get_LWS_INI_EIN() const { return (bool)(raw >> 56 & 0x1); }
        
    /** Sets Engine torque Request Parity (just parity) */
    void set_MPAR_ESP(bool value){ raw = (raw & 0xff7fffffffffffff) | ((uint64_t)value & 0x1) << 55; }

    /** Gets Engine torque Request Parity (just parity) */
    bool get_MPAR_ESP() const { return (bool)(raw >> 55 & 0x1); }
        
    /** Sets Engine torque request dynamic */
    void set_MDYN_ESP(bool value){ raw = (raw & 0xffbfffffffffffff) | ((uint64_t)value & 0x1) << 54; }

    /** Gets Engine torque request dynamic */
    bool get_MDYN_ESP() const { return (bool)(raw >> 54 & 0x1); }
        
    /** Sets drive torque control active */
    void set_AMR_AKT_ESP(bool value){ raw = (raw & 0xffdfffffffffffff) | ((uint64_t)value & 0x1) << 53; }

    /** Gets drive torque control active */
    bool get_AMR_AKT_ESP() const { return (bool)(raw >> 53 & 0x1); }
        
    /** Sets Send cycle time */
    void set_T_Z(BS_300h_T_Z value){ raw = (raw & 0xffe7ffffffffffff) | ((uint64_t)value & 0x3) << 51; }

    /** Gets Send cycle time */
    BS_300h_T_Z get_T_Z() const { return (BS_300h_T_Z)(raw >> 51 & 0x3); }
        
    /** Sets driver brakes parity (straight parity) */
    void set_SFB_PA(bool value){ raw = (raw & 0xfffbffffffffffff) | ((uint64_t)value & 0x1) << 50; }

    /** Gets driver brakes parity (straight parity) */
    bool get_SFB_PA() const { return (bool)(raw >> 50 & 0x1); }
        
    /** Sets driver brakes */
    void set_SFB(BS_300h_SFB value){ raw = (raw & 0xfffcffffffffffff) | ((uint64_t)value & 0x3) << 48; }

    /** Gets driver brakes */
    BS_300h_SFB get_SFB() const { return (BS_300h_SFB)(raw >> 48 & 0x3); }
        
    /** Sets Motor torque toggle 40ms + -10 */
    void set_DMTGL_ART(bool value){ raw = (raw & 0xffff7fffffffffff) | ((uint64_t)value & 0x1) << 47; }

    /** Gets Motor torque toggle 40ms + -10 */
    bool get_DMTGL_ART() const { return (bool)(raw >> 47 & 0x1); }
        
    /** Sets Engine torque request min */
    void set_DMMIN_ART(bool value){ raw = (raw & 0xffffbfffffffffff) | ((uint64_t)value & 0x1) << 46; }

    /** Gets Engine torque request min */
    bool get_DMMIN_ART() const { return (bool)(raw >> 46 & 0x1); }
        
    /** Sets Engine torque request max */
    void set_DMMAX_ART(bool value){ raw = (raw & 0xffffdfffffffffff) | ((uint64_t)value & 0x1) << 45; }

    /** Gets Engine torque request max */
    bool get_DMMAX_ART() const { return (bool)(raw >> 45 & 0x1); }
        
    /** Sets Ford.Engine torque. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_DM_ART(uint16_t value){ raw = (raw & 0xffffe000ffffffff) | ((uint64_t)value & 0x1fff) << 32; }

    /** Gets Ford.Engine torque. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_DM_ART() const { return (uint16_t)(raw >> 32 & 0x1fff); }
        
    /** Sets Motor torque toggle 40ms + -10 */
    void set_MTGL_ESP(bool value){ raw = (raw & 0xffffffff7fffffff) | ((uint64_t)value & 0x1) << 31; }

    /** Gets Motor torque toggle 40ms + -10 */
    bool get_MTGL_ESP() const { return (bool)(raw >> 31 & 0x1); }
        
    /** Sets Engine torque request min */
    void set_MMIN_ESP(bool value){ raw = (raw & 0xffffffffbfffffff) | ((uint64_t)value & 0x1) << 30; }

    /** Gets Engine torque request min */
    bool get_MMIN_ESP() const { return (bool)(raw >> 30 & 0x1); }
        
    /** Sets Engine torque request max */
    void set_MMAX_ESP(bool value){ raw = (raw & 0xffffffffdfffffff) | ((uint64_t)value & 0x1) << 29; }

    /** Gets Engine torque request max */
    bool get_MMAX_ESP() const { return (bool)(raw >> 29 & 0x1); }
        
    /** Sets Ford.Engine torque. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_M_ESP(uint16_t value){ raw = (raw & 0xffffffffe000ffff) | ((uint64_t)value & 0x1fff) << 16; }

    /** Gets Ford.Engine torque. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_M_ESP() const { return (uint16_t)(raw >> 16 & 0x1fff); }
        
    /** Sets raw signal yaw rate without reconciliation / filtering (+ = left). Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_GIER_ROH(uint16_t value){ raw = (raw & 0xffffffffffff0000) | ((uint64_t)value & 0xffff) << 0; }

    /** Gets raw signal yaw rate without reconciliation / filtering (+ = left). Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_GIER_ROH() const { return (uint16_t)(raw >> 0 & 0xffff); }
        
} BS_300;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of BS_328 */
	uint32_t get_canid(){ return BS_328_CAN_ID; }
    /** Sets WMS Parity (straight parity) */
    void set_WMS_PA(bool value){ raw = (raw & 0x7fffffffffffffff) | ((uint64_t)value & 0x1) << 63; }

    /** Gets WMS Parity (straight parity) */
    bool get_WMS_PA() const { return (bool)(raw >> 63 & 0x1); }
        
    /** Sets target wobble moment change. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_WMS(uint16_t value){ raw = (raw & 0x8000ffffffffffff) | ((uint64_t)value & 0x7fff) << 48; }

    /** Gets target wobble moment change. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_WMS() const { return (uint16_t)(raw >> 48 & 0x7fff); }
        
    /** Sets Vehicle lateral acceleration. The focus (+ = left). Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_AY_S(uint8_t value){ raw = (raw & 0xffff00ffffffffff) | ((uint64_t)value & 0xff) << 40; }

    /** Gets Vehicle lateral acceleration. The focus (+ = left). Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_AY_S() const { return (uint8_t)(raw >> 40 & 0xff); }
        
    /** Sets ESP display messages */
    void set_ESP_DSPL(BS_328h_ESP_DSPL value){ raw = (raw & 0xffffffffe0ffffff) | ((uint64_t)value & 0x1f) << 24; }

    /** Gets ESP display messages */
    BS_328h_ESP_DSPL get_ESP_DSPL() const { return (BS_328h_ESP_DSPL)(raw >> 24 & 0x1f); }
        
    /** Sets Emergency braking (brake light blink) */
    void set_NOTBRE(bool value){ raw = (raw & 0xffffffffffbfffff) | ((uint64_t)value & 0x1) << 22; }

    /** Gets Emergency braking (brake light blink) */
    bool get_NOTBRE() const { return (bool)(raw >> 22 & 0x1); }
        
    /** Sets Open clutch */
    void set_KPL_OEF(bool value){ raw = (raw & 0xfffffffffff7ffff) | ((uint64_t)value & 0x1) << 19; }

    /** Gets Open clutch */
    bool get_KPL_OEF() const { return (bool)(raw >> 19 & 0x1); }
        
    /** Sets Message counter. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_BZ328h(uint8_t value){ raw = (raw & 0xfffffffffff8ffff) | ((uint64_t)value & 0x7) << 16; }

    /** Gets Message counter. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_BZ328h() const { return (uint8_t)(raw >> 16 & 0x7); }
        
    /** Sets Impulse ring counter wheel front left (48 per revolution). Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_RIZ_VL(uint8_t value){ raw = (raw & 0xffffffffffff00ff) | ((uint64_t)value & 0xff) << 8; }

    /** Gets Impulse ring counter wheel front left (48 per revolution). Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_RIZ_VL() const { return (uint8_t)(raw >> 8 & 0xff); }
        
    /** Sets Impulse ring counter wheel front right (48 per revolution). Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_RIZ_VR(uint8_t value){ raw = (raw & 0xffffffffffffff00) | ((uint64_t)value & 0xff) << 0; }

    /** Gets Impulse ring counter wheel front right (48 per revolution). Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_RIZ_VR() const { return (uint8_t)(raw >> 0 & 0xff); }
        
} BS_328;



class ECU_ESP_SBC {
	public:
        /**
         * @brief Imports the CAN frame given the CAN ID, CAN Contents, and current timestamp
         *
         * Returns true if the frame was imported successfully, and false if import failed (Due to non-matching CAN ID).
         *
         * NOTE: The endianness of the value cannot be guaranteed. It is up to the caller to correct the byte order!
         */
        bool import_frames(uint64_t value, uint32_t can_id, uint64_t timestamp_now) {
            switch(can_id) {
                case BS_200_CAN_ID:
                    LAST_FRAME_TIMES[0] = timestamp_now;
                    FRAME_DATA[0] = value;
                    return true;
                case BS_208_CAN_ID:
                    LAST_FRAME_TIMES[1] = timestamp_now;
                    FRAME_DATA[1] = value;
                    return true;
                case BS_270_CAN_ID:
                    LAST_FRAME_TIMES[2] = timestamp_now;
                    FRAME_DATA[2] = value;
                    return true;
                case BS_300_CAN_ID:
                    LAST_FRAME_TIMES[3] = timestamp_now;
                    FRAME_DATA[3] = value;
                    return true;
                case BS_328_CAN_ID:
                    LAST_FRAME_TIMES[4] = timestamp_now;
                    FRAME_DATA[4] = value;
                    return true;
                default:
                    return false;
            }
        }
        
        /** Sets data in pointer to BS_200
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_BS_200(uint64_t now, uint64_t max_expire_time, BS_200* dest) const {
            if (LAST_FRAME_TIMES[0] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[0] && now - LAST_FRAME_TIMES[0] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[0];
                return true;
            }
        }
            
        /** Sets data in pointer to BS_208
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_BS_208(uint64_t now, uint64_t max_expire_time, BS_208* dest) const {
            if (LAST_FRAME_TIMES[1] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[1] && now - LAST_FRAME_TIMES[1] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[1];
                return true;
            }
        }
            
        /** Sets data in pointer to BS_270
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_BS_270(uint64_t now, uint64_t max_expire_time, BS_270* dest) const {
            if (LAST_FRAME_TIMES[2] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[2] && now - LAST_FRAME_TIMES[2] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[2];
                return true;
            }
        }
            
        /** Sets data in pointer to BS_300
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_BS_300(uint64_t now, uint64_t max_expire_time, BS_300* dest) const {
            if (LAST_FRAME_TIMES[3] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[3] && now - LAST_FRAME_TIMES[3] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[3];
                return true;
            }
        }
            
        /** Sets data in pointer to BS_328
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_BS_328(uint64_t now, uint64_t max_expire_time, BS_328* dest) const {
            if (LAST_FRAME_TIMES[4] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[4] && now - LAST_FRAME_TIMES[4] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[4];
                return true;
            }
        }
            
	private:
		uint64_t FRAME_DATA[5];
		uint64_t LAST_FRAME_TIMES[5];
};
#endif // __ECU_ESP_SBC_H_

#endif // EGS52_MODE