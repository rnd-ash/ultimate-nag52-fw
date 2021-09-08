#ifndef BS_208H_H_
#define BS_208H_H_

/**
 * AUTOGENERATED BY gen_unions.py
*/
#include <stdint.h>

#define BS_208_ID 0x0208



enum class BS_208H_GMIN_ESP {
    PASSIVE = 0, /** Passive value */
    G1 = 1, /** target gear, lower limit = 1 */
    G2 = 2, /** target gear, lower limit = 2 */
    G3 = 3, /** target gear, lower limit = 3 */
    G4 = 4, /** target gear, lower limit = 4 */
    G5 = 5, /** target gear, lower limit = 5 */
    G6 = 6, /** target gear, lower limit = 6 */
    G7 = 7, /** target gear, lower limit = 7 */
};

enum class BS_208H_GMAX_ESP {
    PASSIVE = 0, /** Passive value */
    G1 = 1, /** target gear, upper limit = 1 */
    G2 = 2, /** target gear, upper limit = 2 */
    G3 = 3, /** target gear, upper limit = 3 */
    G4 = 4, /** target gear, upper limit = 4 */
    G5 = 5, /** target gear, upper limit = 5 */
    G6 = 6, /** target gear, upper limit = 6 */
    G7 = 7, /** target gear, upper limit = 7 */
};

enum class BS_208H_SLV_ESP {
    SKL0 = 0, /** switching characteristic "0" */
    SKL1 = 1, /** switching characteristic "1" */
    SKL2 = 2, /** switching characteristic "2" */
    SKL3 = 3, /** switching characteristic "3" */
    ​​SKL4 = 4, /** switching characteristic "4" */
    SKL5 = 5, /** switching characteristic "5" */
    SKL6 = 6, /** switching characteristic "6" */
    SKL7 = 7, /** switching characteristic "7" */
    SKL8 = 8, /** switching characteristic "8" */
    SKL9 = 9, /** switching characteristic "9" */
    SKL10 = 10, /** switching characteristic "10" */
};

enum class BS_208H_SZS {
    ERR_DIAG = 0, /** system error or diagnosis */
    NORM = 1, /** normal operation */
    NOT_DEFINED = 2, /** not defined */
    EXHAUST = 3, /** Emissions test */
};

enum class BS_208H_ANFN {
    NOT_DEFINED = 0, /** not defined */
    ANF_N = 1, /** "Neutral" request */
    IDLE = 2, /** No requirement */
    SNV = 3, /** signal not available */
};

enum class BS_208H_DRTGHR {
    PASSIVE = 0, /** No direction of rotation detection */
    FORWARD = 1, /** direction of rotation forward */
    REVERSE = 2, /** direction of rotation backwards */
    SNV = 3, /** signal not available */
};

enum class BS_208H_DRTGHL {
    PASSIVE = 0, /** No direction of rotation detection */
    FORWARD = 1, /** direction of rotation forward */
    REVERSE = 2, /** direction of rotation backwards */
    SNV = 3, /** signal not available */
};




typedef union {
    uint8_t bytes[8];
    uint64_t raw;

    // Sets target gear, lower limit
    void set_BS_208H_GMIN_ESP(uint8_t value){ raw = (raw & 0xf8ffffffffffffff) | ((uint64_t)value & 0x7) << 56; }
    // Gets target gear, lower limit
    BS_208H_GMIN_ESP get_GMIN_ESP() { return (BS_208H_GMIN_ESP)(raw >> 56 & 0x7); }

    // Sets target gear, upper limit
    void set_BS_208H_GMAX_ESP(uint8_t value){ raw = (raw & 0xc7ffffffffffffff) | ((uint64_t)value & 0x7) << 59; }
    // Gets target gear, upper limit
    BS_208H_GMAX_ESP get_GMAX_ESP() { return (BS_208H_GMAX_ESP)(raw >> 59 & 0x7); }

    // Sets Target gear request from ART
    void set_MINMAX_ART(bool value){ raw = (raw & 0xbfffffffffffffff) | ((uint64_t)value & 0x1) << 62; }
    // Gets Target gear request from ART
    bool get_MINMAX_ART() { return raw >> 62 & 0x1; }

    // Sets ESP / ART request: "Active downshift"
    void set_AKT_R_ESP(bool value){ raw = (raw & 0x7fffffffffffffff) | ((uint64_t)value & 0x1) << 63; }
    // Gets ESP / ART request: "Active downshift"
    bool get_AKT_R_ESP() { return raw >> 63 & 0x1; }

    // Sets switching line shift ESP
    void set_BS_208H_SLV_ESP(uint8_t value){ raw = (raw & 0xfff0ffffffffffff) | ((uint64_t)value & 0xf) << 48; }
    // Gets switching line shift ESP
    BS_208H_SLV_ESP get_SLV_ESP() { return (BS_208H_SLV_ESP)(raw >> 48 & 0xf); }

    // Sets Cruise control mode off
    void set_TM_AUS(bool value){ raw = (raw & 0xffefffffffffffff) | ((uint64_t)value & 0x1) << 52; }
    // Gets Cruise control mode off
    bool get_TM_AUS() { return raw >> 52 & 0x1; }

    // Sets system status
    void set_BS_208H_SZS(uint8_t value){ raw = (raw & 0xff9fffffffffffff) | ((uint64_t)value & 0x3) << 53; }
    // Gets system status
    BS_208H_SZS get_SZS() { return (BS_208H_SZS)(raw >> 53 & 0x3); }

    // Sets Suppression of dynamic full load downshift
    void set_DDYN_UNT(bool value){ raw = (raw & 0xff7fffffffffffff) | ((uint64_t)value & 0x1) << 55; }
    // Gets Suppression of dynamic full load downshift
    bool get_DDYN_UNT() { return raw >> 55 & 0x1; }

    // Sets ART brake intervention active
    void set_BRE_AKT_ART(bool value){ raw = (raw & 0xffffefffffffffff) | ((uint64_t)value & 0x1) << 44; }
    // Gets ART brake intervention active
    bool get_BRE_AKT_ART() { return raw >> 44 & 0x1; }

    // Sets ESP request: insert "N"
    void set_BS_208H_ANFN(uint8_t value){ raw = (raw & 0xffff9fffffffffff) | ((uint64_t)value & 0x3) << 45; }
    // Gets ESP request: insert "N"
    BS_208H_ANFN get_ANFN() { return (BS_208H_ANFN)(raw >> 45 & 0x3); }

    // Sets ESP brake intervention active
    void set_BRE_AKT_ESP(bool value){ raw = (raw & 0xffff7fffffffffff) | ((uint64_t)value & 0x1) << 47; }
    // Gets ESP brake intervention active
    bool get_BRE_AKT_ESP() { return raw >> 47 & 0x1; }

    // Sets Set braking torque (BR240 factor 1.8 greater)
    void set_MBRE_ESP(short value){ raw = (raw & 0xfffff000ffffffff) | ((uint64_t)value & 0xfff) << 32; }
    // Gets Set braking torque (BR240 factor 1.8 greater)
    short get_MBRE_ESP() { return raw >> 32 & 0xfff; }

    // Sets direction of rotation of rear wheel right
    void set_BS_208H_DRTGHR(uint8_t value){ raw = (raw & 0xffffffff3fffffff) | ((uint64_t)value & 0x3) << 30; }
    // Gets direction of rotation of rear wheel right
    BS_208H_DRTGHR get_DRTGHR() { return (BS_208H_DRTGHR)(raw >> 30 & 0x3); }

    // Sets wheel speed rear right
    void set_DHR(short value){ raw = (raw & 0xffffffffc000ffff) | ((uint64_t)value & 0x3fff) << 16; }
    // Gets wheel speed rear right
    short get_DHR() { return raw >> 16 & 0x3fff; }

    // Sets direction of rotation of rear left wheel
    void set_BS_208H_DRTGHL(uint8_t value){ raw = (raw & 0xffffffffffff3fff) | ((uint64_t)value & 0x3) << 14; }
    // Gets direction of rotation of rear left wheel
    BS_208H_DRTGHL get_DRTGHL() { return (BS_208H_DRTGHL)(raw >> 14 & 0x3); }

    // Sets rear left wheel speed
    void set_DHL(short value){ raw = (raw & 0xffffffffffffc000) | ((uint64_t)value & 0x3fff) << 0; }
    // Gets rear left wheel speed
    short get_DHL() { return raw >> 0 & 0x3fff; }

    /** Imports the frame data from a source */
    void import_frame(uint32_t cid, uint8_t* data, uint8_t len) {
        if (cid == BS_208_ID) {
            for (int i = 0; i < len; i++) {
                bytes[7-i] = data[i];
            }
        }
    }

    /** Exports the frame data to a destination */
    void export_frame(uint32_t* cid, uint8_t* data, uint8_t* len) {
        *cid = BS_208_ID;
        *len = 8;
        for (int i = 0; i < *len; i++) {
            data[i] = bytes[7-i];
        }
    }
} BS_208;

#endif
