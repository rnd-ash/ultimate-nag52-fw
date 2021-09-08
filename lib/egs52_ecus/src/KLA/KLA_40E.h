#ifndef KLA_40EH_H_
#define KLA_40EH_H_

/**
 * AUTOGENERATED BY gen_unions.py
*/
#include <stdint.h>

#define KLA_40E_ID 0x040E






typedef union {
    uint8_t bytes[8];
    uint64_t raw;

    // Sets Heating power requirement
    void set_HZL_ANF(uint8_t value){ raw = (raw & 0x00ffffffffffffff) | ((uint64_t)value & 0xff) << 56; }
    // Gets Heating power requirement
    uint8_t get_HZL_ANF() { return raw >> 56 & 0xff; }

    /** Imports the frame data from a source */
    void import_frame(uint32_t cid, uint8_t* data, uint8_t len) {
        if (cid == KLA_40E_ID) {
            for (int i = 0; i < len; i++) {
                bytes[7-i] = data[i];
            }
        }
    }

    /** Exports the frame data to a destination */
    void export_frame(uint32_t* cid, uint8_t* data, uint8_t* len) {
        *cid = KLA_40E_ID;
        *len = 8;
        for (int i = 0; i < *len; i++) {
            data[i] = bytes[7-i];
        }
    }
} KLA_40E;

#endif
