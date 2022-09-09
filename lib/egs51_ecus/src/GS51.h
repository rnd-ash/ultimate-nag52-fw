
/**
* AUTOGENERATED BY convert.py
* DO NOT EDIT THIS FILE!
*
* IF MODIFICATIONS NEED TO BE MADE, MODIFY can_data.txt!
*
* CAN Defintiion for ECU 'GS51'
*/

#ifdef EGS51_MODE

#ifndef __ECU_GS51_H_
#define __ECU_GS51_H_

#include <stdint.h>
    
#define GS_218_CAN_ID 0x0218

/** Target gear */
enum class GS_218h_GZC {
	G_N = 0, // Destination "N"
	G_D1 = 1, // Destination "1"
	G_D2 = 2, // Destination "2"
	G_D3 = 3, // Destination "3"
	G_D4 = 4, // Destination "4"
	G_D5 = 5, // Destination "5"
	G_R = 6, // Destination "R"
	G_R2 = 7, // Destination "R2"
	G_P = 8, // Destination "P"
	G_SNV = 15, // signal not available
};

/** actual gear */
enum class GS_218h_GIC {
	G_N = 0, // Destination "N"
	G_D1 = 1, // Destination "1"
	G_D2 = 2, // Destination "2"
	G_D3 = 3, // Destination "3"
	G_D4 = 4, // Destination "4"
	G_D5 = 5, // Destination "5"
	G_R = 6, // Destination "R"
	G_R2 = 7, // Destination "R2"
	G_P = 8, // Destination "P"
	G_SNV = 15, // signal not available
};



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of GS_218 */
	uint32_t get_canid(){ return GS_218_CAN_ID; }
    /** Sets Torque request value. 0xFE when inactive. Conversion formula (To raw from real): y=(x-0.0)/2.00 */
    void set_TORQUE_REQ(uint8_t value){ raw = (raw & 0x00ffffffffffffff) | ((uint64_t)value & 0xff) << 56; }

    /** Gets Torque request value. 0xFE when inactive. Conversion formula (To real from raw): y=(2.00x)+0.0 */
    uint8_t get_TORQUE_REQ() const { return (uint8_t)(raw >> 56 & 0xff); }
        
    /** Sets Enable torque request */
    void set_TORQUE_REQ_EN(bool value){ raw = (raw & 0xffbfffffffffffff) | ((uint64_t)value & 0x1) << 54; }

    /** Gets Enable torque request */
    bool get_TORQUE_REQ_EN() const { return (bool)(raw >> 54 & 0x1); }
        
    /** Sets Target gear */
    void set_GZC(GS_218h_GZC value){ raw = (raw & 0xffff0fffffffffff) | ((uint64_t)value & 0xf) << 44; }

    /** Gets Target gear */
    GS_218h_GZC get_GZC() const { return (GS_218h_GZC)(raw >> 44 & 0xf); }
        
    /** Sets actual gear */
    void set_GIC(GS_218h_GIC value){ raw = (raw & 0xfffff0ffffffffff) | ((uint64_t)value & 0xf) << 40; }

    /** Gets actual gear */
    GS_218h_GIC get_GIC() const { return (GS_218h_GIC)(raw >> 40 & 0xf); }
        
    /** Sets Kickdown pressed */
    void set_KICKDOWN(bool value){ raw = (raw & 0xffffffbfffffffff) | ((uint64_t)value & 0x1) << 38; }

    /** Gets Kickdown pressed */
    bool get_KICKDOWN() const { return (bool)(raw >> 38 & 0x1); }
        
    /** Sets error number or counter for calid / CVN transmission. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_FEHLER(uint8_t value){ raw = (raw & 0xfffffffffff0ffff) | ((uint64_t)value & 0xf) << 16; }

    /** Gets error number or counter for calid / CVN transmission. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint8_t get_FEHLER() const { return (uint8_t)(raw >> 16 & 0xf); }
        
} GS_218;



class ECU_GS51 {
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
                case GS_218_CAN_ID:
                    LAST_FRAME_TIMES[0] = timestamp_now;
                    FRAME_DATA[0] = value;
                    return true;
                default:
                    return false;
            }
        }
        
        /** Sets data in pointer to GS_218
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_GS_218(uint64_t now, uint64_t max_expire_time, GS_218* dest) const {
            if (LAST_FRAME_TIMES[0] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[0] && now - LAST_FRAME_TIMES[0] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[0];
                return true;
            }
        }
            
	private:
		uint64_t FRAME_DATA[1];
		uint64_t LAST_FRAME_TIMES[1];
};
#endif // __ECU_GS51_H_

#endif // EGS51_MODE