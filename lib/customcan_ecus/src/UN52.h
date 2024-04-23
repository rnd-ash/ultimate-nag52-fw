
/**
* AUTOGENERATED BY convert.py
* DO NOT EDIT THIS FILE!
*
* IF MODIFICATIONS NEED TO BE MADE, MODIFY can_data.txt!
*
* CAN Defintiion for ECU 'UN52'
*/

#ifndef ECU_UN52_H
#define ECU_UN52_H

#include <stdint.h>
    
#define UN52_400_CUSTOMCAN_CAN_ID 0x0400
#define UN52_410_CUSTOMCAN_CAN_ID 0x0410

/** Target gear */
enum class UN52_400h_T_GEAR_CUSTOMCAN : uint16_t {
	P = 0, // Park
	N = 1, // Neutral
	R1 = 2, // Reverse 1
	R2 = 3, // Reverse 2
	D1 = 4, // Drive 1
	D2 = 5, // Drive 2
	D3 = 6, // Drive 3
	D4 = 7, // Drive 4
	D5 = 8, // Drive 5
	D6 = 9, // Drive 6
	D7 = 10, // Drive 7
	D8 = 11, // Drive 8
	D9 = 12, // Drive 9
	ABORT = 13, // Shift abort
	SNV = 255, // Signal not available
};

/** Actual gear */
enum class UN52_400h_A_GEAR_CUSTOMCAN : uint16_t {
	P = 0, // Park
	N = 1, // Neutral
	R1 = 2, // Reverse 1
	R2 = 3, // Reverse 2
	D1 = 4, // Drive 1
	D2 = 5, // Drive 2
	D3 = 6, // Drive 3
	D4 = 7, // Drive 4
	D5 = 8, // Drive 5
	D6 = 9, // Drive 6
	D7 = 10, // Drive 7
	D8 = 11, // Drive 8
	D9 = 12, // Drive 9
	P_FREE = 13, // Power free (Loss of drive detected)
	SNV = 255, // Signal not available
};

/** Profile */
enum class UN52_400h_PROFILE_CUSTOMCAN : uint16_t {
	P_S = 0, // S profile
	P_C = 1, // C profile
	P_W = 2, // W profile
	P_A = 3, // A profile
	P_M = 4, // M profile
	P_R = 5, // R profile
	P_U = 6, // User custom profile
	SNV = 255, // Profile not available
};



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING1__: 6;
		/** Gearbox OK **/
		bool GB_OK: 1;
		/** Garage shifting **/
		bool G_SHIFT: 1;
		/** Oil temperature **/
		uint8_t T_OEL: 8;
		/** Transmission output speed **/
		uint16_t OUTPUT_RPM: 16;
		/** Transmission input speed **/
		uint16_t INPUT_RPM: 16;
		/** Profile **/
		UN52_400h_PROFILE_CUSTOMCAN PROFILE: 8;
		/** Actual gear **/
		UN52_400h_A_GEAR_CUSTOMCAN A_GEAR: 8;
		/** Target gear **/
		UN52_400h_T_GEAR_CUSTOMCAN T_GEAR: 8;
	} __attribute__((packed));
	/** Gets CAN ID of UN52_400_CUSTOMCAN **/
	uint32_t get_canid(){ return UN52_400_CUSTOMCAN_CAN_ID; }
} UN52_400_CUSTOMCAN;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		/** Torque request target (Nm) **/
		uint16_t TRQ_REQ_TRQ: 16;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint64_t __PADDING1__: 43;
		/** Torque request watchdog toggle bit **/
		bool TRQ_REQ_TOGGLE: 1;
		/** Torque request max request **/
		bool TRQ_REQ_MAX: 1;
		/** Torque request min request **/
		bool TRQ_REQ_MIN: 1;
		/** Torque request control bit 1 **/
		bool TRQ_REQ_CTRL1: 1;
		/** Torque request control bit 0 **/
		bool TRQ_REQ_CTRL0: 1;
	} __attribute__((packed));
	/** Gets CAN ID of UN52_410_CUSTOMCAN **/
	uint32_t get_canid(){ return UN52_410_CUSTOMCAN_CAN_ID; }
} UN52_410_CUSTOMCAN;



class ECU_UN52 {
	public:
        /**
         * @brief Imports the CAN frame given the CAN ID, CAN Contents, and current timestamp
         *
         * Returns true if the frame was imported successfully, and false if import failed (Due to non-matching CAN ID).
         *
         * NOTE: The endianness of the value cannot be guaranteed. It is up to the caller to correct the byte order!
         */
        bool import_frames(uint64_t value, uint32_t can_id, uint32_t timestamp_now) {
            uint8_t idx = 0;
            bool add = true;
            switch(can_id) {
                case UN52_400_CUSTOMCAN_CAN_ID:
                    idx = 0;
                    break;
                case UN52_410_CUSTOMCAN_CAN_ID:
                    idx = 1;
                    break;
                default:
                    add = false;
                    break;
            }
            if (add) {
                LAST_FRAME_TIMES[idx] = timestamp_now;
                FRAME_DATA[idx] = value;
            }
            return add;
        }
        
        /** Sets data in pointer to UN52_400
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_UN52_400(const uint32_t now, const uint32_t max_expire_time, UN52_400_CUSTOMCAN* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[0] <= now && now - LAST_FRAME_TIMES[0] < max_expire_time) {
                dest->raw = FRAME_DATA[0];
                ret = true;
            }
            return ret;
        }
            
        /** Sets data in pointer to UN52_410
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_UN52_410(const uint32_t now, const uint32_t max_expire_time, UN52_410_CUSTOMCAN* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[1] <= now && now - LAST_FRAME_TIMES[1] < max_expire_time) {
                dest->raw = FRAME_DATA[1];
                ret = true;
            }
            return ret;
        }
            
	private:
		uint64_t FRAME_DATA[2];
		uint32_t LAST_FRAME_TIMES[2];
};
#endif // ECU_UN52_H