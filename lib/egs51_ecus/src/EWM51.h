
/**
* AUTOGENERATED BY convert.py
* DO NOT EDIT THIS FILE!
*
* IF MODIFICATIONS NEED TO BE MADE, MODIFY can_data.txt!
*
* CAN Defintiion for ECU 'EWM51'
*/

#ifndef __ECU_EWM51_H_
#define __ECU_EWM51_H_

#include <stdint.h>
    
#define EWM_230_EGS51_CAN_ID 0x0230

/** gear selector lever position (NAG only) */
enum class EWM_230h_WHC_EGS51 : uint16_t {
	D = 5, // selector lever in position "D"
	N = 6, // selector lever in position "N"
	R = 7, // selector lever in position "R"
	P = 8, // selector lever in position "P"
	PLUS = 9, // selector lever in position "+"
	MINUS = 10, // selector lever in position "-"
	N_ZW_D = 11, // selector lever in intermediate position "N-D"
	R_ZW_N = 12, // selector lever in intermediate position "R-N"
	P_ZW_R = 13, // selector lever in intermediate position "P-R"
	SNV = 15, // selector lever position unplausible
};



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint64_t __PADDING1__: 56;
		/** gear selector lever position (NAG only) **/
		EWM_230h_WHC_EGS51 WHC: 4;
		/** barrier magnet energized **/
		bool SPERR: 1;
		/** Kickdown **/
		bool KD: 1;
		/** Driving program button actuated **/
		bool FPT: 1;
		/** Driving program **/
		bool W_S: 1;
	} __attribute__((packed));
	/** Gets CAN ID of EWM_230_EGS51 **/
	uint32_t get_canid(){ return EWM_230_EGS51_CAN_ID; }
} EWM_230_EGS51;



class ECU_EWM51 {
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
                case EWM_230_EGS51_CAN_ID:
                    idx = 0;
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
        
        /** Sets data in pointer to EWM_230
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_EWM_230(const uint32_t now, const uint32_t max_expire_time, EWM_230_EGS51* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[0] <= now && now - LAST_FRAME_TIMES[0] < max_expire_time) {
                dest->raw = FRAME_DATA[0];
                ret = true;
            }
            return ret;
        }
            
	private:
		uint64_t FRAME_DATA[1];
		uint32_t LAST_FRAME_TIMES[1];
};
#endif // __ECU_EWM51_H_