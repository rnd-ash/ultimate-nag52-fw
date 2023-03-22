
/**
* AUTOGENERATED BY convert.py
* DO NOT EDIT THIS FILE!
*
* IF MODIFICATIONS NEED TO BE MADE, MODIFY can_data.txt!
*
* CAN Defintiion for ECU 'TSLM'
*/

#ifndef __ECU_TSLM_H_
#define __ECU_TSLM_H_

#include <stdint.h>
    
#define SBW_RS_ISM_EGS53_CAN_ID 0x0073
#define NM_TSLM_EGS53_CAN_ID 0x042F

/** Transmission Selector Lever Position / Transmission Logging */
enum class SBW_RS_ISM_TSL_Posn_ISM_EGS53 : uint16_t {
	D = 5, // Transmission Selector Lever in position "D"
	N = 6, // Transmission Selector Lever in position "N"
	R = 7, // Transmission Selector Lever in position "R"
	P = 8, // Transmission Selector Lever in position "P"
	PLUS = 9, // Transmission Selector Lever in position "+"
	MINUS = 10, // Transmission Selector Lever in position "-"
	N_ZW_D = 11, // Transmission Selector Lever in Intermediate Position "N-D"
	R_ZW_N = 12, // Transmission Selector Lever in Intermediate Position "R-N"
	P_ZW_R = 13, // Transmission Selector Lever in Intermediate Position "P-R"
	SNA = 15, // Signal Not Available
};

/** Network Management Mode / Network Management Mode */
enum class NM_TSLM_NM_Mode_EGS53 : uint16_t {
	LHOM = 252, // LIMP-HOME Fashion
	RING = 253, // ring fashion
	ALIVE = 254, // Alive mode
	SNA = 255, // Signal Not Available
};

/** Network Management UserData Launch Type / Network Management UserData Sendart */
enum class NM_TSLM_NM_Ud_Launch_EGS53 : uint16_t {
	BROADCAST = 4, // Broadcast or Start Alive
	SNA = 63, // Signal Not Available
};

/** Network Management UserData Service No./netzmanagement UserData service */
enum class NM_TSLM_NM_Ud_Srv_EGS53 : uint16_t {
	DATA_OK_BC = 1, // UserData Transmission OK (Broadcast)
	WAKEUP_SA = 2, // Wakeup status (start alive)
	SBC_STAT_BC = 5, // System Base Chip Status (Broadcast)
	AWAKE_BC = 15, // Stay Awake Reason (Broadcast)
	SNA = 255, // Signal Not Available
};

/** Wakeup Reason / Wake-up */
enum class NM_TSLM_WakeupRsn_TSLM_EGS53 : uint16_t {
	NETWORK = 0, // Wakeup by Network
	SNA = 255, // Signal Not Available
};

/** Network Identification No./netzwerk-id */
enum class NM_TSLM_Nw_Id_EGS53 : uint16_t {
	BACKBONE = 4, // Backbone CAN
	DIAGNOSTICS = 5, // Diagnostics CAN
	BODY = 6, // Body CAN
	CHASSIS = 7, // Chassis CAN
	POWERTRAIN = 8, // Powertrain Can
	PT_SENSOR = 9, // Powertrain Sensor CAN
	DYNAMICS = 11, // Dynamics CAN
	HEADUNIT = 14, // HeadUnit CAN
	IMPACT = 15, // Impact CAN
	MULTIPURPOSE = 16, // Multipurpose CAN
	SNA = 255, // Signal Not Available
};



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		/** CRC Checksum Byte 1 to 7 Accordinging to SAE J1850 / CRC Checksum Byte 1 - 7 to SAE J1850 **/
		uint8_t CRC_SBW_RS_ISM: 8;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING1__: 4;
		/** Message Counter / Message Counter **/
		uint8_t MC_SBW_RS_ISM: 4;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint64_t __PADDING2__: 40;
		/** Transmission Selector Lever Position / Transmission Logging **/
		SBW_RS_ISM_TSL_Posn_ISM_EGS53 TSL_Posn_ISM: 4;
		/** Transmission Selector Lever Motion Lock Active / Gear Select Lock Active **/
		bool TSL_MtnLk_Actv: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		bool __PADDING3__: 1;
		/** Transmission Driving Program Switch State / Status Driving program **/
		bool TxDrvProgSw_Psd_V3: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		bool __PADDING4__: 1;
	} __attribute__((packed));
	/** Gets CAN ID of SBW_RS_ISM_EGS53 **/
	uint32_t get_canid(){ return SBW_RS_ISM_EGS53_CAN_ID; }
} SBW_RS_ISM_EGS53;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		/** Network Identification No./netzwerk-id **/
		NM_TSLM_Nw_Id_EGS53 Nw_Id: 8;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING1__: 8;
		/** Counter for Module Wakeup States During Network Sleep / Counter for ECUs Internal Wachzustäustände during bus rest **/
		uint8_t WakeupCnt: 8;
		/** Wakeup Reason / Wake-up **/
		NM_TSLM_WakeupRsn_TSLM_EGS53 WakeupRsn_TSLM: 8;
		/** Network Management UserData Service No./netzmanagement UserData service **/
		NM_TSLM_NM_Ud_Srv_EGS53 NM_Ud_Srv: 8;
		/** Network Management UserData Launch Type / Network Management UserData Sendart **/
		NM_TSLM_NM_Ud_Launch_EGS53 NM_Ud_Launch: 6;
		/** Network Management Sleep Acknowledge / Network Management Sleep Acknowledge **/
		bool NM_Sleep_Ack: 1;
		/** Network Management Sleep Indication / Network Management Sleep Indication **/
		bool NM_Sleep_Ind: 1;
		/** Network Management Logical Successor / Network Management Logical Successor **/
		uint8_t NM_Successor: 8;
		/** Network Management Mode / Network Management Mode **/
		NM_TSLM_NM_Mode_EGS53 NM_Mode: 8;
	} __attribute__((packed));
	/** Gets CAN ID of NM_TSLM_EGS53 **/
	uint32_t get_canid(){ return NM_TSLM_EGS53_CAN_ID; }
} NM_TSLM_EGS53;



class ECU_TSLM {
	public:
        /**
         * @brief Imports the CAN frame given the CAN ID, CAN Contents, and current timestamp
         *
         * Returns true if the frame was imported successfully, and false if import failed (Due to non-matching CAN ID).
         *
         * NOTE: The endianness of the value cannot be guaranteed. It is up to the caller to correct the byte order!
         */
        bool import_frames(uint64_t value, uint32_t can_id, uint64_t timestamp_now) {
            uint8_t idx = 0;
            bool add = true;
            switch(can_id) {
                case SBW_RS_ISM_EGS53_CAN_ID:
                    idx = 0;
                    break;
                case NM_TSLM_EGS53_CAN_ID:
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
        
        /** Sets data in pointer to SBW_RS_ISM
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_SBW_RS_ISM(uint64_t now, uint64_t max_expire_time, SBW_RS_ISM_EGS53* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[0] <= now && now - LAST_FRAME_TIMES[0] < max_expire_time) {
                dest->raw = FRAME_DATA[0];
                ret = true;
            }
            return ret;
        }
            
        /** Sets data in pointer to NM_TSLM
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_NM_TSLM(uint64_t now, uint64_t max_expire_time, NM_TSLM_EGS53* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[1] <= now && now - LAST_FRAME_TIMES[1] < max_expire_time) {
                dest->raw = FRAME_DATA[1];
                ret = true;
            }
            return ret;
        }
            
	private:
		uint64_t FRAME_DATA[2];
		uint64_t LAST_FRAME_TIMES[2];
};
#endif // __ECU_TSLM_H_