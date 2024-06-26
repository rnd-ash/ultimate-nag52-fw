
/**
* AUTOGENERATED BY convert.py
* DO NOT EDIT THIS FILE!
*
* IF MODIFICATIONS NEED TO BE MADE, MODIFY can_data.txt!
*
* CAN Defintiion for ECU 'HFM'
*/

#ifndef ECU_HFM_H
#define ECU_HFM_H

#include <stdint.h>
    
#define HFM_210_CAN_ID 0x0210
#define HFM_308_CAN_ID 0x0308
#define HFM_608_CAN_ID 0x0608
#define HFM_610_CAN_ID 0x0610

/** shifter module code */
enum class HFM_210h_WHC : uint16_t {
	PN_B = 0, // driving position P/N engaged (automatic transmission only)
	R_B = 3, // driving position reverse
	D4_B = 5, // driving position P/N not engaged (automatic transmission only)
	SCH = 10, // manual transmission
};

/** HFM coding */
enum class HFM_308h_HFM_COD : uint16_t {
	BR202E22 = 40, // M111 E22, model series 202
	BR124E22MPSV = 41, // M111 E22, model series 124 w/ partial intake manifold preheating
	BR124E22OPSV = 42, // M111 E22, model series 124 wo/ partial intake manifold preheating
	BR202E32 = 60, // M104 E32, model series 202
	BR124E32 = 61, // M104 E32, model series 124
	BR140E32 = 62, // M104 E32, model series 140
	BR129E32 = 63, // M104 E32, model series 129
	BR210E32 = 64, // M104 E32, model series 210
	BR202E28 = 65, // M104 E28, model series 202
	BR124E28 = 66, // M104 E28, model series 124
	BR140E28 = 67, // M104 E28, model series 140
	BR129E28 = 68, // M104 E28, model series 129
	BR210E28 = 69, // M104 E28, model series 210
	FFVE32 = 72, // M104 E32, flexible fuel engine
	BR140E32FE = 82, // M104 E32, model series 140, functional testing
};



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint16_t __PADDING1__: 16;
		/** throttle valve actual value implausible **/
		bool DKI_UP_B: 1;
		/** throttle valve target value implausible **/
		bool DKV_UP_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING2__: 4;
		/** vehicle speed signal implausible **/
		bool VSIG_UP_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING3__: 3;
		/** full throttle **/
		bool VG_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING4__: 2;
		/** brake light switch on **/
		bool BLS_B: 1;
		/** emergency mode **/
		bool NOTL_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		bool __PADDING5__: 1;
		/** vehicle speed **/
		uint8_t V_SIGNAL: 8;
		/** throttle valve target value **/
		uint8_t DKV: 8;
		/** throttle valve actual value **/
		uint8_t DKI: 8;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		bool __PADDING6__: 1;
		/** shifter module code **/
		HFM_210h_WHC WHC: 4;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING7__: 3;
	} __attribute__((packed));
	/** Gets CAN ID of HFM_210 **/
	uint32_t get_canid(){ return HFM_210_CAN_ID; }
} HFM_210;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint16_t __PADDING1__: 16;
		/** starter running **/
		bool KL50_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING2__: 6;
		/** air mass signal implausible **/
		bool HFM_UP_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING3__: 8;
		/** engine speed **/
		uint16_t NMOT: 16;
		/** HFM coding **/
		HFM_308h_HFM_COD HFM_COD: 8;
		/** engine speed implausible **/
		bool NMOT_UP_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING4__: 7;
	} __attribute__((packed));
	/** Gets CAN ID of HFM_308 **/
	uint32_t get_canid(){ return HFM_308_CAN_ID; }
} HFM_308;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint16_t __PADDING1__: 16;
		/** intake air temperature **/
		uint8_t T_LUFT: 8;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint32_t __PADDING2__: 22;
		/** intake air temperature implausible **/
		bool TFA_UP_B: 1;
		/** engine temperature implausible **/
		bool TFM_UP_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING3__: 8;
		/** engine temperature **/
		uint8_t T_MOT: 8;
	} __attribute__((packed));
	/** Gets CAN ID of HFM_608 **/
	uint32_t get_canid(){ return HFM_608_CAN_ID; }
} HFM_608;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];
	struct {
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint16_t __PADDING1__: 16;
		/** air mass flow **/
		uint8_t MLE: 8;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint8_t __PADDING2__: 8;
		/** ROZ tempering detection **/
		bool ROZ_TP_B: 1;
		 /** BITFIELD PADDING. DO NOT CHANGE **/
		uint32_t __PADDING3__: 31;
	} __attribute__((packed));
	/** Gets CAN ID of HFM_610 **/
	uint32_t get_canid(){ return HFM_610_CAN_ID; }
} HFM_610;



class ECU_HFM {
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
                case HFM_210_CAN_ID:
                    idx = 0;
                    break;
                case HFM_308_CAN_ID:
                    idx = 1;
                    break;
                case HFM_608_CAN_ID:
                    idx = 2;
                    break;
                case HFM_610_CAN_ID:
                    idx = 3;
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
        
        /** Sets data in pointer to HFM_210
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_210(const uint32_t now, const uint32_t max_expire_time, HFM_210* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[0] <= now && now - LAST_FRAME_TIMES[0] < max_expire_time) {
                dest->raw = FRAME_DATA[0];
                ret = true;
            }
            return ret;
        }
            
        /** Sets data in pointer to HFM_308
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_308(const uint32_t now, const uint32_t max_expire_time, HFM_308* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[1] <= now && now - LAST_FRAME_TIMES[1] < max_expire_time) {
                dest->raw = FRAME_DATA[1];
                ret = true;
            }
            return ret;
        }
            
        /** Sets data in pointer to HFM_608
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_608(const uint32_t now, const uint32_t max_expire_time, HFM_608* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[2] <= now && now - LAST_FRAME_TIMES[2] < max_expire_time) {
                dest->raw = FRAME_DATA[2];
                ret = true;
            }
            return ret;
        }
            
        /** Sets data in pointer to HFM_610
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_610(const uint32_t now, const uint32_t max_expire_time, HFM_610* dest) const {
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[3] <= now && now - LAST_FRAME_TIMES[3] < max_expire_time) {
                dest->raw = FRAME_DATA[3];
                ret = true;
            }
            return ret;
        }
            
	private:
		uint64_t FRAME_DATA[4];
		uint32_t LAST_FRAME_TIMES[4];
};
#endif // ECU_HFM_H