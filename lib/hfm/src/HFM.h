
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
enum class HFM_210h_WHC {
	PN_B = 0, // driving position P/N engaged (automatic transmission only)
	R_B = 3, // driving position reverse
	D4_B = 5, // driving position P/N not engaged (automatic transmission only)
	SCH = 10, // manual transmission
};



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of HFM_210 */
	uint32_t get_canid(){ return HFM_210_CAN_ID; }
    /** Sets shifter module code */
    void set_WHC(HFM_210h_WHC value){ raw = (raw & 0xe1ffffffffffffff) | ((uint64_t)value & 0xf) << 57; }

    /** Gets shifter module code */
    HFM_210h_WHC get_WHC() const { return (HFM_210h_WHC)(raw >> 57 & 0xf); }
        
    /** Sets throttle valve actual value. Conversion formula (To raw from real): y=(x-0.0)/0.35 */
    void set_DKI(uint8_t value){ raw = (raw & 0xff00ffffffffffff) | ((uint64_t)value & 0xff) << 48; }

    /** Gets throttle valve actual value. Conversion formula (To real from raw): y=(0.35x)+0.0 */
    uint8_t get_DKI() const { return (uint8_t)(raw >> 48 & 0xff); }
        
    /** Sets throttle valve target value. Conversion formula (To raw from real): y=(x-0.0)/0.35 */
    void set_DKV(uint8_t value){ raw = (raw & 0xffff00ffffffffff) | ((uint64_t)value & 0xff) << 40; }

    /** Gets throttle valve target value. Conversion formula (To real from raw): y=(0.35x)+0.0 */
    uint8_t get_DKV() const { return (uint8_t)(raw >> 40 & 0xff); }
        
    /** Sets vehicle speed. Conversion formula (To raw from real): y=(x-0.0)/1.20 */
    void set_V_SIGNAL(uint8_t value){ raw = (raw & 0xffffff00ffffffff) | ((uint64_t)value & 0xff) << 32; }

    /** Gets vehicle speed. Conversion formula (To real from raw): y=(1.20x)+0.0 */
    uint8_t get_V_SIGNAL() const { return (uint8_t)(raw >> 32 & 0xff); }
        
    /** Sets emergency mode */
    void set_NOTL_B(bool value){ raw = (raw & 0xffffffffbfffffff) | ((uint64_t)value & 0x1) << 30; }

    /** Gets emergency mode */
    bool get_NOTL_B() const { return (bool)(raw >> 30 & 0x1); }
        
    /** Sets brake light switch on */
    void set_BLS_B(bool value){ raw = (raw & 0xffffffffdfffffff) | ((uint64_t)value & 0x1) << 29; }

    /** Gets brake light switch on */
    bool get_BLS_B() const { return (bool)(raw >> 29 & 0x1); }
        
    /** Sets full throttle */
    void set_VG_B(bool value){ raw = (raw & 0xfffffffffbffffff) | ((uint64_t)value & 0x1) << 26; }

    /** Gets full throttle */
    bool get_VG_B() const { return (bool)(raw >> 26 & 0x1); }
        
    /** Sets vehicle speed signal implausible */
    void set_VSIG_UP_B(bool value){ raw = (raw & 0xffffffffffbfffff) | ((uint64_t)value & 0x1) << 22; }

    /** Gets vehicle speed signal implausible */
    bool get_VSIG_UP_B() const { return (bool)(raw >> 22 & 0x1); }
        
    /** Sets throttle valve target value implausible */
    void set_DKV_UP_B(bool value){ raw = (raw & 0xfffffffffffdffff) | ((uint64_t)value & 0x1) << 17; }

    /** Gets throttle valve target value implausible */
    bool get_DKV_UP_B() const { return (bool)(raw >> 17 & 0x1); }
        
    /** Sets throttle valve actual value implausible */
    void set_DKI_UP_B(bool value){ raw = (raw & 0xfffffffffffeffff) | ((uint64_t)value & 0x1) << 16; }

    /** Gets throttle valve actual value implausible */
    bool get_DKI_UP_B() const { return (bool)(raw >> 16 & 0x1); }
        
} HFM_210;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of HFM_308 */
	uint32_t get_canid(){ return HFM_308_CAN_ID; }
    /** Sets engine speed implausible */
    void set_NMOT_UP_B(bool value){ raw = (raw & 0xfeffffffffffffff) | ((uint64_t)value & 0x1) << 56; }

    /** Gets engine speed implausible */
    bool get_NMOT_UP_B() const { return (bool)(raw >> 56 & 0x1); }
        
    /** Sets engine speed. Conversion formula (To raw from real): y=(x-0.0)/1.00 */
    void set_NMOT(uint16_t value){ raw = (raw & 0xffff0000ffffffff) | ((uint64_t)value & 0xffff) << 32; }

    /** Gets engine speed. Conversion formula (To real from raw): y=(1.00x)+0.0 */
    uint16_t get_NMOT() const { return (uint16_t)(raw >> 32 & 0xffff); }
        
    /** Sets air mass signal implausible */
    void set_HFM_UP_B(bool value){ raw = (raw & 0xffffffffff7fffff) | ((uint64_t)value & 0x1) << 23; }

    /** Gets air mass signal implausible */
    bool get_HFM_UP_B() const { return (bool)(raw >> 23 & 0x1); }
        
    /** Sets starter running */
    void set_KL50_B(bool value){ raw = (raw & 0xfffffffffffeffff) | ((uint64_t)value & 0x1) << 16; }

    /** Gets starter running */
    bool get_KL50_B() const { return (bool)(raw >> 16 & 0x1); }
        
} HFM_308;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of HFM_608 */
	uint32_t get_canid(){ return HFM_608_CAN_ID; }
    /** Sets engine temperature. Conversion formula (To raw from real): y=(x+44.0)/1.16 */
    void set_T_MOT(uint8_t value){ raw = (raw & 0x00ffffffffffffff) | ((uint64_t)value & 0xff) << 56; }

    /** Gets engine temperature. Conversion formula (To real from raw): y=(1.16x)-44.0 */
    uint8_t get_T_MOT() const { return (uint8_t)(raw >> 56 & 0xff); }
        
    /** Sets engine temperature implausible */
    void set_TFM_UP_B(bool value){ raw = (raw & 0xffff7fffffffffff) | ((uint64_t)value & 0x1) << 47; }

    /** Gets engine temperature implausible */
    bool get_TFM_UP_B() const { return (bool)(raw >> 47 & 0x1); }
        
} HFM_608;



typedef union {
	uint64_t raw;
	uint8_t bytes[8];

	/** Gets CAN ID of HFM_610 */
	uint32_t get_canid(){ return HFM_610_CAN_ID; }
    /** Sets ROZ tempering detection */
    void set_ROZ_TP_B(bool value){ raw = (raw & 0xfffffffeffffffff) | ((uint64_t)value & 0x1) << 32; }

    /** Gets ROZ tempering detection */
    bool get_ROZ_TP_B() const { return (bool)(raw >> 32 & 0x1); }
        
    /** Sets air mass flow. Conversion formula (To raw from real): y=(x-0.0)/4.00 */
    void set_MLE(uint8_t value){ raw = (raw & 0xffffffffff00ffff) | ((uint64_t)value & 0xff) << 16; }

    /** Gets air mass flow. Conversion formula (To real from raw): y=(4.00x)+0.0 */
    uint8_t get_MLE() const { return (uint8_t)(raw >> 16 & 0xff); }
        
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
            switch(can_id) {
                case HFM_210_CAN_ID:
                    LAST_FRAME_TIMES[0] = timestamp_now;
                    FRAME_DATA[0] = value;
                    return true;
                case HFM_308_CAN_ID:
                    LAST_FRAME_TIMES[1] = timestamp_now;
                    FRAME_DATA[1] = value;
                    return true;
                case HFM_608_CAN_ID:
                    LAST_FRAME_TIMES[2] = timestamp_now;
                    FRAME_DATA[2] = value;
                    return true;
                case HFM_610_CAN_ID:
                    LAST_FRAME_TIMES[3] = timestamp_now;
                    FRAME_DATA[3] = value;
                    return true;
                default:
                    return false;
            }
        }
        
        /** Sets data in pointer to HFM_210
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_210(const uint32_t now, const uint32_t max_expire_time, HFM_210* dest) const {
            if (LAST_FRAME_TIMES[0] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[0] && now - LAST_FRAME_TIMES[0] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[0];
                return true;
            }
        }
            
        /** Sets data in pointer to HFM_308
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_308(const uint32_t now, const uint32_t max_expire_time, HFM_308* dest) const {
            if (LAST_FRAME_TIMES[1] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[1] && now - LAST_FRAME_TIMES[1] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[1];
                return true;
            }
        }
            
        /** Sets data in pointer to HFM_608
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_608(const uint32_t now, const uint32_t max_expire_time, HFM_608* dest) const {
            if (LAST_FRAME_TIMES[2] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[2] && now - LAST_FRAME_TIMES[2] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[2];
                return true;
            }
        }
            
        /** Sets data in pointer to HFM_610
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_HFM_610(const uint32_t now, const uint32_t max_expire_time, HFM_610* dest) const {
            if (LAST_FRAME_TIMES[3] == 0 || dest == nullptr) { // CAN Frame has not been seen on bus yet / NULL pointer
                return false;
            } else if (now > LAST_FRAME_TIMES[3] && now - LAST_FRAME_TIMES[3] > max_expire_time) { // CAN Frame has not refreshed in valid interval
                return false;
            } else { // CAN Frame is valid! return it
                dest->raw = FRAME_DATA[3];
                return true;
            }
        }
            
	private:
		uint64_t FRAME_DATA[4];
		uint32_t LAST_FRAME_TIMES[4];
};
#endif // ECU_HFM_H