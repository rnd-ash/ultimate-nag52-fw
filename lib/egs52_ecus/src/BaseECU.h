#ifndef __BASE_ECU_H_
#define __BASE_ECU_H_


#include <driver/can.h>

/**
 * Abstracted layer for all ECUs that will be talking to EGS52
 */


struct FrameStatus {
    uint64_t last_rx_timestamp;
    bool is_valid;
};

inline void update_framestatus(FrameStatus* status, uint64_t rx_timestamp) {
    status->last_rx_timestamp = rx_timestamp;
    status->is_valid = true;
}


class BaseECU {
    public:
        /**
         * This function will clear expired CAN frames from the ECU.
         * This is so that if the ECU stops transmitting data over CANBUS,
         * EGS52 can identify this and throw the correct DTC about data not
         * being received over CANBUS, rather than constantly using the last
         * CAN Frame
         */
        virtual void clear_old_frames(uint64_t now);


        /**
         * This functions imports a frame from CAN bus using the frames ID and timestamp (The time now)
         * 
         * Returns true if the frame is for THIS ECU, meaning the CAN class can stop trying insert it to other ECUs.
         */
        virtual bool import_can_frame(can_message_t *f, uint64_t timestamp);
};

#endif