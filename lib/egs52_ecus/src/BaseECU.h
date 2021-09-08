
/**
 * Abstracted layer for all ECUs that will be talking to EGS52
 */

class BaseECU {
    public:
        /**
         * This function will clear expired CAN frames from the ECU.
         * This is so that if the ECU stops transmitting data over CANBUS,
         * EGS52 can identify this and throw the correct DTC about data not
         * being received over CANBUS, rather than constantly using the last
         * CAN Frame
         */
        void clear_old_frames();
};