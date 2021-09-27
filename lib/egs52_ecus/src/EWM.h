#include "BaseECU.h"
#include <EWM/EWM_230.h>

class EWM_ECU : public BaseECU {
    public:
        EWM_ECU() {
            this->ewm230.raw = 0x00000000;
            this->ewm230_status.is_valid = false;
        }

        void clear_old_frames(uint64_t now) {
            if (now - ewm230_status.last_rx_timestamp > 250) { // every 25ms normally
                ewm230_status.is_valid = false;
            }
        }

        bool import_can_frame(can_message_t *f, uint64_t timestamp) {
            switch (f->identifier) {
                case EWM_230_ID:
                    this->ewm230.import_frame(f->identifier, f->data, f->data_length_code);
                    update_framestatus(&this->ewm230_status, timestamp);
                    return true;
                default:
                    return false;
            }
        }

        EWM_230* get_ewm230() {
            if (this->ewm230_status.is_valid) {
                return &this->ewm230;
            } else {
                return nullptr;
            }
        }
    private:
        EWM_230 ewm230;
        FrameStatus ewm230_status;
};
