#include "BaseECU.h"
#include <BS/BS_200.h>
#include <BS/BS_208.h>
#include <BS/BS_270.h>
#include <BS/BS_300.h>


class BS_ECU : public BaseECU {
    public:
        void clear_old_frames(uint64_t now) {
            if (now - bs200_status.last_rx_timestamp > 250) { // every 25ms normally
                bs200_status.is_valid = false;
            }
            if (now -  bs208_status.last_rx_timestamp > 250) { // every 25ms normally
                bs208_status.is_valid = false;
            }
            if (now -  bs270_status.last_rx_timestamp > 250) { // every 25ms normally
                bs270_status.is_valid = false;
            }
            if (now -  bs300_status.last_rx_timestamp > 250) { // every 25ms normally
                bs300_status.is_valid = false;
            }
        }

        bool import_can_frame(can_message_t *f, uint64_t timestamp) {
            switch (f->identifier) {
                case BS_200_ID:
                    this->bs200.import_frame(f->identifier, f->data, f->data_length_code);
                    update_framestatus(&this->bs200_status, timestamp);
                    return true;
                case BS_208_ID:
                    this->bs208.import_frame(f->identifier, f->data, f->data_length_code);
                    update_framestatus(&this->bs208_status, timestamp);
                    return true;
                case BS_270_ID:
                    this->bs270.import_frame(f->identifier, f->data, f->data_length_code);
                    update_framestatus(&this->bs270_status, timestamp);
                    return true;
                case BS_300_ID:
                    this->bs300.import_frame(f->identifier, f->data, f->data_length_code);
                    update_framestatus(&this->bs300_status, timestamp);
                    return true;
                default:
                    return false;
            }
        }

        BS_200* get_bs200() {
            if (this->bs200_status.is_valid) {
                return &this->bs200;
            } else {
                return nullptr;
            }
        }

        BS_208* get_bs208() {
            if (this->bs208_status.is_valid) {
                return &this->bs208;
            } else {
                return nullptr;
            }
        }

        BS_270* get_bs270() {
            if (this->bs270_status.is_valid) {
                return &this->bs270;
            } else {
                return nullptr;
            }
        }

        BS_300* get_bs300() {
            if (this->bs300_status.is_valid) {
                return &this->bs300;
            } else {
                return nullptr;
            }
        }
    private:
        BS_200 bs200;
        BS_208 bs208;
        BS_270 bs270;
        BS_300 bs300;

        FrameStatus bs200_status;
        FrameStatus bs208_status;
        FrameStatus bs270_status;
        FrameStatus bs300_status;
};
