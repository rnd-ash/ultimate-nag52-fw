#ifndef SHIFTER_TRRS_H
#define SHIFTER_TRRS_H

#include "shifter.h"
#include "esp_err.h"

typedef struct
{
    bool a;
    bool b;
    bool c;
    bool d;
    ShifterPosition pos;
} TRRSPos;

const static TRRSPos TRRS_SHIFTER_TABLE[8] = {
    TRRSPos{.a = 1, .b = 1, .c = 1, .d = 0, .pos = ShifterPosition::P},
    TRRSPos{.a = 0, .b = 1, .c = 1, .d = 1, .pos = ShifterPosition::R},
    TRRSPos{.a = 1, .b = 0, .c = 1, .d = 1, .pos = ShifterPosition::N},
    TRRSPos{.a = 0, .b = 0, .c = 1, .d = 0, .pos = ShifterPosition::D},
    TRRSPos{.a = 0, .b = 0, .c = 0, .d = 1, .pos = ShifterPosition::FOUR},
    TRRSPos{.a = 0, .b = 1, .c = 0, .d = 0, .pos = ShifterPosition::THREE},
    TRRSPos{.a = 1, .b = 0, .c = 0, .d = 0, .pos = ShifterPosition::TWO},
    TRRSPos{.a = 1, .b = 1, .c = 0, .d = 1, .pos = ShifterPosition::ONE},
};

class ShifterTrrs : public Shifter
{
public:
    ShifterTrrs(esp_err_t *can_init_status, const char *name, bool *start_enable);
    ShifterPosition get_shifter_position(const uint64_t now, const uint64_t expire_time_ms) override;
    void update_shifter_position(const uint64_t now);

private:
    const uint8_t IO_ADDR = 0x20u;

    const uint8_t RPSolenoidV12 = 0;
    const uint8_t StartEnableV12 = 1;
    // On 1.3 boards, the RP and Start solenoid are flipped!
    const uint8_t RPSolenoidV13 = 1;
    const uint8_t StartEnableV13 = 0;
    // Only on 1.3
    const uint8_t GenMosfet = 2;

    ShifterPosition last_valid_position = ShifterPosition::SignalNotAvailable;
    uint8_t i2c_rx_bytes[2] = {0, 0};
    uint8_t i2c_tx_bytes[2] = {0, 0};
    uint64_t last_i2c_query_time = 0;
    bool *_start_enable;
};

#endif // SHIFTER_TRRS_H