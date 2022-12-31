#ifndef __DTC_MANAGER_H_
#define __DTC_MANAGER_H_

#include <stdint.h>
#include "dtcs.h"

struct EgsSnapShotData {

};

/**
 * @brief DTC Stored state on the TCU
 * 
 * ## About the `counter`
 * 
 * When the TCU start up, the counter is set to 0. Meaning the error has not been checked. Upon every successful conformation, or error, the counter
 * is either decreased or increased. When reaching its negative limit (-128), the DTC is marked as OK. If the counter reaches 127, then the failure
 * is marked as current, the `env_data_first_occurrence` is written to, and the `drive_cycle_count` is increased by 1. If the `drive_cycle_count` is 
 * now more than 1, indicating this is the 2nd or more time this fault has been triggered, then `env_data_most_recent_occurrence` is also written to,
 * the TCU is now considered stored and current. If on the next startup, the counter hits -127 (Error is OK now), then the DTC is just marked as `stored`
 * 
 */
struct DtcState {
    /// @brief DTC Code
    DtcCode code;
    /// @brief DTC Counter for the current drive. This allows for debouncing. -128 = Passed (OK), +127 = Failed (Error), 0 = Not ready.
    int8_t counter;
    /// @brief Drive cycle count of error
    uint8_t drive_cycle_count;
    /// @brief Snapshot data of most recent occurrence
    EgsSnapShotData* env_data_most_recent_occurrence;
    /// @brief Snapshot data of the first occurrence
    EgsSnapShotData* env_data_first_occurrence;
};

class DtcManager {
public:
    DtcManager();
};

#endif