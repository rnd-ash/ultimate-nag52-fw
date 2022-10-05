#ifndef __SHIFT_REPORT_H__
#define __SHIFT_REPORT_H__
#include "common_structs.h"
#include "nvs.h"

#define MAX_REPORTS 10

typedef struct {
    ShiftReport reports[MAX_REPORTS];
    uint8_t index;
    uint16_t crc;
} __attribute__ ((packed)) ShiftReportNvsGroup;

// 0x7D000 from partitions.csv (tcm_shift_store) partition
static_assert(sizeof(ShiftReportNvsGroup) < 0x7D000, "ShiftReportNvsGroup cannot fit into designated partition!");

class ShiftReporter {

public:
    ShiftReporter();
    ~ShiftReporter();

    void add_report(ShiftReport src);

    void save();
    ShiftReportNvsGroup diag_get_nvs_group_ptr() { return *this->report_group; }
private:
    bool has_change = false;
    ShiftReportNvsGroup* report_group;
    uint32_t spiffs_addr;
};


#endif // __SHIFT_REPORT_H__