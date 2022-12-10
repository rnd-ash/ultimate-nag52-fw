#ifndef SHIFT_REPORT_H
#define SHIFT_REPORT_H
#include "common_structs.h"
#include "nvs.h"

static const uint16_t MAX_REPORTS = 10u;

struct __attribute__ ((packed)) ShiftReportNvsGroup{
    ShiftReport reports[MAX_REPORTS];
    uint8_t index;
    uint16_t crc;
};

// 0x7D000 from partitions.csv (tcm_shift_store) partition
static_assert(sizeof(ShiftReportNvsGroup) < 0x7D000u, "ShiftReportNvsGroup cannot fit into designated partition!");

class ShiftReporter {

public:
    ShiftReporter(void);
    ~ShiftReporter(void);

    void add_report(ShiftReport src);

    void save(void);
    ShiftReportNvsGroup diag_get_nvs_group_ptr() { return *this->report_group; }
private:
    bool has_change = false;
    ShiftReportNvsGroup* report_group;
    uint32_t spiffs_addr;
};


#endif // SHIFT_REPORT_H