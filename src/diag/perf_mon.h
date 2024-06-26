#ifndef PERF_MON_H
#define PERF_MON_H

#include <stdint.h>
#include <esp_err.h>

struct CpuStats{
    uint16_t load_core_1;
    uint16_t load_core_2;
} ;

namespace PerfMon {
    void init_perfmon(void);
    void update_sample(void);
    CpuStats get_cpu_stats(void);
}

#endif // PERF_MON_H