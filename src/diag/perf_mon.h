#ifndef __PERF_MON_H_
#define __PERF_MON_H_

#include <stdint.h>

typedef struct {
    uint16_t load_core_1;
    uint16_t load_core_2;
} CpuStats;

void init_perfmon();

CpuStats get_cpu_usage();

#endif // __PERF_MON_H_