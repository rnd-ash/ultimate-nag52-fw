#ifndef __PERF_MON_H_
#define __PERF_MON_H_

#include <stdint.h>

typedef struct {
    volatile uint16_t load_core_1;
    volatile uint16_t load_core_2;
} CpuStats;


bool init_perfmon();
void remove_perfmon();

CpuStats get_cpu_stats();

#endif // __PERF_MON_H_