#ifndef PERF_MON_H
#define PERF_MON_H

#include <stdint.h>

struct CpuStats{
    volatile uint16_t load_core_1;
    volatile uint16_t load_core_2;
} ;


bool init_perfmon(void);
void remove_perfmon(void);

CpuStats get_cpu_stats(void);

#endif // PERF_MON_H