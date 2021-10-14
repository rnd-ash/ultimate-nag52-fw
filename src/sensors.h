#ifndef __SENSORS_H_
#define __SENSORS_H_

#include <stdint.h>

namespace Sensors {
    bool init_sensors();

    uint32_t read_n3_rpm();
    uint32_t read_n2_rpm();

    bool read_vbatt(int* dest);
    bool read_atf_temp(int* dest);

    bool parking_lock_engaged(bool* dest);
}

#endif // __SENSORS_H_