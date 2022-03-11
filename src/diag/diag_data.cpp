#include "diag_data.h"
#include "sensors.h"

DATA_GEARBOX_SENSORS get_gearbox_sensors() {
    DATA_GEARBOX_SENSORS ret = {};
    RpmReading d;
    bool b = false;
    if (!Sensors::read_input_rpm(&d, false)) {
        ret.n2_rpm = 0xFFFF;
        ret.n3_rpm = 0xFFFF;
        ret.calc_rpm = 0xFFFF;
    } else {
        ret.n2_rpm = d.n2_raw;
        ret.n3_rpm = d.n3_raw;
        ret.calc_rpm = d.calc_rpm;
    }
    if(!Sensors::read_atf_temp(&ret.atf_temp_c)) {
        ret.atf_temp_c = 0xFFFF;
    }
    if (!Sensors::read_vbatt(&ret.v_batt)) {
        ret.v_batt = 0xFFFF;
    }
    if (Sensors::parking_lock_engaged(&b)) {
         ret.parking_lock = b;
    } else {
        ret.parking_lock = 0xFF;
    }
    ret.rli = RLI_GEARBOX_SENSORS;
    return ret;
}