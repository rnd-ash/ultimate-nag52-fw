#include "diag_data.h"
#include "sensors.h"
#include "solenoids/solenoids.h"

DATA_GEARBOX_SENSORS get_gearbox_sensors() {
    DATA_GEARBOX_SENSORS ret = {};
    RpmReading d;
    bool b = false;
    if (!Sensors::read_input_rpm(&d, false)) {
        ret.n2_rpm = 0xFFFF;
        ret.n3_rpm = 0xFFFF;
        ret.calculated_rpm = 0xFFFF;
    } else {
        ret.n2_rpm = d.n2_raw;
        ret.n3_rpm = d.n3_raw;
        ret.calculated_rpm = d.calc_rpm;
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
    return ret;
}

DATA_SOLENOIDS get_solenoid_data() {
    DATA_SOLENOIDS ret = {};

    ret.mpc_current = sol_mpc->get_current_estimate();
    ret.spc_current = sol_spc->get_current_estimate();
    ret.tcc_current = sol_tcc->get_current_estimate();
    ret.y3_current = sol_y3->get_current_estimate();
    ret.y4_current = sol_y4->get_current_estimate();
    ret.y5_current = sol_y5->get_current_estimate();

    ret.mpc_pwm = sol_mpc->get_pwm();
    ret.spc_pwm = sol_spc->get_pwm();
    ret.tcc_pwm = sol_tcc->get_pwm();
    ret.y3_pwm = sol_y3->get_pwm();
    ret.y4_pwm = sol_y4->get_pwm();
    ret.y5_pwm = sol_y5->get_pwm();

    return ret;
}