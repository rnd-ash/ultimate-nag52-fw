#include "sw_fader.h"
#include "solenoids.h"

void fader_loop() {
    Solenoid* pwm_solenoids[2] = { sol_mpc, sol_spc };
    while(1) {

        

        vTaskDelay(5);
    }
}