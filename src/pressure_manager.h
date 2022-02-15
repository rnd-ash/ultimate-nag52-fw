#ifndef __PRESSURE_MANAGER_H_
#define __PRESSURE_MANAGER_H_

#include <common_structs.h>
#include "profiles.h"
#include <gearbox_config.h>

// Default maps (Needs modifying!)

/*
    Large NAG W5A580 gearbox pressure maps
*/

#ifdef LARGE_NAG

// 1 -> 2 upshift
const pressure_map spc_1_2 = {540, 530, 500, 490, 480, 470, 460, 450, 440, 430, 420};
const pressure_map mpc_1_2 = {540, 530, 500, 490, 480, 470, 460, 450, 440, 430, 420};

// 2 -> 3 upshift
const pressure_map spc_2_3 = {490, 480, 470, 460, 440, 430, 420, 400, 390, 380, 360};
const pressure_map mpc_2_3 = {490, 480, 470, 460, 440, 430, 420, 400, 390, 380, 360};


// 3 -> 4 upshift
const pressure_map spc_3_4 = {450, 440, 430, 420, 410, 400, 390, 380, 370, 360, 350};
const pressure_map mpc_3_4 = {450, 440, 430, 420, 410, 400, 390, 380, 370, 360, 350};


// 4 -> 5 upshift
const pressure_map spc_4_5 = {490, 480, 470, 450, 430, 420, 410, 400, 380, 360, 340};
const pressure_map mpc_4_5 = {490, 480, 470, 450, 430, 420, 410, 400, 380, 360, 340};


// 2 -> 1 downshift
const pressure_map spc_2_1 = {510, 495, 490, 485, 480, 475, 470, 460, 450, 420, 400};
const pressure_map mpc_2_1 = {510, 495, 490, 485, 480, 475, 470, 460, 450, 420, 400};

// 3 -> 2 downshift
const pressure_map spc_3_2 = {450, 425, 400, 390, 380, 370, 360, 350, 340, 330, 320};
const pressure_map mpc_3_2 = {450, 425, 400, 390, 380, 370, 360, 350, 340, 330, 320};

// 4 -> 3 downshift
const pressure_map spc_4_3 = {430, 420, 410, 400, 390, 380, 370, 350, 330, 310, 290};
const pressure_map mpc_4_3 = {430, 420, 410, 400, 390, 380, 370, 350, 330, 310, 290};

// 5 -> 4 downshift
const pressure_map spc_5_4 = {430, 420, 395, 385, 385, 370, 360, 360, 350, 340, 325};
const pressure_map mpc_5_4 = {430, 420, 395, 385, 385, 370, 360, 360, 350, 340, 325};

#else

/*
    Small NAG W5A330 gearbox pressure maps
*/

// 1 -> 2 upshift
const pressure_map spc_1_2 = {540, 530, 520, 510, 500, 490, 480, 470, 460, 450, 440};
const pressure_map mpc_1_2 = {540, 530, 520, 510, 500, 490, 480, 470, 460, 450, 440};

// 2 -> 3 upshift
const pressure_map spc_2_3 = {510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 400};
const pressure_map mpc_2_3 = {510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 400};


// 3 -> 4 upshift
const pressure_map spc_3_4 = {510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 400};
const pressure_map mpc_3_4 = {510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 400};


// 4 -> 5 upshift
const pressure_map spc_4_5 = {510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 400};
const pressure_map mpc_4_5 = {510, 500, 490, 480, 470, 460, 450, 440, 430, 420, 400};


// 2 -> 1 downshift
const pressure_map spc_2_1 = {510, 495, 490, 485, 480, 475, 470, 460, 450, 420, 400};
const pressure_map mpc_2_1 = {510, 495, 490, 485, 480, 475, 470, 460, 450, 420, 400};

// 3 -> 2 downshift
const pressure_map spc_3_2 = {450, 425, 400, 390, 380, 370, 360, 350, 340, 330, 320};
const pressure_map mpc_3_2 = {450, 425, 400, 390, 380, 370, 360, 350, 340, 330, 320};

// 4 -> 3 downshift
const pressure_map spc_4_3 = {440, 435, 430, 420, 410, 400, 380, 360, 340, 320, 300};
const pressure_map mpc_4_3 = {440, 435, 430, 420, 410, 400, 380, 360, 340, 320, 300};

// 5 -> 4 downshift
const pressure_map spc_5_4 = {430, 420, 395, 385, 385, 370, 360, 360, 350, 340, 325};
const pressure_map mpc_5_4 = {430, 420, 395, 385, 385, 370, 360, 360, 350, 340, 325};

#endif

const float pressure_temp_normalizer[17] = {
    0.6, 0.62, 0.65, 0.68, 0.71, // -40-0C (0-40)
    0.75, 0.80, 0.84, 0.88, 0.92, // 10-50C (50-90)
    0.95, 0.98, 1, 1.01, 1.015, 1.02, 1.025 //60C+ (100-160)
};

// 0, 1k, 2k, 3k, 4k, 5k, 6k, 7k, 8k RPM
const float rpm_normalizer[9] = {1.04, 1.02, 1.00, 0.98, 0.96, 0.94, 0.92, 0.9, 0.88};

// RPM vs MPC pressure when driving (0-8000RPM)
const uint16_t mpc_hold_pressure[9] = {300, 320, 340, 360, 370, 380, 390, 400, 400};

class PressureManager {

public:
    PressureManager(SensorData* sensor_ptr) {
        this->sensor_data = sensor_ptr;
    }
    /**
     * @brief Performs a gear change
     * 
     * @param shift_request Which gear change needs to be made?
     * @param sensors Pointer to gearbox sensor data
     * @param profile Drive profile currently running when the shift request was called. This is
     * used to get the shift harshness and shift firmness request
     * 
     * @return Shift response data about how the shift went
     */
    ShiftResponse perform_and_monitor_shift(ProfileGearChange shift_request,  AbstractProfile* profile);
    /**
     * @brief In the event a gear change is in progress, and the gearbox needs to abort the shift
     * call this function. This ONLY applies to up shifting, where the gearbox may need to abort
     * the upshift due to a sudden load demand change
     * 
     * @return Boolean value indicating if the abort request can be carried out. Sometimes
     * it is not possible for the gearbox to abort the gear change (EG, already at red-line when upshifting
     * cannot result in an abort shift as that would over-rev the engine)
     * 
     */
    void abort_shift();

    /**
     * @brief Get the shift data object for the requested shift
     * 
     * @param shift_firmness Firmness of the shift (higher = firmer shift)
     * @param shift_speed Speed of the shift (higer = faster shift)
     * @return ShiftData 
     */
    ShiftData get_shift_data(SensorData* sensors, ProfileGearChange shift_request, ShiftCharacteristics chars);

    /**
     * @brief Gets the holding pressure in order to keeps MPC clutches activated (When NOT shifting), without too much pressure
     * to blow the gearbox to bits!
     * 
     */
    uint16_t get_mpc_active_duty_percent();
private:
    bool abort = false;
    SensorData* sensor_data;
};

#endif