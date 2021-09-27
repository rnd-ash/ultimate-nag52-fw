#ifndef __GEARBOX_H_
#define __GEARBOX_H_


#include <config.h>
#include "sensors.h"
#include "../canbus/abstract_can.h"
#include <freertos/task.h>

enum ShiftResult {
    Failed,
    ShiftAlreadyPending,
    OK
}



class Gearbox {
    public:
        Gearbox();
        ~Gearbox();
    private:
        /**
         * @brief Task handler for the task which switches gears
         * 
         */
        TaskHandle_t* shifter_task = nullptr;

        /**
         * @brief Task handler for the gearbox's logic loop
         * 
         */
        TaskHandle_t* logic_loop = nullptr;

        Gear target_gear;
        Gear actual_gear;
}

#endif // __GEARBOX_H_