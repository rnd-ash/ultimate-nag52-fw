#ifndef PINS_H
#define PINS_H

#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>

static const uint16_t NUM_TEMP_POINTS = 22u;

struct temp_reading_t{
    // Voltage in mV
    uint16_t v; 
    // ATF Temp in degrees C * 10
    int temp; 
};

const static temp_reading_t atf_temp_lookup_V12[NUM_TEMP_POINTS] = {
//    mV, Temp(x10)
// mV Values are calibrated on 3.45V rail
// as that is how much the ATF sensor power gets
    {725, -400},
    {784, -300},
    {842, -200},
    {900, -100},
    {957, 0},
    {1013, 100},
    {1068, 200},
    {1123, 300},
    {1177, 400},
    {1230, 500},
    {1281, 600},
    {1332, 700},
    {1384, 800},
    {1438, 900},
    {1488, 1000},
    {1538, 1100},
    {1587, 1200},
    {1636, 1300},
    {1685, 1400},
    {1732, 1500},
    {1779, 1600},
    {1826, 1700}
};

const static temp_reading_t atf_temp_lookup_V11[NUM_TEMP_POINTS] = {
//    mV, Temp(x10)
// mV Values are calibrated on 3.45V rail
// as that is how much the ATF sensor power gets
    {446, -400},
    {461, -300},
    {476, -200},
    {491, -100},
    {507, 0},
    {523, 100},
    {540, 200},
    {557, 300},
    {574, 400},
    {592, 500},
    {610, 600},
    {629, 700},
    {648, 800},
    {669, 900},
    {690, 1000},
    {711, 1100},
    {732, 1200},
    {755, 1300},
    {778, 1400},
    {802, 1500},
    {814, 1600},
    {851, 1700}
};

typedef struct {
    //adc2_channel_t batt_channel;
    //adc2_channel_t atf_channel;
    adc_channel_t adc_batt;
    adc_channel_t adc_atf;
    const temp_reading_t* atf_calibration_curve;
    float current_sense_multi;
} SensorFuncData;

class BoardGpioMatrix {
public:
    gpio_num_t can_tx_pin;
    gpio_num_t can_rx_pin;

    gpio_num_t spkr_pin;

    gpio_num_t vsense_pin;
    gpio_num_t atf_pin;
    gpio_num_t n3_pin;
    gpio_num_t n2_pin;
    
    gpio_num_t io_pin; // Only on 1.3 and newer

    gpio_num_t y3_sense;
    gpio_num_t y3_pwm;

    gpio_num_t y4_sense;
    gpio_num_t y4_pwm;

    gpio_num_t y5_sense;
    gpio_num_t y5_pwm;

    gpio_num_t mpc_sense;
    gpio_num_t mpc_pwm;

    gpio_num_t spc_sense;
    gpio_num_t spc_pwm;

    gpio_num_t tcc_sense;
    gpio_num_t tcc_pwm;

    gpio_num_t i2c_sda; // Only on 1.2 and newer
    gpio_num_t i2c_scl; // Only on 1.2 and newer

    SensorFuncData sensor_data;
};

/**
 * @brief GPIO Matrix for the Red beta PCB (V1.1 12/12/21)
 * 
 * NOTE: This will be removed from the code base once all beta boards have been replaced
 * 
 */
class BoardV11GpioMatrix: public BoardGpioMatrix {
public:
    BoardV11GpioMatrix(void);
};

/**
 * @brief GPIO Matrix for the Gen 1 production PCB (V1.2 07/07/22)
 * 
 */
class BoardV12GpioMatrix: public BoardGpioMatrix {
public:
    BoardV12GpioMatrix(void);
};

/**
 * @brief GPIO Matrix for the Gen 2 production PCB (V1.3 29/11/22)
 * NOTE: This PCB is in development, so its matrix can change
 */
class BoardV13GpioMatrix: public BoardGpioMatrix {
public:
    BoardV13GpioMatrix(void);
};


extern BoardGpioMatrix* pcb_gpio_matrix;

#endif