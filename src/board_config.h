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

typedef enum {
    PCA_NUM_NC = -1,
    PCA_NUM_0 = 0,
    PCA_NUM_1 = 1,
    PCA_NUM_2 = 2,
    PCA_NUM_3 = 3,
    PCA_NUM_4 = 4,
    PCA_NUM_5 = 5,
    PCA_NUM_6 = 6,
    PCA_NUM_7 = 7,
} pca_num_t;

class BoardGpioMatrix {
public:
    SensorFuncData sensor_data;
    
    gpio_num_t can_tx_pin;
    gpio_num_t can_rx_pin;

    gpio_num_t spkr_pin;

    gpio_num_t vsense_pin;
    gpio_num_t atf_pin;
    gpio_num_t n3_pin;
    gpio_num_t n2_pin;
    
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
    gpio_num_t tcc_zener;

    // only on board versions 1.2 and newer
    gpio_num_t i2c_sda; 
    gpio_num_t i2c_scl;

    // inputs
    pca_num_t i2c_expander_trrs_a;
    pca_num_t i2c_expander_trrs_b;
    pca_num_t i2c_expander_trrs_c;
    pca_num_t i2c_expander_trrs_d;
    pca_num_t i2c_expander_brake_light_switch;
    pca_num_t i2c_expander_program_button;
    pca_num_t i2c_expander_kickdown_switch;
    pca_num_t i2c_expander_solenoid_power_overload;

    // outputs
    pca_num_t i2c_expander_rp_solenoid_enabler;
    pca_num_t i2c_expander_start_enabler;
    pca_num_t i2c_expander_gearbox_protection_enabler;
    pca_num_t i2c_expander_solenoid_power_cutoff;

    // only on board versions 1.3 and newer
    gpio_num_t io_pin;

    // Only on board version 1.4 and newer
    // (NOTE: TFT Sensor will be marked as NC)
    bool has_external_adc_tft_vsol;
    uint8_t external_adc_addr;
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
 */
class BoardV13GpioMatrix: public BoardGpioMatrix {
public:
    BoardV13GpioMatrix(void);
};

/**
 * @brief GPIO Matrix for the 1.4 PCB (HGS OR EGS enclosure) (V1.4 10/11/23)
 */
class BoardV14GpioMatrix: public BoardGpioMatrix {
public:
    BoardV14GpioMatrix(void);
};


extern BoardGpioMatrix* pcb_gpio_matrix;

#endif