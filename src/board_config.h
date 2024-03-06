#ifndef PINS_H
#define PINS_H

#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>
#include "ioexpander.h"

static const uint16_t NUM_TEMP_POINTS = 25u;

struct temp_reading_t{
    // Voltage in mV
    uint16_t v; 
    // ATF Temp in degrees C * 10
    int temp; 
};

// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
const static temp_reading_t atf_temp_lookup_V12[NUM_TEMP_POINTS] = {
//    mV, Temp(x10)
// mV Values are calibrated on 3.45V rail
// as that is how much the ATF sensor power gets
    {686, -500},
    {738, -400},
    {792, -300},
    {847, -200},
    {903, -100},
    {959,  0},
    {1015, 100},
    {1071, 200},
    {1100, 250},
    {1128, 300},
    {1183, 400},
    {1238, 500},
    {1292, 600},
    {1346, 700},
    {1399, 800},
    {1450, 900},
    {1501, 1000},
    {1551, 1100},
    {1599, 1200},
    {1623, 1250},
    {1647, 1300},
    {1692, 1400},
    {1737, 1500},
    {1781, 1600},
    {1823, 1700},
};

// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
const static temp_reading_t atf_temp_lookup_V11[NUM_TEMP_POINTS] = {
//    mV, Temp(x10)
// mV Values are calibrated on 3.45V rail
// as that is how much the ATF sensor power gets
    {436, -500},
    {449, -400},
    {462, -300},
    {477, -200},
    {492, -100},
    {508, 0},
    {524, 100},
    {541, 200},
    {550, 250},
    {558, 300},
    {576, 400},
    {595, 500},
    {614, 600},
    {634, 700},
    {654, 800},
    {674, 900},
    {695, 1000},
    {716, 1100},
    {738, 1200},
    {749, 1250},
    {760, 1300},
    {782, 1400},
    {804, 1500},
    {827, 1600},
    {850, 1700},
};

struct SensorFuncData {
    //adc2_channel_t batt_channel;
    //adc2_channel_t atf_channel;
    adc_channel_t adc_batt;
    adc_channel_t adc_atf;
    const temp_reading_t* atf_calibration_curve;
    float current_sense_multi;
} ;

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

    // only on board versions 1.2 and newer
    gpio_num_t i2c_sda; 
    gpio_num_t i2c_scl;

    // only on board versions 1.3 and newer
    gpio_num_t io_pin;

    // returns the state of the driving program switch, if connected to the board
    virtual bool is_program_switch_pressed(void) { return false;};
    virtual bool is_data_valid(const uint32_t expire_time_ms) { return false; };
    virtual void read_input_signals(void) {};
	virtual void write_output_signals(void) {};
	virtual uint8_t get_trrs(void) {return UINT8_MAX;};
	virtual bool is_kickdown_pressed(void) {return false;};
	virtual bool is_brake_light_switch_pressed(void){return false;};
	virtual void set_rp_solenoid(const bool rp_solenoid_enabled){};
	virtual void set_start(const bool start_enabled){};
	virtual void set_gearbox_protection(const bool gearbox_protection_enabled){};
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

class BoardGpioMatrixWithIOExpander: public BoardGpioMatrix{
public:
    bool is_program_switch_pressed(void) override;
    bool is_data_valid(const uint32_t expire_time_ms) override;
    void read_input_signals(void) override;
	void write_output_signals(void) override;
	uint8_t get_trrs(void) override;
	bool is_kickdown_pressed(void) override;
	bool is_brake_light_switch_pressed(void) override;
	void set_rp_solenoid(const bool rp_solenoid_enabled) override;
	void set_start(const bool start_enabled) override;
	void set_gearbox_protection(const bool gearbox_protection_enabled) override;
};

/**
 * @brief GPIO Matrix for the Gen 1 production PCB (V1.2 07/07/22)
 * 
 */
class BoardV12GpioMatrix: public BoardGpioMatrixWithIOExpander {
public:
    BoardV12GpioMatrix(void);
};

/**
 * @brief GPIO Matrix for the Gen 2 production PCB (V1.3 29/11/22)
 * NOTE: This PCB is in development, so its matrix can change
 */
class BoardV13GpioMatrix: public BoardGpioMatrixWithIOExpander {
public:
    BoardV13GpioMatrix(void);
};


extern BoardGpioMatrix* pcb_gpio_matrix;

#endif