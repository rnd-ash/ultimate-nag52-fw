#include "board_config.h"
#include "esp_log.h"

BoardV11GpioMatrix::BoardV11GpioMatrix() {
    ESP_LOGI("GPIO_MATRIX", "GPIO Matrix version 1.1 (12/12/21)");
    this->io_pin     = gpio_num_t::GPIO_NUM_NC; // Not available on this board version
    this->i2c_sda    = gpio_num_t::GPIO_NUM_NC; // Not available on this board version
    this->i2c_scl    = gpio_num_t::GPIO_NUM_NC; // Not available on this board version
    this->can_tx_pin = gpio_num_t::GPIO_NUM_5; // CAN TWAI Tx
    this->can_rx_pin = gpio_num_t::GPIO_NUM_4; // CAN TWAI Rx
    this->spkr_pin   = gpio_num_t::GPIO_NUM_15; // Piezo speaker
    this->vsense_pin = gpio_num_t::GPIO_NUM_25; // Battery voltage feedback
    this->atf_pin    = gpio_num_t::GPIO_NUM_26; // ATF temp sensor and lockout
    this->n3_pin     = gpio_num_t::GPIO_NUM_27; // N3 speed sensor
    this->n2_pin     = gpio_num_t::GPIO_NUM_14; // N2 speed sensor
    this->y3_sense   = gpio_num_t::GPIO_NUM_36; // Y3 (1-2/4-5) shift solenoid (Current feedback)
    this->y3_pwm     = gpio_num_t::GPIO_NUM_23; // Y3 (1-2/4-5) shift solenoid (PWM output)
    this->y4_sense   = gpio_num_t::GPIO_NUM_39; // Y4 (3-4) shift solenoid (Current feedback)
    this->y4_pwm     = gpio_num_t::GPIO_NUM_22; // Y4 (3-4) shift solenoid (PWM output)
    this->y5_sense   = gpio_num_t::GPIO_NUM_35; // Y5 (2-3) shift solenoid (Current feedback)
    this->y5_pwm     = gpio_num_t::GPIO_NUM_19; // Y5 (2-3) shift solenoid (PWM output)
    this->mpc_sense  = gpio_num_t::GPIO_NUM_34; // Modulating pressure solenoid (Current feedback)
    this->mpc_pwm    = gpio_num_t::GPIO_NUM_21; // Modulating pressure solenoid (PWM output)
    this->spc_sense  = gpio_num_t::GPIO_NUM_32; // Shift pressure solenoid (Current feedback)
    this->spc_pwm    = gpio_num_t::GPIO_NUM_12; // Shift pressure solenoid (PWM output)
    this->tcc_sense  = gpio_num_t::GPIO_NUM_33; // Torque converter solenoid(Current feedback)
    this->tcc_pwm    = gpio_num_t::GPIO_NUM_13; // Torque converter solenoid (PWM output)
    this->tcc_zener  = gpio_num_t::GPIO_NUM_NC; // Torque converter solenoid (Zener off)
    this->sensor_data = SensorFuncData {
        .adc_batt     = adc_channel_t::ADC_CHANNEL_8,
        .adc_atf      = adc_channel_t::ADC_CHANNEL_9,
        .atf_calibration_curve = atf_temp_lookup_V11,
        .current_sense_multi = 2.0,
    };

    // I/O expander inputs
    this->i2c_expander_trrs_a = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_trrs_b = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_trrs_c = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_trrs_d = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_brake_light_switch = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_program_button = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_kickdown_switch = pca_num_t::PCA_NUM_NC; // Not available on this board version

    // I/O expander outputs
    this->i2c_expander_rp_solenoid_enabler = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_start_enabler = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_gearbox_protection_enabler = pca_num_t::PCA_NUM_NC; // Not available on this board version

    this->i2c_expander_solenoid_power_overload = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_solenoid_power_cutoff = pca_num_t::PCA_NUM_NC; // Not available on this board version

    this->has_external_adc_tft_vsol = false; // Not available on this board version
    this->external_adc_addr = 0x00; // Not available on this board version
}

BoardV12GpioMatrix::BoardV12GpioMatrix() {
    ESP_LOGI("GPIO_MATRIX", "GPIO Matrix version 1.2 (07/07/22)");
    this->io_pin = gpio_num_t::GPIO_NUM_NC; // Not available on this board version

    this->can_tx_pin = gpio_num_t::GPIO_NUM_5; // CAN TWAI Tx
    this->can_rx_pin = gpio_num_t::GPIO_NUM_18; // CAN TWAI Rx
    this->spkr_pin   = gpio_num_t::GPIO_NUM_4; // Piezo speaker
    this->vsense_pin = gpio_num_t::GPIO_NUM_25; // Battery voltage feedback
    this->atf_pin    = gpio_num_t::GPIO_NUM_27; // ATF temp sensor and lockout
    this->n3_pin     = gpio_num_t::GPIO_NUM_14; // N3 speed sensor
    this->n2_pin     = gpio_num_t::GPIO_NUM_26; // N2 speed sensor
    this->y3_sense   = gpio_num_t::GPIO_NUM_36; // Y3 (1-2/4-5) shift solenoid (Current feedback)
    this->y3_pwm     = gpio_num_t::GPIO_NUM_23; // Y3 (1-2/4-5) shift solenoid (PWM output)
    this->y4_sense   = gpio_num_t::GPIO_NUM_39; // Y4 (3-4) shift solenoid (Current feedback)
    this->y4_pwm     = gpio_num_t::GPIO_NUM_22; // Y4 (3-4) shift solenoid (PWM output)
    this->y5_sense   = gpio_num_t::GPIO_NUM_35; // Y5 (2-3) shift solenoid (Current feedback)
    this->y5_pwm     = gpio_num_t::GPIO_NUM_19; // Y5 (2-3) shift solenoid (PWM output)
    this->mpc_sense  = gpio_num_t::GPIO_NUM_34; // Modulating pressure solenoid (Current feedback)
    this->mpc_pwm    = gpio_num_t::GPIO_NUM_21; // Modulating pressure solenoid (PWM output)
    this->spc_sense  = gpio_num_t::GPIO_NUM_32; // Shift pressure solenoid (Current feedback)
    this->spc_pwm    = gpio_num_t::GPIO_NUM_12; // Shift pressure solenoid (PWM output)
    this->tcc_sense  = gpio_num_t::GPIO_NUM_33; // Torque converter solenoid(Current feedback)
    this->tcc_pwm    = gpio_num_t::GPIO_NUM_13; // Torque converter solenoid (PWM output)
    this->tcc_zener  = gpio_num_t::GPIO_NUM_NC; // Torque converter solenoid (Zener off)
    this->i2c_sda    = gpio_num_t::GPIO_NUM_15; // I2C clock
    this->i2c_scl    = gpio_num_t::GPIO_NUM_2; // I2C data 
    this->sensor_data = SensorFuncData {
        .adc_batt     = adc_channel_t::ADC_CHANNEL_8,
        .adc_atf      = adc_channel_t::ADC_CHANNEL_7,
        .atf_calibration_curve = atf_temp_lookup_V12,
        .current_sense_multi = 1.0,
    };

    // I/O expander inputs
    this->i2c_expander_trrs_a = pca_num_t::PCA_NUM_5; // TRRS A
    this->i2c_expander_trrs_b = pca_num_t::PCA_NUM_6; // TRRS B
    this->i2c_expander_trrs_c = pca_num_t::PCA_NUM_3; // TRRS C
    this->i2c_expander_trrs_d = pca_num_t::PCA_NUM_4; // TRRS D
    this->i2c_expander_brake_light_switch = pca_num_t::PCA_NUM_0; // brake light switch (BLS)
    this->i2c_expander_program_button = pca_num_t::PCA_NUM_1; // driving program switch
    this->i2c_expander_kickdown_switch = pca_num_t::PCA_NUM_2; // kickdown switch

    // I/O expander outputs
    this->i2c_expander_rp_solenoid_enabler = pca_num_t::PCA_NUM_0; // enabler for RP-solenoid at TRRS-shifter
    this->i2c_expander_start_enabler = pca_num_t::PCA_NUM_1; // start enabler
    this->i2c_expander_gearbox_protection_enabler = pca_num_t::PCA_NUM_NC; // gearbox protection activated (see GSA-bit)

    this->i2c_expander_solenoid_power_overload = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_solenoid_power_cutoff = pca_num_t::PCA_NUM_NC; // Not available on this board version

    this->has_external_adc_tft_vsol = false; // Not available on this board version
    this->external_adc_addr = 0x00; // Not available on this board version
}

BoardV13GpioMatrix::BoardV13GpioMatrix() {
    ESP_LOGI("GPIO_MATRIX", "GPIO Matrix version 1.3 (29/11/22)");
    this->io_pin     = gpio_num_t::GPIO_NUM_4; 
    this->can_tx_pin = gpio_num_t::GPIO_NUM_5; // CAN TWAI Tx
    this->can_rx_pin = gpio_num_t::GPIO_NUM_18; // CAN TWAI Rx
    this->spkr_pin   = gpio_num_t::GPIO_NUM_0; // Piezo speaker
    this->vsense_pin = gpio_num_t::GPIO_NUM_25; // Battery voltage feedback
    this->atf_pin    = gpio_num_t::GPIO_NUM_27; // ATF temp sensor and lockout
    this->n3_pin     = gpio_num_t::GPIO_NUM_14; // N3 speed sensor
    this->n2_pin     = gpio_num_t::GPIO_NUM_26; // N2 speed sensor
    this->y3_sense   = gpio_num_t::GPIO_NUM_36; // Y3 (1-2/4-5) shift solenoid (Current feedback)
    this->y3_pwm     = gpio_num_t::GPIO_NUM_23; // Y3 (1-2/4-5) shift solenoid (PWM output)
    this->y4_sense   = gpio_num_t::GPIO_NUM_39; // Y4 (3-4) shift solenoid (Current feedback)
    this->y4_pwm     = gpio_num_t::GPIO_NUM_22; // Y4 (3-4) shift solenoid (PWM output)
    this->y5_sense   = gpio_num_t::GPIO_NUM_35; // Y5 (2-3) shift solenoid (Current feedback)
    this->y5_pwm     = gpio_num_t::GPIO_NUM_19; // Y5 (2-3) shift solenoid (PWM output)
    this->mpc_sense  = gpio_num_t::GPIO_NUM_34; // Modulating pressure solenoid (Current feedback)
    this->mpc_pwm    = gpio_num_t::GPIO_NUM_21; // Modulating pressure solenoid (PWM output)
    this->spc_sense  = gpio_num_t::GPIO_NUM_32; // Shift pressure solenoid (Current feedback)
    this->spc_pwm    = gpio_num_t::GPIO_NUM_12; // Shift pressure solenoid (PWM output)
    this->tcc_sense  = gpio_num_t::GPIO_NUM_33; // Torque converter solenoid(Current feedback)
    this->tcc_pwm    = gpio_num_t::GPIO_NUM_13; // Torque converter solenoid (PWM output)
    this->tcc_zener  = gpio_num_t::GPIO_NUM_NC; // Torque converter solenoid (Zener off)
    this->i2c_sda    = gpio_num_t::GPIO_NUM_15; // I2C clock
    this->i2c_scl    = gpio_num_t::GPIO_NUM_2; // I2C data 
    this->sensor_data = SensorFuncData {
        .adc_batt = adc_channel_t::ADC_CHANNEL_8,
        .adc_atf = adc_channel_t::ADC_CHANNEL_7,
        .atf_calibration_curve = atf_temp_lookup_V12,
        .current_sense_multi = 1.0,
    };

    // I/O expander inputs
    this->i2c_expander_trrs_a = pca_num_t::PCA_NUM_5; // TRRS A
    this->i2c_expander_trrs_b = pca_num_t::PCA_NUM_6; // TRRS B
    this->i2c_expander_trrs_c = pca_num_t::PCA_NUM_4; // TRRS C
    this->i2c_expander_trrs_d = pca_num_t::PCA_NUM_3; // TRRS D
    this->i2c_expander_brake_light_switch = pca_num_t::PCA_NUM_0; // brake light switch (BLS)
    this->i2c_expander_program_button = pca_num_t::PCA_NUM_1; // driving program switch
    this->i2c_expander_kickdown_switch = pca_num_t::PCA_NUM_2; // kickdown switch

    // I/O expander outputs
    this->i2c_expander_rp_solenoid_enabler = pca_num_t::PCA_NUM_1;  // enabler for RP-solenoid at TRRS-shifter
    this->i2c_expander_start_enabler = pca_num_t::PCA_NUM_0; // start enabler
    this->i2c_expander_gearbox_protection_enabler = pca_num_t::PCA_NUM_2; // gearbox protection activated (GSA-bit)

    this->i2c_expander_solenoid_power_overload = pca_num_t::PCA_NUM_NC; // Not available on this board version
    this->i2c_expander_solenoid_power_cutoff = pca_num_t::PCA_NUM_NC; // Not available on this board version

    this->has_external_adc_tft_vsol = false; // Not available on this board version
    this->external_adc_addr = 0x00; // Not available on this board version
}

BoardV14GpioMatrix::BoardV14GpioMatrix() {
    ESP_LOGI("GPIO_MATRIX", "GPIO Matrix version 1.4 (11/10/23)");
    this->io_pin     = gpio_num_t::GPIO_NUM_4; 
    this->can_tx_pin = gpio_num_t::GPIO_NUM_5; // CAN TWAI Tx
    this->can_rx_pin = gpio_num_t::GPIO_NUM_18; // CAN TWAI Rx
    this->spkr_pin   = gpio_num_t::GPIO_NUM_0; // Piezo speaker
    this->vsense_pin = gpio_num_t::GPIO_NUM_25; // Battery voltage feedback
    this->atf_pin    = gpio_num_t::GPIO_NUM_NC; // ATF temp sensor and lockout
    this->n3_pin     = gpio_num_t::GPIO_NUM_14; // N3 speed sensor
    this->n2_pin     = gpio_num_t::GPIO_NUM_26; // N2 speed sensor
    this->y3_sense   = gpio_num_t::GPIO_NUM_36; // Y3 (1-2/4-5) shift solenoid (Current feedback)
    this->y3_pwm     = gpio_num_t::GPIO_NUM_23; // Y3 (1-2/4-5) shift solenoid (PWM output)
    this->y4_sense   = gpio_num_t::GPIO_NUM_39; // Y4 (3-4) shift solenoid (Current feedback)
    this->y4_pwm     = gpio_num_t::GPIO_NUM_22; // Y4 (3-4) shift solenoid (PWM output)
    this->y5_sense   = gpio_num_t::GPIO_NUM_35; // Y5 (2-3) shift solenoid (Current feedback)
    this->y5_pwm     = gpio_num_t::GPIO_NUM_19; // Y5 (2-3) shift solenoid (PWM output)
    this->mpc_sense  = gpio_num_t::GPIO_NUM_34; // Modulating pressure solenoid (Current feedback)
    this->mpc_pwm    = gpio_num_t::GPIO_NUM_21; // Modulating pressure solenoid (PWM output)
    this->spc_sense  = gpio_num_t::GPIO_NUM_32; // Shift pressure solenoid (Current feedback)
    this->spc_pwm    = gpio_num_t::GPIO_NUM_12; // Shift pressure solenoid (PWM output)
    this->tcc_sense  = gpio_num_t::GPIO_NUM_33; // Torque converter solenoid(Current feedback)
    this->tcc_pwm    = gpio_num_t::GPIO_NUM_13; // Torque converter solenoid (PWM output)
    this->tcc_zener  = gpio_num_t::GPIO_NUM_27; // Torque converter solenoid (Zener off)
    this->i2c_sda    = gpio_num_t::GPIO_NUM_15; // I2C clock
    this->i2c_scl    = gpio_num_t::GPIO_NUM_2; // I2C data 
    this->sensor_data = SensorFuncData {
        .adc_batt = adc_channel_t::ADC_CHANNEL_8,
        .adc_atf = adc_channel_t::ADC_CHANNEL_7,
        .atf_calibration_curve = atf_temp_lookup_V12,
        .current_sense_multi = 1.0,
    };

    // I/O expander inputs
    this->i2c_expander_trrs_a = pca_num_t::PCA_NUM_5; // TRRS A
    this->i2c_expander_trrs_b = pca_num_t::PCA_NUM_6; // TRRS B
    this->i2c_expander_trrs_c = pca_num_t::PCA_NUM_4; // TRRS C
    this->i2c_expander_trrs_d = pca_num_t::PCA_NUM_3; // TRRS D
    this->i2c_expander_brake_light_switch = pca_num_t::PCA_NUM_0; // brake light switch (BLS)
    this->i2c_expander_program_button = pca_num_t::PCA_NUM_1; // driving program switch
    this->i2c_expander_kickdown_switch = pca_num_t::PCA_NUM_2; // kickdown switch

    // I/O expander outputs
    this->i2c_expander_rp_solenoid_enabler = pca_num_t::PCA_NUM_1;  // enabler for RP-solenoid at TRRS-shifter
    this->i2c_expander_start_enabler = pca_num_t::PCA_NUM_0; // start enabler
    this->i2c_expander_gearbox_protection_enabler = pca_num_t::PCA_NUM_2; // gearbox protection activated (GSA-bit)

    this->i2c_expander_solenoid_power_overload = pca_num_t::PCA_NUM_7; // Solenoid high side switch overload
    this->i2c_expander_solenoid_power_cutoff = pca_num_t::PCA_NUM_3; // Solenoid high side switch cutoff

    this->has_external_adc_tft_vsol = true;
    this->external_adc_addr = 0xFD;
}

BoardGpioMatrix* pcb_gpio_matrix = nullptr;