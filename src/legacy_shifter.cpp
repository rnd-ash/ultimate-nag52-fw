#include "legacy_shifter.h"

#ifdef BOARD_V2
#include "pins.h"
#include "driver/i2c.h"
#include "macros.h"

#define IO_ADDR 0x20

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

LegacyShifter::LegacyShifter() {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = 400000;
    esp_err_t res;
    res = i2c_driver_install(I2C_NUM_0, i2c_mode_t::I2C_MODE_MASTER, 0, 0, 0);
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "I2C", "Failed to install driver %s", esp_err_to_name(res));
    }
    res = i2c_param_config(I2C_NUM_0, &conf);
    if (res != ESP_OK) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "I2C", "Failed to set param config %s", esp_err_to_name(res));
    }
    // Now set P0 as all inputs, and P1 as all outputs  
    
}

void read_state() {
    uint8_t response[2] = {0,0};
    uint8_t req[2] = {0, 0};
    //i2c_master_write_read_device(I2C_NUM_0, IO_ADDR, req, 2, response, 2, 5);
    i2c_master_read_from_device(I2C_NUM_0, IO_ADDR, response, 2, 5);
    ESP_LOG_LEVEL(ESP_LOG_INFO, "LS", "%02X %02X", response[0], response[1]);
}

bool LegacyShifter::kickdown_pressed() {
    read_state();
    return false;
}

ShifterPosition LegacyShifter::get_shifter_position() {
    uint8_t req[2] = {0};
    uint8_t response[2] = {0,0};
    esp_err_t e = i2c_master_write_read_device(I2C_NUM_0, IO_ADDR, req, 1, response, 2, 5);
    //esp_err_t e = i2c_master_read_from_device(I2C_NUM_0, IO_ADDR, response, 2, 1);
    if (e != ESP_OK) {
        // Error, SNV
        ESP_LOGE("LS", "Could not query I2C: %s", esp_err_to_name(e));
        return ShifterPosition::SignalNotAvaliable;
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "LS", BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(response[0]));
    return ShifterPosition::SignalNotAvaliable;
}

#endif