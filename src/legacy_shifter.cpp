#include "legacy_shifter.h"

#ifdef BOARD_V2
#include "pins.h"
#include "driver/i2c.h"

#define IO_ADDR 0b0100100

LegacyShifter::LegacyShifter() {
    int i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = 100000;
    i2c_param_config(0, &conf);
    i2c_driver_install(0, i2c_mode_t::I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_LEVEL1);

    // Now set P0 as all inputs, and P1 as all outputs
    
}

void read_state() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t response[2] = {0,0};
    i2c_master_write_byte(cmd, (IO_ADDR << 1) | I2C_MASTER_READ, true); // Address
    i2c_master_read_byte(cmd, &response[0], i2c_ack_type_t::I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &response[1], i2c_ack_type_t::I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
}

bool LegacyShifter::kickdown_pressed() {
    read_state();
    return false;
}

#endif