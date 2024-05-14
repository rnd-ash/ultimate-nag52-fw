#include "ext_adc.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "clock.hpp"
#include "board_config.h"

ExternalADC::ExternalADC(gpio_num_t sda, gpio_num_t scl)
{
	if ((gpio_num_t::GPIO_NUM_NC != sda) && (gpio_num_t::GPIO_NUM_NC != scl))
	{
		// init I/O expander module
		i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = sda,
			.scl_io_num = scl,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master = {
				.clk_speed = 100000u},
			.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL};
		init_status = i2c_driver_install(I2C_NUM_0, i2c_mode_t::I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_SHARED);
		if (ESP_OK == init_status)
		{
			init_status = i2c_param_config(I2C_NUM_0, &conf);
			if (ESP_OK == init_status)
			{
				while(1) {
					printf("ADC OK\n");
					uint8_t req[2] = {0x00, 0x00};
					uint8_t req[2] = {0x00, 0x00};
					i2c_master_write_read_device(I2C_NUM_0, 0x48, 2,)
					vTaskDelay(1000);
				}
			}
 			else
			{
				ESP_LOG_LEVEL(ESP_LOG_ERROR, "EXTADC", "Failed to set param config");
			}
		}
		else
		{
			ESP_LOG_LEVEL(ESP_LOG_ERROR, "EXTADC", "Failed to install driver");
		}
	}
	else
	{
		ESP_LOG_LEVEL(ESP_LOG_ERROR, "EXTADC", "Cannot launch IOExpander on board without I2C!");
		init_status = ESP_ERR_INVALID_VERSION;
	}
}

ExternalADC* ext_adc = nullptr;