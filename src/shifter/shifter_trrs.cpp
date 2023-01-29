#include "shifter_trrs.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"

#include "esp_log.h"
#include "driver/i2c.h"

const uint8_t IO_ADDR = 0x20u;

ShifterTrrs::ShifterTrrs(esp_err_t *can_init_status, const char *name, bool *start) : _start_enable(start)
{
	if (gpio_num_t::GPIO_NUM_NC == pcb_gpio_matrix->i2c_sda || gpio_num_t::GPIO_NUM_NC == pcb_gpio_matrix->i2c_scl)
	{
		ESP_LOG_LEVEL(ESP_LOG_ERROR, "SHIFTER", "Cannot launch TRRS on board without I2C!");
		*can_init_status = ESP_ERR_INVALID_VERSION;
	}
	if (ESP_OK == *can_init_status)
	{
		// Init TRRS sensors
		i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = pcb_gpio_matrix->i2c_sda,
			.scl_io_num = pcb_gpio_matrix->i2c_scl,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
		};
		conf.master.clk_speed = 400000u;
		*can_init_status = i2c_driver_install(I2C_NUM_0, i2c_mode_t::I2C_MODE_MASTER, 0, 0, 0);
		if (ESP_OK == *can_init_status)
		{
			*can_init_status = i2c_param_config(I2C_NUM_0, &conf);
			if (ESP_OK != *can_init_status)
			{
				ESP_LOG_LEVEL(ESP_LOG_ERROR, name, "Failed to set param config");
			}
		}
		else
		{
			ESP_LOG_LEVEL(ESP_LOG_ERROR, name, "Failed to install driver");
		}
	}
}

ShifterPosition ShifterTrrs::get_shifter_position(const uint64_t now, const uint64_t expire_time_ms)
{
	ShifterPosition ret = ShifterPosition::SignalNotAvailable;
	if ((now - last_i2c_query_time) < expire_time_ms)
	{
		// Data is valid time range!
		uint8_t tmp = _i2c_rx_bytes[0];
		bool TRRS_A = (tmp & (uint8_t)BIT(5u)) != 0;
		bool TRRS_B = (tmp & (uint8_t)BIT(6u)) != 0;
		bool TRRS_C;
		bool TRRS_D;
		if (2 == BOARD_CONFIG.board_ver)
		{ // V1.2 layout
			TRRS_C = (tmp & (uint8_t)BIT(3u)) != 0;
			TRRS_D = (tmp & (uint8_t)BIT(4u)) != 0;
		}
		else
		{ // V1.3+ layout
			TRRS_C = (tmp & (uint8_t)BIT(4u)) != 0;
			TRRS_D = (tmp & (uint8_t)BIT(3u)) != 0;
		}

		if (!TRRS_A && !TRRS_B && !TRRS_C && !TRRS_D)
		{ // Intermediate position, now work out which one
			switch (this->last_valid_position)
			{
			case ShifterPosition::P:
				ret = ShifterPosition::P_R;
				break;
			case ShifterPosition::R:
				ret = ShifterPosition::R_N;
				break;
			case ShifterPosition::D:
				ret = ShifterPosition::N_D;
				break;
			case ShifterPosition::N:
				ret = ShifterPosition::N_D;
				break;
			default:
				ret = ShifterPosition::SignalNotAvailable;
				break;
			}
		}
		else
		{
			// Check truth table
			for (uint8_t i = 0; i < 8; i++)
			{
				TRRSPos pos = TRRS_SHIFTER_TABLE[i];
				if (pos.a == TRRS_A && pos.b == TRRS_B && pos.c == TRRS_C && pos.d == TRRS_D)
				{
					ret = pos.pos;
					break;
				}
			}
		}
	}
	return ret;
}

void ShifterTrrs::update_shifter_position(const uint64_t now)
{
	    if (now - last_i2c_query_time > 50) {
            // Query I2C IO Expander
            uint8_t req[2] = {0,0};
            esp_err_t e = i2c_master_write_read_device(I2C_NUM_0, IO_ADDR, req, 1, i2c_rx_bytes, 2, 5);
            if (ESP_OK != e) {
                // Error, SNV
                ESP_LOGE("LS", "Could not query I2C: %s", esp_err_to_name(e));
            } else {
				last_i2c_query_time = now;
            }
            
            // Set RP and Start pins on IO expander to be outputs
            // IO 0+1 - OUTPUT
            // IO 2-7 - INPUT
            uint8_t write_buffer[2] = {0x07,0xFF}; // Set IO (0x06 + port 1 (0x01))
            if (_start_enable) {
                write_buffer[1] = write_buffer[1] & ~(BIT(1));
            }

            i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, write_buffer, 2, 50);
        }
}
