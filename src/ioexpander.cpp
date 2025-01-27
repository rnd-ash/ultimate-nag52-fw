#include "ioexpander.h"
#include "esp_log.h"
#include "clock.hpp"
#include "board_config.h"

IOExpander::IOExpander(gpio_num_t sda, gpio_num_t scl)
{
	if ((gpio_num_t::GPIO_NUM_NC != sda) && (gpio_num_t::GPIO_NUM_NC != scl))
	{
		// init I/O expander module
		const i2c_master_bus_config_t conf = {
			.i2c_port = I2C_NUM_0,
			.sda_io_num = sda,
			.scl_io_num = scl,
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7,
			.intr_priority = 0,
			.trans_queue_depth = 4,
			.flags {
				.enable_internal_pullup = true
			}
		};

		i2c_master_bus_handle_t bus_handle;
		init_status = i2c_new_master_bus(&conf, &bus_handle);
		if (ESP_OK == init_status)
		{
			i2c_device_config_t dev_cfg = {
				.dev_addr_length = I2C_ADDR_BIT_LEN_7,
				.device_address = IO_ADDR,
				.scl_speed_hz = 100000u,
			};
			init_status = i2c_master_bus_add_device(bus_handle, &dev_cfg, &this->dev_handle);
			if (ESP_OK == init_status)
			{
				// set I/O 1 as output
				i2c_tx_bytes[0] = (uint8_t)PCAReg::CONFIG1;
				i2c_tx_bytes[1] = 0x00;
				init_status = i2c_master_transmit(this->dev_handle, i2c_tx_bytes, 2, 50);
				if (ESP_OK == init_status)
				{
					// set I/O 0 as inputs
					i2c_tx_bytes[0] = (uint8_t)PCAReg::CONFIG0;
					i2c_tx_bytes[1] = 0xFF;
					init_status = i2c_master_transmit(this->dev_handle, i2c_tx_bytes, 2, 50);
					i2c_tx_bytes[0] = (uint8_t)PCAReg::POLARITY0;
					i2c_tx_bytes[1] = 0x0u;
					init_status = i2c_master_transmit(this->dev_handle, i2c_tx_bytes, 2, 50);
					i2c_tx_bytes[0] = (uint8_t)PCAReg::OUTPUT1;
					i2c_tx_bytes[1] = 0x0u;
					init_status = i2c_master_transmit(this->dev_handle, i2c_tx_bytes, 2, 50);
					if (ESP_OK != init_status)
					{
						ESP_LOG_LEVEL(ESP_LOG_ERROR, name, "Failed to set input reg");
					}
				}
				else
				{
					ESP_LOG_LEVEL(ESP_LOG_ERROR, name, "Failed to set output reg");
				}
			}
			else
			{
				ESP_LOG_LEVEL(ESP_LOG_ERROR, name, "Failed to set param config");
			}
		}
		else
		{
			ESP_LOG_LEVEL(ESP_LOG_ERROR, name, "Failed to install driver");
		}
	}
	else
	{
		ESP_LOG_LEVEL(ESP_LOG_ERROR, name, "Cannot launch IOExpander on board without I2C!");
		init_status = ESP_ERR_INVALID_VERSION;
	}
}

esp_err_t IOExpander::init_state(void) const
{
	return init_status;
}

void IOExpander::read_from_ioexpander(void)
{
	if (init_status == ESP_OK)
	{
		uint32_t now = GET_CLOCK_TIME();
		if (50u < (now - last_i2c_query_time))
		{
			// query I2C I/O expander
			uint8_t req[2] = {(uint8_t)PCAReg::INPUT0, 0};
			esp_err_t e = i2c_master_transmit_receive(this->dev_handle, req, 1, i2c_rx_bytes, 2, 5);
			if (ESP_OK == e)
			{
				last_i2c_query_time = now;
			}
			else
			{
				ESP_LOGE(name, "Could not read from I2C I/O expander: %s", esp_err_to_name(e));
			}
		}
	}
}

void IOExpander::write_to_ioexpander(void)
{
	if (init_status == ESP_OK)
	{
		// write to I2C I/O expander
		esp_err_t e = i2c_master_transmit(this->dev_handle, i2c_tx_bytes, 2, 5);
		if (ESP_OK != e)
		{
			ESP_LOGE(name, "Could not write to I2C I/O expander: %s", esp_err_to_name(e));
		}
	}
}

bool IOExpander::is_data_valid(const uint32_t expire_time_ms) const
{
	return expire_time_ms > (GET_CLOCK_TIME() - last_i2c_query_time);
}

inline bool IOExpander::get_bool_value(const pca_num_t bit, const uint8_t *i2c_rx_bytes)
{
	return (i2c_rx_bytes[0] >> bit) & 0b1;
}

inline void IOExpander::set_value(const bool value, const pca_num_t bit, uint8_t *i2c_tx_bytes)
{
	// reset bit and keep existing buffer
	i2c_tx_bytes[1] &= ~(BIT(bit));
	// set bit
	i2c_tx_bytes[1] |= ((uint8_t)value) << bit;
}

uint8_t IOExpander::get_trrs(void)
{
	uint8_t result = get_bool_value(i2c_expander_trrs_a, i2c_rx_bytes);
	result |= (get_bool_value(i2c_expander_trrs_b, i2c_rx_bytes) << 1);
	result |= (get_bool_value(i2c_expander_trrs_c, i2c_rx_bytes) << 2);
	result |= (get_bool_value(i2c_expander_trrs_d, i2c_rx_bytes) << 3);
	return result;
}

bool IOExpander::get_kickdown(void)
{
	return get_bool_value(i2c_expander_kickdown_switch, i2c_rx_bytes);
}

bool IOExpander::is_program_switch_pressed(void)
{
	return get_bool_value(i2c_expander_program_button, i2c_rx_bytes);
}

bool IOExpander::is_brake_light_switch_pressed(void)
{
	return get_bool_value(i2c_expander_brake_light_switch, i2c_rx_bytes);
}

void IOExpander::set_rp_solenoid(const bool rp_solenoid_enabled)
{
	set_value(rp_solenoid_enabled, i2c_expander_rp_solenoid_enabler, i2c_tx_bytes);
}

void IOExpander::set_start(const bool start_enabled)
{
	set_value(start_enabled, i2c_expander_start_enabler, i2c_tx_bytes);
}

void IOExpander::set_gearbox_protection(const bool gearbox_protection_enabled)
{
	set_value(gearbox_protection_enabled, i2c_expander_gearbox_protection_enabler, i2c_tx_bytes);
}

void IOExpander::debug_get_registers(uint8_t* ll, uint8_t* hb) {
	*ll = this->i2c_rx_bytes[0];
	*hb = this->i2c_rx_bytes[1];
}

IOExpander *ioexpander = nullptr;
