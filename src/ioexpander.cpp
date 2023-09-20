#include "ioexpander.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "clock.hpp"

IOExpander::IOExpander(void)
{
	if ((gpio_num_t::GPIO_NUM_NC != pcb_gpio_matrix->i2c_sda) && (gpio_num_t::GPIO_NUM_NC != pcb_gpio_matrix->i2c_scl))
	{
		// init I/O expander module
		i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = pcb_gpio_matrix->i2c_sda,
			.scl_io_num = pcb_gpio_matrix->i2c_scl,
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
				// set I/O 1 as output
				i2c_tx_bytes[0] = (uint8_t)PCAReg::CONFIG1;
				i2c_tx_bytes[1] = 0x0u;
				init_status = i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, i2c_tx_bytes, 2, 50);
				if (ESP_OK == init_status)
				{
					// set I/O 0 as inputs
					i2c_tx_bytes[0] = (uint8_t)PCAReg::CONFIG0;
					i2c_tx_bytes[1] = 0xFF;
					init_status = i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, i2c_tx_bytes, 2, 50);
					i2c_tx_bytes[0] = (uint8_t)PCAReg::OUTPUT1;
					i2c_tx_bytes[1] = 0x0u;
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
			uint8_t req[1] = {0};
			esp_err_t e = i2c_master_write_read_device(I2C_NUM_0, IO_ADDR, req, 1, i2c_rx_bytes, 2, 5);
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
		esp_err_t e = i2c_master_write_to_device(I2C_NUM_0, IO_ADDR, i2c_tx_bytes, 2, 5);
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
	uint8_t result = get_bool_value(pcb_gpio_matrix->i2c_expander_trrs_a, i2c_rx_bytes);
	result |= (get_bool_value(pcb_gpio_matrix->i2c_expander_trrs_b, i2c_rx_bytes) << 1);
	result |= (get_bool_value(pcb_gpio_matrix->i2c_expander_trrs_c, i2c_rx_bytes) << 2);
	result |= (get_bool_value(pcb_gpio_matrix->i2c_expander_trrs_d, i2c_rx_bytes) << 3);
	return result;
}

bool IOExpander::get_kickdown(void)
{
	return get_bool_value(pcb_gpio_matrix->i2c_expander_kickdown_switch, i2c_rx_bytes);
}

ProfileSwitchPos IOExpander::get_program_switch(void)
{
	bool tmp = get_bool_value(pcb_gpio_matrix->i2c_expander_program_button, i2c_rx_bytes);
	ProfileSwitchPos result = tmp ? ProfileSwitchPos::Top : ProfileSwitchPos::Bottom;
	return result;
}

bool IOExpander::get_brake_light_switch(void)
{
	return get_bool_value(pcb_gpio_matrix->i2c_expander_brake_light_switch, i2c_rx_bytes);
}

void IOExpander::set_rp_solenoid(const bool rp_solenoid_enabled)
{
	set_value(rp_solenoid_enabled, pcb_gpio_matrix->i2c_expander_rp_solenoid_enabler, i2c_tx_bytes);
}

void IOExpander::set_start(const bool start_enabled)
{
	set_value(start_enabled, pcb_gpio_matrix->i2c_expander_start_enabler, i2c_tx_bytes);
}

void IOExpander::set_gearbox_protection(const bool gearbox_protection_enabled)
{
	set_value(gearbox_protection_enabled, pcb_gpio_matrix->i2c_expander_gearbox_protection_enabler, i2c_tx_bytes);
}

IOExpander *ioexpander = nullptr;