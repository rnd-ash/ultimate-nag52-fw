#ifndef IOEXPANDER_H
#define IOEXPANDER_H

#include <stdint.h>
#include "esp_err.h"
#include "shifter/shifter.h"
#include "board_config.h"

class IOExpander {
public:
	IOExpander(void);
	esp_err_t init_state(void) const;
	void read_from_ioexpander(void);
	void write_to_ioexpander(void);
	bool is_data_valid(const uint32_t expire_time_ms) const;
	uint8_t get_trrs(void);
	bool get_kickdown(void);
	ProfileSwitchPos get_program_switch(void);
	bool get_brake_light_switch(void);
	void set_rp_solenoid(const bool rp_solenoid_enabled);
	void set_start(const bool start_enabled);
	void set_gearbox_protection(const bool gearbox_protection_enabled);
	void debug_get_registers(uint8_t* ll, uint8_t* hb);
private:
    const uint8_t IO_ADDR = 0x20u;
	const char* name = "IOEXPANDER";

    uint8_t i2c_rx_bytes[2] = {0, 0};
    uint8_t i2c_tx_bytes[2] = {0, 0};
    uint32_t last_i2c_query_time = 0u;

	esp_err_t init_status = ESP_FAIL;
	
	enum PCAReg
	{
		INPUT0 = 0u,
		INPUT1 = 1u,
		OUTPUT0 = 2u,
		OUTPUT1 = 3u,
		POLARITY0 = 4u,
		POLARITY1 = 5u,
		CONFIG0 = 6u,
		CONFIG1 = 7u
	};

	inline static bool get_bool_value(const pca_num_t bit, const uint8_t* i2c_rx_bytes);
	inline static void set_value(const bool value, const pca_num_t bit, uint8_t* i2c_tx_bytes);
};

extern IOExpander* ioexpander;

#endif