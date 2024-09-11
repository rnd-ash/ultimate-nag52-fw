#ifndef IOEXPANDER_H
#define IOEXPANDER_H

#include <stdint.h>
#include "esp_err.h"
#include "shifter/shifter.h"

enum pca_num_t{
    PCA_NUM_NC = -1,
    PCA_NUM_0 = 0,
    PCA_NUM_1 = 1,
    PCA_NUM_2 = 2,
    PCA_NUM_3 = 3,
    PCA_NUM_4 = 4,
    PCA_NUM_5 = 5,
    PCA_NUM_6 = 6,
    PCA_NUM_7 = 7
} ;

class IOExpander {
public:
	IOExpander(gpio_num_t sda, gpio_num_t scl);

	esp_err_t init_state(void) const;
	void read_from_ioexpander(void);
	void write_to_ioexpander(void);
	bool is_data_valid(const uint32_t expire_time_ms) const;
	uint8_t get_trrs(void);
	bool get_kickdown(void);
	bool is_program_switch_pressed(void);
	bool is_brake_light_switch_pressed(void);
	void set_rp_solenoid(const bool rp_solenoid_enabled);
	void set_start(const bool start_enabled);
	void set_gearbox_protection(const bool gearbox_protection_enabled);

	// inputs
    pca_num_t i2c_expander_trrs_a						= PCA_NUM_NC;
    pca_num_t i2c_expander_trrs_b						= PCA_NUM_NC;
    pca_num_t i2c_expander_trrs_c						= PCA_NUM_NC;
    pca_num_t i2c_expander_trrs_d						= PCA_NUM_NC;
    pca_num_t i2c_expander_brake_light_switch			= PCA_NUM_NC;
    pca_num_t i2c_expander_program_button				= PCA_NUM_NC;
    pca_num_t i2c_expander_kickdown_switch				= PCA_NUM_NC;

    // outputs
    pca_num_t i2c_expander_rp_solenoid_enabler			= PCA_NUM_NC;
    pca_num_t i2c_expander_start_enabler				= PCA_NUM_NC;
    pca_num_t i2c_expander_gearbox_protection_enabler	= PCA_NUM_NC;
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