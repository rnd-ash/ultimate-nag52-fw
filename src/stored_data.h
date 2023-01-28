#ifndef STORED_DATA_H
#define STORED_DATA_H

#include "esp_err.h"

class StoredData {
	public:
    	esp_err_t init_status(void) const;
        /**
         * @brief Resets the map data to the stock map from the TCU firmware (maps.cpp)
         * THIS RESETS THE MAP TO FIRMWARE DEFAULT - ALL CHANGES WILL BE LOST!
         */
        // bool reset_from_default_eeprom(void);

		virtual esp_err_t read_from_eeprom(const char *key_name, uint16_t expected_size) = 0;

        /**
         * @brief Reloads the previously saved map from EEPROM into the map (Undo function)
         */
        esp_err_t reload_from_eeprom(void);

        uint16_t get_data_element_count(void);

        const int16_t* get_default_data(void);
        int16_t* get_current_eeprom_data(void);

        const char* get_data_name(void);

	protected:
        esp_err_t init_state;
		const char* data_name;
        uint16_t data_element_count;
        const int16_t* default_data;
};

#endif // STORED_DATA_H