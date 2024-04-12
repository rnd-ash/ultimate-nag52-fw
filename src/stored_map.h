#ifndef STORED_MAP_H
#define STORED_MAP_H

#include "../lib/core/lookupmap.h"
#include "esp_err.h"
#include "stored_table.h"

class StoredMap : public LookupAllocMap, public StoredData {

    public:
        StoredMap(
            const char* eeprom_key_name,
            const uint16_t data_element_count,
            const int16_t* x_headers,
            const int16_t* y_headers,
            const uint16_t x_size,
            const uint16_t y_size,
            const int16_t* default_map
        );

        /**
         * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
         * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
         * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
         */
        esp_err_t replace_data_content(const int16_t* new_data, uint16_t content_len);

        esp_err_t reset_from_flash() override;

        /**
         * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
         */
        esp_err_t save_to_eeprom(void) override;

        /**
         * @brief Resets the map data to the stock map from the TCU firmware (maps.cpp)
         * THIS RESETS THE MAP TO FIRMWARE DEFAULT - ALL CHANGES WILL BE LOST!
         */
        // bool reset_from_default_eeprom(void);

        uint16_t get_map_element_count(void);

        const int16_t* get_default_map_data(void);
        int16_t* get_current_eeprom_map_data(void);

        const char* get_map_name(void);

    private:
        const int16_t* default_map;
        esp_err_t read_from_eeprom(const char* key_name, uint16_t expected_size) override;
};

#endif // STORED_MAP_H