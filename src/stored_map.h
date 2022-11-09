#ifndef __STORED_MAP_H_
#define __STORED_MAP_H_

#include <tcm_maths.h>


class StoredTcuMap {

    public:
        StoredTcuMap(
            const char* eeprom_key_name,
            const uint16_t map_size,
            const int16_t* x_headers,
            const int16_t* y_headers,
            int16_t x_size,
            int16_t y_size,
            const int16_t* default_map
        );

        bool init_ok();

        /**
         * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
         */
        bool save_to_eeprom();

        /**
         * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
         * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
         * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
         */
        bool replace_map_content(int16_t* new_data, uint16_t content_len);

        /**
         * @brief Reloads the previously saved map from EEPROM into the map (Undo function)
         */
        bool reload_from_eeprom();

        /**
         * @brief Resets the map data to the stock map from the TCU firmware (maps.cpp)
         * THIS RESETS THE MAP TO FIRMWARE DEFAULT - ALL CHANGES WILL BE LOST!
         */
        bool reset_from_default_eeprom();

        // See TcmMap.cpp
        float get_value(float x_value, float y_value);

        uint16_t get_map_element_count();

        const int16_t* get_default_map_data();
        int16_t* get_current_map_data();

    private:
        TcmMap* internal;
        const char* map_name;
        uint16_t map_size;
        const int16_t* default_map;
        bool read_from_eeprom(const char* key_name, uint16_t expected_size);
};

#endif