#ifndef STORED_TABLE_H
#define STORED_TABLE_H

#include "esp_err.h"
#include "../../lib/core/lookuptable.h"
#include "stored_data.h"

class StoredTable : public LookupTable, public StoredData{
    public:
        StoredTable(
            const char* eeprom_key_name,
            const uint16_t data_element_count,
            const int16_t* x_headers,
            uint16_t x_element_count,
            const int16_t* default_data
        );
    
        /**
         * @brief Replace map contents with new data (Keeping it in memory, call `save_to_eeprom` to write it to the TCU's EEPROM)
         * Note. This is a temporary replace. If you power the car down, changes made will be lost unless they
         * are written to EEPROM. This also acts as a failsafe in the event of a bad map edit, just reboot the car!
         */
        esp_err_t replace_data_content(int16_t* new_data, uint16_t content_len);
        esp_err_t save_to_eeprom(void) override;
        esp_err_t read_from_eeprom(const char* key_name, uint16_t expected_size) override;        
        
};

#endif // STORED_TABLE_H