#ifndef TABLEHEADER_H
#define TABLEHEADER_H

#include <stdint.h>

class LookupHeader {

    public:
        
        /// @brief manages a table header for a lookup table or lookup map
        /// @param _header 
        /// @param _length 
        LookupHeader(const int16_t* _header, const uint16_t _size);

        /// @brief frees the allocated memory
        ~LookupHeader(void);

        /// @brief Sets the indices idx_min and idx_max in between the value is found in headers. idx_min and idx_max are identical, if value is an element of headers.
        /// @param value the value to be searched in the sorted list of headers
        /// @param idx_min the index of the element in headers, which is smaller than or equal to value
        /// @param idx_max the index of the element in headers, which is greater than or equal to value
        void set_indices(const float value, uint16_t* idx_min, uint16_t* idx_max);

        int16_t get_value(const uint16_t index);
        uint16_t get_size(void);
        int16_t* get_data(void);
        
    private:

        /// @brief is true, if the memory allocation for the headers was successful
        bool allocation_successful;

        /// @brief points to the array with the headers
        int16_t* header;

        /// @brief indicates the length of the headers
        uint16_t size;
};

#endif