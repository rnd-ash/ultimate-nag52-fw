#ifndef TABLEHEADER_H
#define TABLEHEADER_H

#include <stdint.h>

class LookupHeader {
    public:
        int16_t get_value(const uint16_t index) const;
        uint16_t get_size(void) const;
        int16_t* get_data(void);  
    protected:
        int16_t* header;
        uint16_t size;

};

class LookupAllocHeader: public LookupHeader {
    public:
        /// @brief manages a table header for a lookup table or lookup map
        /// @param _header 
        /// @param _length 
        LookupAllocHeader(const int16_t* _header, const uint16_t _size);
        /// @brief frees the allocated memory
        ~LookupAllocHeader(void);    
    private:
        /// @brief is true, if the memory allocation for the headers was successful
        bool allocation_successful;
};

class LookupRefHeader: public LookupHeader {
    public:
        /// @brief manages a table header for a lookup table or lookup map
        /// @param _header 
        /// @param _length 
        LookupRefHeader(int16_t* _header, const uint16_t _size);
};

#endif