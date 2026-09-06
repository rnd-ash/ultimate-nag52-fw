#ifndef TABLEHEADER_H
#define TABLEHEADER_H

#include <stdint.h>

class LookupHeader {
    public:
        // Virtual: LookupMap/LookupTable hold a LookupHeader* that may point to a
        // LookupAllocHeader, and delete through that base pointer. Without this,
        // the derived destructor never runs and the header allocation leaks.
        virtual ~LookupHeader(void) = default;
        int16_t get_value(const uint16_t index) const;
        uint16_t get_size(void) const;
        int16_t* get_data(void) const;
    protected:
        int16_t* header = nullptr;
        uint16_t size = 0u;

};

class LookupAllocHeader: public LookupHeader {
    public:
        /// @brief manages a table header for a lookup table or lookup map
        /// @param _header 
        /// @param _length 
        LookupAllocHeader(const int16_t* _header, const uint16_t _size);
        /// @brief frees the allocated memory
        ~LookupAllocHeader(void) override;
    private:
        /// @brief is true, if the memory allocation for the headers was successful
        bool allocation_successful = false;
};

class LookupRefHeader: public LookupHeader {
    public:
        /// @brief manages a table header for a lookup table or lookup map
        /// @param _header 
        /// @param _length 
        LookupRefHeader(int16_t* _header, const uint16_t _size);
};

#endif