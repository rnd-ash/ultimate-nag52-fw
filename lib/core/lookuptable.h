#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include "lookupheader.h"

using namespace std;

class LookupTable {
    public:
        float get_value(float xValue);
        /// @brief This functions generates a corresponding header-value based on the parameter. This function does only work on tables with increasing x-values.
        /// @param xValue the value to be looked up
        /// @return the interpolated header-value
        float get_header_interpolated(const float value) const;
        void get_x_headers(uint16_t *size, int16_t **headers);
    protected:
        uint16_t xHeaderSize;
        uint16_t dataSize;
        int16_t* data;
        LookupHeader* xHeader;
};

class LookupAllocTable: public LookupTable {
    public:
        LookupAllocTable(const int16_t* _xHeader, const uint16_t _xHeaderSize);
        LookupAllocTable(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _data, const uint16_t _dataSize);
        ~LookupAllocTable(void);
        int16_t* get_current_data(void);
        bool set_data(const int16_t* _data, uint16_t _dataSize);
        bool is_allocated(void) const;
        bool add_data(const int16_t* map, const uint16_t size);
    private:
        bool allocation_successful;
};

class LookupRefTable: public LookupTable {
    public:
        LookupRefTable(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _data, uint16_t _dataSize);
};

#endif /* LOOKUPTABLE_H */