#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include "lookupheader.h"

using namespace std;

class LookupTable {
    public:
        float get_value(float xValue);
        bool add_value(const int16_t sample_point_value, const uint16_t x_value, float threshold);        
        void get_x_headers(uint16_t *size, int16_t **headers);
        int16_t* get_current_data(void);
        const LookupHeader* get_header(void);
        uint16_t data_size(void) const;
    protected:
        uint16_t x_header_size;
        uint16_t dataSize;
        int16_t* data;
        LookupHeader* x_header;
    private:
        inline float interpolate_x(const float x_value, uint16_t* idx_min, uint16_t* idx_max);
};

class LookupAllocTable: public LookupTable {
    public:
        LookupAllocTable(const int16_t* _xHeader, const uint16_t _xHeaderSize);
        LookupAllocTable(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _data, const uint16_t _dataSize);
        ~LookupAllocTable(void);
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