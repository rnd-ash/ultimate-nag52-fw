#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include "lookupheader.h"

using namespace std;

class LookupTable {
    public:
        LookupTable(const int16_t* _x_header, const uint16_t _x_header_size, int16_t *_data, const uint16_t _dataSize);
        float get_value(float xValue);
        bool add_value(const int16_t sample_point_value, const uint16_t x_value, float threshold);
        /// @brief This functions generates a corresponding header-value based on the parameter. This function does only work on tables with increasing x-values.
        /// @param value the value to be looked up
        /// @return the interpolated header-value
        float get_header_interpolated(const float value) const;
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
        LookupAllocTable(const int16_t* _x_header, const uint16_t _x_header_size);
        LookupAllocTable(const int16_t* _x_header, const uint16_t _x_header_size, const int16_t* _data, const uint16_t _dataSize);
        ~LookupAllocTable(void);
        bool set_data(const int16_t* _data, uint16_t _dataSize);
        bool is_allocated(void) const;
        bool add_data(const int16_t* map, const uint16_t size);
    private:
        bool allocation_successful;
};

class LookupRefTable: public LookupTable {
    public:
        LookupRefTable(int16_t* _x_header, uint16_t _x_header_size, int16_t* _data, uint16_t _dataSize);
};

#endif /* LOOKUPTABLE_H */