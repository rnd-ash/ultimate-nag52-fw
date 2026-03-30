#ifndef LOOKUPMAP_H
#define LOOKUPMAP_H

#include "lookuptable.h"

class LookupMap {
    public:
        float get_value(const int16_t x_value, const int16_t y_value);
        bool add_value(const int16_t sample_point_value, const int16_t x_value, const int16_t y_value, const float threshold);
        void get_y_headers(uint16_t *size, int16_t **headers);
        int16_t* get_current_data(void) const;
        void get_x_headers(uint16_t *size, int16_t **headers);
        uint16_t data_size();
    protected:
        LookupTable* table;
        LookupHeader* x_header;
        uint16_t x_header_size;
        LookupHeader* y_header;
        uint16_t y_header_size;
    private:
        inline float interpolate_xy(const float x, const float y, uint16_t* x_idx_min, uint16_t* x_idx_max, uint16_t* y_idx_min, uint16_t* y_idx_max, int16_t* x1, int16_t* x2, int16_t* y1, int16_t* y2);
};

class LookupAllocMap : public LookupMap {
    public:
        LookupAllocMap(const int16_t* _x_header, const uint16_t _x_header_size, const int16_t* _y_header, const uint16_t _y_header_size, const int16_t* _data, const uint16_t _data_size);
        bool add_data(const int16_t* map, const uint16_t size);
        bool is_allocated(void) const;
        ~LookupAllocMap();
};

class LookupRefMap : public LookupMap {
    public:
        LookupRefMap(int16_t* _x_header, const uint16_t _x_header_size, int16_t* _y_header, const uint16_t _y_header_size, int16_t* _data, const uint16_t _data_size);
};

class LookupByteMap : public LookupMap {
    public:
        LookupByteMap(uint8_t* _x_header, const uint16_t _x_header_size, uint8_t* _y_header, const uint16_t _y_header_size, uint8_t* _data, const uint16_t _data_size);
        bool is_allocated(void) const;
        bool add_data(const uint8_t* map, const uint16_t size);
        ~LookupByteMap();
    private:
        int16_t* x_alloc;
        int16_t* y_alloc;
        int16_t* z_alloc;
        uint16_t z_size;
};

#endif /* lookupmap.h */