#ifndef LOOKUPMAP_H
#define LOOKUPMAP_H

#include "lookuptable.h"

class LookupMap : public LookupTable {
    public:
        float get_value(const float xValue, const float yValue);
        void get_y_headers(uint16_t *size, int16_t **headers);
        float get_x_header_interpolated(const float value, const int16_t y) const;
    protected:
        LookupHeader* yHeader;
        uint16_t yHeaderSize;
};

class LookupAllocMap : public LookupAllocTable, public LookupMap {
    public:
        LookupAllocMap(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _yHeader, const uint16_t _yHeaderSize, const int16_t* _data, const uint16_t _dataSize);
        ~LookupAllocMap();
};

class LookupRefMap : public LookupRefTable, public LookupMap {
    public:
        LookupRefMap(int16_t* _xHeader, const uint16_t _xHeaderSize, int16_t* _yHeader, const uint16_t _yHeaderSize, int16_t* _data, const uint16_t _dataSize);
};

#endif /* lookupmap.h */