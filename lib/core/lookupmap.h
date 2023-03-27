#ifndef LOOKUPMAP_H
#define LOOKUPMAP_H

#include "lookuptable.h"

class LookupMap : public LookupTable {

    public:
        LookupMap(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _yHeader, const uint16_t _yHeaderSize, const int16_t* _data, const uint16_t _dataSize);

        float get_value(const float xValue, const float yValue);
        void get_y_headers(uint16_t *size, int16_t **headers);

    protected:
        LookupHeader* yHeader;

    private:
        uint16_t yHeaderSize;
};

#endif /* lookupmap.h */