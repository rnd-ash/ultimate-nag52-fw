#ifndef LOOKUPMAP_H
#define LOOKUPMAP_H

#include "lookuptable.h"

class LookupMap : public LookupTable {

    public:
        LookupMap(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _yHeader, uint16_t _yHeaderSize, int16_t* _data, uint16_t _dataSize);

        float getValue(float xValue, float yValue);
    protected:
        TableHeader yHeader;
};

#endif /* lookupmap.h */