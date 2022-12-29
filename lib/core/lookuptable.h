#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include "lookupheader.h"

class LookupTable{
    public:
        LookupTable(int16_t* _xHeader, uint16_t _xHeaderSize);
        LookupTable(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _data, uint16_t _dataSize);
        ~LookupTable(void);

        bool setData(int16_t* _data, uint16_t _dataSize);
        bool isAllocated(void);
        float getValue(float xValue);

    protected:
        LookupHeader xHeader;
        int16_t* data;
        uint16_t dataSize;

    private:
        bool allocation_successful;
};

#endif /* LOOKUPTABLE_H */