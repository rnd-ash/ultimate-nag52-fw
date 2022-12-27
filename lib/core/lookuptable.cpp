#include "lookuptable.h"
#include "esp_heap_caps.h"
#include <string.h>
#include "tcu_maths.h"

LookupTable::LookupTable(int16_t *_xHeader, uint16_t _xHeaderSize) : xHeader(_xHeader, _xHeaderSize)
{
    dataSize = 0u;
    data = nullptr;
    allocation_successful = false;
}

LookupTable::LookupTable(int16_t *_xHeader, uint16_t _xHeaderSize, int16_t *_data, uint16_t _dataSize) : LookupTable(_xHeader, _xHeaderSize)
{
    dataSize = _dataSize;
    data = static_cast<int16_t*>(heap_caps_malloc(dataSize * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    allocation_successful = (nullptr != data);
    if (allocation_successful)
    {
        (void)memcpy(data, _data, dataSize);
    }
}

LookupTable::~LookupTable(void)
{
    heap_caps_free(data);
}

bool LookupTable::setData(int16_t* _data, uint16_t _dataSize)
{
    bool result = false;
    dataSize = _dataSize;
    if(allocation_successful) {
        heap_caps_free(data);
    }
    data = static_cast<int16_t*>(heap_caps_malloc(dataSize * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    allocation_successful = (nullptr != data);
    if (allocation_successful)
    {
        (void)memcpy(data, _data, dataSize);
    }
    return result;
}

bool LookupTable::isAllocated(void)
{
    return allocation_successful;
}

float LookupTable::getValue(float xValue)
{
    uint16_t    idx_min;
    uint16_t    idx_max;
    
    // part 1 - identification of the indices for x-value
    xHeader.setIndices(xValue, &idx_min, &idx_max);

    // part 2: do the interpolation
    int16_t x1 = xHeader.getValue(idx_min);
    int16_t x2 = xHeader.getValue(idx_max);
    
    return interpolate((float)data[idx_min], (float)data[idx_max], x1, x2, xValue);
}
