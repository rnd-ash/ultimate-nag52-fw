#include "lookuptable.h"
#include "../hal/hardwareabstractionlayer.h"
#include <string.h>
#include "tcu_maths.h"

LookupTable::LookupTable(const int16_t *_xHeader, const uint16_t _xHeaderSize)
{
    dataSize = 0u;
    data = nullptr;
    allocation_successful = false;
    xHeaderSize = _xHeaderSize;
    xHeader = new LookupHeader(_xHeader, _xHeaderSize);
}

LookupTable::LookupTable(const int16_t *_xHeader, const uint16_t _xHeaderSize, const int16_t *_data, const uint16_t _dataSize) : LookupTable(_xHeader, _xHeaderSize)
{
    dataSize = _dataSize;
    xHeaderSize = _xHeaderSize;
    data = static_cast<int16_t*>(MALLOC(dataSize * sizeof(int16_t)));
    allocation_successful = (nullptr != data);
    if (allocation_successful)
    {
        (void)memcpy(data, _data, dataSize*sizeof(int16_t));
    }
}

LookupTable::~LookupTable(void)
{
    FREE(data);
}

bool LookupTable::set_data(int16_t* _data, uint16_t _dataSize)
{
    bool result = false;
    dataSize = _dataSize;
    if(allocation_successful) {
        FREE(data);
    }
    data = static_cast<int16_t*>(heap_caps_malloc(dataSize * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    allocation_successful = (nullptr != data);
    if (allocation_successful)
    {
        (void)memcpy(data, _data, dataSize*sizeof(int16_t));
    }
    return result;
}

bool LookupTable::is_allocated(void)
{
    return allocation_successful;
}

float LookupTable::get_value(float xValue)
{
    uint16_t    idx_min;
    uint16_t    idx_max;
    
    // part 1 - identification of the indices for x-value
    xHeader->set_indices(xValue, &idx_min, &idx_max);

    // part 2: do the interpolation
    int16_t x1 = xHeader->get_value(idx_min);
    int16_t x2 = xHeader->get_value(idx_max);
    
    return interpolate((float)data[idx_min], (float)data[idx_max], x1, x2, xValue);
}

int16_t* LookupTable::get_current_data(void) {
    return data;
}

void LookupTable::get_x_headers(uint16_t *size, int16_t **headers){
    *size = xHeaderSize;
    *headers = xHeader->get_data();
}

bool LookupTable::add_data(const int16_t* map, const uint16_t size) {
    bool result = false;
    if (nullptr != map)
    {
        if (size == dataSize)
        {
            (void)memcpy(this->data, map, size * sizeof(int16_t));
            result = true;
        }
    }
    return result;
}
