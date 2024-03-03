#include "lookuptable.h"
#include "tcu_alloc.h"
#include <string.h>
#include "tcu_maths_impl.h"

// Lookup table base implementations

float LookupTable::get_value(float xValue)
{
    uint16_t    idx_min;
    uint16_t    idx_max;
    
    // part 1 - identification of the indices for x-value
    search_value<int16_t>(xValue, xHeader->get_data(), xHeaderSize, &idx_min, &idx_max);
    
    // part 2: do the interpolation
    int16_t x1 = xHeader->get_value(idx_min);
    int16_t x2 = xHeader->get_value(idx_max);
    
    return interpolate((float)data[idx_min], (float)data[idx_max], x1, x2, xValue);
}

float LookupTable::get_header_interpolated(const float value) const
{
    uint16_t    idvalue_min;
    uint16_t    idvalue_max;

    // part 1 - identification of the indices for x-value
    search_value<int16_t>(value, data, dataSize, &idvalue_min, &idvalue_max);

    // part 2: do the interpolation
    const float value1 = (float)xHeader->get_value(idvalue_min);
    const float value2 = (float)xHeader->get_value(idvalue_max);
    
    return value1 + progress_between_targets(value, data[idvalue_min], data[idvalue_max]) * (value2 - value1);
}

void LookupTable::get_x_headers(uint16_t *size, int16_t **headers){
    *size = xHeaderSize;
    *headers = xHeader->get_data();
}

int16_t* LookupTable::get_current_data(void) {
    return data;
}

const LookupHeader* LookupTable::get_header(void) {
    return this->xHeader;
}

uint16_t LookupTable::data_size(void) const {
    return this->dataSize;
}

// Alloc table implementation
LookupAllocTable::LookupAllocTable(const int16_t *_xHeader, uint16_t _xHeaderSize)
:LookupTable()
{
    dataSize = 0u;
    data = nullptr;
    allocation_successful = false;
    xHeaderSize = _xHeaderSize;
    xHeader = new LookupAllocHeader(_xHeader, _xHeaderSize);
}

LookupAllocTable::LookupAllocTable(const int16_t *_xHeader, const uint16_t _xHeaderSize, const int16_t *_data, const uint16_t _dataSize)
:LookupTable()
{
    dataSize = _dataSize;
    xHeaderSize = _xHeaderSize;
    data = static_cast<int16_t*>(TCU_HEAP_ALLOC(dataSize * sizeof(int16_t)));
    allocation_successful = (nullptr != data);
    if (allocation_successful)
    {
        (void)memcpy(data, _data, dataSize*sizeof(int16_t));
    }
    this->xHeader = new LookupAllocHeader(_xHeader, _xHeaderSize);
}

LookupAllocTable::~LookupAllocTable()
{
    TCU_FREE(data);
}

bool LookupAllocTable::add_data(const int16_t* map, const uint16_t size) {
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

bool LookupAllocTable::set_data(const int16_t* _data, uint16_t _dataSize)
{
    bool result = false;
    dataSize = _dataSize;
    if(allocation_successful) {
        TCU_FREE(data);
    }
    data = static_cast<int16_t*>(TCU_HEAP_ALLOC(dataSize * sizeof(int16_t)));
    allocation_successful = (nullptr != data);
    if (allocation_successful)
    {
        (void)memcpy(data, _data, dataSize*sizeof(int16_t));
    }
    return result;
}

bool LookupAllocTable::is_allocated(void) const
{
    return allocation_successful;
}

LookupRefTable::LookupRefTable(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _data, uint16_t _dataSize)
:LookupTable() {
    this->xHeader = new LookupRefHeader(_xHeader, _xHeaderSize);
    this->data = _data;
    this->dataSize = _dataSize;
}

