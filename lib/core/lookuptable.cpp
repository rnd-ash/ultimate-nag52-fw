#include "lookuptable.h"
#include "tcu_alloc.h"
#include <string.h>
#include "tcu_maths_impl.h"

// Lookup table base implementations
LookupTable::LookupTable(const int16_t *_x_header, const uint16_t _x_header_size, int16_t *_data, const uint16_t _dataSize)
{
    this->x_header = new LookupHeader(const_cast<int16_t*>(_x_header), _x_header_size);
    this->data = _data;
    this->dataSize = _dataSize;
}

float LookupTable::get_value(float xValue)
{
    uint16_t    idx_min;
    uint16_t    idx_max;
    
    // part 1 - identification of the indices for x-value
    search_value<int16_t>(xValue, x_header->get_data(), x_header_size, &idx_min, &idx_max);
    
    // part 2: do the interpolation
    int16_t x1 = x_header->get_value(idx_min);
    int16_t x2 = x_header->get_value(idx_max);
    
    return interpolate((float)data[idx_min], (float)data[idx_max], x1, x2, xValue);
}

bool LookupTable::add_value(const int16_t sample_point_value, const uint16_t x_value, float threshold)
{
    // calibration parameter
    const float adapt_gain = 0.20F;
    
    uint16_t    idx_min;
    uint16_t    idx_max;
 
    // interpolation to get the current map value at the given x position
    const float interp = interpolate_x(x_value, &idx_min, &idx_max);
    
    // weight calculation
    const float w_x = (x_value - (float)x_header->get_value(idx_min)) / ((float)x_header->get_value(idx_max) - (float)x_header->get_value(idx_min));
    
    // deviatation
    const float delta = (float)sample_point_value - interp;
    // rating    
    const bool significant_change = (((float)abs(delta) / interp) > threshold);    
    // correction calculation
    const float corr = delta * adapt_gain;

    // map adaptation
    data[idx_min] = clampint16((int32_t)data[idx_min] + (int32_t)(corr * (1.0F - w_x)));
    data[idx_max] = clampint16((int32_t)data[idx_max] + (int32_t)(corr * w_x));
    return significant_change;
}

float LookupTable::get_header_interpolated(const float value) const
{
    uint16_t    idvalue_min;
    uint16_t    idvalue_max;

    // part 1 - identification of the indices for x-value
    search_value<int16_t>(value, data, dataSize, &idvalue_min, &idvalue_max);

    // part 2: do the interpolation
    const float value1 = (float)x_header->get_value(idvalue_min);
    const float value2 = (float)x_header->get_value(idvalue_max);
    
    return value1 + progress_between_targets(value, data[idvalue_min], data[idvalue_max]) * (value2 - value1);
}

void LookupTable::get_x_headers(uint16_t *size, int16_t **headers){
    *size = x_header_size;
    *headers = x_header->get_data();
}

int16_t* LookupTable::get_current_data(void) {
    return data;
}

const LookupHeader* LookupTable::get_header(void) {
    return this->x_header;
}

uint16_t LookupTable::data_size(void) const {
    return this->dataSize;
}

inline float LookupTable::interpolate_x(const float x_value, uint16_t *idx_min, uint16_t *idx_max)
{
    // part 1 - identification of the indices for x-value
    search_value<int16_t>(x_value, x_header->get_data(), x_header->get_size(), idx_min, idx_max);
    
    // part 2: do the interpolation on x-axis
    const int16_t x1 = x_header->get_value(*idx_min);
    const int16_t x2 = x_header->get_value(*idx_max);
    
    // interpolation 
    return interpolate((float)data[(*idx_min)],(float)data[(*idx_max)], x1, x2, x_value);
}

// Alloc table implementation
LookupAllocTable::LookupAllocTable(const int16_t *_xHeader, uint16_t _xHeaderSize) : LookupTable(_xHeader, _xHeaderSize, nullptr, 0u)
{
    // dataSize = 0u;
    // data = nullptr;
    // x_header_size = _xHeaderSize;
    x_header = new LookupAllocHeader(_xHeader, _xHeaderSize);
    allocation_successful = false;
}

LookupAllocTable::LookupAllocTable(const int16_t *_xHeader, const uint16_t _xHeaderSize, const int16_t *_data, const uint16_t _dataSize) : LookupTable(_xHeader, _xHeaderSize, static_cast<int16_t*>(TCU_HEAP_ALLOC(_dataSize * sizeof(int16_t))), _dataSize)
{
    // dataSize = _dataSize;
    // x_header_size = _xHeaderSize;
    // data = static_cast<int16_t*>(TCU_HEAP_ALLOC(dataSize * sizeof(int16_t)));
    allocation_successful = (nullptr != data);
    if (allocation_successful)
    {
        (void)memcpy(data, _data, dataSize*sizeof(int16_t));
    }
    this->x_header = new LookupAllocHeader(_xHeader, _xHeaderSize);
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

LookupRefTable::LookupRefTable(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _data, uint16_t _dataSize) : LookupTable(_xHeader, _xHeaderSize, _data, _dataSize)
{
    this->x_header = new LookupRefHeader(_xHeader, _xHeaderSize);
}

