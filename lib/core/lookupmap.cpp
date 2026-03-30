#include "lookupmap.h"
#include "tcu_maths_impl.h"
#include "tcu_alloc.h"

float LookupMap::get_value(const int16_t x_value, const int16_t y_value)
{
    uint16_t    idx_min;
    uint16_t    idx_max;
    uint16_t    idy_min;
    uint16_t    idy_max;
    const int16_t* data = this->table->get_current_data();

    // part 1a - identification of the indices for x-value
    search_value<int16_t>(x_value, x_header->get_data(), x_header->get_size(), &idx_min, &idx_max);
    
    // part 1b - identification of the indices for y-value
    search_value<int16_t>(y_value, y_header->get_data(), y_header->get_size(), &idy_min, &idy_max);
    
    // part 2: do the interpolation
    const int16_t x1 = x_header->get_value(idx_min);
    const int16_t x2 = x_header->get_value(idx_max);
    const int16_t y1 = y_header->get_value(idy_min);
    const int16_t y2 = y_header->get_value(idy_max);

    // some precalculations for making the code more readable, although somewhat inefficient
    const float f_11 = (float)data[(idy_min * x_header_size) + idx_min];
    const float f_12 = (float)data[(idy_min * x_header_size) + idx_max];
    const float f_21 = (float)data[(idy_max * x_header_size) + idx_min];
    const float f_22 = (float)data[(idy_max * x_header_size) + idx_max];

    // interpolation on x-axis for smaller y-index
    const float f_11f_12_interpolated = interpolate(f_11, f_12, x1, x2, (float)x_value);
    // interpolation on x-axis for greater y-index
    const float f_21f_22_interpolated = interpolate(f_21, f_22, x1, x2, (float)x_value);
    // bilinear interpolation, not always efficient, but with more or less constant runtime
    // also see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/ for mathematical background
    return interpolate(f_11f_12_interpolated, f_21f_22_interpolated, y1, y2, (float)y_value);
}

bool LookupMap::add_value(const int16_t sample_point_value, const int16_t x_value, const int16_t y_value, const float threshold)
{
    // calibration parameter
    const float adapt_gain = 0.20F;
    
    // local variables
    int16_t* data = table->get_current_data();

    uint16_t    idx_min;
    uint16_t    idx_max;
    uint16_t    idy_min;
    uint16_t    idy_max;

    int16_t x1;
    int16_t x2;
    int16_t y1;
    int16_t y2;

    // interpolation to get the current map value at the given x/y position
    const float interp = interpolate_xy(x_value, y_value, &idx_min, &idx_max, &idy_min, &idy_max, &x1, &x2, &y1, &y2);
    
    bool significant_change = false;

    if((INT16_MAX != x1) && (INT16_MAX != x2) && (INT16_MAX != y1) && (INT16_MAX != y2)){
        // weights calculation
        const float w_x = (float)(x_value - x1) / (float)(x2 - x1);
        const float w_y = (float)(y_value - y1) / (float)(y2 - y1);

        // deviatation
        const float delta = (float)sample_point_value - interp;
        // rating    
        significant_change = (((float)abs(delta) / interp) > threshold);    
        // correction calculation
        const float corr = delta * adapt_gain;

        // map adaptation
        data[idy_min * x_header_size + idx_min] = clampint16((int32_t)data[idy_min * x_header_size + idx_min] + (int32_t)(corr * (1.0F - w_x) * (1.0F - w_y)));
        data[idy_min * x_header_size + idx_max] = clampint16((int32_t)data[idy_min * x_header_size + idx_max] + (int32_t)(corr * w_x * (1.0F - w_y)));
        data[idy_max * x_header_size + idx_min] = clampint16((int32_t)data[idy_max * x_header_size + idx_min] + (int32_t)(corr * (1.0F - w_x) * w_y));
        data[idy_max * x_header_size + idx_max] = clampint16((int32_t)data[idy_max * x_header_size + idx_max] + (int32_t)(corr * w_x * w_y));
    }
    return significant_change;
}

void LookupMap::get_y_headers(uint16_t *size, int16_t **headers){
    *size = y_header_size;
    *headers = y_header->get_data();
}

int16_t* LookupMap::get_current_data(void) const {
    return this->table->get_current_data();
}

void LookupMap::get_x_headers(uint16_t *size, int16_t **headers) {
    return table->get_x_headers(size, headers);
}

uint16_t LookupMap::data_size() {
    return this->table->data_size();
}

inline float LookupMap::interpolate_xy(const float x_value, const float y_value, uint16_t* idx_min, uint16_t* idx_max, uint16_t* idy_min, uint16_t* idy_max, int16_t* x1, int16_t* x2, int16_t* y1, int16_t* y2)
{
    // part 1a - identification of the indices for x-value
    search_value<int16_t>(x_value, x_header->get_data(), x_header->get_size(), idx_min, idx_max);
    
    // part 1b - identification of the indices for y-value
    search_value<int16_t>(y_value, y_header->get_data(), y_header->get_size(), idy_min, idy_max);
    
    // part 2: do the interpolation
    *x1 = x_header->get_value(*idx_min);
    *x2 = x_header->get_value(*idx_max);
    *y1 = y_header->get_value(*idy_min);
    *y2 = y_header->get_value(*idy_max);
    const int16_t* data = table->get_current_data();

    // some precalculations for making the code more readable, although somewhat inefficient
    const float f_11 = (float)data[((*idy_min) * x_header->get_size()) + (*idx_min)];
    const float f_12 = (float)data[((*idy_min) * x_header->get_size()) + (*idx_max)];
    const float f_21 = (float)data[((*idy_max) * x_header->get_size()) + (*idx_min)];
    const float f_22 = (float)data[((*idy_max) * x_header->get_size()) + (*idx_max)];

    // interpolation on x-axis for smaller y-index
    const float f_11f_12_interpolated = interpolate(f_11, f_12, *x1, *x2, x_value);
    // interpolation on x-axis for greater y-index
    const float f_21f_22_interpolated = interpolate(f_21, f_22, *x1, *x2, x_value);
    // bilinear interpolation, not always efficient, but with more or less constant runtime
    // also see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/ for mathematical background
    return interpolate(f_11f_12_interpolated, f_21f_22_interpolated, *y1, *y2, y_value);
}

LookupAllocMap::LookupAllocMap(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _yHeader, const uint16_t _yHeaderSize, const int16_t* _data, const uint16_t _dataSize) {
    this->table = new LookupAllocTable(_xHeader, _xHeaderSize, _data, _dataSize);
    this->x_header = const_cast<LookupHeader*>(this->table->get_header());
    this->x_header_size = _xHeaderSize;
    this->y_header = new LookupAllocHeader(_yHeader, _yHeaderSize);
    this->y_header_size = _yHeaderSize;
}

LookupAllocMap::~LookupAllocMap() {
    delete this->y_header;
    delete this->table;
}

bool LookupAllocMap::add_data(const int16_t* map, const uint16_t size) {
    return reinterpret_cast<LookupAllocTable*>(this->table)->add_data(map, size);
}

bool LookupAllocMap::is_allocated(void) const {
    return reinterpret_cast<LookupAllocTable*>(this->table)->is_allocated();
}

LookupRefMap::LookupRefMap(int16_t* _xHeader, const uint16_t _xHeaderSize, int16_t* _yHeader, const uint16_t _yHeaderSize, int16_t* _data, const uint16_t _dataSize) {
    this->table = new LookupRefTable(_xHeader, _xHeaderSize, _data, _dataSize);
    this->x_header = const_cast<LookupHeader*>(this->table->get_header());
    this->x_header_size = _xHeaderSize;
    this->y_header = new LookupRefHeader(_yHeader, _yHeaderSize);
    this->y_header_size = _yHeaderSize;
}

LookupByteMap::LookupByteMap(uint8_t* _xHeader, const uint16_t _xHeaderSize, uint8_t* _yHeader, const uint16_t _yHeaderSize, uint8_t* _data, const uint16_t _dataSize) {
    this->x_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_xHeaderSize * sizeof(int16_t)));
    this->y_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_yHeaderSize * sizeof(int16_t)));
    this->z_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_dataSize * sizeof(int16_t)));

    for (auto i = 0; i < _xHeaderSize; i++) {
        this->x_alloc[i] = _xHeader[i];
    }
    for (auto i = 0; i < _yHeaderSize; i++) {
        this->y_alloc[i] = _yHeader[i];
    }
    for (auto i = 0; i < _dataSize; i++) {
        this->z_alloc[i] = _data[i];
    }
    if (nullptr != this->x_alloc && nullptr != this->y_alloc && nullptr != this->z_alloc) {
        this->table = new LookupRefTable(x_alloc, _xHeaderSize, z_alloc, _dataSize);
        this->x_header = const_cast<LookupHeader*>(this->table->get_header());
        this->x_header_size = _xHeaderSize;    
        this->y_header = new LookupRefHeader(y_alloc, _yHeaderSize);
        this->y_header_size = _yHeaderSize;
    }
}

bool LookupByteMap::is_allocated(void) const {
    return nullptr != this->x_alloc && nullptr != this->y_alloc && nullptr != this->z_alloc;
}

bool LookupByteMap::add_data(const uint8_t* map, const uint16_t size) {
    if (size != this->z_size) {
        return false;
    } else {
        for (auto i = 0; i < size; i++) {
            this->z_alloc[i] = map[i];
        }
        return true;
    }
}

LookupByteMap::~LookupByteMap() {
    delete this->table;
    delete this->y_header;
    TCU_FREE(this->x_alloc);
    TCU_FREE(this->y_alloc);
    TCU_FREE(this->z_alloc);
}