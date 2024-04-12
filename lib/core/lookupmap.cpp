#include "lookupmap.h"
#include "tcu_maths_impl.h"

float LookupMap::get_value(const float xValue, const float yValue)
{
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;
    const LookupHeader* xHeader = this->table->get_header();
    const int16_t* data = this->table->get_current_data();

    // part 1a - identification of the indices for x-value
    search_value<int16_t>(xValue, xHeader->get_data(), xHeader->get_size(), &x_idx_min, &x_idx_max);
    
    // part 1b - identification of the indices for y-value
    search_value<int16_t>(yValue, yHeader->get_data(), yHeader->get_size(), &y_idx_min, &y_idx_max);
    
    // part 2: do the interpolation
    const int16_t x1 = xHeader->get_value(x_idx_min);
    const int16_t x2 = xHeader->get_value(x_idx_max);
    const int16_t y1 = yHeader->get_value(y_idx_min);
    const int16_t y2 = yHeader->get_value(y_idx_max);

    // some precalculations for making the code more readable, although somewhat inefficient
    const float f_11 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_min];
    const float f_12 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_max];
    const float f_21 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_min];
    const float f_22 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_max];

    // interpolation on x-axis for smaller y-index
    const float f_11f_12_interpolated = interpolate(f_11, f_12, x1, x2, xValue);
    // interpolation on x-axis for greater y-index
    const float f_21f_22_interpolated = interpolate(f_21, f_22, x1, x2, xValue);
    // bilinear interpolation, not always efficient, but with more or less constant runtime
    // also see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/ for mathematical background
    return interpolate(f_11f_12_interpolated, f_21f_22_interpolated, y1, y2, yValue);
}

void LookupMap::get_y_headers(uint16_t *size, int16_t **headers){
    *size = yHeaderSize;
    *headers = yHeader->get_data();
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

float LookupMap::get_x_header_interpolated(const float value, const int16_t y) const
{
    const LookupHeader* xHeader = this->table->get_header();
    const int16_t* data = this->table->get_current_data();
    // isolate the row
    int16_t row[xHeader->get_size()] = {0};
    for (uint16_t i = 0; i < xHeader->get_size(); i++)
    {
        row[i] = data[i*yHeaderSize];
    }
    
    uint16_t    idvalue_min;
    uint16_t    idvalue_max;

    // part 1 - identification of the indices for x-value
    search_value<int16_t>(value, row, xHeader->get_size(), &idvalue_min, &idvalue_max);

    // part 2: do the interpolation
    const float value1 = (float)xHeader->get_value(idvalue_min);
    const float value2 = (float)xHeader->get_value(idvalue_max);
    
    return value1 + progress_between_targets(value, row[idvalue_min], row[idvalue_max]) * (value2 - value1);
}

LookupAllocMap::LookupAllocMap(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _yHeader, const uint16_t _yHeaderSize, const int16_t* _data, const uint16_t _dataSize) {
    this->table = new LookupAllocTable(_xHeader, _xHeaderSize, _data, _dataSize);
    this->yHeader = new LookupAllocHeader(_yHeader, _yHeaderSize);
    this->yHeaderSize = _yHeaderSize;
}

LookupAllocMap::~LookupAllocMap() {
    delete this->yHeader;
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
    this->yHeader = new LookupRefHeader(_yHeader, _yHeaderSize);
    this->yHeaderSize = _yHeaderSize;
}