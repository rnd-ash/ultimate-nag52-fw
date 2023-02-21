#include "lookupmap.h"
#include "tcu_maths.h"

LookupMap::LookupMap(const int16_t *_xHeader, const uint16_t _xHeaderSize, const int16_t *_yHeader, const uint16_t _yHeaderSize, const int16_t *_data, const uint16_t _dataSize) :
    LookupTable(_xHeader, _xHeaderSize, _data, _dataSize)
{
    yHeaderSize = _yHeaderSize;
    yHeader = new LookupHeader(_yHeader, _yHeaderSize);
}

float LookupMap::get_value(const float xValue, const float yValue)
{
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;

    // part 1a - identification of the indices for x-value
    xHeader->set_indices(xValue, &x_idx_min, &x_idx_max);

    // part 1b - identification of the indices for y-value
    yHeader->set_indices(yValue, &y_idx_min, &y_idx_max);
    
    // part 2: do the interpolation
    int16_t x1 = xHeader->get_value(x_idx_min);
    int16_t x2 = xHeader->get_value(x_idx_max);
    int16_t y1 = yHeader->get_value(y_idx_min);
    int16_t y2 = yHeader->get_value(y_idx_max);

    // some precalculations for making the code more readable, although somewhat inefficient
    float f_11 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_min];
    float f_12 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_max];
    float f_21 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_min];
    float f_22 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_max];

    // interpolation on x-axis for smaller y-index
    float f_11f_12_interpolated = interpolate(f_11, f_12, x1, x2, xValue);
    // interpolation on x-axis for greater y-index
    float f_21f_22_interpolated = interpolate(f_21, f_22, x1, x2, xValue);
    // bilinear interpolation, not always efficient, but with more or less constant runtime
    // also see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/ for mathematical background
    return interpolate(f_11f_12_interpolated, f_21f_22_interpolated, y1, y2, yValue);
}

void LookupMap::get_y_headers(uint16_t *size, int16_t **headers){
    *size = yHeaderSize;
    *headers = yHeader->get_data();
}