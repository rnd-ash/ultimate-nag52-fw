#include "tcumap.h"
#include "string.h"
#include "esp_heap_caps.h"

TcuMap::TcuMap(uint16_t X_Size, uint16_t Y_size, const int16_t* x_ids, const int16_t* y_ids) {

    this->x_size = X_Size;
    this->y_size = Y_size;

    this->alloc_ok = false;

    this->x_headers = static_cast<int16_t*>(heap_caps_malloc(x_size*sizeof(int16_t), MALLOC_CAP_SPIRAM));
    this->y_headers = static_cast<int16_t*>(heap_caps_malloc(y_size*sizeof(int16_t), MALLOC_CAP_SPIRAM));
    this->data = static_cast<int16_t*>(heap_caps_malloc(y_size*x_size*sizeof(int16_t), MALLOC_CAP_SPIRAM));

    if (!((nullptr == this->x_headers) || (nullptr == this->y_headers) || (nullptr == this->data))) {
        // Allocation succeeded!
        (void)memcpy(this->x_headers, x_ids, (sizeof(int16_t) * x_size));
        (void)memcpy(this->y_headers, y_ids, (sizeof(int16_t) * y_size));
        (void)memset(this->data, 0, y_size * (x_size * sizeof(int16_t)));
        this->alloc_ok = true;
    }
}

bool TcuMap::add_data(int16_t* map, uint16_t size) {
    bool result = false;
    if (nullptr != map)
    {
        if (((uint16_t) size) == ((this->x_size) * (this->y_size)))
        {
            (void)memcpy(this->data, map, size * sizeof(int16_t));
            result = true;
        }
    }
    return result;
}

bool TcuMap::allocate_ok(void) const {
    return this->alloc_ok;
}

/* unused code, thus commented to be compliant with MISRA C 2012*/
// int16_t* TcuMap::get_row(uint16_t id) {
//     // return null-pointer, if row is out of range
//     return (id >= (this->y_size)) ? nullptr : (&this->data[id*this->x_size]);
// }

inline void TcuMap::set_indices(const int16_t value, uint16_t *idx_min, uint16_t *idx_max, const int16_t *headers, uint16_t size){
    // Set minimum index to the first element of the field.
    *idx_min = 0u;
    // Set maximum index to the last element of the field.
    *idx_max = size - 1u;
    // Check, if search value is smaller than smallest element of the field.
    if (value > headers[0]){
        if (value < headers[*idx_max]){
            // Search value is in between the limits of the smallest and the biggest element of the field.
            do{
                // Calculate the middle of the remaining list. If the size is odd, it is rounded down.
                uint16_t idx_mid = (*idx_min + *idx_max) >> 1;
                if (value < headers[idx_mid]) {
                    // Search value is smaller than the element in the middle of the remaining list.
                    *idx_max = idx_mid;
                }
                else if (value > headers[idx_mid]){
                    // Search value is bigger than the element in the middle of the remaining list.
                    *idx_min = idx_mid;
                }
                else {
                    // Search value is also an element of the field.
                    *idx_min = idx_mid;
                    *idx_max = idx_mid;
                }
                // Reduce the remaining search area until it is narrowed down to two consecutive elements.
            } while (1u < ((*idx_max) - (*idx_min)));
        }
        else {
            // Search value is as big as or bigger then the biggest element in the field.
            *idx_min = *idx_max;
        }
    }
    else {
        // Search value is as small as or smaller than smallest element of the field.
        *idx_max = *idx_min;
    }
}

inline float TcuMap::interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x) {
    // cast values from signed integer values to floating values in order to avoid casting the same value twice
    const float x_1_f = (float)x_1;
    const float x_2_f = (float)x_2;
    // See https://en.wikipedia.org/wiki/Linear_interpolation for details. Return f_1, if x_1 and x_2 are identical.
    return (x_1 != x_2) ? f_1 + ((f_2 - f_1) / (x_2_f - x_1_f)) * (x - x_1_f) : f_1;
}

float TcuMap::get_value(float x_value, float y_value) {
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;
    
    // part 1a - identification of the indices for x-value
    set_indices(x_value, &x_idx_min, &x_idx_max, this->x_headers, this->x_size);

    // part 1b - identification of the indices for y-value
    set_indices(y_value, &y_idx_min, &y_idx_max, this->y_headers, this->y_size);

    // part 2: do the interpolation
    int16_t x1 = this->x_headers[x_idx_min];
    int16_t x2 = this->x_headers[x_idx_max];
    int16_t y1 = this->y_headers[y_idx_min];
    int16_t y2 = this->y_headers[y_idx_max];

    // some precalculations for making the code more readable, although somewhat inefficient
    float f_11 = (float)data[(y_idx_min * this->x_size) + x_idx_min];
    float f_12 = (float)data[(y_idx_min * this->x_size) + x_idx_max];
    float f_21 = (float)data[(y_idx_max * this->x_size) + x_idx_min];
    float f_22 = (float)data[(y_idx_max * this->x_size) + x_idx_max];

    // interpolation on x-axis for smaller y-index
    float f_11f_12_interpolated = interpolate(f_11, f_12, x1, x2, x_value);
    // interpolation on x-axis for greater y-index
    float f_21f_22_interpolated = interpolate(f_21, f_22, x1, x2, x_value);
    // bilinear interpolation, not always efficient, but with more or less constant runtime
    // also see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/ for mathematical background
    return interpolate(f_11f_12_interpolated, f_21f_22_interpolated, y1, y2, y_value);
}

int16_t* TcuMap::get_current_data(void) {
    return this->data;
}