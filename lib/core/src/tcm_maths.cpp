#include "tcm_maths.h"
#include "string.h"
#include "esp_heap_caps.h"

// float scale_number(int16_t raw, int16_t new_min, int16_t new_max, int16_t raw_min, int16_t raw_max) {
//     int16_t raw_temp = MAX(raw_min, MIN(raw, raw_max));
//     return (((((float)new_max) - ((float)new_min)) * (((float)raw_temp) - ((float)raw_min))) / (((float)raw_max) - ((float)raw_min))) + ((float)new_min);
// }

float scale_number(float raw, float new_min, float new_max, float raw_min, float raw_max) {
    float raw_limited = MAX(raw_min, MIN(raw, raw_max));
    return (((new_max - new_min) * (raw_limited - raw_min)) / (raw_max - raw_min)) + new_min;
}

TcmMap::TcmMap(uint16_t X_Size, uint16_t Y_size, const int16_t* x_ids, const int16_t* y_ids) {

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

bool TcmMap::add_data(int16_t* map, uint16_t size) {
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

bool TcmMap::allocate_ok(void) const {
    return this->alloc_ok;
}

void TcmMap::get_x_headers(uint16_t* dest_size, int16_t** dest) {
    *dest = this->x_headers;
    *dest_size = this->x_size;
}

void TcmMap::get_y_headers(uint16_t* dest_size, int16_t** dest) {
    *dest = this->y_headers;
    *dest_size = this->y_size;
}

/* unused code, thus commented to be compliant with MISRA C 2012*/
// int16_t* TcmMap::get_row(uint16_t id) {
//     // return null-pointer, if row is out of range
//     return (id >= (this->y_size)) ? nullptr : (&this->data[id*this->x_size]);
// }

inline void TcmMap::set_indices(const int16_t value, uint16_t *idx_min, uint16_t *idx_max, const int16_t *headers, uint16_t size){
    *idx_min = 0u;
    *idx_max = size - 1u;
    if (value <= headers[0]){
        if (value >= headers[*idx_max]){
            do{
                uint16_t idx_mid = (*idx_min + *idx_max) >> 1;
                if (value < headers[idx_mid]) {
                    *idx_max = idx_mid;
                }
                else if (value > headers[idx_mid]){
                    *idx_min = idx_mid;
                }
                else {
                    *idx_min = idx_mid;
                    *idx_max = idx_mid;
                }
            } while (1u < ((*idx_max) - (*idx_min)));
        }
        else {
            *idx_min = *idx_max;
        }
    }
    else {
        *idx_max = *idx_min;
    }
}

float TcmMap::get_value(float x_value, float y_value) {
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;

    // part 1: identify the indices
    // lookup of indices for X-value
    set_indices(x_value, &x_idx_min, &x_idx_max, this->x_headers, this->x_size);

    // lookup of indices for Y-value
    set_indices(y_value, &y_idx_min, &y_idx_max, this->y_headers, this->y_size);

    // part 2: do the bilinear interpolation
    // see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/
    float x1 = (float)this->x_headers[x_idx_min];
    float x2 = (float)this->x_headers[x_idx_max];
    float y1 = (float)this->y_headers[y_idx_min];
    float y2 = (float)this->y_headers[y_idx_max];
    float x2x1 = x2 - x1;
    float y2y1 = y2 - y1;
    float x2x = x2 - x_value;
    float y2y = y2 - y_value;
    float yy1 = y_value - y1;
    float xx1 = x_value - x1;
    return ((((float)data[(y_idx_min * this->x_size) + x_idx_min]) * x2x * y2y) + (((float)data[(y_idx_max * this->x_size) + x_idx_min]) * xx1 * y2y) + (((float)data[(y_idx_min * this->x_size) + x_idx_max]) * x2x * yy1) + (((float)data[(y_idx_max * this->x_size) + x_idx_max]) * xx1 * yy1)) / (x2x1 * y2y1);
}
int16_t* TcmMap::get_current_data() {
    return this->data;
}