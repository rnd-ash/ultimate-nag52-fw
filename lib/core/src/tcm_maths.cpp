#include "tcm_maths.h"
#include "string.h"
#include "esp_heap_caps.h"







float scale_number(int16_t raw, int16_t new_min, int16_t new_max, int16_t raw_min, int16_t raw_max) {
    raw = MAX(raw_min, MIN(raw, raw_max));
    return ((float)((new_max-new_min) * (raw - raw_min)) / ((float)(raw_max-raw_min))) + (float)new_min;
}









TcmMap::TcmMap(uint16_t X_Size, uint16_t Y_size, const int16_t* x_ids, const int16_t* y_ids) {

    this->x_size = X_Size;
    this->y_size = Y_size;

    this->alloc_ok = false;

    this->x_headers = (int16_t*)heap_caps_malloc(x_size*sizeof(int16_t), MALLOC_CAP_SPIRAM);
    this->y_headers = (int16_t*)heap_caps_malloc(y_size*sizeof(int16_t), MALLOC_CAP_SPIRAM);
    this->data = (int16_t*)heap_caps_malloc(y_size*x_size*sizeof(int16_t), MALLOC_CAP_SPIRAM);

    if (this->x_headers == nullptr || this->y_headers == nullptr || this->data == nullptr) {
        // Allocation failed!
        return;
    }

    memcpy(this->x_headers, x_ids, sizeof(int16_t)*x_size);
    memcpy(this->y_headers, y_ids, sizeof(int16_t)*y_size);
    memset(this->data, 0, y_size*x_size*sizeof(int16_t));
    this->alloc_ok = true;
}

bool TcmMap::add_data(int16_t* map, int size) {
    if (map == nullptr) {
        return false;
    }
    if (size != this->x_size*this->y_size) {
        return false;
    }
    memcpy(this->data, map, size*sizeof(int16_t));
    return true;
}

bool TcmMap::allocate_ok() {
    return this->alloc_ok;
}

int16_t* TcmMap::get_row(uint16_t id) {
    if (id >= this->y_size) {
        return nullptr; // Row out of range
    }
    return &this->data[id*this->x_size];
}


float TcmMap::get_value(float x_value, float y_value) {
    uint16_t x_idx_min = 0;
    uint16_t x_idx_max = 0;

    uint16_t y_idx_min = 0;
    uint16_t y_idx_max = 0;
    
    if (this->x_headers[0] >= x_value) {
        x_idx_min = 0;
        x_idx_max = 0;
    } else if (this->x_headers[this->x_size-1] <= x_value) {
        x_idx_min = this->x_size-1;
        x_idx_max = this->x_size-1;
    } else {
        // Lookup X value
        for (uint16_t i = 0; i < this->x_size-1; i++) {
            if (x_value == this->x_headers[i]) {
                x_idx_min = i;
                x_idx_max = i;
                break;
            }

            if (this->x_headers[i] <= x_value && this->x_headers[i+1] > x_value) {
                x_idx_min = i;
                x_idx_max = i+1;
                break;
            }
        }
    }


    // Lookup Y value
    if (this->y_headers[0] >= y_value) { // Check 0th value and below
        y_idx_min = 0;
        y_idx_max = 0;
    } else if (this->y_headers[this->y_size-1] <= y_value) { // Check nth value and above
        y_idx_min = this->y_size-1;
        y_idx_max = this->y_size-1;
    } else {
        // Lookup X value
        for (uint16_t i = 0; i < this->y_size-1; i++) {
            if (y_value == this->y_headers[i]) {
                y_idx_min = i;
                y_idx_max = i;
                break;
            }

            if (this->y_headers[i] <= y_value && this->y_headers[i+1] > y_value) {
                y_idx_min = i;
                y_idx_max = i+1;
                break;
            }
        }
    }

    if (x_idx_max == x_idx_min && y_idx_max == y_idx_min) {
        return data[(y_idx_min*this->x_size)+x_idx_min];
    } else {
        float dx;
        float dy;
        float rx_min;
        float rx_max;
        float ry_min;
        float ry_max;
        if (x_idx_max == x_idx_min) {
            dx = 1.0;
            rx_min = 0.5;
            rx_max = 0.5;
        } else {
            dx = this->x_headers[x_idx_max] - this->x_headers[x_idx_min];
            rx_min = 1.0 - ((x_value-this->x_headers[x_idx_min])/dx);
            rx_max = 1.0-rx_min;
        }
        if (y_idx_max == y_idx_min) {
            dy = 1.0;
            ry_min = 0.5;
            ry_max = 0.5;
        } else {
            dy = this->y_headers[y_idx_max] - this->y_headers[y_idx_min];
            ry_min = 1.0 - ((y_value-this->y_headers[y_idx_min])/dy);
            ry_max = 1.0-ry_min;
        }

        // Now to combine the data
        // First do linear interpolation on X axis for both Y elements

        // For min y
        float x_ymin = (rx_min*data[(y_idx_min*this->x_size)+x_idx_min]) + (rx_max*data[(y_idx_min*this->x_size)+x_idx_max]);
        // For max y
        float x_ymax = (rx_min*data[(y_idx_max*this->x_size)+x_idx_min]) + (rx_max*data[(y_idx_max*this->x_size)+x_idx_max]);

        return (x_ymin*ry_min) + (x_ymax*ry_max);
    }

    return 0;

}