#include "lookupmap.h"
#include "tcu_maths_impl.h"
#include "tcu_alloc.h"
#include "clock.h"
#include <string.h>

LookupMap::~LookupMap(void)
{
    delete this->table;
    this->table = nullptr;
    delete this->yHeader;
    this->yHeader = nullptr;
}

float LookupMap::get_value(const float xValue, const float yValue) {
    return this->get_value(xValue, yValue, 0);
}

float LookupMap::get_value(const float xValue, const float yValue, const uint8_t lookup_cache_idx)
{
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;
    const LookupHeader* xHeader = this->table->get_header();
    const int16_t* data = this->table->get_current_data();

    if (lookup_cache_idx < MAX_LOOKUP_CACHE) {
        this->lookup_cache[lookup_cache_idx].x_val = xValue;
        this->lookup_cache[lookup_cache_idx].y_val = yValue;
        this->lookup_cache[lookup_cache_idx].timestamp_ms = GET_CLOCK_TIME();
    }

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

void LookupMap::get_y_headers(uint16_t *size, int16_t **headers) const {
    *size = yHeaderSize;
    *headers = yHeader->get_data();
}

int16_t* LookupMap::get_current_data(void) const {
    return this->table->get_current_data();
}

void LookupMap::get_x_headers(uint16_t *size, int16_t **headers) const {
    return table->get_x_headers(size, headers);
}

uint16_t LookupMap::data_size() const {
    return this->table->data_size();
}

void LookupMap::copy_lookup_cache(LookupCache* dest) const {
    if (dest == nullptr) {
        return;
    }
    memcpy(dest, this->lookup_cache, sizeof(this->lookup_cache));
}

float LookupMap::get_x_header_interpolated(const float value, const int16_t y) const
{
    const LookupHeader* xHeader = this->table->get_header();
    const int16_t* data = this->table->get_current_data();
    if (nullptr == xHeader || nullptr == data || 0u == xHeader->get_size()) {
        return 0.0f;
    }
    const uint16_t x_size = xHeader->get_size();

    // Isolate the row for y. Data is laid out row major as
    // data[(y_idx * x_size) + x_idx] - the same indexing get_value() uses - so
    // a row is contiguous and can be pointed at directly.
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;
    search_value<int16_t>(y, yHeader->get_data(), yHeader->get_size(), &y_idx_min, &y_idx_max);
    const int16_t* row = &data[y_idx_min * x_size];

    uint16_t    idvalue_min;
    uint16_t    idvalue_max;

    // part 1 - identification of the indices for x-value
    search_value<int16_t>((int16_t)value, row, x_size, &idvalue_min, &idvalue_max);

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
    // table and yHeader are released by ~LookupMap
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

LookupByteMap::LookupByteMap(const uint8_t* _xHeader, const uint16_t _xHeaderSize, const uint8_t* _yHeader, const uint16_t _yHeaderSize, const uint8_t* _data, const uint16_t _dataSize)
    : x_alloc(nullptr), y_alloc(nullptr), z_alloc(nullptr), z_size(_dataSize) {
    this->x_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_xHeaderSize * sizeof(int16_t)));
    this->y_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_yHeaderSize * sizeof(int16_t)));
    this->z_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_dataSize * sizeof(int16_t)));

    if (nullptr == this->x_alloc || nullptr == this->y_alloc || nullptr == this->z_alloc) {
        return;
    }

    for (auto i = 0; i < _xHeaderSize; i++) {
        this->x_alloc[i] = _xHeader[i];
    }
    for (auto i = 0; i < _yHeaderSize; i++) {
        this->y_alloc[i] = _yHeader[i];
    }
    for (auto i = 0; i < _dataSize; i++) {
        this->z_alloc[i] = _data[i];
    }
    this->table = new LookupRefTable(x_alloc, _xHeaderSize, z_alloc, _dataSize);
    this->yHeader = new LookupRefHeader(y_alloc, _yHeaderSize);
    this->yHeaderSize = _yHeaderSize;
}

bool LookupByteMap::is_allocated(void) const {
    return nullptr != this->x_alloc && nullptr != this->y_alloc && nullptr != this->z_alloc;
}

bool LookupByteMap::add_data(const uint8_t* map, const uint16_t size) {
    if (nullptr == map || nullptr == this->z_alloc || size != this->z_size) {
        return false;
    } else {
        for (auto i = 0; i < size; i++) {
            this->z_alloc[i] = map[i];
        }
        return true;
    }
}

LookupByteMap::~LookupByteMap() {
    // table and yHeader are released by ~LookupMap. They are Ref types that only
    // point into the buffers below, so the order of release does not matter.
    TCU_FREE(this->x_alloc);
    this->x_alloc = nullptr;
    TCU_FREE(this->y_alloc);
    this->y_alloc = nullptr;
    TCU_FREE(this->z_alloc);
    this->z_alloc = nullptr;
}
