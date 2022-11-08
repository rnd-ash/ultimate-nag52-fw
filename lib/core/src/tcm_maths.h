#ifndef TCM_MATHS_H
#define TCM_MATHS_H

#include <stdint.h>

// Core maths and calculation stuff this the TCM uses

#ifndef MAX
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

// float scale_number(int16_t raw, int16_t new_min, int16_t new_max, int16_t raw_min, int16_t raw_max);
float scale_number(float raw, float new_min, float new_max, float raw_min, float raw_max);

class TcmMap {
public:
    /// Creates a 0'ed map
    TcmMap(uint16_t X_Size, uint16_t Y_size, const int16_t* x_ids, const int16_t* y_ids);
    /// MUST call after constructor to ensure that memory was allocated
    /// OK for the map!
    bool allocate_ok(void) const;
    /// Adds a new row to the map
    bool add_data(int16_t* map, uint16_t size);

    int16_t* get_row(uint16_t id);


    float get_value(float x_value, float y_value);

    /**
     * @brief Returns pointer to the current stored map data
     * 
     * @return int16_t* 
     */
    int16_t* get_current_data();
private:
    int16_t* data;
    int16_t* x_headers;
    int16_t* y_headers;
    uint16_t x_size;
    uint16_t y_size;
    bool alloc_ok;
    inline static void set_indices(const int16_t value, uint16_t *idx_min, uint16_t *idx_max, const int16_t *headers, uint16_t size);
};

#endif // TCM_MATHS_H