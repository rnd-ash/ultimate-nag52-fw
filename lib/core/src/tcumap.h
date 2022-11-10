#ifndef TCUMAP_H
#define TCUMAP_H

#include <stdint.h>

class TcuMap {
public:
    /// Creates a 0'ed map
    TcuMap(uint16_t X_Size, uint16_t Y_size, const int16_t* x_ids, const int16_t* y_ids);
    /// MUST call after constructor to ensure that memory was allocated
    /// OK for the map!
    bool allocate_ok(void) const;
    /// Adds a new row to the map
    bool add_data(int16_t* map, uint16_t size);

    int16_t* get_row(uint16_t id);

    /**
     * @brief Search for the value at a given x- and y- value of the underlying 3D-map
     * 
     * @return The interpolated value of the 3D-map.
    */
    float get_value(float x_value, float y_value);

     /**
     * @brief Returns pointer to the current stored map data
     * 
     * @return int16_t* 
     */
    int16_t* get_current_data(void);
    
private:
    int16_t* data;
    int16_t* x_headers;
    int16_t* y_headers;
    uint16_t x_size;
    uint16_t y_size;
    bool alloc_ok;
    
    /**
     * @brief Sets the indices idx_min and idx_max in between the value is found in headers.
    */
    inline static void set_indices(const int16_t value, uint16_t *idx_min, uint16_t *idx_max, const int16_t *headers, uint16_t size);

    /**
     * @brief Calulates interpolated value between given values of function f_1 and f_2 for given value x.
    */
    inline static float interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x);    
};

#endif // TCUMAP_H