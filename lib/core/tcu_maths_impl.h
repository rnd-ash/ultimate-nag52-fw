#include "tcu_maths.h"

#ifndef TCU_MATHS_IMPL_H
#define TCU_MATHS_IMPL_H

template <typename T> void search_value(const T value, const T *values, const uint16_t size, uint16_t *idvalue_min, uint16_t *idvalue_max)
{
	// Set minimum index to the first element of the field.
    *idvalue_min = 0u;
    // Set maximum index to the last element of the field.
    *idvalue_max = size - 1u;
    // Check, if search value is smaller than smallest element of the field.
    if (value > values[0]) {
        if (value < values[*idvalue_max]) {
            // Search value is in between the limits of the smallest and the biggest element of the field.
            do {
                // Calculate the middle of the remaining list. If the size is odd, it is rounded down.
                uint16_t idvalue_mid = (*idvalue_min + *idvalue_max) >> 1;
                if (value < values[idvalue_mid]) {
                    // Search value is smaller than the element in the middle of the remaining list.
                    *idvalue_max = idvalue_mid;
                }
                else if (value > values[idvalue_mid]) {
                    // Search value is bigger than the element in the middle of the remaining list.
                    *idvalue_min = idvalue_mid;
                }
                else {
                    // Search value is also an element of the field.
                    *idvalue_min = idvalue_mid;
                    *idvalue_max = idvalue_mid;
                }
                // Reduce the remaining search area until it is narrowed down to two consecutive elements.
            } while (1u < ((*idvalue_max) - (*idvalue_min)));
        }
        else {
            // Search value is as big as or bigger then the biggest element in the field.
            *idvalue_min = *idvalue_max;
        }
    }
    else {
        // Search value is as small as or smaller than smallest element of the field.
        *idvalue_max = *idvalue_min;
    }
}


#endif