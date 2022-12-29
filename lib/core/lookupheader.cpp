#include "lookupheader.h"
#include "../hal/hardwareabstractionlayer.h"
#include <string.h>

LookupHeader::LookupHeader(const int16_t *_header, const uint16_t _size)
{
    size = _size;
    header = static_cast<int16_t*>(MALLOC(size * sizeof(int16_t)));
    allocation_successful = (nullptr != header);
    if(allocation_successful){
        (void)memcpy(header, _header, size);
    }
}

LookupHeader::~LookupHeader(void)
{
    if(allocation_successful){
        FREE(header);
    }
}

void LookupHeader::setIndices(const float value, uint16_t *idx_min, uint16_t *idx_max)
{
    // Set minimum index to the first element of the field.
    *idx_min = 0u;
    // Set maximum index to the last element of the field.
    *idx_max = size - 1u;
    // Check, if search value is smaller than smallest element of the field.
    if (value > (float)header[0]) {
        if (value < (float)header[*idx_max]) {
            // Search value is in between the limits of the smallest and the biggest element of the field.
            do {
                // Calculate the middle of the remaining list. If the size is odd, it is rounded down.
                uint16_t idx_mid = (*idx_min + *idx_max) >> 1;
                if (value < (float)header[idx_mid]) {
                    // Search value is smaller than the element in the middle of the remaining list.
                    *idx_max = idx_mid;
                }
                else if (value > (float)header[idx_mid]) {
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

int16_t LookupHeader::getValue(const uint16_t index)
{
    int16_t result = INT16_MAX;
    if(index <= size){
        result = header[index];
    }
    return result;
}

uint16_t LookupHeader::getSize(void)
{
    return size;
}
