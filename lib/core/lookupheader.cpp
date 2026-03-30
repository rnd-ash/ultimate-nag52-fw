#include "lookupheader.h"
#include "tcu_alloc.h"
#include <string.h>

LookupHeader::LookupHeader(const int16_t *_header, const uint16_t _size)
{
    this->header = const_cast<int16_t*>(_header);
    this->size = _size;
}

LookupHeader::~LookupHeader(void)
{
    delete[] header;
}

int16_t LookupHeader::get_value(const uint16_t index) const
{
    int16_t result = INT16_MAX;
    if(index < size){
        result = header[index];
    }
    return result;
}

uint16_t LookupHeader::get_size(void) const
{
    return size;
}

int16_t * LookupHeader::get_data(void) const
{
    return header;
}

LookupAllocHeader::LookupAllocHeader(const int16_t *_header, const uint16_t _size) : LookupHeader(static_cast<int16_t*>(TCU_HEAP_ALLOC(_size * sizeof(int16_t))), _size)
{
    allocation_successful = (nullptr != header);
    if(allocation_successful){
        (void)memcpy(header, _header, size*sizeof(int16_t));
    }
}

LookupAllocHeader::~LookupAllocHeader(void)
{
    if(allocation_successful){
        TCU_FREE(header);
    }
}

LookupRefHeader::LookupRefHeader(int16_t *_header, const uint16_t _size) : LookupHeader(_header, _size)
{
    // Nothing to do here
}
