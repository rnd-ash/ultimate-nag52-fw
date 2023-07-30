#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include "lookupheader.h"

using namespace std;

class LookupTable{
    public:
        LookupTable(const int16_t* _xHeader, const uint16_t _xHeaderSize);
        LookupTable(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _data, const uint16_t _dataSize);
        ~LookupTable(void);

        bool set_data(int16_t* _data, uint16_t _dataSize);
        bool is_allocated(void);
        float get_value(float xValue);
        /// @brief This functions generates a corresponding header-value based on the parameter. This function does only work on tables with increasing x-values.
        /// @param xValue the value to be looked up
        /// @return the interpolated header-value
        float get_header_interpolated(const float value);
        int16_t* get_current_data(void);
        void get_x_headers(uint16_t *size, int16_t **headers);
        bool add_data(const int16_t* map, const uint16_t size);

    protected:
        LookupHeader* xHeader;
        int16_t* data;
        uint16_t dataSize;

    private:
        bool allocation_successful;
        uint16_t xHeaderSize;

};

#endif /* LOOKUPTABLE_H */