#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include "lookupheader.h"

using namespace std;

class LookupTable {
    public:
        // Virtual: LookupMap holds a LookupTable* that may point to a
        // LookupAllocTable, and deletes through that base pointer. Without this,
        // the derived destructor never runs and the data allocation leaks.
        virtual ~LookupTable(void);
        float get_value(float xValue);
        /// @brief This functions generates a corresponding header-value based on the parameter. This function does only work on tables with increasing x-values.
        /// @param xValue the value to be looked up
        /// @return the interpolated header-value
        float get_header_interpolated(const float value) const;
        void get_x_headers(uint16_t *size, int16_t **headers) const;
        int16_t* get_current_data(void);
        const LookupHeader* get_header(void) const;
        uint16_t data_size(void) const;
    protected:
        uint16_t xHeaderSize = 0u;
        uint16_t dataSize = 0u;
        int16_t* data = nullptr;
        LookupHeader* xHeader = nullptr;
};

class LookupAllocTable: public LookupTable {
    public:
        LookupAllocTable(const int16_t* _xHeader, const uint16_t _xHeaderSize);
        LookupAllocTable(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _data, const uint16_t _dataSize);
        ~LookupAllocTable(void) override;
        bool set_data(const int16_t* _data, uint16_t _dataSize);
        bool is_allocated(void) const;
        bool add_data(const int16_t* map, const uint16_t size);
    private:
        bool allocation_successful = false;
};

class LookupRefTable: public LookupTable {
    public:
        LookupRefTable(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _data, uint16_t _dataSize);
};

#endif /* LOOKUPTABLE_H */