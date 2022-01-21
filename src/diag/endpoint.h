#ifndef DIAG_ENDPOINT_H__
#define DIAG_ENDPOINT_H__

#include <stdint.h>

typedef struct DiagMessage {
    uint16_t data_size;
    uint8_t data[1024]; // 1KB messages max (EGS52/3 is always max 256 bytes)
};

/**
 * @brief Abstract endpoint
 * 
 */
class AbstractEndpoint {
    
};

/**
 * @brief Endpoint for USB communication with ultimate-nag52 custom USB util
 * 
 */
class UsbEndpoint: public AbstractEndpoint {

};

/**
 * @brief Endpoint for ISO-TP communication with OBD readers
 * 
 */
class CanEndpoint: public AbstractEndpoint {

};


#endif