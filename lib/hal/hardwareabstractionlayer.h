#ifndef HARDWAREABSTRACTIONLAYER_H
#define HARDWAREABSTRACTIONLAYER_H

#include "esp_heap_caps.h"

#define MALLOC(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)

#define FREE(ptr) heap_caps_free(ptr)

#endif