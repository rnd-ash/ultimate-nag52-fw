#ifndef __TCU_ALLOC_H__
#define __TCU_ALLOC_H__

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32 == 1
    #include "esp_heap_caps.h"
    #define TCU_HEAP_ALLOC(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
    #define TCU_FREE(ptr) heap_caps_free(ptr)
    #define TCU_IRAM_ALLOC(size) heap_caps_malloc(size, MALLOC_CAP_INTERNAL)
#else
    #include <string.h>
    #define TCU_HEAP_ALLOC(size) malloc(size)
    #define TCU_FREE(ptr) free(ptr)
    #define TCU_IRAM_ALLOC(size) malloc(size)
#endif

#endif