#include "all_keys.h"

#define CREATE_NAME(def, str) const char* NVS_KEY_##def = str;
FOR_LIST(CREATE_NAME)

#define GET_KEY_NAME(def, str) &NVS_KEY_##def,

const char** ALL_NVS_KEYS[] = {
    FOR_LIST(GET_KEY_NAME)
};

int ALL_NVS_KEYS_LEN = sizeof(ALL_NVS_KEYS)/sizeof(const char*);