/*
 * SPDX-FileCopyrightText: 2017-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "esp_efuse.h"
#include <assert.h>
#include "efuse.h"

// md5_digest_table d6c1172944e02bf3f301a89c299ce6c7
// This file was generated from the file efuse.csv. DO NOT CHANGE THIS FILE MANUALLY.
// If you want to change some fields, you need to change efuse.csv file
// then run `efuse_common_table` or `efuse_custom_table` command it will generate this file.
// To show efuse_table run the command 'show_efuse_table'.

#define MAX_BLK_LEN CONFIG_EFUSE_MAX_BLK_LEN

// The last free bit in the block is counted over the entire file.
#define LAST_FREE_BIT_BLK3 40

_Static_assert(LAST_FREE_BIT_BLK3 <= MAX_BLK_LEN, "The eFuse table does not match the coding scheme. Edit the table and restart the efuse_common_table or efuse_custom_table command to regenerate the new files.");

static const esp_efuse_desc_t BOARD_VER[] = {
    {EFUSE_BLK3, 0, 8}, 	 // PCB version,
};

static const esp_efuse_desc_t M_DAY[] = {
    {EFUSE_BLK3, 8, 8}, 	 // Manufacture day of month,
};

static const esp_efuse_desc_t M_WEEK[] = {
    {EFUSE_BLK3, 16, 8}, 	 // Manufacture week of year,
};

static const esp_efuse_desc_t M_MONTH[] = {
    {EFUSE_BLK3, 24, 8}, 	 // Manufacture month of year,
};

static const esp_efuse_desc_t M_YEAR[] = {
    {EFUSE_BLK3, 32, 8}, 	 // Manufacture year,
};





const esp_efuse_desc_t* ESP_EFUSE_BOARD_VER[] = {
    &BOARD_VER[0],    		// PCB version
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_M_DAY[] = {
    &M_DAY[0],    		// Manufacture day of month
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_M_WEEK[] = {
    &M_WEEK[0],    		// Manufacture week of year
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_M_MONTH[] = {
    &M_MONTH[0],    		// Manufacture month of year
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_M_YEAR[] = {
    &M_YEAR[0],    		// Manufacture year
    NULL
};
