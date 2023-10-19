#include "maps.h"

const uint8_t STRONGEST_LOADED_CLUTCH_SMALL_NAG[8] = {3, 4, 0, 1, 1, 1, 5, 5};
const uint8_t STRONGEST_LOADED_CLUTCH_LARGE_NAG[8] = {3, 2, 2, 1, 1, 1, 2, 2};

const int16_t CLUTCH_FRICTION_MAP[NUM_MECHANICAL_CALIBRATIONS][9*6] = {
    {
/*                    SET 0 (SMALL NAG)  */
/*      K1    K2    K3    B1    B2    B3 */
        7003, 0   , 0   , 4765, 0   , 0   , // N
        0,    0   , 4331, 3004, 3194, 0   , // D1 337Nm or 425Nm
        2708, 0   , 2656, 0   , 1959, 0   , // D2 398Nm or 501Nm
        0,    1652, 0   , 0   , 1329, 0   , // D3 652Nm or 822Nm
        1403, 2507, 1376, 0   , 0   , 0   , // D4 429Nm or 541Nm
        0,    2089, 1146, 795 , 0   , 0   , // D5 516Nm or 650Nm
        0,    0   , 4331, 3004, 0   , 4204, // R1 256Nm or 323Nm
        2708, 0   , 2656, 0   , 0   , 2579  // R2 417Nm or 501Nm
    },
    {
/*  SET 1 (SMALL NAG - Weaker B2 and K1) */
/*      K1    K2    K3    B1    B2    B3 */
        9338, 0   , 0   , 7147, 0   , 0   , // N
        0,    0   , 5774, 4506, 4259, 0   , // D1 186Nm or 235Nm
        3611, 0   , 3542, 0   , 2612, 0   , // D2 298Nm or 376Nm
        0,    2202, 0   , 0   , 1772, 0   , // D3 489Nm or 616Nm
        1870, 3343, 1835, 0   , 0   , 0   , // D4 322Nm or 406Nm
        0,    2785, 1529, 1193, 0   , 0   , // D5 387Nm or 487Nm
        0,    0   , 5774, 4506, 0   , 5606, // R1 186Nm or 235Nm
        3611, 0   , 3542, 0   , 0   , 3438  // R2 298Nm or 376Nm
    },
    {
/*              SET 2 (Large NAG)        */
/*      K1    K2    K3    B1    B2    B3 */
        5639, 0   , 0   , 4765, 0   , 0   , // N
        0,    0   , 5774, 3004, 2679, 0   , // D1 279Nm or 352Nm
        2203, 0   , 3542, 0   , 1633, 0   , // D2 461Nm or 580Nm
        0,    1321, 0   , 0   , 1109, 0   , // D3 816Nm or 1028Nm
        1144, 2007, 1835, 0   , 0   , 0   , // D4 537Nm or 676Nm
        0,    1669, 1529, 804 , 0   , 0   , // D5 645Nm or 813Nm
        0,    0   , 5774, 3054, 0   , 3380, // R1 280Nm or 353Nm
        2203, 0   , 3542, 0   , 0   , 2060  // R2 461Nm or 580Nm
    },
    {
/*                    SET 3              */
/*      K1    K2    K3    B1    B2    B3 */
        5651, 0   , 0   , 3574, 0   , 0   , // N
        0,    0   , 3845, 2303, 2685, 0   , // D1 280Nm or 353 NM
        2214, 0   , 2338, 0   , 1633, 0   , // D2 461Nm or 580 NM
        0,    1321, 0   , 0   , 1109, 0   , // D3 816Nm or 1028 NM
        1150, 2007, 1214, 0   , 0   , 0   , // D4 537Nm or 676 NM
        0,    1668, 1009,  604, 0   , 0   , // D5 646Nm or 814 NM
        0,    0   , 3845, 2303, 0   , 3387, // R1 280Nm or 353 NM
        2214, 0   , 2338, 0   , 0   , 2060  // R2 461Nm or 580 NM
    },
    {
/*                    SET 4 (700NM+)     */
/*      K1    K2    K3    B1    B2    B3 */
        4709, 0   , 0   , 3574, 0   , 0   , // N
        0,    0   , 3076, 2303, 2685, 0   , // D1 350Nm or 441Nm
        1845, 0   , 1871, 0   , 1633, 0   , // D2 576Nm or 725Nm
        0,    1101, 0   , 0   , 1109, 0   , // D3 979Nm or 1233Nm
         958, 1673,  971, 0   , 0   , 0   , // D4 644Nm or 811Nm
        0,    1390,  807,  604, 0   , 0   , // D5 775Nm or 976Nm
        0,    0   , 3076, 2303, 0   , 3387, // R1 350Nm or 441Nm
        1845, 0   , 1871, 0   , 0   , 2060  // R2 576Nm or 725Nm
    }
};

const int16_t CLUTCH_RELEASE_SPRING_MAP[NUM_SPRING_CALIBRATIONS][6] = {
//   K1    K2    K3    B1    B2    B3
    {1270, 846 , 1205, 1139, 1289, 448 }, // Set 0
    {1270, 846 , 1636, 1139, 1289, 448 }, // Set 1
};



const int16_t HYDRALIC_PCS_MAP[NUM_HYDRALIC_CALIBRATIONS][8*4] = {
    {
        /*      mBar                                    */
        /* 0    50    600  1000  2350  5600  6600  7700 */
        1300, 1100, 1085,  954,  700,  450,  350, 200, // -25C
        1277, 1077,  925,  830,  675,  415,  320,   0, //  20C
        1200, 1000,  835,  780,  650,  400,  288,   0, //  60C
        1175,  975,  795,  745,  625,  370,  260,   0  // 150C
    },
    {
        /*      mBar                                    */
        /* 0   100  1300  1800  3250  7100  8200  9700 */
        1255, 1155,  945,  880,  710,  410,  320, 150, // -25C
        1155, 1055,  860,  810,  685,  395,  300,   0, //  20C
        1010,  990,  805,  765,  660,  385,  275,   0, //  60C
        1075,  965,  770,  730,  620,  345,  235,   0  // 150C
    }
};

const int16_t HYDRALIC_PCS_MAP_X_HEADER[NUM_HYDRALIC_CALIBRATIONS][8] = {
    {0,  50,  600, 1000, 2350, 4500, 6600, 7700},
    {0, 100, 1300, 1800, 3250, 7100, 8200, 9700}
};