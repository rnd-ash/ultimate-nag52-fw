#ifndef GEARBOX_CONFIG_H
#define GEARBOX_CONFIG_H

#define GB_EFFICIENCY 0.9 // 90% *TODO* - Does this changed based on GB physical vairent?

// https://en.wikipedia.org/wiki/Mercedes-Benz_5G-Tronic_transmission
#define RAT_1_LARGE 3.595
#define RAT_2_LARGE 2.186
#define RAT_3_LARGE 1.405
#define RAT_4_LARGE 1.000
#define RAT_5_LARGE 0.831
#define RAT_R1_LARGE -3.167
#define RAT_R2_LARGE -1.926
#define MAX_TORQUE_RATING_NM_LARGE 580

#define RAT_1_SMALL 3.951
#define RAT_2_SMALL 2.423
#define RAT_3_SMALL 1.486
#define RAT_4_SMALL 1.000
#define RAT_5_SMALL 0.833
#define RAT_R1_SMALL -3.147
#define RAT_R2_SMALL -1.930
#define MAX_TORQUE_RATING_NM_SMALL 330

#endif // GEARBOX_CONFIG_H