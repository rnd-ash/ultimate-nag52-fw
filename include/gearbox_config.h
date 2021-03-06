#ifndef __GEARBOX_CONFIG_H_
#define __GEARBOX_CONFIG_H_


// https://en.wikipedia.org/wiki/Mercedes-Benz_5G-Tronic_transmission
    #define RAT_1_LARGE 3.5876 // 1.64 (1->2 D_RATIO)
    #define RAT_2_LARGE 2.1862 // 1.55 (2->3 D_RATIO)
    #define RAT_3_LARGE 1.4054 // 1.41 (3->4 D_RATIO)
    #define RAT_4_LARGE 1.0000 // 1.21 (4->5 D_RATIO)
    #define RAT_5_LARGE 0.8314
    #define RAT_R1_LARGE -3.1605
    #define RAT_R2_LARGE -1.9259
    #define MAX_TORQUE_RATING_NM_LARGE 580

    #define RAT_1_SMALL 3.9319 // 1.63 (1->2 D_RATO)
    #define RAT_2_SMALL 2.4079 // 1.62 (2->3 D_RATO)
    #define RAT_3_SMALL 1.4857 // 1.49 (3->4 D_RATO)
    #define RAT_4_SMALL 1.0000 // 1.20 (4->5 D_RATO)
    #define RAT_5_SMALL 0.8305
    #define RAT_R1_SMALL -3.1002
    #define RAT_R2_SMALL -1.8986
    #define MAX_TORQUE_RATING_NM_SMALL 330

#define EGS52_MODE // EGS52 CAN layer
//#define EGS53_MODE // EGS53 CAN layer

#endif // __GEARBOX_CONFIG_H_