#ifndef __GEARBOX_CONFIG_H_
#define __GEARBOX_CONFIG_H_

#define LARGE_NAG // Uncomment if you have W580

// https://en.wikipedia.org/wiki/Mercedes-Benz_5G-Tronic_transmission
#ifdef LARGE_NAG
    #define RAT_1 3.5876 // 1.64 (1->2 D_RATIO)
    #define RAT_2 2.1862 // 1.55 (2->3 D_RATIO)
    #define RAT_3 1.4054 // 1.41 (3->4 D_RATIO)
    #define RAT_4 1.0000 // 1.21 (4->5 D_RATIO)
    #define RAT_5 0.8314
    #define RAT_R1 -3.1605
    #define RAT_R2 -1.9259
    #define MAX_TORQUE_RATING_NM 580 // Little less than actual max of 580
#else
    #define RAT_1 3.9319 // 1.63 (1->2 D_RATO)
    #define RAT_2 2.4079 // 1.62 (2->3 D_RATO)
    #define RAT_3 1.4857 // 1.49 (3->4 D_RATO)
    #define RAT_4 1.0000 // 1.20 (4->5 D_RATO)
    #define RAT_5 0.8305
    #define RAT_R1 -3.1002
    #define RAT_R2 -1.8986
    #define MAX_TORQUE_RATING_NM 330 // Little less than actual max of 330
#endif

//#define EGS52_MODE // EGS52 CAN layer
#define EGS53_MODE // EGS53 CAN layer

#endif // __GEARBOX_CONFIG_H_