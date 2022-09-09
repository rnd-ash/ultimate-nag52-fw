#ifndef __GEARBOX_CONFIG_H_
#define __GEARBOX_CONFIG_H_


#define BOARD_V2 // Comment this line for the red beta boards!!!


#define GB_EFFICIENCY 0.9 // 90% *TODO* - Does this changed based on GB physical vairent?

// https://en.wikipedia.org/wiki/Mercedes-Benz_5G-Tronic_transmission
    #define RAT_1_LARGE 3.5876 // 1.64 (1->2 D_RATIO)
    #define RAT_2_LARGE 2.1862 // 1.55 (2->3 D_RATIO)
    #define RAT_3_LARGE 1.4054 // 1.41 (3->4 D_RATIO)
    #define RAT_4_LARGE 1.0000 // 1.21 (4->5 D_RATIO)
    #define RAT_5_LARGE 0.8314
    #define RAT_R1_LARGE -3.1605
    #define RAT_R2_LARGE -1.9259
    #define MAX_TORQUE_RATING_NM_LARGE 580

    #define RAT_1_SMALL 3.951 // 1.63 (1->2 D_RATO)
    #define RAT_2_SMALL 2.423 // 1.62 (2->3 D_RATO)
    #define RAT_3_SMALL 1.486 // 1.49 (3->4 D_RATO)
    #define RAT_4_SMALL 1.000 // 1.20 (4->5 D_RATO)
    #define RAT_5_SMALL 0.833
    #define RAT_R1_SMALL -3.147
    #define RAT_R2_SMALL -1.930
    #define MAX_TORQUE_RATING_NM_SMALL 330

// The old EGS51, EGS52 and EGS53 MODE FLAGS ARE NOW MOVED TO PLATFORMIO.INI!

#endif // __GEARBOX_CONFIG_H_