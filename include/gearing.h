#ifndef GEARING_H_
#define GEARING_H_

#include <config.h>


#if LARGE_NAG == true

/** Gear ratios for W5A580 (Large 722.6) **/

#define GEAR_D1 3.5876
#define GEAR_D2 2.1862
#define GEAR_D3 1.4054
#define GEAR_D4 1.0000
#define GEAR_D5 0.8314
#define GEAR_R1 -3.1605
#define GEAR_R2 -1.9259

#else

/** Gear ratios for W5A330 (Small 722.6) **/

#define GEAR_D1 3.9319
#define GEAR_D2 2.4079
#define GEAR_D3 1.4857
#define GEAR_D4 1.0000
#define GEAR_D5 0.8305
#define GEAR_R1 -3.1002
#define GEAR_R2 -1.8986

#endif


 #endif // GEARING_H_