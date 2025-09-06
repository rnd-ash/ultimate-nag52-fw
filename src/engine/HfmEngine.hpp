#ifndef HFMENGINE_HPP
#define HFMENGINE_HPP

#include <stdint.h>

class HfmEngine
{
private:
    /* number of cylinders [-] */
    uint8_t n_cylinders;

    /* engine displacement [ccm] */
    uint16_t V_H;

    /* air density [kg/m³]*/
    float current_air_pressure = 0.0f;
    /* intakte air temperature [K] */
    float current_intake_air_temp = 0.0f;

public:
    HfmEngine(/* args */);
    ~HfmEngine();
    
    /* required for KM6 (1997-1) HFM: 210h */

    /* returns true if air condition is on */
    bool get_KLIMA(void);

    /* returns true if electric engine management demands an active down shifting */
    bool get_AKT_R(void);

    /* returns true if electric engine management demands driveaway in 2nd gear */
    bool get_ANF2_EMS(void);

    /* returns true if electric engine management demands keeping the gear */
    bool get_GHALT_EMS(void);

    /* returns true if limp mode is activated */
    bool get_NOTL(void);

    /* returns true if cruise control is active */
    bool get_TM_REG(void);

    /* required for KM6 (1997-1) HFM: 308h */

    /* returns true if gearbox protection request  is acknowledged */
    bool get_GSQ(void);
    
    /* returns true if warmup of engine is active */
    bool get_WLAK(void);

    /* returns true if engine speed is limited*/
    bool get_N_MAX_BG(void);

    /* returns true if opening of torque converter lockup clutch is requested */
    bool get_KUEB_O_A(void);

    /* returns engine speed [rpm] */
    uint16_t get_NMOT(void);

    /* returns the index switching curve adjustment (0-10) */
    uint8_t get_SLV(void);

    /* required for KM6 (1997-1) HFM: 310h */

    /* returns mass volumetric efficiency [%] */
    float get_ML(void);

    /* returns factor for degradation of maximum engine torque dependent on decreasing atmospheric pressure [-] */
    float get_FMMOTMAX(void);

    /* returns the indicated engine torque [Nm] */
    uint16_t get_MMOTI(void);

    /* returns the engine drag torque [Nm]*/
    uint16_t get_MMOTSCH(void);
};

#endif