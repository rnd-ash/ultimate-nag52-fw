#include "can_hfm.h"
#include "driver/twai.h"
#include "driver/i2c_master.h"
// #include "board_config.h"
#include "nvs/eeprom_config.h"
#include "tcu_maths.h"
// #include "ioexpander.h"

HfmCan::HfmCan(const char *name, uint8_t tx_time_ms, Shifter *shifter) : EgsBaseCan(name, tx_time_ms, 125000u, shifter)
{    
    this->start_enable = true;
    can_init_status = ESP_OK;
    // if (ShifterStyle::TRRS == (ShifterStyle)VEHICLE_CONFIG.shifter_style)
    // {   
    //     this->start_enable = true;
    //     can_init_status = ESP_OK;
    // }
    // else
    // {
    //     can_init_status = ESP_ERR_INVALID_ARG;
    //     // Hfm-CAN has 125kbit/s; EWM requires 500kbit/s-CAN
    //     ESP_LOGE("INIT", "ERROR. CAN mode is set to Hfm-CAN (125kbit/s), but shifter is set to EWM (500kbit/s)! Set shifter to TRRS instead!");
    // }
}

uint16_t HfmCan::generateWheelData(const uint32_t expire_time_ms) const
{
    uint16_t result = UINT16_MAX;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        if (!hfm210.VSIG_UP_B)
        {
            if (0u < VEHICLE_CONFIG.wheel_circumference)
            {
                // V_SIGNAL is rather inaccurate, since it considers a fixed wheel circumference, that might deviate from the actually mounted tires
                // convert from km/h to m/min first and divide by circumference, which originally is in mm
                // 1. v_Veh = V_SIGNAL * 1.2 // v_Veh in km/h
                // 2. v_Veh_in_m_per_min = v_Veh * 1000 / 60
                // 3. wheel_circumference_in_m = wheel_circumference / 1000
                // 4. wheel_rpm = v_Veh_in_m_per_min / wheel_circumference_in_m
                // 5. wheel_rpm_double = 2 * wheel_rpm
                // <=> wheel_rpm_double = 2 * (v_Veh_in_m_per_min / (wheel_circumference / 1000))
                // <=> wheel_rpm_double = 2 * ((v_Veh * 1000 / 60) / (wheel_circumference / 1000))
                // <=> wheel_rpm_double = 2 * ((V_SIGNAL * 1.2 * 1000 / 60) / (wheel_circumference / 1000))
                // <=> wheel_rpm_double = 2 * ((V_SIGNAL * 1200 / 60) * (1000 / wheel_circumference))
                // <=> wheel_rpm_double = 2 * ((V_SIGNAL * 20) * (1000 / wheel_circumference))
                // <=> wheel_rpm_double = (2 * 20 * 1000 * V_SIGNAL) / wheel_circumference
                // <=> wheel_rpm_double = (40000 * V_SIGNAL) / wheel_circumference
                result = (40000 * ((int)hfm210.V_SIGNAL)) / (int)VEHICLE_CONFIG.wheel_circumference;
            }
        }
    }
    // TODO: should be read from sensor through GPIO and would look like the following:
    // 1. ticks = 0;
    // 2. gearbox_output_rpm = ticks / ((int)VEHICLE_CONFIG.parking_lock_gear_teeth);
    // 3. wheel_rpm = gearbox_output_rpm / (VEHICLE_CONFIG.diff_ratio / 1000);
    // 4. wheel_rpm_double = 2 * wheel_rpm;
    // <=> wheel_rpm_double = 2 * (gearbox_output_rpm / (VEHICLE_CONFIG.diff_ratio / 1000));
    // <=> wheel_rpm_double = 2 * (1000 * gearbox_output_rpm / VEHICLE_CONFIG.diff_ratio);
    // <=> wheel_rpm_double = (2000 * gearbox_output_rpm) / VEHICLE_CONFIG.diff_ratio;
    // <=> wheel_rpm_double = (2000 * (ticks / ((int)VEHICLE_CONFIG.parking_lock_gear_teeth))) / VEHICLE_CONFIG.diff_ratio;
    // <=> wheel_rpm_double = (2000 * ticks) / (((int)VEHICLE_CONFIG.diff_ratio) * ((int)VEHICLE_CONFIG.parking_lock_gear_teeth));
    return result;
}

uint16_t HfmCan::get_front_right_wheel(const uint32_t expire_time_ms)
{
    return generateWheelData(expire_time_ms);
}

uint16_t HfmCan::get_front_left_wheel(const uint32_t expire_time_ms)
{
    return generateWheelData(expire_time_ms);
}

uint16_t HfmCan::get_rear_right_wheel(const uint32_t expire_time_ms)
{
    return generateWheelData(expire_time_ms);
}

uint16_t HfmCan::get_rear_left_wheel(const uint32_t expire_time_ms)
{
    return generateWheelData(expire_time_ms);
}

EngineType HfmCan::get_engine_type(const uint32_t expire_time_ms)
{
    // M104 & M111 are always petrol; this function can be used to avoid wrong usage of the CAN-layer
    EngineType result = EngineType::Unknown;
    HFM_308 hfm308;
    if (hfm_ecu.get_HFM_308(GET_CLOCK_TIME(), expire_time_ms, &hfm308))
    {
        switch (hfm308.HFM_COD)
        {
        case HFM_308h_HFM_COD::BR202E22:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR124E22MPSV:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR124E22OPSV:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR202E32:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR124E32:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR140E32:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR129E32:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR210E32:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR202E28:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR124E28:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR140E28:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR129E28:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR210E28:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::FFVE32:
            result = EngineType::Petrol;
            break;
        case HFM_308h_HFM_COD::BR140E32FE:
            result = EngineType::Petrol;
            break;
        default:
            break;
        }
    }
    return result;
}

bool HfmCan::get_engine_is_limp(const uint32_t expire_time_ms)
{
    bool result = false;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        result = hfm210.NOTL_B;
    }
    return result;
}

bool HfmCan::get_kickdown(const uint32_t expire_time_ms)
{
    // TODO: check if there is a difference to full throttle and kick down
    bool result = false;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        result |= hfm210.VG_B;
    }
    return result;
}

uint8_t HfmCan::get_pedal_value(const uint32_t expire_time_ms)
{
    uint8_t result = UINT8_MAX;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        if (!hfm210.DKI_UP_B)
        {
            uint8_t dki = hfm210.DKI;
            if (VEHICLE_CONFIG.throttlevalve_maxopeningangle > dki)
            {
                result = (uint8_t)(100.F * (((float)dki) / ((float)VEHICLE_CONFIG.throttlevalve_maxopeningangle)));
            }
        }
    }
    return result;
}

CanTorqueData HfmCan::get_torque_data(const uint32_t expire_time_ms) {
    return TORQUE_NDEF;
}

/*
int HfmCan::get_static_engine_torque(const uint32_t expire_time_ms)
{
    int result = INT_MAX;
    HFM_308 hfm308;
    if (this->hfm_ecu.get_HFM_308(GET_CLOCK_TIME(), expire_time_ms, &hfm308))
    {
        if (hfm308.HFM_UP_B)
        {
            HFM_610 hfm610;
            if (this->hfm_ecu.get_HFM_610(GET_CLOCK_TIME(), expire_time_ms, &hfm610))
            {
                float mle = ((float)(hfm610.MLE)) * air_mass_factor;
                float nmot = ((float)(get_engine_rpm(expire_time_ms)));
                // constant * mass air flow / engine speed
                result = (int)(VEHICLE_CONFIG.c_eng * mle / nmot);
            }
        }
    }
    return result;
}

int HfmCan::get_driver_engine_torque(const uint32_t expire_time_ms)
{
    // init result value
    int result = INT_MAX;
    // check if maximum opening angle of throttle valve is set
    if (0u < VEHICLE_CONFIG.throttlevalve_maxopeningangle)
    {
        HFM_210 hfm210;
        if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
        {
            // check if throttle valve actual value is plausible
            if (!hfm210.DKI_UP_B)
            {
                uint8_t dki = hfm210.DKI;
                uint8_t dkv = 0u;
                // check if throttle valve target value is plausible
                if (!hfm210.DKV_UP_B)
                {
                    dkv = hfm210.DKV;
                }
                // use the higher value to ensure the requested value is collected and convert it to angle in degrees
                uint8_t dk = MAX(dki, dkv);
                // calculate relative openening of the throttle valve
                float rel = (1.F - cosine[dk]) / (1.F - cosine[VEHICLE_CONFIG.throttlevalve_maxopeningangle]);
                // calculate the static engine torque
                float m_stat = (float)(this->get_static_engine_torque(expire_time_ms));
                // calculate the
                result = (int)(rel * m_stat);
            }
        }
    }
    return result;
}

int HfmCan::get_maximum_engine_torque(const uint32_t expire_time_ms)
{
    int result = INT_MAX;
    HFM_308 hfm308;
    if (this->hfm_ecu.get_HFM_308(GET_CLOCK_TIME(), expire_time_ms, &hfm308))
    {
        if (!hfm308.NMOT_UP_B)
        {
            float nmot = ((float)(hfm308.NMOT));
            result = (int)(enginemaxtorque->get_value(nmot));
        }
    }
    return result;
}
*/
int16_t HfmCan::get_engine_coolant_temp(const uint32_t expire_time_ms)
{
    int16_t result = INT16_MAX;
    HFM_608 hfm608;
    if (this->hfm_ecu.get_HFM_608(GET_CLOCK_TIME(), expire_time_ms, &hfm608))
    {
        // ESP_LOGI("Hfm-CAN", "Coolant temperature: %d",hfm608.T_MOT);
        if (!hfm608.TFM_UP_B)
        {
            result = (int16_t)((((float)(hfm608.T_MOT)) * temperature_factor) + temperature_offset);
        }
    }
    return result;
}

int16_t HfmCan::get_engine_oil_temp(const uint32_t expire_time_ms)
{
    // Not available on Hfm-ECUs
    return INT16_MAX;
}

int16_t HfmCan::get_engine_iat_temp(const uint32_t expire_time_ms)
{
    int16_t result = INT16_MAX;
    HFM_608 hfm608;
    if (this->hfm_ecu.get_HFM_608(GET_CLOCK_TIME(), expire_time_ms, &hfm608))
    {
        ESP_LOGI("Hfm-CAN", "Intake air temperature: %d",hfm608.T_LUFT);
        if (!hfm608.TFA_UP_B)
        {
            result = (int16_t)((((float)(hfm608.T_LUFT)) * temperature_factor) + temperature_offset);
        }
    }
    return result;
}

uint16_t HfmCan::get_engine_rpm(const uint32_t expire_time_ms)
{
    uint16_t result = UINT16_MAX;
    HFM_308 hfm308;
    if (this->hfm_ecu.get_HFM_308(GET_CLOCK_TIME(), expire_time_ms, &hfm308))
    {
        if (!hfm308.NMOT_UP_B)
        {
            result = hfm308.NMOT;
        }
    }
    return result;
}

bool HfmCan::get_is_starting(const uint32_t expire_time_ms)
{
    bool result = false;
    HFM_308 hfm308;
    if (this->hfm_ecu.get_HFM_308(GET_CLOCK_TIME(), expire_time_ms, &hfm308))
    {
        result = hfm308.KL50_B;
    }
    return result;
}

bool HfmCan::get_is_brake_pressed(const uint32_t expire_time_ms)
{
    bool result = false;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        result = hfm210.BLS_B;
    }
    return result;
}

void HfmCan::on_rx_frame(uint32_t id, uint8_t dlc, uint64_t data, const uint32_t timestamp)
{
    this->hfm_ecu.import_frames(data, id, timestamp);
}