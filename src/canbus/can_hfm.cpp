#include "can_hfm.h"
#include "driver/twai.h"
#include "driver/i2c_master.h"
#include "nvs/eeprom_config.h"
#include "tcu_maths.h"
#include <tcu_maths_impl.h>
#include "maps.h"
#include "engines/hfm_engine.hpp"

HfmCan::HfmCan(const char *name, uint8_t tx_time_ms) : EgsBaseCan(name, tx_time_ms, 125000u)
{
    ESP_LOGI(name, "SETUP CALLED");
    hfm_engine = new HfmEngine();

    this->start_enable = true;
    can_init_status = ESP_OK;
}

uint16_t HfmCan::generateWheelData(const uint32_t expire_time_ms) const
{
    uint16_t result = 0;
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
                result = (uint16_t)((40000.F * ((float)hfm210.V_SIGNAL)) / (float)VEHICLE_CONFIG.wheel_circumference);
            }
        }
    }
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
    bool result = false;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        // validity check for DKV
        if(!hfm210.DKV_UP_B){
            result = (hfm210.DKV == VEHICLE_CONFIG.throttlevalve_maxopeningangle);
        }
        result |= hfm210.VG_B;
    }
    return result;
}

uint8_t HfmCan::get_pedal_value(const uint32_t expire_time_ms)
{
    uint8_t result = UINT8_MAX;
    float dkv = 0.F;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        if(!hfm210.DKV_UP_B){
            dkv = (float)(hfm210.DKV);
            dkv *= (float)PEDAL_VALUE_LIMIT;
            // scale value to max throttle opening angle, so that it can be used for interpolation in the engine class
            dkv /= (float)VEHICLE_CONFIG.throttlevalve_maxopeningangle;
        }
    }
    result = (uint8_t)dkv;
    return result;
}

CanTorqueData HfmCan::get_torque_data(const uint32_t expire_time_ms)
{
    // higher expiration time for frames above 0x600, since they are not changing that fast
    const uint32_t expire_time_hfm_can = 250u; // [ms]

    // calculate torque values
    CanTorqueData result = TORQUE_NDEF;

    // obtain values from the CAN-bus to calculate the indicated and the driver torque
    uint16_t n_mot = get_engine_rpm(expire_time_ms); // todo check for uint16max
    float mle = 0.F;
    uint8_t dki = UINT8_MAX;
    uint8_t dkv = UINT8_MAX;
    
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm210))
    {
        if(!hfm210.DKI_UP_B){
            dki = hfm210.DKI;
        }
        if(!hfm210.DKV_UP_B){
            dkv = hfm210.DKV;
        }

        HFM_610 hfm610;
        if (this->hfm_ecu.get_HFM_610(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm610))
        {
            // conversion from raw value to [kg/h]
            mle = ((float)(hfm610.MLE)) * AIR_MASS_FACTOR;
            // add measured mass air flow to map for current engine speed and throttle position, so that it can be used for interpolation later on
            hfm_engine->add_to_mass_air_flow_map(n_mot, dki, mle);
        }
    }

    // minimum torque
    result.m_min = 0; // TODO: find a way to get this value

    // maximum torque
    result.m_max = hfm_engine->get_max_torque(n_mot); // interpolate from torque map based on engine speed

    // indicated torque
    // ratio of current mass air flow to maximum mass air flow at current engine speed, multiplied by maximum torque at current engine speed
    result.m_ind = INT16_MAX; // default value in case of invalid data
    result.m_converted_driver = INT16_MAX; // default value in case of invalid data
    float max_mass_air_flow = hfm_engine->get_max_mass_air_flow(n_mot);
    if(0.F < max_mass_air_flow)
    {
        result.m_ind = (int16_t)(MIN(1.F, mle / max_mass_air_flow) * ((float)(result.m_max)));
        
        // driver torque - comes from the accelerator pedal or cruise control
        if (dkv < UINT8_MAX)
        {
            result.m_converted_driver = (int16_t)((hfm_engine->get_mass_air_flow(n_mot, dkv) / max_mass_air_flow) * ((float)(result.m_max)));
            ESP_LOGI("HFM-CAN", "DKV: %u, max_throttle_value: %u, m_max: %u, driver torque (before): %u Nm", dkv, VEHICLE_CONFIG.throttlevalve_maxopeningangle, result.m_max, result.m_converted_driver);
        }
    }    

    // static torque
    if ((0 < n_mot) && (UINT16_MAX != n_mot))
    {
        // result.m_converted_static = (int16_t)(c_engine * (mle / 3600.F) / n_mot_SI); // constant * mass air flow / engine speed
    }
   
    // ESP_LOGI("HFM-CAN", "Indicated: %u Nm, Static: %u Nm, driver torque: %u Nm (before)", result.m_ind, result.m_converted_static, result.m_converted_driver);
    if (INT16_MAX != result.m_converted_static && INT16_MAX != result.m_converted_driver)
    {
        int16_t static_converted = result.m_converted_static;
        int16_t tmp = result.m_converted_driver;
        int16_t driver_converted = static_converted;
        int16_t indicated = 0;
        // Calculate converted torque from ESP
        if (INT16_MAX != result.m_max && INT16_MAX != result.m_min)
        {
            tmp = MAX(MIN(result.m_converted_driver, result.m_max), result.m_min);
        }
        if (tmp <= 0)
        {
            tmp = MIN(tmp, static_converted);
        }
        driver_converted = tmp;

        // Check if freezing torque should be done
        bool active_shift = actual_gear != target_gear;
        bool trq_req_en = MMAX_EGS;
        if (active_shift && trq_req_en)
        {
            this->freeze_torque = true; // Gear shift and we have started a torque request, freeze it
        }
        else if (!active_shift)
        {
            this->freeze_torque = false; // No gear shift, unfreeze it
        }
        // Change torque values based on freezing or not
        if (this->freeze_torque)
        {
            driver_converted = MAX(driver_converted - this->req_static_torque_delta, static_converted);
        }
        else
        {
            this->req_static_torque_delta = driver_converted - static_converted;
        }
        if (driver_converted > 0)
        {
            indicated = driver_converted;
        }

        HFM_628 hfm628;
        if (hfm_ecu.get_HFM_628(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm628))
        {
            if (hfm628.KLIMA_B)
            {
                // Typical drag torque for an air conditioning compressor in a combustion engine car
                // is in the range of 10-30 Nm depending on compressor type and operating conditions.
                const int16_t M_KOMP = 20;

                driver_converted -= M_KOMP;
                static_converted -= M_KOMP;
            }
        }

        // result.m_ind = indicated;
        result.m_converted_driver = driver_converted;
        result.m_converted_static = static_converted;
    }

    return result;
}

float HfmCan::get_ML(const uint32_t expire_time_ms)
{
    const uint32_t expire_time_hfm_can = 600u; // [ms]
    float result = 0.F;
    float mle = 0.F;
    int16_t air_pressure = 0;
    HFM_608 hfm608;
    if (this->hfm_ecu.get_HFM_608(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm608))
    {
        if(!hfm608.HOH_UP_B)
        {
            air_pressure = ((int16_t)(hfm608.P_HOH) * ALTITUDE_PRESSURE_FACTOR); 
        }
    }
    HFM_610 hfm610;
    if (this->hfm_ecu.get_HFM_610(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm610))
    {
        mle = ((float)(hfm610.MLE)) * AIR_MASS_FACTOR;
    }
    hfm_engine->get_ML(mle, this->get_engine_iat_temp(expire_time_ms), air_pressure);
    return result;
}

int16_t HfmCan::get_engine_coolant_temp(const uint32_t expire_time_ms)
{
    const uint32_t expire_time_hfm_can = 125u; // [ms]
    int16_t result = INT16_MAX;
    HFM_608 hfm608;
    if (this->hfm_ecu.get_HFM_608(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm608))
    {
        if (!hfm608.TFM_UP_B)
        {
            result = COOLANT_TEMPERATURE[hfm608.T_MOT];
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
    const uint32_t expire_time_hfm_can = 125u; // [ms]
    int16_t result = INT16_MAX;
    HFM_608 hfm608;
    if (this->hfm_ecu.get_HFM_608(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm608))
    {
        result = INTAKE_AIR_TEMPERATURE[hfm608.T_LUFT];
    }
    return result;
}

uint16_t HfmCan::get_engine_rpm(const uint32_t expire_time_ms)
{
    uint16_t result = UINT16_MAX;
    HFM_308 hfm308;
    if (this->hfm_ecu.get_HFM_308(GET_CLOCK_TIME(), expire_time_ms, &hfm308))
    {
        result = hfm308.NMOT;
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

void HfmCan::set_actual_gear(GearboxGear actual)
{
    actual_gear = actual;
}

void HfmCan::set_target_gear(GearboxGear target)
{
    target_gear = target;
}

void HfmCan::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm)
{
    MMAX_EGS = TorqueRequestBounds::LessThan != limit_type;
}

void HfmCan::on_rx_frame(uint32_t id, uint8_t dlc, uint64_t data, const uint32_t timestamp)
{
    this->hfm_ecu.import_frames(data, id, timestamp);
}

void HfmCan::set_drive_profile(GearboxProfile p)
{
}