#include "can_hfm.h"
#include "driver/twai.h"
#include "driver/i2c_master.h"
#include "nvs/eeprom_config.h"
#include "tcu_maths.h"
#include <tcu_maths_impl.h>
#include "maps.h"
#include "engines/hfm_engine.hpp"
#include <cmath>

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
        if (!hfm210.DKV_UP_B)
        {
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
        if (!hfm210.DKV_UP_B)
        {
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
    CanTorqueData result = TORQUE_NDEF; // default value in case of invalid data

    // obtain values from the CAN-bus to calculate the indicated and the driver torque
    uint16_t n_mot = get_engine_rpm(expire_time_ms); // todo check for uint16max

    if (UINT16_MAX != n_mot)
    {
        float mle = 0.F;
        float max_mass_air_flow = 0.F;

        uint8_t dki = UINT8_MAX;
        uint8_t dkv = UINT8_MAX;

        //minimum torque
        int16_t m_loss = 0;
        int16_t m_loss_ac = 0;
        int16_t m_loss_generator = 0;
        if (0 < n_mot)
        {
            m_loss = int16_t(a + b *(float)n_mot/1000.F + c * powf((float)n_mot/1000.F, 2)); // Torque loss in the drivetrain, calculated from a quadratic fit of measured torque loss on the dyno
            HFM_628 hfm628;
            if (hfm_ecu.get_HFM_628(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm628))
            {
                if (hfm628.KLIMA_B)
                {
                    m_loss_ac = (int16_t)(K_AC / ((float)n_mot));
                }
            }
            m_loss_generator = (int16_t)(K_GENERATOR / ((float)n_mot)); // Torque loss due to generator, modeled as inversely proportional to engine speed
        }
        result.m_min = -(m_loss + m_loss_ac + m_loss_generator); // Torque loss is represented as negative torque
    
        // maximum torque
        result.m_max = hfm_engine->get_max_torque(n_mot); // interpolate from torque map based on engine speed

        HFM_610 hfm610;
        if (this->hfm_ecu.get_HFM_610(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm610))
        {
            // conversion from raw value to [kg/h]
            mle = ((float)(hfm610.MLE)) * AIR_MASS_FACTOR;
            max_mass_air_flow = hfm_engine->get_max_mass_air_flow(n_mot);

            // indicated torque
            // ratio of current mass air flow to maximum mass air flow at current engine speed, multiplied by maximum torque at current engine speed
            result.m_ind = 0;
            result.m_converted_static = 0;
            if (0.F < max_mass_air_flow)
            {
                result.m_converted_static = (int16_t)(MIN(1.F, mle / max_mass_air_flow) * ((float)(result.m_max)));
            }
            result.m_ind = result.m_converted_static + result.m_min; // Indicated torque is the static torque plus the minimum torque (which is negative, since it's a loss)
            result.m_ind = LIMIT(result.m_ind, result.m_min, result.m_max); // Limit indicated torque to min and max torque
            
            HFM_210 hfm210;
            if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_hfm_can, &hfm210))
            {
                if (!hfm210.DKI_UP_B)
                {
                    dki = hfm210.DKI;
                    // add measured mass air flow to map for current engine speed and throttle position, so that it can be used for interpolation later on
                    hfm_engine->add_to_mass_air_flow_map(n_mot, dki, mle);
                }
                if (!hfm210.DKV_UP_B)
                {
                    dkv = hfm210.DKV;

                    // driver torque - comes from the accelerator pedal or cruise control
                    result.m_converted_driver = 0;
                    if (0.F < max_mass_air_flow)
                    {
                        result.m_converted_driver = (int16_t)((hfm_engine->get_mass_air_flow(n_mot, dkv) / max_mass_air_flow) * ((float)(result.m_max)));
                    }
                    result.m_converted_driver = LIMIT(result.m_converted_driver, 0, result.m_max); // Limit driver demanded torque to min and max torque
                    
                    bool freeze = MMAX_EGS;
                    // Change torque values based on freezing or not
                    if (freeze)
                    {
                        result.m_converted_driver = MAX(result.m_converted_driver - this->req_static_torque_delta, result.m_converted_static);
                    }
                    else
                    {
                        this->req_static_torque_delta = result.m_converted_driver - result.m_converted_static;
                    }

                    // calculating torque loss due to air conditioning, if the air conditioning is on
                    // result.m_converted_driver -= m_loss_ac;
                    result.m_converted_static -= m_loss_ac;
                }
            }
        }
        // ESP_LOGI("HFM-CAN", "N: %u rpm, DKI: %u, DKV: %u, MLE: %.0f kg/h, MAF_max: %.0f kg/h, max_throttle_value: %.2f, M_MAX: %u Nm, M_DRIVER: %u Nm, M_IND: %u Nm, M_STAT: %u Nm, M_LOSS: %u Nm", n_mot, dki, dkv, mle, max_mass_air_flow, ((float)VEHICLE_CONFIG.throttlevalve_maxopeningangle) * .35F, result.m_max, result.m_converted_driver, result.m_ind, result.m_converted_static, m_loss);
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
        if (!hfm608.HOH_UP_B)
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