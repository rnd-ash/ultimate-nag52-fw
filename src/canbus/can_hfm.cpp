#include "can_hfm.h"
#include "driver/twai.h"
#include "driver/i2c_master.h"
#include "nvs/eeprom_config.h"
#include "tcu_maths.h"
#include <tcu_maths_impl.h>

HfmCan::HfmCan(const char *name, uint8_t tx_time_ms) : EgsBaseCan(name, tx_time_ms, 125000u)
{   
    ESP_LOGI(name, "SETUP CALLED");
    this->start_enable = true;
    can_init_status = ESP_OK;

    // set the value of the max_throttle-value
    // avoid division by zero, if value is not set
    if (0 < VEHICLE_CONFIG.throttlevalve_maxopeningangle)
    {
        max_throttle_value = VEHICLE_CONFIG.throttlevalve_maxopeningangle;
    }
    for(uint16_t i = 0; i <= UINT8_MAX; i++)
    {
        phi_throttle[i] = (1.F - cosine[i]) / (1.F - cosine[max_throttle_value]);
    }
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
    // TODO: check if there is a difference to full throttle and kick down
    bool result = false;
    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {
        result = hfm210.VG_B;
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
        dkv = MIN(((float)(hfm210.DKV)), max_throttle_value); // limitation to maximum opening angle of throttle valve
        dkv *= (float)PEDAL_VALUE_LIMIT;
        dkv /= (float)max_throttle_value;
    }
    result = (uint8_t)dkv;
    return result;
}

CanTorqueData HfmCan::get_torque_data(const uint32_t expire_time_ms) {

    const bool CALC_C_ENGINE = false;    

    // calculate torque values
    CanTorqueData result = TORQUE_NDEF;

    // obtain values from the CAN-bus to calculate the indicated and the driver torque
    uint16_t n_mot = get_engine_rpm(expire_time_ms);
    float n_mot_SI = ((float)n_mot) / 60.F; // conversion from [1/min] to [1/s]
    float mle = 0.F;

    HFM_610 hfm610;
    // if (this->hfm_ecu.get_HFM_610(GET_CLOCK_TIME(), 1000, &hfm610))
    if (this->hfm_ecu.get_HFM_610(GET_CLOCK_TIME(), expire_time_ms, &hfm610))
    {
        mle = ((float)(hfm610.MLE)) * AIR_MASS_FACTOR;              // conversion from raw value to [kg/h]
        mle /= 3600.F;                                              // conversion from [kg/h] to [kg/s]
    }

    HFM_210 hfm210;
    if (this->hfm_ecu.get_HFM_210(GET_CLOCK_TIME(), expire_time_ms, &hfm210))
    {        
        // indicated torque
        uint8_t dki = 0;
        dki = MIN(hfm210.DKI, max_throttle_value); // limitation to maximum opening angle of throttle valve
        result.m_ind = (int16_t)(phi_throttle[dki] * (float)(result.m_max));

        // driver torque - comes from the accelerator pedal or cruise control
        uint8_t dkv = 0;
        dkv = MIN(hfm210.DKV, max_throttle_value); // limitation to maximum opening angle of throttle valve
        result.m_converted_driver = (int16_t)(phi_throttle[dkv] * (float)(result.m_max));
    }

    // minimum torque
    result.m_min = 0; // TODO: find a way to get this value

    // maximum torque
    result.m_max = interpolate_linear_array(n_mot, M_MAX_LEN, n, M_MAX);

    //static torque
    if(0 < n_mot) {
        result.m_converted_static = (int16_t)(c_engine * mle / n_mot_SI); // constant * mass air flow / engine speed
    }
    
    if (INT16_MAX != result.m_converted_static && INT16_MAX != result.m_converted_driver)
    {
        int16_t static_converted = result.m_converted_static;
        int16_t tmp = result.m_converted_driver;
        int16_t driver_converted = static_converted;
        int16_t indicated = 0;
        // Calculate converted torque from ESP
        if (INT16_MAX != result.m_max && INT16_MAX != result.m_min) {
            tmp = MAX(MIN(result.m_converted_driver, result.m_max), result.m_min);
        }
        if (tmp <= 0) {
            tmp = MIN(tmp, static_converted);
        }
        driver_converted = tmp;

        // Check if freezing torque should be done
        bool active_shift = actual_gear != target_gear;
        bool trq_req_en = MMAX_EGS;
        if (active_shift && trq_req_en) {
            this->freeze_torque = true; // Gear shift and we have started a torque request, freeze it
        } else if (!active_shift) {
            this->freeze_torque = false; // No gear shift, unfreeze it
        }
        // Change torque values based on freezing or not
        if (this->freeze_torque) {
            driver_converted = MAX(driver_converted - this->req_static_torque_delta, static_converted);
        } else {
            this->req_static_torque_delta = driver_converted - static_converted;
        }
        if (driver_converted > 0) {
            indicated = driver_converted;
        }

        HFM_628 hfm628;
        if (hfm_ecu.get_HFM_628(GET_CLOCK_TIME(), expire_time_ms, &hfm628)) {
            if (hfm628.KLIMA_B) {
                // Typical drag torque for an air conditioning compressor in a combustion engine car
                // is in the range of 10-30 Nm depending on compressor type and operating conditions.
                const int16_t M_KOMP = 20;

                driver_converted -= M_KOMP;
                static_converted -= M_KOMP;
            }
        } 
        
        result.m_ind = indicated;
        result.m_converted_driver = driver_converted;
        result.m_converted_static = static_converted;
    }

    // calculate c_engine values
    if (CALC_C_ENGINE)
    {
        float c_engine_tmp = 0.F;
        uint8_t load = 0;
        HFM_308 hfm308;
        if (this->hfm_ecu.get_HFM_308(GET_CLOCK_TIME(), 1000, &hfm308))
        {
            load = hfm308.LAST;
        }
        if(200u == load){
            c_engine_tmp = ((uint32_t)(result.m_max)) * n_mot_SI / mle;
            uint8_t dki = 0;
            // log messages
            // ESP_LOGI("HFM-CAN", "MLE: %.2f kg/s, n_mot: %d 1/min, m_max: %i, c_eng: %.2f, load: %d, DKI: %d, phi_throttle: %.2f, m_ind: %i Nm, m_ind (alt): %i, cosine[dki]: %.2f, cosine[max]: %.2f", mle, n_mot, result.m_max, c_engine_tmp, load, dki_uint, phi_throttle_ind, result.m_ind, ((int16_t)(phi_throttle_ind * ((float)result.m_max))), cosine[dki_uint], cosine[VEHICLE_CONFIG.throttlevalve_maxopeningangle]);
            ESP_LOGI("HFM-CAN", "MLE: %.2f kg/s, n_mot: %d 1/min, m_max: %i, c_eng: %.2f, load: %d, phi_throttle: %.2f, m_ind: %i Nm, m_ind (alt): %i, cosine[dki]: %.2f, cosine[max]: %.2f", mle, n_mot, result.m_max, c_engine_tmp, load, phi_throttle[dki], result.m_ind, ((int16_t)(phi_throttle[dki] * ((float)result.m_max))), cosine[dki], cosine[max_throttle_value]);
        }
    }

    return result;
}

float HfmCan::get_ML(const uint32_t expire_time_ms)
{
    /* mass volumetric efficiency */
    float lambda_l = 0.0f;
    float m_Air = 0.0f; // TODO: use MLE
    float m_theoretical = 0.0f; // TODO: 
    /* engine displacement [m³]*/
    //float engine_displacement = ((float)V_H) * 0.001f;
    /* air density [kg/m³] */
    float rho_theoretical = 1.188f;
    /* temperature [K] */
    float T_theoretical = get_engine_iat_temp(expire_time_ms);
    /* pressure [N/m²]*/
    float p_theoretical = this->current_air_pressure; // TODO: use P_HOH
    rho_theoretical = T_theoretical / p_theoretical;

    m_theoretical = rho_theoretical * V_H / ((float)n_cylinders);
    if (0.0f != m_theoretical) {
        lambda_l = m_Air / m_theoretical;
    }
    return lambda_l;
}

int16_t HfmCan::get_engine_coolant_temp(const uint32_t expire_time_ms)
{
    int16_t result = INT16_MAX;
    HFM_608 hfm608;
    if (this->hfm_ecu.get_HFM_608(GET_CLOCK_TIME(), expire_time_ms, &hfm608))
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
    int16_t result = INT16_MAX;
    HFM_608 hfm608;
    if (this->hfm_ecu.get_HFM_608(GET_CLOCK_TIME(), expire_time_ms, &hfm608))
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