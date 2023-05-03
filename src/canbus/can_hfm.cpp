#include "can_hfm.h"
#include "driver/twai.h"
#include "driver/i2c.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "tcu_maths.h"


HfmCan::HfmCan(const char* name, uint8_t tx_time_ms, uint32_t baud) : EgsBaseCan(name, tx_time_ms, baud) {
    ESP_LOGI("ClassicEGS", "SETUP CALLED");
    if(ShifterStyle::TRRS != VEHICLE_CONFIG.shifter_style){
        // Hfm-CAN has 125kbit/s; EWM requires 500kbit/s-CAN
        ESP_LOGE("INIT", "ERROR. CAN mode is set to Hfm-CAN (125kbit/s), but shifter is set to (500kbit/s)! Set shifter to TRRS instead!");                        
    }
    this->start_enable = true;
}

WheelData HfmCan::generateWheelData(uint64_t now, uint64_t expire_time_ms) {
    // TODO: should be read from sensor through GPIO instead of using the inaccurate vehicle speed
    WheelData result = WheelData {
        .double_rpm = 0,
        .current_dir = WheelDirection::SignalNotAvailable
    };
    HFM_210 hfm210;
    if(this->hfm_ecu.get_HFM_210(now, expire_time_ms, &hfm210)){
        if (!hfm210.get_VSIG_UP_B())
        {
            if (0 < VEHICLE_CONFIG.wheel_circumference)
            {
                // convert from km/h to m/min first and divide by circumference, which originally is in mm
                result.double_rpm = ((((double)hfm210.get_V_SIGNAL()) * 1.2 * 1000.0) / 60.0) * (1000.0 / ((double)VEHICLE_CONFIG.wheel_circumference));
                switch (hfm210.get_WHC())
                {
                case HFM_210h_WHC::PN_B:
                    result.current_dir = WheelDirection::Stationary;
                    break;
                case HFM_210h_WHC::R_B:
                    result.current_dir = WheelDirection::Reverse;
                    break;
                case HFM_210h_WHC::D4_B:
                    result.current_dir = WheelDirection::Forward;
                    break;
                default:
                    result.current_dir = WheelDirection::SignalNotAvailable;
                    break;
                }
            }
        }
    }
    return result;
}

WheelData HfmCan::get_front_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    return generateWheelData(now, expire_time_ms);
}

WheelData HfmCan::get_front_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    return generateWheelData(now, expire_time_ms);
}

WheelData HfmCan::get_rear_right_wheel(uint64_t now, uint64_t expire_time_ms) {
    return generateWheelData(now, expire_time_ms);
}

WheelData HfmCan::get_rear_left_wheel(uint64_t now, uint64_t expire_time_ms) {
    return generateWheelData(now, expire_time_ms);
}

ShifterPosition HfmCan::get_shifter_position(uint64_t now, uint64_t expire_time_ms) {
    return shifter->get_shifter_position(now, expire_time_ms);
}

EngineType HfmCan::get_engine_type(uint64_t now, uint64_t expire_time_ms) {
    // M104 & M111 are always petrol
    // TODO: Additional logic can check, if car-type sent on CAN matches the engine type
    return EngineType::Petrol;
}

bool HfmCan::get_engine_is_limp(uint64_t now, uint64_t expire_time_ms) {
    bool result = false;
    HFM_210 hfm210;
    if(this->hfm_ecu.get_HFM_210(now, expire_time_ms, &hfm210)){
        result = hfm210.get_NOTL_B();
    }
    return result;
}

bool HfmCan::get_kickdown(uint64_t now, uint64_t expire_time_ms) {
    // TODO: check if there is a difference to full throttle and kick down
    // TODO: since kick-down switch is directly connected to the TCU, it could make more sense to read that signal instead --> could be faster
    bool result = false;
    HFM_210 hfm210;
    if(this->hfm_ecu.get_HFM_210(now, expire_time_ms, &hfm210)){
        result = hfm210.get_VG_B();
    }
    return result;
}

uint8_t HfmCan::get_pedal_value(uint64_t now, uint64_t expire_time_ms) { // TODO
    // TODO: convert from throttle valve-value
    // TODO: requires the maximum possible throttle valve-value to calculate the relative position --> parameter in VEHICLE_CONFIG
    uint8_t result = UINT8_MAX;
    HFM_210 hfm210;
    if(this->hfm_ecu.get_HFM_210(now, expire_time_ms, &hfm210)){
        if(!hfm210.get_DKI_UP_B()){
            uint8_t dki = hfm210.get_DKI();
            result = (250u < dki) ? 250u : dki;
        }
    }
    return result;
}

int HfmCan::get_static_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    int result = INT_MAX;
    HFM_308 hfm308;
    if(this->hfm_ecu.get_HFM_308(now, expire_time_ms, &hfm308)){
        if(hfm308.get_HFM_UP_B()){
            HFM_610 hfm610;
            if(this->hfm_ecu.get_HFM_610(now, expire_time_ms, &hfm610)){
                // TODO: calculate using int-values
                float mle = ((float)(hfm610.get_MLE())) * 4.F;                
                float nmot = ((float)(get_engine_rpm(now, expire_time_ms)));
                // constant * mass air flow / engine speed
                // TODO: convert c_eng, if need
                result = (int)(VEHICLE_CONFIG.c_eng * mle / nmot);
            }    
        }
    }
    return result;
}

int HfmCan::get_driver_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    // init result value
    int result = INT_MAX;
    // check if maximum opening angle of throttle valve is set
    if(0u < VEHICLE_CONFIG.throttlevalve_maxopeningangle) {
        HFM_210 hfm210;
        if(this->hfm_ecu.get_HFM_210(now, expire_time_ms, &hfm210)){
            // check if throttle valve actual value is plausible
            if(!hfm210.get_DKI_UP_B()){
                uint8_t dki = hfm210.get_DKI();
                uint8_t dkv = 0u;
                // check if throttle valve target value is plausible
                if(!hfm210.get_DKV_UP_B()){
                    dkv = hfm210.get_DKV();
                }
                // use the higher value to ensure the requested value is collected and convert it to angle in degrees
                uint8_t dk = MAX(dki, dkv);
                // calculate relative openening of the throttle valve
                float rel =  (1.F - cosine[dk]) / (1.F - cosine[VEHICLE_CONFIG.throttlevalve_maxopeningangle]);
                // calculate the static engine torque
                float m_stat = (float)(this->get_static_engine_torque(now, expire_time_ms));
                // calculate the 
                result = (int)(rel * m_stat);
            }
        }
    }
    return result;
}

int HfmCan::get_maximum_engine_torque(uint64_t now, uint64_t expire_time_ms) { // TODO
    int result = INT_MAX;
    HFM_308 hfm308;
    if(this->hfm_ecu.get_HFM_308(now, expire_time_ms, &hfm308)){
        if(!hfm308.get_NMOT_UP_B()){
            float nmot = ((float)(hfm308.get_NMOT()));
            result = (int)(enginemaxtorque->get_value(nmot));
        }
    }
    return result;
}

int HfmCan::get_minimum_engine_torque(uint64_t now, uint64_t expire_time_ms) {
    // Always 0, since Hfm-ECUs do not calculate inertia
    return 0;
}

PaddlePosition HfmCan::get_paddle_position(uint64_t now, uint64_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t HfmCan::get_engine_coolant_temp(uint64_t now, uint64_t expire_time_ms) {
    int16_t result = INT16_MAX;
    HFM_608 hfm608;
    if(this->hfm_ecu.get_HFM_608(now, expire_time_ms, &hfm608)) {
        if(!hfm608.get_TFM_UP_B()){
            result = (int16_t)((((float)(hfm608.get_T_MOT())) * 1.6F) - 44.F);
        }
    }
    return result;
}

int16_t HfmCan::get_engine_oil_temp(uint64_t now, uint64_t expire_time_ms) {
    // Not available on Hfm-ECUs
    return INT16_MAX;
}

int16_t HfmCan::get_engine_iat_temp(uint64_t now, uint64_t expire_time_ms) {
    // TODO
    return INT16_MAX;
}

uint16_t HfmCan::get_engine_rpm(uint64_t now, uint64_t expire_time_ms) {
    uint16_t result = UINT16_MAX;
    HFM_308 hfm308;
    if(this->hfm_ecu.get_HFM_308(now, expire_time_ms, &hfm308)){
        if(!hfm308.get_NMOT_UP_B()){
            result = hfm308.get_NMOT();
        }
    }
    return result;
}

bool HfmCan::get_is_starting(uint64_t now, uint64_t expire_time_ms) { // TODO
    bool result =  false;
    HFM_308 hfm308;
    if(this->hfm_ecu.get_HFM_308(now, expire_time_ms, &hfm308)){
        result = hfm308.get_KL50_B();
    }
    return result;
}

bool HfmCan::get_is_brake_pressed(uint64_t now, uint64_t expire_time_ms) {
    bool result = false;
    HFM_210 hfm210;
    if(this->hfm_ecu.get_HFM_210(now, expire_time_ms, &hfm210)){
        result = hfm210.get_BLS_B();
    }
    return result;
}

bool HfmCan::get_profile_btn_press(uint64_t now, uint64_t expire_time_ms) {
    // Only applicable for EWM (500kbit/s), which is not compatible to 125kbit/s-Hfm-CAN
    return false;
}

void HfmCan::set_clutch_status(ClutchStatus status) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_actual_gear(GearboxGear actual) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_target_gear(GearboxGear target) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_safe_start(bool can_start) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_gearbox_temperature(uint16_t temp) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_input_shaft_speed(uint16_t rpm) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_is_all_wheel_drive(bool is_4wd) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_wheel_torque(uint16_t t) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_shifter_position(ShifterPosition pos) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_gearbox_ok(bool is_ok) {
    // TODO: should ground the connector to the Hfm-ECU
}

void HfmCan::set_torque_request(TorqueRequest request, float amount_nm) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_error_check_status(SystemStatusCheck ssc) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_turbine_torque_loss(uint16_t loss_nm) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_drive_profile(GearboxProfile p) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_display_msg(GearboxMessage msg) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::set_wheel_torque_multi_factor(float ratio) {
    // nothing is sent on Hfm-CAN
}

void HfmCan::tx_frames() {
    // None to transmit on HFM!
}

void HfmCan::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, uint64_t timestamp) {
    this->hfm_ecu.import_frames(data, id, timestamp);
}

void HfmCan::on_rx_done(uint64_t now_ts) {
    if(ShifterStyle::TRRS == VEHICLE_CONFIG.shifter_style) {
        (static_cast<ShifterTrrs*>(shifter))->update_shifter_position(now_ts);
    }
}