#include "can_egs52.h"
#include "driver/twai.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "../shifter/shifter_ewm.h"
#include "../shifter/shifter_trrs.h"
#include "ioexpander.h"
#include "egs_calibration/calibration_structs.h"

Egs52Can::Egs52Can(const char *name, uint8_t tx_time_ms, uint32_t baud, Shifter *shifter) : EgsBaseCan(name, tx_time_ms, baud, shifter) {
    // Set default values
    this->gs218.raw = 0;
    this->gs338.raw = ~0;
    this->gs418.raw = ~0;
    this->gs218.MKRIECH = 0;
    this->gs418.FMRAD = 1.0;
    this->set_target_gear(GearboxGear::SignalNotAvailable);
    this->set_actual_gear(GearboxGear::SignalNotAvailable);
    this->set_shifter_position(ShifterPosition::P);
    this->gs418.KD = false;
    this->gs418.SCHALT = false; // Auto is 0, manual is 1
    this->gs218.SCHALT = false;
    this->gs218.GIC = GS_218h_GIC_EGS52::G_SNV;
    gs218.CALID_CVN_AKT = true;
    gs218.G_G = false;
    this->gs218.ALF = true; // Fix for KG systems where cranking would stop when TCU turns on
    // Set profile to N/A for now
    this->set_drive_profile(GearboxProfile::Underscore);
    // Set no message
    this->set_display_msg(GearboxMessage::None);
    if (VEHICLE_CONFIG.is_four_matic != 0) {
        this->gs418.ALLRAD = true;
    } else {
        this->gs418.ALLRAD = false;
    }
    this->gs418.FRONT = false; // Primary rear wheel drive
    this->gs418.CVT = false; // Not CVT gearbox]
    if (MECH_PTR->gb_ty == 0) {
        this->gs418.MECH = GS_418h_MECH_EGS52::GROSS;
    } else {
        this->gs418.MECH = GS_418h_MECH_EGS52::KLEIN;
    }
    // New! GS_338 data for newer K-Matrix cars (W216)
    this->gs338.NTURBINE = 0;
    this->gs338.MABS_SW_ANFB = 0;
    this->gs338.M_VORSTR = 0;
    this->gs338.RACE_START = GS_338h_RACE_START_EGS52::OFF;
    this->gs338.KID = 0;
    this->gs338.NAK_TGL = false;
    this->gs338.NAK_PA = false;
    this->gs338.MIL_ANF_GS = false;
    this->gs338.NAB = 0xFFFF; // Passive as we don't report this (TODO - G class SHOULD report this)
}

uint16_t Egs52Can::get_front_right_wheel(const uint32_t expire_time_ms)
{ // TODO
	return UINT16_MAX;
}

uint16_t Egs52Can::get_front_left_wheel(const uint32_t expire_time_ms) { // TODO
    return UINT16_MAX;
}

uint16_t Egs52Can::get_rear_right_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS52 bs208;
    uint16_t ret = UINT16_MAX;
    if (this->esp_ecu.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (BS_208h_DRTGHR_EGS52::SNV != bs208.DRTGHR) {
            ret = bs208.DHR;
        }
        
    }
    return ret;
}

uint16_t Egs52Can::get_rear_left_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS52 bs208;
    uint16_t ret = UINT16_MAX;
    if (this->esp_ecu.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (BS_208h_DRTGHL_EGS52::SNV != bs208.DRTGHL) {
            ret = bs208.DHL;
        }
        
    }
    return ret;
}

ShifterPosition Egs52Can::internal_can_shifter_get_shifter_position(const uint32_t expire_time_ms) {
	ShifterPosition ret = ShifterPosition::SignalNotAvailable;
	EWM_230_EGS52 dest;
	if (this->ewm_ecu.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest))
	{
		switch (dest.WHC)
		{
		case EWM_230h_WHC_EGS52::D:
			ret = ShifterPosition::D;
			break;
		case EWM_230h_WHC_EGS52::N:
			ret = ShifterPosition::N;
			break;
		case EWM_230h_WHC_EGS52::R:
			ret = ShifterPosition::R;
			break;
		case EWM_230h_WHC_EGS52::P:
			ret = ShifterPosition::P;
			break;
		case EWM_230h_WHC_EGS52::PLUS:
			ret = ShifterPosition::PLUS;
			break;
		case EWM_230h_WHC_EGS52::MINUS:
			ret = ShifterPosition::MINUS;
			break;
		case EWM_230h_WHC_EGS52::N_ZW_D:
			ret = ShifterPosition::N_D;
			break;
		case EWM_230h_WHC_EGS52::R_ZW_N:
			ret = ShifterPosition::R_N;
			break;
		case EWM_230h_WHC_EGS52::P_ZW_R:
			ret = ShifterPosition::P_R;
			break;
		case EWM_230h_WHC_EGS52::SNV:
			break;
		default:
			break;
		}
	}
	return ret;
}

EngineType Egs52Can::get_engine_type(const uint32_t expire_time_ms) {
    MS_608_EGS52 ms608;
    // This signal can be valid for up to 1000ms
    if (this->ecu_ms.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        switch (ms608.get_FCOD_MOT()) {
            case MS_608h_FCOD_MOT_EGS52::OM611DE22LA100:
            case MS_608h_FCOD_MOT_EGS52::OM640DE20LA60:
            case MS_608h_FCOD_MOT_EGS52::OM642DE30LA160:
                return EngineType::Diesel;
            default:
                return EngineType::Unknown;
        }
    } else {
        return EngineType::Unknown;
    }
}

bool Egs52Can::get_engine_is_limp(const uint32_t expire_time_ms) { // TODO
    return false;
}

bool Egs52Can::get_kickdown(const uint32_t expire_time_ms) { // TODO
    return false;
}

uint8_t Egs52Can::get_pedal_value(const uint32_t expire_time_ms) { // TODO
    MS_210_EGS52 ms210;
    // This signal can be valid for up to 1000ms
    if (this->ecu_ms.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        return ms210.PW;
    } else {
        return 0xFF;
    }
}

CanTorqueData Egs52Can::get_torque_data(const uint32_t expire_time_ms) {
    MS_312_EGS52 ms312;
    MS_212_EGS52 ms212;
    MS_210_EGS52 ms210;
    CanTorqueData ret = TORQUE_NDEF;
    if (this->ecu_ms.get_MS_312(GET_CLOCK_TIME(), expire_time_ms, &ms312)) {
        if (UINT16_MAX != ms312.M_STA) {ret.m_converted_static = ((int16_t)ms312.M_STA / 4) - 500;}
        if (UINT16_MAX != ms312.M_MIN) {ret.m_min = ((int16_t)ms312.M_MIN / 4) - 500;}
        if (UINT16_MAX != ms312.M_MAX) {ret.m_max = ((int16_t)ms312.M_MAX / 4) - 500;}
    }
    if (this->ecu_ms.get_MS_212(GET_CLOCK_TIME(), expire_time_ms, &ms212)) {
        if (UINT16_MAX != ms212.M_ESPV) {ret.m_converted_driver = ((int16_t)ms212.M_ESPV / 4) - 500;}
    }
    if (INT16_MAX != ret.m_max) {
        // Get factor to correct it by
        if (this->ecu_ms.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
            if (UINT8_MAX != ms210.FMMOTMAX) {ret.m_max = (float)ret.m_max * ((float)ms210.FMMOTMAX * 0.0078);}
        }
    }
    if (INT16_MAX != ret.m_converted_static && INT16_MAX != ret.m_converted_driver) {
        int static_converted = ret.m_converted_static;
        int tmp = ret.m_converted_driver;
        int driver_converted = static_converted;
        int indicated = 0;
        // Calculate converted torque from ESP
        // Chrysler cars don't seem to report MAX/MIN
        if (INT_MAX != ret.m_max && INT_MAX != ret.m_min) {
            tmp = MIN(ret.m_converted_driver, ret.m_max);
        }
        if (tmp <= 0) {
            tmp = MIN(tmp, static_converted);
        }
        driver_converted = tmp;

        // Check if freezing torque should be done
        bool active_shift = (uint8_t)this->gs418.GIC != (uint8_t)this->gs418.GZC;
        bool trq_req_en = this->gs218.MMIN_EGS != 0 || this->gs218.MMAX_EGS != 0;
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
        KLA_410_EGS52 kl410;
        if (this->ezs_ecu.get_KLA_410(GET_CLOCK_TIME(), expire_time_ms, &kl410)) {
            if (UINT8_MAX != kl410.M_KOMP) {
                driver_converted -= (kl410.M_KOMP / 4);
                static_converted -= (kl410.M_KOMP / 4);
            }
        }
        ret.m_ind = indicated;
        ret.m_converted_driver = driver_converted;
        ret.m_converted_static = static_converted;
    }
    return ret;
}

PaddlePosition Egs52Can::get_paddle_position(const uint32_t expire_time_ms) {
    SBW_232 sbw;
    if (misc_ecu.get_SBW_232(GET_CLOCK_TIME(), expire_time_ms, &sbw)) { // 50ms timeout
        switch (sbw.get_LRT_PM3()) {
            case SBW_232h_LRT_PM3::PLUS:
                return PaddlePosition::Plus;
            case SBW_232h_LRT_PM3::MINUS:
                return PaddlePosition::Minus;
            case SBW_232h_LRT_PM3::SNV:
                return PaddlePosition::SNV;
            default:
                return PaddlePosition::None;
        }
    } else {
        return PaddlePosition::SNV;
    }
}

int16_t Egs52Can::get_engine_coolant_temp(const uint32_t expire_time_ms) {
    MS_608_EGS52 ms608;
    int16_t res = INT16_MAX;
    if (ecu_ms.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_MOT != UINT8_MAX) {
            res = ms608.T_MOT-40;
        }
    }
    return res;
}

int16_t Egs52Can::get_engine_oil_temp(const uint32_t expire_time_ms) {
    MS_308_EGS52 ms308;
    int16_t res = INT16_MAX;
    if (ecu_ms.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        if (ms308.T_OEL != UINT8_MAX) {
            res = ms308.T_OEL-40;
        }
    }
    return res;
}

int16_t Egs52Can::get_engine_iat_temp(const uint32_t expire_time_ms) {
    MS_608_EGS52 ms608;
    int16_t res = INT16_MAX;
    if (ecu_ms.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_LUFT != UINT8_MAX) {
            res = ms608.T_LUFT-40;
        }
    }
    return res;
}

uint16_t Egs52Can::get_engine_rpm(const uint32_t expire_time_ms) {
    MS_308_EGS52 ms308;
    if (ecu_ms.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        return ms308.NMOT;
    } else {
        return UINT16_MAX;
    }
}

bool Egs52Can::get_is_starting(const uint32_t expire_time_ms) { // TODO
    MS_308_EGS52 ms308;
    if (ecu_ms.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        return ms308.ANL_LFT;
    } else {
        return false;
    }
}

bool Egs52Can::get_is_brake_pressed(const uint32_t expire_time_ms) {
    BS_200_EGS52 bs200;
    if (this->esp_ecu.get_BS_200(GET_CLOCK_TIME(), expire_time_ms, &bs200)) {
        return bs200.BLS == BS_200h_BLS_EGS52::BREMSE_BET;
    } else {
        return false;
    }
}

bool Egs52Can::get_profile_btn_press(const uint32_t expire_time_ms) {
	bool result = false;
	EWM_230_EGS52 ewm;
    if (this->ewm_ecu.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &ewm)) {
        result = ewm.FPT;
    }
    return result;
}

ProfileSwitchPos Egs52Can::get_profile_switch_pos(const uint32_t expire_time_ms) {
    EWM_230_EGS52 ewm;
    ProfileSwitchPos result = ProfileSwitchPos::SNV;
    if (this->ewm_ecu.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &ewm)) {
        result = ewm.W_S ? ProfileSwitchPos::Top : ProfileSwitchPos::Bottom;
    }
    return result;
}

uint16_t Egs52Can::get_fuel_flow_rate(const uint32_t expire_time_ms) {
    MS_608_EGS52 ms608;
    if (this->ecu_ms.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        return (uint16_t)((float)ms608.VB*0.868);
    } else {
        return 0;
    }
}

TransferCaseState Egs52Can::get_transfer_case_state(const uint32_t expire_time_ms) {
    if (VEHICLE_CONFIG.jeep_chrysler) {
        // Handling for Jeep cars. This appears to be part of the MS608 frame
        MS_608_EGS52 ms608;
        if (this->ecu_ms.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
            switch (ms608.bytes[0]) {
                case 0x03:
                    return TransferCaseState::Hi;// - High
                case 0x04:
                    return TransferCaseState::Neither;// - Neutral
                case 0x05:
                    return TransferCaseState::Low;// - Low range
                default:
                    return TransferCaseState::SNA;
            }
        } else {
            return TransferCaseState::SNA;
        }
    } else {
        // Mercedes handling - Query the dedicated VG ECU
        VG_428 vg428;
        if (this->misc_ecu.get_VG_428(GET_CLOCK_TIME(), expire_time_ms, &vg428)) {
            switch (vg428.get_VG_GANG()) {
                case VG_428h_VG_GANG::HI:
                    return TransferCaseState::Hi;
                case VG_428h_VG_GANG::LO:
                    return TransferCaseState::Low;
                case VG_428h_VG_GANG::N:
                    return TransferCaseState::Neither;
                case VG_428h_VG_GANG::SH_IPG:
                    return TransferCaseState::Switching;
                case VG_428h_VG_GANG::SNV:
                default:
                    return TransferCaseState::SNA;
            }
        } else {
            return TransferCaseState::SNA;
        }
    }
}

bool Egs52Can::engine_ack_torque_request(const uint32_t expire_time_ms) {
    MS_212_EGS52 ms212;
    if (this->ecu_ms.get_MS_212(GET_CLOCK_TIME(), expire_time_ms, &ms212)) {
        return ms212.M_EGS_Q;
    } else {
        return false;
    }
}

bool Egs52Can::esp_torque_intervention_active(const uint32_t expire_time_ms) {
    BS_300_EGS52 bs300;
    int r = false;
    if (this->esp_ecu.get_BS_300(GET_CLOCK_TIME(), expire_time_ms, &bs300)) {
        r = bs300.MMIN_ESP || bs300.MMAX_ESP;
    }
    return r;
}

bool Egs52Can::is_cruise_control_active(const uint32_t expire_time_ms) {
    BS_300_EGS52 bs300;
    int r = false;
    if (this->esp_ecu.get_BS_300(GET_CLOCK_TIME(), expire_time_ms, &bs300)) {
        r = bs300.DMMAX_ART || bs300.DMMIN_ART;
    }
    return r;
}

int Egs52Can::cruise_control_torque_demand(const uint32_t expire_time_ms) {
    BS_300_EGS52 bs300;
    int r = INT_MAX;
    if (this->esp_ecu.get_BS_300(GET_CLOCK_TIME(), expire_time_ms, &bs300)) {
        r = (bs300.DM_ART/4) - 500.0;
    }
    return r;
}

int Egs52Can::esp_torque_demand(const uint32_t expire_time_ms) {
    BS_300_EGS52 bs300;
    int r = INT_MAX;
    if (this->esp_ecu.get_BS_300(GET_CLOCK_TIME(), expire_time_ms, &bs300)) {
        r = (bs300.M_ESP/4) - 500.0;
    }
    return r;
}

TccReqState Egs52Can::get_engine_tcc_override_request(const uint32_t expire_time_ms) {
    MS_210_EGS52 ms210;
    MS_308_EGS52 ms308;
    TccReqState ret = TccReqState::None;
    // Check for slip first
    if (this->ecu_ms.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        if (ms210.KUEB_S_A) {
            ret = TccReqState::Slipping;
        }
    }
    // Then check if engine wants open
    if (this->ecu_ms.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        if (ms308.KUEB_O_A) {
            ret = TccReqState::Open;
        }
    }
    return ret;
}

void Egs52Can::set_clutch_status(TccClutchStatus status) {
    switch(status) {
        case TccClutchStatus::Open:
            gs218.K_G_B = false;
            gs218.K_O_B = true;
            gs218.K_S_B = false;
            break;
        case TccClutchStatus::OpenToSlipping:
            gs218.K_G_B = false;
            gs218.K_O_B = true;
            gs218.K_S_B = true;
            break;
        case TccClutchStatus::Slipping:
            gs218.K_G_B = false;
            gs218.K_O_B = false;
            gs218.K_S_B = true;
            break;
        case TccClutchStatus::SlippingToClosed:
            gs218.K_G_B = true;
            gs218.K_O_B = false;
            gs218.K_S_B = true;
            break;
        case TccClutchStatus::Closed:
            gs218.K_G_B = true;
            gs218.K_O_B = false;
            gs218.K_S_B = false;
            break;
        default:
            break;
    }
}

void Egs52Can::set_actual_gear(GearboxGear actual) {
    switch (actual) {
        case GearboxGear::Park:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_P;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_P;
            break;
        case GearboxGear::Reverse_First:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_R;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_R2;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_R2;
            break;
        case GearboxGear::Neutral:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_N;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_N;
            break;
        case GearboxGear::First:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_D1;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D1;
            break;
        case GearboxGear::Second:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_D2;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D2;
            break;
        case GearboxGear::Third:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_D3;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_D4;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_D5;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D5;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs418.GIC = GS_418h_GIC_EGS52::G_SNV;
            this->gs218.GIC = GS_218h_GIC_EGS52::G_SNV;
            break;
    }
}

void Egs52Can::set_target_gear(GearboxGear target) {
    switch (target) {
        case GearboxGear::Park:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_P;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_P;
            break;
        case GearboxGear::Reverse_First:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_R;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_R2;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_R2;
            break;
        case GearboxGear::Neutral:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_N;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_N;
            break;
        case GearboxGear::First:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_D1;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D1;
            break;
        case GearboxGear::Second:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_D2;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D2;
            break;
        case GearboxGear::Third:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_D3;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_D4;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_D5;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D5;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs418.GZC = GS_418h_GZC_EGS52::G_SNV;
            this->gs218.GZC = GS_218h_GZC_EGS52::G_SNV;
            break;
    }
}

void Egs52Can::set_safe_start(bool can_start) {
    this->gs218.ALF = can_start;
    if (ShifterStyle::TRRS == shifter->get_shifter_type()) { // TODO - Find a way to disable this
        ioexpander->set_start(can_start);
    }
    // ioexpander->set_start(can_start);
}

void Egs52Can::set_gearbox_temperature(int16_t temp) {
    this->gs418.T_GET = (MAX(temp, -50) + 50) & 0xFF;
}

void Egs52Can::set_input_shaft_speed(uint16_t rpm) {
    gs338.NTURBINE = rpm;
}

void Egs52Can::set_is_all_wheel_drive(bool is_4wd) {
    this->gs418.ALLRAD = is_4wd;
}

void Egs52Can::set_wheel_torque(uint16_t t) {

}

void Egs52Can::set_shifter_position(ShifterPosition pos) {
    switch (pos) {
        case ShifterPosition::P:
            gs418.WHST = GS_418h_WHST_EGS52::P;
            break;
        case ShifterPosition::R:
            gs418.WHST = GS_418h_WHST_EGS52::R;
            break;
        case ShifterPosition::N:
            gs418.WHST = GS_418h_WHST_EGS52::N;
            break;
        case ShifterPosition::D:
            gs418.WHST = GS_418h_WHST_EGS52::D;
            break;
        case ShifterPosition::SignalNotAvailable:
            gs418.WHST = GS_418h_WHST_EGS52::SNV;
            break;
        default: 
            break;
    }
}

void Egs52Can::set_gearbox_ok(bool is_ok) {
    gs218.GET_OK = is_ok; // Gearbox OK
    gs218.GSP_OK = is_ok; // Gearbox profile OK
    gs218.GS_NOTL = !is_ok; // Emergency mode activated
}

void Egs52Can::set_garage_shift_state(bool enable) {
    gs218.KS = enable;
}

void Egs52Can::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    if (control_type == TorqueRequestControlType::None) {
        gs218.DYN0_AMR_EGS = false;
        gs218.DYN1_EGS = false;
    } else if (control_type == TorqueRequestControlType::FastAsPossible) {
        gs218.DYN0_AMR_EGS = true;
        gs218.DYN1_EGS = false;
    } else if (control_type == TorqueRequestControlType::BackToDemandTorque) {
        gs218.DYN0_AMR_EGS = true;
        gs218.DYN1_EGS = true;
    } else { // Normal speed
        gs218.DYN0_AMR_EGS = true;
        gs218.DYN1_EGS = false;
    }
    if (control_type != TorqueRequestControlType::None) {
        gs218.M_EGS = (amount_nm + 500) * 4;
        if (limit_type == TorqueRequestBounds::LessThan) {
            gs218.MMIN_EGS = true;
            gs218.MMAX_EGS = false;
        } else if (limit_type == TorqueRequestBounds::MoreThan) {
            gs218.MMIN_EGS = false;
            gs218.MMAX_EGS = true;
        } else {
            gs218.MMIN_EGS = true;
            gs218.MMAX_EGS = true;
        }
    } else {
        gs218.MMIN_EGS = false;
        gs218.MMAX_EGS = false;
        gs218.M_EGS = 0;
    }
}

void Egs52Can::set_error_check_status(SystemStatusCheck ssc) {
    switch(ssc) {
        case SystemStatusCheck::Error:
            gs218.FEHLPRF_ST = GS_218h_FEHLPRF_ST_EGS52::ERROR;
            break;
        case SystemStatusCheck::Waiting:
            gs218.FEHLPRF_ST = GS_218h_FEHLPRF_ST_EGS52::WAIT;
            break;
        case SystemStatusCheck::OK:
            gs218.FEHLPRF_ST = GS_218h_FEHLPRF_ST_EGS52::OK;
            break;
        default:
            break;
    }
}


void Egs52Can::set_turbine_torque_loss(uint16_t loss_nm) {
    if (loss_nm > 0xFE/4) {
        loss_nm = 0xFE; // 0xFF implies implausible
    }
    gs418.M_VERL = loss_nm*4;
}

void Egs52Can::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
    switch(g) {
        case GearboxDisplayGear::P:
            this->gs418.FSC = 'P';
            break;
        case GearboxDisplayGear::N:
            this->gs418.FSC = 'N';
            break;
        case GearboxDisplayGear::R:
            this->gs418.FSC = 'R';
            break;
        case GearboxDisplayGear::One:
            this->gs418.FSC = '1';
            break;
        case GearboxDisplayGear::Two:
            this->gs418.FSC = '2';
            break;
        case GearboxDisplayGear::Three:
            this->gs418.FSC = '3';
            break;
        case GearboxDisplayGear::Four:
            this->gs418.FSC = '4';
            break;
        case GearboxDisplayGear::Five:
            this->gs418.FSC = '5';
            break;
        case GearboxDisplayGear::A:
            this->gs418.FSC = 'A';
            break;
        case GearboxDisplayGear::D:
            this->gs418.FSC = 'D';
            break;
        case GearboxDisplayGear::Failure:
            this->gs418.FSC = 'F';
            break;
        case GearboxDisplayGear::SNA:
        default:
            this->gs418.FSC = ' ';
            break;

    }
}

void Egs52Can::set_drive_profile(GearboxProfile p) {
    this->curr_profile_bit = p;
    gs218.HSM = false;
    switch (p) {
        case GearboxProfile::Agility:
            gs418.FPC = 'A';
            break;
        case GearboxProfile::Comfort:
            gs418.FPC = 'C';
            break;
        case GearboxProfile::Winter:
            gs418.FPC = 'W';
            gs218.HSM = true;
            break;
        case GearboxProfile::Failure:
            gs418.FPC = 'F';
            break;
        case GearboxProfile::Standard:
            gs418.FPC = 'S';
            break;
        case GearboxProfile::Manual:
            gs418.FPC = 'M';
            gs218.HSM = true;
            break;
        case GearboxProfile::Individual:
            gs418.FPC = 'I';
            break;
        case GearboxProfile::Race:
            gs418.FPC = 'R';
            gs218.HSM = true;
            break;
        case GearboxProfile::Underscore:
            gs418.FPC = '_';
            break;
        default:
            break;
    }
    // Update display message as well
    this->set_display_msg(this->curr_message);
}

void Egs52Can::set_abort_shift(bool is_aborting){
    this->gs218.GZC = GS_218h_GZC_EGS52::G_ABBRUCH;
}

void Egs52Can::set_fake_engine_rpm(uint16_t rpm) {
    this->fake_rpm = rpm;
}

void Egs52Can::set_display_msg(GearboxMessage msg) {
    this->curr_message = msg;
    if (msg == GearboxMessage::Upshift) {
        gs418.FPC = 24;
    } else if (msg == GearboxMessage::Downshift) {
        gs418.FPC = 25;
    }
}

void Egs52Can::set_wheel_torque_multi_factor(float ratio) {
    if (ratio == -1) {
        gs418.FMRAD = 0x7FF; // Implausible
    } else {
        uint16_t fmrad_int = MIN((uint16_t)(ratio * 0.05), 2046);
        gs418.FMRAD = fmrad_int;
    }
}

/**
 * Parity calculation for torque numbers on GS218 and GS418
 * 
 * Each torque request member is a struct of 16 bits comprised of the following fields:
 * 1. Toggle bit (1 bit)
 * 2. Max request - bool (1 bit)
 * 3. Min request - bool (1 bit)
 * 4. Required torque - 13 bits
 */
inline bool calc_torque_parity(uint16_t s) {
    uint16_t p = s;
    p ^= (p >> 1);
    p ^= (p >> 2);
    p ^= (p >> 4);
    p ^= (p >> 8);
    return (p & 1) == 1;
}


void Egs52Can::tx_frames() {
    tx.data_length_code = 8; // Always
    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    gs_338tx = {gs338.raw};
    gs_218tx = {gs218.raw};
    gs_418tx = {gs418.raw};

    // Firstly we have to deal with toggled bits!
    // As toggle bits need to be toggled every 40ms,
    // and egs52 Tx interval is 20ms,
    // we can achieve this with 2 booleans
    gs_218tx.MTGL_EGS = toggle;
    gs_338tx.NAK_TGL = toggle;
    gs_418tx.FMRADTGL = toggle;
    // Now do parity calculations
    gs_218tx.MPAR_EGS = calc_torque_parity(gs_218tx.raw >> 48);
    // Includes FMRAD and FMRADTGL signal in this mask
    gs_418tx.FMRADPAR = calc_torque_parity(gs_418tx.raw & 0x47FF);
    if (time_to_toggle) {
        toggle = !toggle;
    }
    time_to_toggle = !time_to_toggle;
    
    // Now set CVN Counter (Increases every frame)
    gs_218tx.FEHLER = cvn_counter;
    cvn_counter++;

    // Now send CAN Data!

    /**
     * TX order of EGS52:
     * GS_338
     * GS_218
     * GS_418
     * GS_CUSTOM_558 ;)
     */
    //if (this->fake_rpm != 0 && this->ecu_ms.get_MS_308(esp_timer_get_time()/1000, 500, &ms308)) {
    //    ms308.set_NMOT(this->fake_rpm);
    //    tx.identifier = MS_308_CAN_ID;
    //    to_bytes(ms308.raw, tx.data);
    //    twai_transmit(&tx, 5);
    //}
    tx.identifier = GS_338_EGS52_CAN_ID;
    to_bytes(gs_338tx.raw, tx.data);
    twai_transmit(&tx, 5);
    tx.identifier = GS_218_EGS52_CAN_ID;
    to_bytes(gs_218tx.raw, tx.data);
    twai_transmit(&tx, 5);
    tx.identifier = GS_418_EGS52_CAN_ID;
    to_bytes(gs_418tx.raw, tx.data);
    twai_transmit(&tx, 5);
    //if (this->fake_rpm != 0 && this->ecu_ms.get_MS_308(esp_timer_get_time()/1000, 500, &ms308)) {
    //    ms308.set_NMOT(this->fake_rpm);
    //    tx.identifier = MS_308_CAN_ID;
    //    to_bytes(ms308.raw, tx.data);
    //    twai_transmit(&tx, 5);
    //}
}

void Egs52Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) {
    if(this->ecu_ms.import_frames(data, id, timestamp)) {
        //if (this->fake_rpm != 0 && id == MS_308_CAN_ID) {
        //    uint64_t d = (data & 0xff0000ffffffffff) | ((uint64_t)this->fake_rpm & 0xffff) << 40;
        //    twai_message_t tx;
        //    tx.data_length_code = 8;
        //    tx.identifier = MS_308_CAN_ID;
        //    to_bytes(d, tx.data);
        //    twai_transmit(&tx, 5);
        //}
    } else if (this->esp_ecu.import_frames(data, id, timestamp)) {
    } else if (this->ewm_ecu.import_frames(data, id, timestamp)) {
    } else if (this->misc_ecu.import_frames(data, id, timestamp)) {
    } else if (this->ezs_ecu.import_frames(data, id, timestamp)) {
    }
}