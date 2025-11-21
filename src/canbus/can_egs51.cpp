#include "can_egs51.h"

#include "driver/twai.h"
#include "driver/i2c_master.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "shifter/shifter_trrs.h"
#include "shifter/shifter_ewm.h"

Egs51Can::Egs51Can(const char *name, uint8_t tx_time_ms, uint32_t baud, Shifter *shifter) : EgsBaseCan(name, tx_time_ms, baud, shifter) 
{
    ESP_LOGI("EGS51", "SETUP CALLED");
    this->gs218.TORQUE_REQ = 0xFE;
    this->gs218.bytes[7] = 0xFE;
    this->gs218.bytes[4] = 0x48;
    this->gs218.bytes[3] = 0x64;
}

uint16_t Egs51Can::get_front_right_wheel(const uint32_t expire_time_ms)
{
	return UINT16_MAX;
}

uint16_t Egs51Can::get_front_left_wheel(const uint32_t expire_time_ms) {
    return UINT16_MAX;
}

uint16_t Egs51Can::get_rear_right_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    uint16_t ret = UINT16_MAX;
    if (this->esp51.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (BS_208h_DRTGHR_EGS51::SNV != bs208.DRTGHR) {
            ret = bs208.DHR;
        }
        
    }
    return ret;
}

uint16_t Egs51Can::get_rear_left_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    uint16_t ret = UINT16_MAX;
    if (this->esp51.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (BS_208h_DRTGHL_EGS51::SNV != bs208.DRTGHL) {
            ret = bs208.DHL;
        }
        
    }
    return ret;
}

EngineType Egs51Can::get_engine_type(const uint32_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs51Can::get_engine_is_limp(const uint32_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_kickdown(const uint32_t expire_time_ms) {
    bool ret  = false;
    // Only for the CAN shifter
    EWM_230_EGS52 dest;
	if (this->ewm.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest)){
        ret  = dest.KD;
    }
    return ret;
}

uint8_t Egs51Can::get_pedal_value(const uint32_t expire_time_ms) { // TODO
    MS_210_EGS51 ms210;
    if (this->ms51.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        return ms210.PW;
    } else {
        return 0xFF;
    }
}

CanTorqueData Egs51Can::get_torque_data(const uint32_t expire_time_ms) {
    CanTorqueData ret = TORQUE_NDEF;
    MS_310_EGS51 ms310;
    MS_210_EGS51 ms210;
    uint16_t m_esp = INT16_MAX;
    if (this->ms51.get_MS_310(GET_CLOCK_TIME(), expire_time_ms, &ms310) &&
        this->ms51.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        if (UINT8_MAX != ms310.IND_TORQUE) {
            ret.m_ind = ((int16_t)ms310.IND_TORQUE)*3;
        }
        if (UINT8_MAX != ms310.MIN_TORQUE) {
            ret.m_min = ((int16_t)ms310.MIN_TORQUE)*3;
        }
        if (UINT8_MAX != ms310.MAX_TORQUE) {
            ret.m_max = ((int16_t)ms310.MAX_TORQUE)*3;
            // TODO -> ms310.MAX_TRQ_FACTOR
        }
        if (UINT8_MAX != ms210.M_ESP) {
            m_esp = ((int16_t)ms210.M_ESP)*3;
        }
    }
    if (
        INT16_MAX != ret.m_min &&
        INT16_MAX != ret.m_max &&
        INT16_MAX != ret.m_ind &&
        INT16_MAX != m_esp
    ) {
        ret.m_ind = MIN(ret.m_ind, ret.m_max); // Limit indicated torque to max torque
        ret.m_ind = MAX(ret.m_min, ret.m_ind); // Floor indicated torque to min torque

        m_esp = MIN(m_esp, ret.m_max); // Limit ESP torque to max torque
        m_esp = MAX(ret.m_min, m_esp); // Floor ESP torque to min torque

        int16_t driver_converted = ret.m_ind;
        int16_t static_converted = ret.m_ind;

        if (m_esp > ret.m_ind) {
            driver_converted = m_esp;
        }

        bool active_shift = (uint8_t)this->gs218.GIC != (uint8_t)this->gs218.GZC;
        bool trq_req_en = this->gs218.TORQUE_REQ != 0xFE;
        if (active_shift && trq_req_en) {
            this->freeze_torque = true; // Gear shift and we have started a torque request, freeze it
        } else if (!active_shift) {
            this->freeze_torque = false; // No gear shift, unfreeze it
        }
        // Change torque values based on freezing or not
        if (this->freeze_torque) {
            ret.m_converted_driver = MAX(driver_converted - this->req_static_torque_delta, static_converted);
        } else {
            this->req_static_torque_delta = driver_converted - static_converted;
        }

        ret.m_converted_driver = driver_converted;
        ret.m_converted_static = static_converted;
    }
    return ret;
}

PaddlePosition Egs51Can::get_paddle_position(const uint32_t expire_time_ms) {
    return PaddlePosition::SNV;
}

int16_t Egs51Can::get_engine_coolant_temp(const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    int16_t res = INT16_MAX;
    if (this->ms51.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_MOT != UINT8_MAX) {
            res = ms608.T_MOT - 40;
        }
    }
    return res;
}

int16_t Egs51Can::get_engine_oil_temp(const uint32_t expire_time_ms) {
    MS_308_EGS51 ms308;
    int16_t res = INT16_MAX;
    if (this->ms51.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        if (ms308.T_OEL != UINT8_MAX) {
            res = ms308.T_OEL - 40;
        }
    }
    return res;
}

int16_t Egs51Can::get_engine_iat_temp(const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    int16_t res = INT16_MAX;
    if (this->ms51.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_LUFT != UINT8_MAX) {
            res = ms608.T_LUFT - 40;
        }
    }
    return res;
}

uint16_t Egs51Can::get_engine_rpm(const uint32_t expire_time_ms) {
    MS_308_EGS51 ms308;
    if (this->ms51.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        return ms308.NMOT;
    } else {
        return UINT16_MAX;
    }
}

bool Egs51Can::get_is_starting(const uint32_t expire_time_ms) { // TODO
    return false;
}

bool Egs51Can::get_is_brake_pressed(const uint32_t expire_time_ms) {
    return false;
}

bool Egs51Can::get_profile_btn_press(const uint32_t expire_time_ms) {
    return false;
}

uint16_t Egs51Can::get_fuel_flow_rate(const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    if (this->ms51.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        return (uint16_t)((float)ms608.VB*0.868);
    } else {
        return 0;
    }
}

void Egs51Can::set_clutch_status(TccClutchStatus status) {
    switch(status) {
        case TccClutchStatus::Open:
            gs218.TCC_SHUT = false;
            gs218.TCC_OPEN = true;
            gs218.TCC_SLIPPING = false;
            break;
        case TccClutchStatus::OpenToSlipping:
            gs218.TCC_SHUT = false;
            gs218.TCC_OPEN = true;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::Slipping:
            gs218.TCC_SHUT = false;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::SlippingToClosed:
            gs218.TCC_SHUT = true;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::Closed:
            gs218.TCC_SHUT = true;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = false;
            break;
        default:
            break;
    }
}

void Egs51Can::set_actual_gear(GearboxGear actual) {
    switch(actual) {
        case GearboxGear::First:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D5;
            break;
        case GearboxGear::Park:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_P;
            break;
        case GearboxGear::Neutral:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_N;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_SNV;
            break;
    }
}

void Egs51Can::set_target_gear(GearboxGear target) {
    switch(target) {
        case GearboxGear::First:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D5;
            break;
        case GearboxGear::Park:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_P;
            break;
        case GearboxGear::Neutral:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_N;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_SNV;
            break;
    }
}

ShifterPosition Egs51Can::internal_can_shifter_get_shifter_position(const uint32_t expire_time_ms) {
	ShifterPosition ret = ShifterPosition::SignalNotAvailable;
	EWM_230_EGS52 dest;
	if (this->ewm.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest))
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

void Egs51Can::set_gearbox_temperature(int16_t temp) {
}

void Egs51Can::set_input_shaft_speed(uint16_t rpm) {
}

void Egs51Can::set_is_all_wheel_drive(bool is_4wd) {
}

void Egs51Can::set_wheel_torque(uint16_t t) {
}

void Egs51Can::set_shifter_position(ShifterPosition pos) {
    if (ShifterPosition::N == pos || ShifterPosition::P == pos) {
        this->gs218.NEUTRAL = true;
    } else {
        this->gs218.NEUTRAL = false;
    }
}

void Egs51Can::set_gearbox_ok(bool is_ok) {
    this->gs218.GEARBOX_OK = is_ok;
    this->gs218.LIMP_MODE = !is_ok;
}

void Egs51Can::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    if (control_type == TorqueRequestControlType::None) {
        this->gs218.TORQUE_REQ_EN = false;
        this->gs218.SE = false;
        this->gs218.TORQUE_REQ = 0xFE;
    } else {
        // Just enable the request
        this->gs218.TORQUE_REQ_EN = true;
        this->gs218.SE = true;
        this->gs218.TORQUE_REQ = amount_nm/3;
    }
}

void Egs51Can::set_garage_shift_state(bool enable) {
    this->gs218.GARAGE_SHIFT = enable;
}

void Egs51Can::set_error_check_status(SystemStatusCheck ssc) {
}


void Egs51Can::set_turbine_torque_loss(uint16_t loss_nm) {
}

void Egs51Can::set_display_gear(GearboxDisplayGear g, bool manual_mode) {
}

void Egs51Can::set_drive_profile(GearboxProfile p) {
}

void Egs51Can::set_display_msg(GearboxMessage msg) {
}

void Egs51Can::set_wheel_torque_multi_factor(float ratio) {
}

void Egs51Can::set_safe_start(bool can_start) {
    if (ioexpander) { // Do this in CAN HAL - When Gearbox commands it
        ioexpander->set_start(can_start);
    }
}

void Egs51Can::tx_frames() {
    tx.data_length_code = 6;
    GS_218_EGS51 gs_218tx;
    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    gs_218tx = {gs218.raw};
    // Now set CVN Counter (Increases every frame)
    gs_218tx.FEHLER = cvn_counter;
    cvn_counter++;
    tx.identifier = GS_218_EGS51_CAN_ID;
    to_bytes(gs_218tx.raw, tx.data);
    twai_transmit(&tx, 5);
}

void Egs51Can::on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) {
    if (this->ms51.import_frames(data, id, timestamp)) {
    } else if (this->esp51.import_frames(data, id, timestamp)) {
    } else if (this->ewm.import_frames(data, id, timestamp)) {
    }
}