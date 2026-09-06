#include "can_egs51.h"

#include "driver/twai.h"
#include "driver/i2c_master.h"
#include "board_config.h"
#include "nvs/eeprom_config.h"
#include "shifter/shifter_trrs.h"
#include "shifter/shifter_ewm.h"
#include "can_egs51_logic.h"
#include "tcu_maths.h"

Egs51Can::Egs51Can(const char *name, uint8_t tx_time_ms, uint32_t baud, Shifter *shifter) : EgsBaseCan(name, tx_time_ms, baud, shifter) 
{
    ESP_LOGI("EGS51", "SETUP CALLED");
    this->gs218.TORQUE_REQ = 0xFE;
    this->gs218.bytes[7] = 0xFE;
    this->gs218.bytes[4] = 0x48;
    this->gs218.bytes[3] = 0x64;
}

wheel_rpm_2x_t Egs51Can::get_front_right_wheel(const uint32_t expire_time_ms)
{
    BS_200_EGS51 bs200;
    if (this->esp51.get_BS_200(GET_CLOCK_TIME(), expire_time_ms, &bs200)) {
        return WheelSpeed::from_raw_2x(egs51_decode_wheel_speed_or_sna(bs200.DVR));
    }
    return WheelSpeed::INVALID;
}

wheel_rpm_2x_t Egs51Can::get_front_left_wheel(const uint32_t expire_time_ms) {
    BS_200_EGS51 bs200;
    if (this->esp51.get_BS_200(GET_CLOCK_TIME(), expire_time_ms, &bs200)) {
        return WheelSpeed::from_raw_2x(egs51_decode_wheel_speed_or_sna(bs200.DVL));
    }
    return WheelSpeed::INVALID;
}

wheel_rpm_2x_t Egs51Can::get_rear_right_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    wheel_rpm_2x_t ret = WheelSpeed::INVALID;
    if (this->esp51.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (0x3FFF != bs208.DHR) {
            ret = WheelSpeed::from_raw_2x(bs208.DHR);
        }
        
    }
    return ret;
}

wheel_rpm_2x_t Egs51Can::get_rear_left_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    wheel_rpm_2x_t ret = WheelSpeed::INVALID;
    if (this->esp51.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (0x3FFF != bs208.DHL) {
            ret = WheelSpeed::from_raw_2x(bs208.DHL);
        }
        
    }
    return ret;
}

EngineType Egs51Can::get_engine_type(const uint32_t expire_time_ms) {
    return EngineType::Unknown;
}

bool Egs51Can::get_engine_is_limp(const uint32_t expire_time_ms) { // TODO
    MS_308_EGS51 ms308;
    if (this->ms51.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        return egs51_infer_engine_limp(ms308.TEMP_KL, ms308.UEHITZ, ms308.DIAG_KL);
    }
    return false;
}

bool Egs51Can::get_kickdown(const uint32_t expire_time_ms) {
    bool ret  = false;
    // Only for the CAN shifter
    EWM_230_EGS51 dest;
	if (this->ewm.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest)){
        ret  = dest.KD;
    }
    return ret;
}

pedal_pos_t Egs51Can::get_pedal_value(const uint32_t expire_time_ms) {
    MS_210_EGS51 ms210;
    if (this->ms51.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        return Pedal::from_raw(ms210.PW);
    } else {
        return Pedal::INVALID;
    }
}

CanTorqueData Egs51Can::get_torque_data(const uint32_t expire_time_ms) {
    CanTorqueData ret = TORQUE_NDEF;
    MS_310_EGS51 ms310;
    MS_210_EGS51 ms210;
    // EGS51 does not use the (Nm + 500) * 4 encoding the other buses do - MS_310
    // carries a single unsigned byte at 3 Nm per bit, so there is no offset and
    // no negative range. Decode it here rather than through Torque::from_can_raw().
    static constexpr int16_t EGS51_NM_PER_BIT = 3;
    // Was declared uint16_t while holding a signed torque compared against
    // m_min, which can be negative on other buses. Nm are signed.
    int16_t m_esp = INT16_MAX;
    int16_t m_ind = INT16_MAX;
    int16_t m_min = INT16_MAX;
    int16_t m_max = INT16_MAX;
    if (this->ms51.get_MS_310(GET_CLOCK_TIME(), expire_time_ms, &ms310) &&
        this->ms51.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        if (UINT8_MAX != ms310.IND_TORQUE) {
            m_ind = ((int16_t)ms310.IND_TORQUE) * EGS51_NM_PER_BIT;
        }
        if (UINT8_MAX != ms310.MIN_TORQUE) {
            m_min = ((int16_t)ms310.MIN_TORQUE) * EGS51_NM_PER_BIT;
        }
        if (UINT8_MAX != ms310.MAX_TORQUE) {
            m_max = ((int16_t)ms310.MAX_TORQUE) * EGS51_NM_PER_BIT;
            m_max = egs51_apply_max_torque_factor(m_max, ms310.MAX_TRQ_FACTOR);
        }
        if (UINT8_MAX != ms210.M_ESP) {
            m_esp = ((int16_t)ms210.M_ESP) * EGS51_NM_PER_BIT;
        }
        ret.m_ind = Torque::from_nm(m_ind);
        ret.m_min = Torque::from_nm(m_min);
        ret.m_max = Torque::from_nm(m_max);
    }
    if (
        INT16_MAX != m_min &&
        INT16_MAX != m_max &&
        INT16_MAX != m_ind &&
        INT16_MAX != m_esp
    ) {
        m_ind = MIN(m_ind, m_max); // Limit indicated torque to max torque
        m_ind = MAX(m_min, m_ind); // Floor indicated torque to min torque

        m_esp = MIN(m_esp, m_max); // Limit ESP torque to max torque
        m_esp = MAX(m_min, m_esp); // Floor ESP torque to min torque

        int16_t converted_torque = m_ind;
        int16_t static_converted = converted_torque;

        if (m_esp > converted_torque) {
            converted_torque = m_esp;
        }

        bool freeze = this->gs218.TORQUE_REQ_EN;
        Egs51FreezeResult freeze_result = egs51_apply_freeze_logic(
            freeze,
            converted_torque,
            static_converted,
            this->req_static_torque_delta
        );
        ret.m_ind = Torque::from_nm(m_ind);
        // Preserve freeze-adjusted driver torque instead of overwriting it later.
        ret.m_converted_driver = Torque::from_nm(freeze_result.driver_converted);
        this->req_static_torque_delta = freeze_result.req_static_torque_delta;
        ret.m_converted_static = Torque::from_nm(static_converted);
    }
    return ret;
}

PaddlePosition Egs51Can::get_paddle_position(const uint32_t expire_time_ms) {
    return PaddlePosition::SNV;
}

temp_c_t Egs51Can::get_engine_coolant_temp(const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    temp_c_t res = Temp::INVALID;
    if (this->ms51.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_MOT != UINT8_MAX) {
            res = Temp::from_can_u8_offset40(ms608.T_MOT);
        }
    }
    return res;
}

temp_c_t Egs51Can::get_engine_oil_temp(const uint32_t expire_time_ms) {
    MS_308_EGS51 ms308;
    temp_c_t res = Temp::INVALID;
    if (this->ms51.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        if (ms308.T_OEL != UINT8_MAX) {
            res = Temp::from_can_u8_offset40(ms308.T_OEL);
        }
    }
    return res;
}

temp_c_t Egs51Can::get_engine_iat_temp(const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    temp_c_t res = Temp::INVALID;
    if (this->ms51.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_LUFT != UINT8_MAX) {
            res = Temp::from_can_u8_offset40(ms608.T_LUFT);
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

bool Egs51Can::get_is_starting(const uint32_t expire_time_ms) {
    MS_308_EGS51 ms308;
    if (this->ms51.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        return ms308.ANL_LFT;
    }
    return false;
}

bool Egs51Can::get_is_brake_pressed(const uint32_t expire_time_ms) {
    BS_200_EGS51 bs200;
    if (this->esp51.get_BS_200(GET_CLOCK_TIME(), expire_time_ms, &bs200)) {
        return bs200.BLS == BS_200h_BLS_EGS51::BREMSE_BET;
    }
    return false;
}

bool Egs51Can::get_profile_btn_press(const uint32_t expire_time_ms) {
    EWM_230_EGS51 ewm_data;
    if (this->ewm.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &ewm_data)) {
        return ewm_data.FPT;
    }
    return false;
}

ProfileSwitchPos Egs51Can::get_profile_switch_pos(const uint32_t expire_time_ms) {
    EWM_230_EGS51 ewm_data;
    if (this->ewm.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &ewm_data)) {
        return ewm_data.W_S ? ProfileSwitchPos::Top : ProfileSwitchPos::Bottom;
    }
    return ProfileSwitchPos::SNV;
}

uint16_t Egs51Can::get_fuel_flow_rate(const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    if (this->ms51.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        return TCU_ROUND_TO_U16_SAT((float)ms608.VB * 0.868f);
    } else {
        return 0;
    }
}

void Egs51Can::set_clutch_status(TccClutchStatus status) {
    switch(status) {
        case TccClutchStatus::Open:
            gs218.TCC_CLOSED = false;
            gs218.TCC_OPEN = true;
            gs218.TCC_SLIPPING = false;
            break;
        case TccClutchStatus::OpenToSlipping:
            gs218.TCC_CLOSED = false;
            gs218.TCC_OPEN = true;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::Slipping:
            gs218.TCC_CLOSED = false;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::SlippingToClosed:
            gs218.TCC_CLOSED = true;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::Closed:
            gs218.TCC_CLOSED = true;
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
    EWM_230_EGS51 dest;
	if (this->ewm.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest))
	{
		switch (dest.WHC)
		{
        case EWM_230h_WHC_EGS51::D:
			ret = ShifterPosition::D;
			break;
        case EWM_230h_WHC_EGS51::N:
			ret = ShifterPosition::N;
			break;
        case EWM_230h_WHC_EGS51::R:
			ret = ShifterPosition::R;
			break;
        case EWM_230h_WHC_EGS51::P:
			ret = ShifterPosition::P;
			break;
        case EWM_230h_WHC_EGS51::PLUS:
			ret = ShifterPosition::PLUS;
			break;
        case EWM_230h_WHC_EGS51::MINUS:
			ret = ShifterPosition::MINUS;
			break;
        case EWM_230h_WHC_EGS51::N_ZW_D:
			ret = ShifterPosition::N_D;
			break;
        case EWM_230h_WHC_EGS51::R_ZW_N:
			ret = ShifterPosition::R_N;
			break;
        case EWM_230h_WHC_EGS51::P_ZW_R:
			ret = ShifterPosition::P_R;
			break;
        case EWM_230h_WHC_EGS51::SNV:
			break;
		default:
			break;
		}
	}
	return ret;
}

void Egs51Can::set_gearbox_temperature(temp_c_t temp) {
}

void Egs51Can::set_input_shaft_speed(uint16_t rpm) {
}

void Egs51Can::set_is_all_wheel_drive(bool is_4wd) {
    this->gs218.FWD = !is_4wd;
}

void Egs51Can::set_wheel_torque(uint16_t t) {
}

void Egs51Can::set_shifter_position(ShifterPosition pos) {
    if (ShifterPosition::N == pos || ShifterPosition::P == pos) {
        this->gs218.PN = true;
    } else {
        this->gs218.PN = false;
    }
}

void Egs51Can::set_gearbox_ok(bool is_ok) {
    this->gs218.GB_OK = is_ok;
    this->gs218.LIMP_MODE = !is_ok;
}

void Egs51Can::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    (void)limit_type;
    if (control_type == TorqueRequestControlType::None) {
        this->gs218.TORQUE_REQ_EN = false;
        this->gs218.SE = false;
        this->gs218.TORQUE_REQ = 0xFE;
    } else {
        // Just enable the request
        this->gs218.TORQUE_REQ_EN = true;
        this->gs218.SE = true;
        this->gs218.TORQUE_REQ = egs51_torque_request_to_raw(amount_nm);
    }
}

void Egs51Can::set_garage_shift_state(bool enable, bool to_d) {
    (void)to_d;
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
    this->gs218.CAN_START = can_start;
    if (ioexpander) { // Do this in CAN HAL - When Gearbox commands it
        ioexpander->set_start(can_start);
    }
}

void Egs51Can::set_tcc_trq_multiplier(float multi) {
    gs218.TCC_MULTI = egs51_tcc_multiplier_to_raw(multi);
}

void Egs51Can::tx_frames() {
    tx.data_length_code = 6;
    GS_218_EGS51 gs_218tx;
    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    gs_218tx = {gs218.raw};
    gs_218tx.KICKDOWN = get_kickdown(300);
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