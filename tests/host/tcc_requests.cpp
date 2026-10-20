// Real state processor/class; map storage, fill hydraulics and CAN are stubbed.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
enum class GearboxGear { First, Second, Third, Fourth, Fifth, Neutral };
enum class TccClutchStatus { Open };
enum class TccReqState { None, Open, Slipping };
struct SensorData { int atf_temp=50, input_rpm=1500, converted_torque=150,
                       pedal_delta_per_second=0, pedal_pos=100; };
struct PressureManager {};
struct AbstractProfile {};
struct { int engine_drag_torque=200; } VEHICLE_CONFIG;
struct Settings {
    bool adapt_enable=false, enable_d1=true, enable_d2=true, enable_d3=true,
         enable_d4=true, enable_d5=true, unlock_load_upshifts=false,
         unlock_load_downshifts=false, unlock_coasting_upshifts=false,
         unlock_coasting_downshifts=false, react_on_engine_slip_request=true,
         react_on_engine_open_request=true;
} TCC_CURRENT_SETTINGS;
struct FakeCan {
    TccReqState request=TccReqState::None;
    TccReqState get_engine_tcc_override_request(uint32_t expiry) { assert(expiry==500); return request; }
} can;
FakeCan* egs_can_hal=&can;
struct StoredMap {
    int get_value(int, int) { return 500; }
    void add_value(int, int, int, double) {}
    void save_to_eeprom() {}
} map;
const int16_t TCC_ADAPT_MAP_X[6]={}, TCC_ADAPT_MAP_Y[6]={};
template<class T> float interpolate_linear_array(int, int, const T*, const T*) { return 0; }
#include "production.h"
TorqueConverter::TorqueConverter(uint16_t) {
    tcc_adapt_map_d1=tcc_adapt_map_d2=tcc_adapt_map_d3=tcc_adapt_map_d4=tcc_adapt_map_d5=&map;
}
void TorqueConverter::calculate_min_pressure(SensorData*, GearboxGear) { min_tcc_pressure=100; }
void TorqueConverter::fill_tcc(GearboxGear, SensorData*) { ++command_p_stage; tcc_shift_pressure=700; }
void engaged(TorqueConverter& t, bool filling) {
    t.current_tcc_state=filling ? InternalTccState::Open : InternalTccState::Slipping;
    t.target_tcc_state=InternalTccState::Slipping;
    t.tcc_commanded_pressure=600; t.slip_target=50; t.actual_slip_abs=50;
    t.command_p_stage=filling ? 1 : 0;
}
void released(TorqueConverter& t, SensorData& sd, GearboxGear g) {
    for (int i=0; i<10; ++i) t.process_open_or_slip_state(&sd,g);
    assert(t.current_tcc_state==InternalTccState::Open);
    assert(t.target_tcc_state==InternalTccState::Open);
    assert(t.tcc_commanded_pressure==0);
    assert(t.command_p_stage==0);
}
int main() {
    SensorData sd;
    bool* unlocks[]={&TCC_CURRENT_SETTINGS.unlock_load_upshifts,
                    &TCC_CURRENT_SETTINGS.unlock_load_downshifts,
                    &TCC_CURRENT_SETTINGS.unlock_coasting_upshifts,
                    &TCC_CURRENT_SETTINGS.unlock_coasting_downshifts};
    for (int scenario=0;scenario<4;++scenario) {
        const bool up=scenario%2==0;
        sd.pedal_pos=scenario<2 ? 100 : 0;
        for (int selected=0;selected<4;++selected) {
            for (bool filling : {false,true}) {
                TorqueConverter t(300); engaged(t,filling); t.shift_start(up,false);
                *unlocks[selected]=true;
                if (selected==scenario) released(t,sd,GearboxGear::Third);
                else {
                    t.process_open_or_slip_state(&sd,GearboxGear::Third);
                    assert(t.target_tcc_state==InternalTccState::Slipping);
                    assert(t.tcc_commanded_pressure>0);
                }
                *unlocks[selected]=false;
            }
        }
    }
    sd.pedal_pos=100;
    TorqueConverter open(300); open.shift_start(true,false); open.slip_target=20;
    open.process_open_or_slip_state(&sd,GearboxGear::Third);
    assert(open.tcc_commanded_pressure==0 && open.command_p_stage==0);
    open.shift_end(); open.process_open_or_slip_state(&sd,GearboxGear::Third);
    assert(open.target_tcc_state==InternalTccState::Slipping && open.command_p_stage==1);
    for (bool filling : {false,true}) {
        TorqueConverter t(300); engaged(t,filling); can.request=TccReqState::Open;
        released(t,sd,GearboxGear::Third);
    }
    TorqueConverter t(300); engaged(t,false); t.slip_target=10;
    TCC_CURRENT_SETTINGS.react_on_engine_open_request=false;
    t.process_open_or_slip_state(&sd,GearboxGear::Third);
    assert(t.target_tcc_state==InternalTccState::Slipping && t.slip_target==10);
    can.request=TccReqState::Slipping;
    t.process_open_or_slip_state(&sd,GearboxGear::Third);
    assert(t.slip_target==50 && t.target_tcc_state==InternalTccState::Slipping);
    TCC_CURRENT_SETTINGS.react_on_engine_open_request=true;
    TCC_CURRENT_SETTINGS.react_on_engine_slip_request=false; t.slip_target=10;
    t.process_open_or_slip_state(&sd,GearboxGear::Third);
    assert(t.slip_target==10 && t.target_tcc_state==InternalTccState::Slipping);
    // A slip request must not become an open request when slip reaction is disabled.
    can.request=TccReqState::None; t.process_open_or_slip_state(&sd,GearboxGear::Third);
    assert(t.target_tcc_state==InternalTccState::Slipping);
    puts("PASS: four independent shift flags, fill/slip release, engine request gating");
}
