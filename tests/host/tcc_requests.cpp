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
    TccReqState get_engine_tcc_override_request(uint32_t expiry) { assert(expiry==100); return request; }
} can;
FakeCan* egs_can_hal=&can;
struct StoredMap {
    int get_value(int, int) { return 50; }
    void add_value(int, int, int, double) {}
    void save_to_eeprom() {}
} map;
const int16_t TCC_ADAPT_MAP_X[6]={}, TCC_ADAPT_MAP_Y[6]={};
template<class T> float interpolate_linear_array(int, int, const T*, const T*) { return 0; }
#include "production.h"
TorqueConverter::TorqueConverter(uint16_t) {
    slip_rpm_target_map=&map;
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
 for (bool filling : {false,true}) {
  for (int scenario=0;scenario<5;++scenario) {
   TCC_CURRENT_SETTINGS={};
   TorqueConverter t(300); engaged(t,filling); t.pulling=true;
   can.request=scenario==0 ? TccReqState::Open : TccReqState::None;
   t.is_shifting=scenario!=0; t.upshifting=scenario==1 || scenario==2;
   sd.pedal_pos=(scenario==2 || scenario==4)?0:100;
   if(scenario==1) TCC_CURRENT_SETTINGS.unlock_load_upshifts=true;
   if(scenario==2) TCC_CURRENT_SETTINGS.unlock_coasting_upshifts=true;
   if(scenario==3) TCC_CURRENT_SETTINGS.unlock_load_downshifts=true;
   if(scenario==4) TCC_CURRENT_SETTINGS.unlock_coasting_downshifts=true;
   t.slip_target=t.calculate_slip_target(&sd);
   assert(t.slip_target>110);
   for(int i=0;i<12;++i) t.process_open_or_slip_state(&sd,GearboxGear::Third);
   assert(t.current_tcc_state==InternalTccState::Open && t.target_tcc_state==InternalTccState::Open && t.tcc_commanded_pressure==0);
  }
 }
 // Ignored requests and ordinary slip/hysteresis remain available.
 TCC_CURRENT_SETTINGS={}; TCC_CURRENT_SETTINGS.react_on_engine_open_request=false;
 can.request=TccReqState::Open; sd.pedal_pos=100;
 TorqueConverter t(300); engaged(t,false); t.pulling=true;
 t.slip_target=t.calculate_slip_target(&sd); assert(t.slip_target==50);
 t.process_open_or_slip_state(&sd,GearboxGear::Third);
 assert(t.target_tcc_state==InternalTccState::Slipping && t.tcc_commanded_pressure>0);
 can.request=TccReqState::Slipping;
 assert(t.calculate_slip_target(&sd)==60);
 t.slip_target=100; t.process_open_or_slip_state(&sd,GearboxGear::Third);
 assert(t.target_tcc_state==InternalTccState::Slipping);
 puts("PASS: engine Open and all four shift unlocks release from fill/slip; ignored Open, slip request and hysteresis preserved");
}
