// Actual complete garage-engagement block; pressure hardware, clock and CAN are fake.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <algorithm>
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
enum class GearboxGear { First, Second, Third, Fourth, Fifth, Reverse_First, Reverse_Second, Neutral, Park };
enum class ShifterPosition { D, R, N, P };
enum class Clutch { B2, B3, K2 };
enum class GearChange { _IDLE, _1_2, _4_5 };
enum class ShiftCircuit { sc_1_2, sc_2_3, sc_3_4 };
enum class InterpType { Linear };
float interpolate_float(float x,float a,float b,float low,float high,InterpType) {
    return a+(b-a)*std::clamp((x-low)/(high-low),0.0f,1.0f);
}
float linear_ramp_with_timer(float from,float to,uint8_t timer) { return timer ? from+(to-from)/timer : to; }
struct SensorData { int output_rpm=0,input_rpm=96,pedal_pos=20,engine_rpm=700,atf_temp=50; };
struct Config {};
int calc_input_rpm_from_req_gear(int output,GearboxGear,Config*) { return output; }
bool is_fwd_gear(GearboxGear g) { return g<=GearboxGear::Fifth; }
bool is_controllable_gear(GearboxGear g) { return g<GearboxGear::Neutral; }
struct PressureManager {
    int mod=0,shift=0; bool circuits[3]={};
    int get_spring_pressure(Clutch) { return 1000; }
    int calculate_centrifugal_force_for_clutch(Clutch,int,int) { return 0; }
    int get_max_shift_pressure(int) { return 10000; }
    int get_max_solenoid_pressure() { return 15000; }
    void set_shift_circuit(ShiftCircuit c,bool on) { circuits[static_cast<int>(c)]=on; }
    void set_target_modulating_pressure(int p) { mod=p; }
    void set_target_shift_pressure(int p) { shift=p; }
    void update_pressures(GearboxGear,GearChange) {}
    void set_spc_p_max() { shift=15000; }
} pm;
PressureManager* pressure_manager=&pm;
namespace ShiftHelpers { int correct_shift_shift_pressure(PressureManager*,int p,int) { return p; } }
struct Can { bool garage=false; void set_garage_shift_state(bool active,bool) { garage=active; } } can;
Can* egs_can_hal=&can;
class Gearbox {
public:
    SensorData sensor_data; GearboxGear target_gear=GearboxGear::Second,result=GearboxGear::Neutral;
    ShifterPosition shifter_pos=ShifterPosition::D;
    Config gearboxConfig; PressureManager* pressure_mgr=&pm; bool tcu_restarted=false;
    struct { bool active=false; int p_off=0,p_on=0,s_off=0,s_on=0,shift_phase=0,sync_rpm=0; } algo_feedback;
    void run();
};
Gearbox* active=nullptr;
int ticks=0,cancel_after=0;
void vTaskDelay(int ms) {
    assert(ms==20);
    if (++ticks>1500) throw std::runtime_error("garage retry did not terminate");
    if (cancel_after && ticks==cancel_after) active->shifter_pos=ShifterPosition::P;
}
#include "production.h"
void run(Gearbox& g) { active=&g; ticks=0; pm=PressureManager{}; can.garage=false; g.run();
    assert(!can.garage); for (bool circuit:pm.circuits) assert(!circuit);
}
int main() {
    for (int rpm:{34,96,349}) {
        Gearbox g; g.sensor_data.input_rpm=rpm; run(g);
        assert(g.result==GearboxGear::Second && ticks<200);
    }
    Gearbox reverse; reverse.target_gear=GearboxGear::Reverse_First;
    reverse.shifter_pos=ShifterPosition::R; reverse.sensor_data.pedal_pos=0;
    reverse.sensor_data.input_rpm=34; run(reverse); assert(reverse.result==GearboxGear::Reverse_First);
    Gearbox fail; fail.sensor_data.input_rpm=600; run(fail);
    assert(fail.result==GearboxGear::Neutral && ticks<700);
    Gearbox cancel; cancel_after=5; run(cancel); cancel_after=0;
    assert(cancel.result==GearboxGear::Park && ticks==5);
    puts("PASS: standstill turbine drag, reverse, failed engagement retry, cancellation and cleanup");
}
