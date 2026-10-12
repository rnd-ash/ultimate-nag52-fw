"""Execute pedal fault qualification and production fault/recovery gates."""
from pathlib import Path
import unittest
from owner_port_test import compile_run
from owner_port_test import block

ROOT = Path(__file__).resolve().parents[2]


class PedalTests(unittest.TestCase):
    def test_complete_can_domain_and_explicit_fault(self):
        compile_run(r'''
#include "models/egs51_pedal.h"
#include <cassert>
int main() {
 for (int raw=0; raw<256; ++raw) {
  auto sample=Egs51Pedal::qualify(raw);
  assert(sample.valid==(raw<=250));
  assert(sample.pedal==(raw<=250 ? raw : 64));
  sample=Egs51Pedal::qualify(raw,true);
  assert(!sample.valid && sample.pedal==64);
 }
}
''')

    def test_loss_recovery_pending_and_active_shift_ownership(self):
        source = (ROOT / 'src/gearbox.cpp').read_text()
        methods = '\n'.join(block(source, marker) for marker in (
            'void Gearbox::update_pedal_state(', 'void Gearbox::set_torque_request('))
        # Run the actual new guard, separately from map decisions that need a
        # full profile. The target/torque methods above are complete methods.
        gate_start = source.index('if (this->pedal_input_valid && speeds_valid && is_fwd_gear(')
        gate_end = source.index('\n', gate_start)
        decision_condition = source[gate_start:gate_end].strip()[4:-1]
        spawn_guard = block(source, 'if (this->target_gear != this->actual_gear && !this->shifting &&')
        compile_run(r'''
#include "models/egs51_pedal.h"
#include <atomic>
#include <cassert>
enum class GearboxGear {Park, First, Second, Third, Fourth, Fifth, Neutral};
bool is_fwd_gear(GearboxGear g) {return g>=GearboxGear::First && g<=GearboxGear::Fifth;}
enum class TorqueRequestControlType {None,Active};
enum class TorqueRequestBounds {LessThan};
namespace DownshiftObserver {enum class State {Unavailable,NoRequest};}
struct CAN {
 TorqueRequestControlType ty=TorqueRequestControlType::None;float amount=0;
 void set_torque_request(TorqueRequestControlType t,TorqueRequestBounds,float a) {ty=t;amount=a;}
} can;
auto egs_can_hal=&can;
struct Gearbox;
void xTaskCreatePinnedToCore(void(*)(),const char*,int,Gearbox*,int,int*,int);
struct Gearbox {
 std::atomic<bool> engine_running{true},pedal_input_valid{false};
 GearboxGear actual_gear=GearboxGear::Second,target_gear=GearboxGear::Third;
 bool shifting=false,agility_inputs_valid=true;
 bool ask_upshift=true,ask_downshift=true,manual_shift=true;
 unsigned pedal_last=0;
 struct {unsigned pedal_pos=150;bool kickdown_pressed=true;} sensor_data;
 struct {float torque_req_amount;TorqueRequestControlType ctrl_type;TorqueRequestBounds bounds;} output_data;
 void update_pedal_state(uint8_t);
 void set_torque_request(TorqueRequestControlType,TorqueRequestBounds,float);
 bool may_consult_maps(bool speeds_valid) {
 return '''+decision_condition+r''';
 }
 bool spawned=false;int shift_task=0;
 static void start_shift_thread() {}
 void maybe_spawn() {
'''+spawn_guard+r'''
 }
};
void xTaskCreatePinnedToCore(void(*)(),const char*,int, Gearbox* g,int,int*,int) {g->spawned=true;}
'''+methods+r'''
int main() {
 Gearbox g; g.update_pedal_state(150);
 assert(g.pedal_input_valid && g.sensor_data.pedal_pos==150 && g.pedal_last==150);
 g.set_torque_request(TorqueRequestControlType::Active,TorqueRequestBounds::LessThan,100);
 assert(can.ty==TorqueRequestControlType::Active);
 g.update_pedal_state(255);
 assert(!g.pedal_input_valid && g.sensor_data.pedal_pos==64);
 assert(!g.sensor_data.kickdown_pressed);
 assert(!g.ask_upshift && !g.ask_downshift && !g.manual_shift);
 assert(g.target_gear==g.actual_gear && g.engine_running);
 assert(can.ty==TorqueRequestControlType::None && can.amount==0);
 // Simulate a pending road request arriving after the fault was processed.
 g.target_gear=GearboxGear::Third;g.maybe_spawn();assert(!g.spawned);
 assert(!g.may_consult_maps(true));
 for(int i=0;i<100;++i) {
  g.set_torque_request(TorqueRequestControlType::Active,TorqueRequestBounds::LessThan,200);
  assert(can.ty==TorqueRequestControlType::None && can.amount==0);
 }
 // An active executor retains its target and running/pressure ownership.
 g.shifting=true;g.update_pedal_state(255);
 assert(g.target_gear==GearboxGear::Third && g.shifting && g.engine_running);
 g.update_pedal_state(0);assert(g.pedal_input_valid && g.sensor_data.pedal_pos==0);
 assert(g.may_consult_maps(true) && !g.may_consult_maps(false));
 g.set_torque_request(TorqueRequestControlType::Active,TorqueRequestBounds::LessThan,80);
 assert(can.ty==TorqueRequestControlType::Active && can.amount==80);
 // Pedal loss does not prevent range/garage requests.
 g.shifting=false;g.target_gear=GearboxGear::Neutral;g.update_pedal_state(255);
 assert(g.target_gear==GearboxGear::Neutral);g.maybe_spawn();assert(g.spawned);
 g.spawned=false;g.actual_gear=GearboxGear::Neutral;g.target_gear=GearboxGear::Second;
 g.update_pedal_state(255);g.maybe_spawn();assert(g.spawned);
}
''')
