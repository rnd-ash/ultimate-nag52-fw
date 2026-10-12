// Actual state/torque/output methods; fake CAN and solenoid writes.
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include "src/models/egs51_engine_rpm.h"
enum class TorqueRequestControlType {None,Reduction};
enum class TorqueRequestBounds {LessThan};
enum class TccClutchStatus {Open};
enum class ShiftCircuit {sc_1_2,sc_2_3,sc_3_4};
enum class GearChange {_IDLE};
struct Solenoid {int current=500,duty=500; void set_current_target(int x){current=x;} void set_duty(int x){duty=x;}} mpc,spc,tcc;
Solenoid *sol_mpc=&mpc,*sol_spc=&spc,*sol_tcc=&tcc;
struct Can {TorqueRequestControlType request=TorqueRequestControlType::Reduction; float amount=200;
 void set_torque_request(TorqueRequestControlType x,TorqueRequestBounds,float v){request=x;amount=v;}
 void set_clutch_status(TccClutchStatus){}
} can;
Can* egs_can_hal=&can;
struct Pressure {int tcc=600,updates=0,circuits=0; void set_target_tcc_pressure(int x){tcc=x;}
 void set_shift_circuit(ShiftCircuit,bool){++circuits;} void update_pressures(int,GearChange){++updates;}
} pressure;
struct Gearbox {
 std::atomic<bool> engine_running{false}, pedal_input_valid{true}; bool engine_rpm_valid=false;
 struct {uint16_t engine_rpm=0,input_rpm=200,output_rpm=0;} sensor_data;
 struct {float torque_req_amount=0; TorqueRequestControlType ctrl_type=TorqueRequestControlType::None; TorqueRequestBounds bounds=TorqueRequestBounds::LessThan;} output_data;
 int actual_gear=3,target_gear=3; float tcc_percent=20; Pressure* pressure_mgr=&pressure;
 void update_engine_state(uint16_t); void set_torque_request(TorqueRequestControlType,TorqueRequestBounds,float);
 void stopped_outputs(bool,bool); void idle_outputs(bool);
};
#include "production.h"
int main(){
 Gearbox g;
 for(int i=0;i<128;++i){g.update_engine_state(UINT16_MAX);assert(!g.engine_running && !g.engine_rpm_valid && g.sensor_data.engine_rpm==750);}
 assert(g.actual_gear==3 && g.target_gear==3);
 g.stopped_outputs(true,false); assert(mpc.current==500 && spc.current==500 && pressure.tcc==0 && tcc.duty==0);
 g.set_torque_request(TorqueRequestControlType::Reduction,TorqueRequestBounds::LessThan,200);
 assert(can.request==TorqueRequestControlType::None && can.amount==0);
 g.update_engine_state(800);assert(g.engine_running && g.engine_rpm_valid);
 g.set_torque_request(TorqueRequestControlType::Reduction,TorqueRequestBounds::LessThan,200);assert(can.amount==200);
 g.update_engine_state(UINT16_MAX);assert(!g.engine_running && g.actual_gear==3 && can.amount==0);
 g.update_engine_state(450);assert(g.engine_running);g.update_engine_state(449);assert(!g.engine_running);
 g.stopped_outputs(true,false);assert(mpc.current==500); // Low RPM alone is not shutdown.
 g.update_engine_state(0);g.sensor_data.output_rpm=20;g.stopped_outputs(true,false);assert(mpc.current==500);
 g.sensor_data.output_rpm=0;g.stopped_outputs(false,false);assert(mpc.current==500);
 g.stopped_outputs(true,true);assert(mpc.current==500);
 g.stopped_outputs(true,false);assert(mpc.current==0 && spc.current==0 && pressure.circuits==3);
 g.idle_outputs(false);assert(pressure.updates==0); // Do not overwrite confirmed shutdown.
 g.update_engine_state(UINT16_MAX);g.idle_outputs(false);assert(pressure.updates==1);
 g.idle_outputs(true);assert(pressure.updates==1); // Active shift owns hydraulics.
 puts("PASS: missing/recovered/stopped RPM, gear preservation, torque suppression and guarded pressure shutdown");
}
