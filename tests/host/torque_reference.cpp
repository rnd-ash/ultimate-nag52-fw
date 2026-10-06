// Actual reference method and both executor CAN-output blocks; hydraulic phases are omitted.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
struct SensorData { int16_t indicated_torque=300, converted_driver_torque=300; };
enum class TorqueRequestBounds { LessThan };
enum class TorqueRequestControlType { None, BackToDemandTorque, NormalSpeed };
struct Request { float amount=0; TorqueRequestBounds bounds{}; TorqueRequestControlType ty{}; };
struct Interface { bool trq_req_en=true; Request* ptr_w_trq_req; };
class ShiftingAlgorithm {
public:
    int16_t trq_req_reference=0;
    int16_t trq_req_reference_torque(SensorData* sd);
    SensorData* sd; Interface* sid;
    uint16_t torque_req_out=0; bool trq_req_up_ramp=false;
};
class CrossoverShift : public ShiftingAlgorithm { public: void send(); };
class ReleasingShift : public ShiftingAlgorithm { public: void send(); };
#include "production.h"
template<class Executor> void verify() {
    Executor t; SensorData sd; Request request; Interface sid{true,&request};
    t.sd=&sd; t.sid=&sid;
    for (int tick=0;tick<50;++tick) {
        t.torque_req_out=60; t.send();
        assert(request.amount==240);
        sd.indicated_torque=static_cast<int16_t>(request.amount); // simulated response, not a plant
    }
    t.torque_req_out=600; t.send(); assert(request.amount==60); // retained floor
    sd.indicated_torque=60; sd.converted_driver_torque=100;
    t.torque_req_out=60; t.send(); assert(request.amount==40); // real driver lift
    sd.indicated_torque=0; sd.converted_driver_torque=0;
    t.torque_req_out=60; t.send(); assert(request.amount==0);
    sd.indicated_torque=-30; sd.converted_driver_torque=-10;
    t.torque_req_out=60; t.send(); assert(request.amount==0);
    sid.trq_req_en=false; t.torque_req_out=60; t.send();
    assert(request.ty==TorqueRequestControlType::None && request.amount==0);
    sid.trq_req_en=true; t.torque_req_out=0; t.send();
    assert(request.ty==TorqueRequestControlType::None);
    Executor next; next.sd=&sd; next.sid=&sid;
    sd.indicated_torque=200; sd.converted_driver_torque=200;
    next.torque_req_out=40; next.trq_req_up_ramp=true; next.send();
    assert(request.amount==160 && request.ty==TorqueRequestControlType::BackToDemandTorque);
}
int main() { verify<CrossoverShift>(); verify<ReleasingShift>(); puts("PASS: repeated engine responses, demand lift, floor and request gating in both executors"); }
