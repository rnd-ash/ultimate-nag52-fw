// Actual production CAN torque decoder and recovery helper; decoded frames/clock are fake.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include "src/canbus/egs51_demand.h"
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))
uint32_t now=0;
#define GET_CLOCK_TIME() now
struct MS_310_EGS51 { uint8_t IND_TORQUE=60,MIN_TORQUE=10,MAX_TORQUE=200,DRG_TORQUE=10,MAX_TRQ_FACTOR=128; };
struct MS_210_EGS51 { uint8_t M_ESP=60; };
struct CanTorqueData { int16_t m_min=INT16_MAX,m_max=INT16_MAX,m_ind=INT16_MAX,
    m_converted_driver=INT16_MAX,m_converted_static=INT16_MAX; };
const CanTorqueData TORQUE_NDEF{};
struct Frames {
    MS_310_EGS51 torque; MS_210_EGS51 demand; bool valid=true;
    bool get_MS_310(uint32_t,uint32_t,MS_310_EGS51* result) { *result=torque;return valid; }
    bool get_MS_210(uint32_t,uint32_t,MS_210_EGS51* result) { *result=demand;return valid; }
};
class Egs51Can {
public:
    Frames ms51; struct { bool TORQUE_REQ_EN=false; } gs218;
    int16_t drag_trq=0;
    Egs51Torque::DemandCorrection demand_correction;
    CanTorqueData get_torque_data(uint32_t);
};
#include "production.h"
int main() {
    Egs51Can can;
    assert(can.get_torque_data(100).m_converted_driver==150); // learns zero offset
    can.gs218.TORQUE_REQ_EN=true;can.ms51.torque.IND_TORQUE=40;now=20;
    assert(can.get_torque_data(100).m_converted_driver==150);
    can.gs218.TORQUE_REQ_EN=false;now=80;
    assert(can.get_torque_data(100).m_converted_driver==150);
    can.gs218.TORQUE_REQ_EN=true;now=100;
    assert(can.get_torque_data(100).m_converted_driver==150); // old decoder relearns reduction, returns 90
    can.gs218.TORQUE_REQ_EN=false;now=599;
    assert(can.get_torque_data(100).m_converted_driver==150);
    now=600;can.get_torque_data(100); // exact hold boundary resumes learning
    now=620;can.gs218.TORQUE_REQ_EN=true;
    assert(can.get_torque_data(100).m_converted_driver==90);
    // Genuine demand lift and current maximum remain live.
    can.ms51.demand.M_ESP=20;can.ms51.torque.IND_TORQUE=20;now=640;
    assert(can.get_torque_data(100).m_converted_driver==30);
    can.ms51.torque.MAX_TORQUE=15;now=660;
    assert(can.get_torque_data(100).m_converted_driver<=15);
    can.ms51.valid=false;now=700;
    assert(can.get_torque_data(100).m_converted_driver==INT16_MAX);
    // Rejected active reads must not restart the recovery timer.
    Egs51Can invalid;
    invalid.get_torque_data(100);invalid.gs218.TORQUE_REQ_EN=true;now=1000;
    invalid.ms51.torque.IND_TORQUE=40;invalid.get_torque_data(100);
    invalid.ms51.valid=false;now=1499;invalid.get_torque_data(100);
    invalid.ms51.valid=true;invalid.gs218.TORQUE_REQ_EN=false;now=1500;invalid.get_torque_data(100);
    invalid.gs218.TORQUE_REQ_EN=true;now=1520;
    assert(invalid.get_torque_data(100).m_converted_driver==90);
    Egs51Torque::DemandCorrection wrap;
    assert(wrap.update(UINT32_MAX-200,true,150,150,300)==150);
    assert(wrap.update(298,false,150,90,300)==150); // 499 ms
    assert(wrap.update(299,false,150,90,300)==150); // 500 ms, relearn
    assert(wrap.update(320,true,150,90,300)==90);
    puts("PASS: request gaps, exact recovery boundary, lift/limits, invalid frames and clock wrap");
}
