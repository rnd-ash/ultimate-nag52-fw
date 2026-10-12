// Real recorder, packed protocol and RLI trace dispatch; platform services are fake.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define ESP_LOG_LEVEL(...) ((void)0)
uint32_t now=0;
#define GET_CLOCK_TIME() now
bool fail_alloc=false; int allocation_calls=0;
void* alloc(size_t bytes) { ++allocation_calls; return fail_alloc ? nullptr : std::malloc(bytes); }
#define TCU_HEAP_ALLOC(n) alloc(n)
struct SensorData { uint16_t input_rpm=2000,output_rpm=1000,engine_rpm=2100;
                    int16_t input_torque=150; uint8_t pedal_pos=100; };
struct ShiftAlgoFeedback { uint16_t p_on=2000,p_off=3000; int16_t s_on=100;
    uint8_t shift_phase=2,subphase_shift=1,subphase_mod=3; };
struct { uint16_t diff_ratio=3070,wheel_circumference=1975; } VEHICLE_CONFIG;
struct MechanicalCalibration { uint16_t ratio_table[8]={0,3500,2200,1500,1000,800,3000,2000}; } mech;
MechanicalCalibration* MECH_PTR=&mech;
constexpr int SID_READ_DATA_LOCAL_IDENT=0x21,RLI_SHIFT_TRACE=0x33,
    NRC_CONDITIONS_NOT_CORRECT_REQ_SEQ_ERROR=0x22;
bool positive=false,negative=false; size_t reply_size=0; uint8_t reply[256]={};
void make_diag_neg_msg(int sid,int nrc) { assert(sid==0x21 && nrc==0x22);negative=true; }
void make_diag_pos_msg(int sid,int rli,uint8_t* data,size_t size) {
    assert(sid==0x21 && rli==0x33 && size<=sizeof(reply));
    positive=true;reply_size=size;memcpy(reply,data,size);
}
#include "production.h"
void sample(bool shifting,uint8_t phase=2) {
    SensorData sd; ShiftAlgoFeedback algo;algo.shift_phase=phase;
    now+=20;ShiftTrace::sample(&sd,&algo,shifting,3,4,4500,5000,3,120,150);
}
int main() {
    ShiftTrace::init();sample(false);
    assert(!ShiftTrace::is_enabled() && ShiftTrace::get_header()==nullptr && allocation_calls==0);
    fail_alloc=true;request_trace();
    assert(negative && !positive && !ShiftTrace::is_enabled() && allocation_calls==1);
    fail_alloc=false;negative=false;request_trace();
    assert(positive && !negative && ShiftTrace::is_enabled() && allocation_calls==2);
    assert(reply_size==sizeof(ShiftTraceHeader));
    const ShiftTraceHeader* h=ShiftTrace::get_header();
    assert(h->magic==SHIFT_TRACE_MAGIC && h->version==3 && h->sample_size==30 && h->capacity==512);
    sample(false,0);sample(true,2);sample(true,3);sample(false,4);
    assert(h->seq==4 && h->n_events==1 && h->events[0].done && h->events[0].quality.duration_ms==40);
    assert(trace_ring[2].phase==3 && trace_ring[2].trq_req_amount==120 && trace_ring[2].engine_torque==150);
    assert(trace_ring[2].spc==4500 && trace_ring[2].mpc==5000 && trace_ring[2].gear==0x34);
    for(int event=0;event<6;++event) { sample(true);sample(false); }
    assert(h->n_events==4 && h->events[3].done);
    for(int tick=0;tick<520;++tick) sample(false,5);
    const auto* last=&trace_ring[(h->seq-1)%SHIFT_TRACE_CAPACITY];
    assert(last->t_ms==now && last->phase==5);
    uint32_t seq=h->seq;ShiftTrace::set_enabled(false);sample(true);
    assert(ShiftTrace::get_header()==nullptr && h->seq==seq);
    assert(ShiftTrace::set_enabled(true) && allocation_calls==2);
    sample(true);assert(h->events[h->n_events-1].seq_start==seq);
    sample(false);assert(h->events[h->n_events-1].done);
    // Allocation is process-lifetime in firmware; release it in the host fixture.
    free(trace_ring);trace_ring=nullptr;
    puts("PASS: lazy allocation/failure, RLI wire layout, capture, completion, ring wrap and re-enable");
}
