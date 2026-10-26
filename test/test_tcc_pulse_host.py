"""Run production TCC state transitions and output ownership against fake hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest
def block(source, marker):
    start = source.index(marker)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


ROOT = Path(__file__).resolve().parents[1]


def run_cpp(program):
    with tempfile.TemporaryDirectory() as directory:
        p = Path(directory)
        (p / 'test.cpp').write_text(program)
        result = subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra',
                                 '-fsanitize=address,undefined,float-cast-overflow',
                                 '-fno-sanitize-recover=all', '-pthread',
                                 '-I', str(ROOT), str(p / 'test.cpp'), '-o', str(p / 'test')],
                                capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(result.stderr)
        result = subprocess.run([str(p / 'test')], capture_output=True, text=True, timeout=15)
        if result.returncode:
            raise AssertionError(result.stderr)


class SolenoidSafetyTests(unittest.TestCase):
    def test_tcc_output_and_timer_failures(self):
        source = (ROOT / 'src/solenoids/inrush_solenoid.cpp').read_text()
        methods = '\n'.join(block(source, marker) for marker in (
            'void IRAM_ATTR InrushControlSolenoid::force_off_locked',
            'void IRAM_ATTR InrushControlSolenoid::write_legacy_duty_locked',
            'void InrushControlSolenoid::isr_disable',
            'void InrushControlSolenoid::isr_enable',
            'bool InrushControlSolenoid::is_disabled',
            'bool IRAM_ATTR InrushControlSolenoid::on_alarm',
            'void InrushControlSolenoid::__write_pwm',
            'void InrushControlSolenoid::set_duty'))
        header = '\n'.join(line for line in (ROOT / 'src/solenoids/inrush_solenoid.h').read_text().splitlines()
                           if not line.startswith('#include'))
        run_cpp(r'''
#include <cassert>
#include <initializer_list>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include <limits>
#define IRAM_ATTR
#include "src/solenoids/tcc_pulse.h"
#define portMUX_TYPE std::recursive_mutex
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(p) (p)->lock()
#define portEXIT_CRITICAL(p) (p)->unlock()
#define portENTER_CRITICAL_ISR portENTER_CRITICAL
#define portEXIT_CRITICAL_ISR portEXIT_CRITICAL
#define ESP_LOGE(...) ((void)0)
using esp_err_t=int;
constexpr int ESP_OK=0,ESP_ERR_INVALID_STATE=1,GPIO_NUM_NC=-1,LEDC_HIGH_SPEED_MODE=0;
using gpio_num_t=int;using ledc_timer_t=int;using ledc_channel_t=int;using adc_channel_t=int;
struct Timer { uint64_t now=10000, alarm=0; bool running=false, fail=false; } hw;
using gptimer_handle_t=Timer*;
struct gptimer_alarm_event_data_t { uint64_t alarm_value; };
struct gptimer_alarm_config_t { uint64_t alarm_count=0; };
unsigned ledc=0; bool pins[8]={};
int ledc_set_duty(int,int,unsigned duty){assert(duty<=4096);ledc=duty;return 0;}
int ledc_update_duty(int,int){return 0;}
int LEDC=0;constexpr int LEDC_DUTY_DIR_INCREASE=1;
void ledc_ll_set_duty_int_part(int*,int,int,unsigned duty){assert(duty<=4096);ledc=duty;}
void ledc_ll_set_duty_direction(int*,int,int,int){}
void ledc_ll_set_duty_num(int*,int,int,int){}
void ledc_ll_set_duty_cycle(int*,int,int,int){}
void ledc_ll_set_duty_scale(int*,int,int,int){}
void ledc_ll_set_sig_out_en(int*,int,int,bool){}
void ledc_ll_set_duty_start(int*,int,int,bool){}
int ledc_stop(int,int,int){ledc=0;return 0;}
void gpio_set_level(int pin,bool level){assert(pin>=0 && pin<8);pins[pin]=level;}
int gptimer_stop(Timer* t){t->running=false;return 0;}
int gptimer_start(Timer* t){t->running=true;return 0;}
int gptimer_get_raw_count(Timer* t,uint64_t* c){*c=t->now;return 0;}
int gptimer_set_alarm_action(Timer* t,gptimer_alarm_config_t* a){
    if(t->fail)return 1;
    assert(a->alarm_count>t->now);t->alarm=a->alarm_count;return 0;
}
struct PwmSolenoid {
    int channel=0; const char* name="test"; uint16_t pwm=0,pwm_raw=0;
    virtual void pre_current_test(){};virtual void post_current_test(){};
};
#define private public
''' + header + r'''
#undef private
InrushControlSolenoid::InrushControlSolenoid(const char*,int,int pin,int zener,int,int,uint16_t,uint16_t,uint16_t){
    pwm_pin=pin;zener_pin=zener;timer=&hw;
}
void InrushControlSolenoid::pre_current_test(){isr_disable();}
void InrushControlSolenoid::post_current_test(){isr_enable();}
''' + methods + r'''
int main(){
    InrushControlSolenoid gpio("test",0,1,2,0,0,100,700,10);
    // Enumerate every 12-bit command, plus oversized inputs, in both hardware modes.
    for(bool mode : {false,true}) {
        InrushControlSolenoid s("test",0,1,mode?2:-1,0,0,100,700,10);
        for(unsigned duty=0;duty<=65535;duty++) {
            s.set_duty(duty);s.__write_pwm(1,1);s.pulse.reset();
            for(int edge=0;edge<40;edge++) {
                auto e=s.pulse.next(mode);
                assert(e.ticks>0 && e.ticks<=TccPulse::period && e.duty<=4096);
                if(duty==0)assert(e.duty==0);
            }
        }
    }
    // Full hold must never produce an unbounded full-on inrush/zero interval.
    gpio.set_duty(4096);gpio.__write_pwm(1,1);
    for(int i=0;i<100;i++){auto e=gpio.pulse.next(true);assert(e.ticks==5000);assert(e.duty==(i%2?4096:0));}
    // Preserve the nominal 35% energized carrier during the hold phase.
    gpio.set_duty(2048);gpio.__write_pwm(1,1);gpio.pulse.reset();
    assert(gpio.pulse.next(true).ticks==25000);
    auto low=gpio.pulse.next(true);assert(low.duty==0 && low.ticks==6500);
    auto high=gpio.pulse.next(true);assert(high.duty==4096 && high.ticks==3500);
    // Exact legacy failure: shrink duty 1800 -> 1200 after 7000 hold
    // ticks. The old driver returned an ON edge with a zero alarm interval.
    gpio.set_duty(1800);gpio.__write_pwm(6.f/7.f,1);gpio.pulse.reset();
    assert(gpio.pulse.next(true).ticks==25000);
    auto before_shrink=gpio.pulse.next(true);
    assert(before_shrink.duty==0 && before_shrink.ticks==7000);
    gpio.set_duty(1200);
    auto after_shrink=gpio.pulse.next(true);
    assert(after_shrink.duty==4096 && after_shrink.ticks==3000);
    // Finish the original cycle, then adopt the shorter command.
    unsigned remaining_ticks=0;
    while(gpio.pulse.phase!=0) {
        auto e=gpio.pulse.next(true);assert(e.ticks>0);
        remaining_ticks+=e.ticks;
    }
    assert(25000+7000+3000+remaining_ticks==100000);
    assert(gpio.pulse.next(true).ticks==25000);
    assert(gpio.pulse.cycle.hold==4296);
    // Defensive malformed-carrier path must fail off, never return ON/0.
    gpio.pulse.cycle.off_ticks=0;
    auto malformed=gpio.pulse.next(true);
    assert(malformed.ticks>0 && malformed.duty==0);
    // Command changes cannot shrink an in-flight hold below elapsed time.
    for(unsigned i=0;i<10000;i++) {
        gpio.set_duty((i*7919)%65536);gpio.__write_pwm(i%2?0.001f:100.f,1);
        auto e=gpio.pulse.next(true);assert(e.ticks>0 && e.ticks<=100000);
    }
    gpio.__write_pwm(std::numeric_limits<float>::quiet_NaN(),1);
    assert(gpio.pulse.command.on_ticks>0 && gpio.pulse.command.off_ticks>0);
    // Resume from the initial disabled state and preserve idempotent enables.
    assert(gpio.is_disabled());
    gpio.isr_enable();gpio.isr_enable();assert(!gpio.is_disabled() && hw.running);
    gpio.set_duty(2048);gptimer_alarm_event_data_t event{hw.now-1000000};
    gpio.on_alarm(&hw,&event);assert(hw.alarm>=hw.now+1000);
    gpio.isr_disable();gpio.isr_disable();
    gpio.on_alarm(&hw,&event);assert(!pins[1] && !pins[2]);
    gpio.isr_enable();assert(!gpio.is_disabled() && hw.running);
    // Failed rearm forces off. No ISR spin waiting for hardware.
    hw.fail=true;gpio.on_alarm(&hw,&event);assert(gpio.is_disabled() && !pins[1]);hw.fail=false;
    // Legacy pause must stop the actual PWM peripheral, not just its GPIO latch.
    InrushControlSolenoid legacy("test",0,1,-1,0,0,100,700,10);
    legacy.isr_enable();legacy.set_duty(2000);legacy.on_alarm(&hw,&event);assert(ledc==4096);
    legacy.isr_disable();assert(ledc==0);
    legacy.on_alarm(&hw,&event);assert(ledc==0);
    legacy.isr_enable();legacy.set_duty(0);assert(ledc==0);
    // Exercise task/ISR sharing under real host locks.
    std::thread writer([&]{for(int i=0;i<10000;i++)gpio.set_duty(i%4097);});
    std::thread updater([&]{for(int i=0;i<10000;i++)gpio.__write_pwm(1,1);});
    writer.join();updater.join();
}
''')


if __name__ == "__main__":
    unittest.main()
