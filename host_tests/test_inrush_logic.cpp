#include <cstdint>
#include <iostream>

#include "inrush_solenoid_logic.h"

namespace {

int g_failures = 0;

void expect_eq_u32(const char* name, uint32_t actual, uint32_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << expected << " actual=" << actual << "\n";
        g_failures++;
    }
}

void expect_true(const char* name, bool value) {
    if (!value) {
        std::cerr << "FAIL: " << name << " expected=true actual=false\n";
        g_failures++;
    }
}

void expect_false(const char* name, bool value) {
    if (value) {
        std::cerr << "FAIL: " << name << " expected=false actual=true\n";
        g_failures++;
    }
}

void test_hold_step_uses_remaining_time_without_underflow() {
    InrushHoldStepPlan plan = inrush_plan_hold_step(
        3000,
        7000,
        false,
        2000,
        10000
    );

    expect_eq_u32("step uses pwm_on when within remaining", plan.step_us, 3000);
    expect_eq_u32("next total increments by step", plan.next_total_us, 5000);
    expect_false("hold not completed", plan.hold_completed);
}

void test_hold_step_caps_to_remaining_time() {
    InrushHoldStepPlan plan = inrush_plan_hold_step(
        3000,
        7000,
        true,
        9500,
        10000
    );

    expect_eq_u32("step caps to remaining", plan.step_us, 500);
    expect_eq_u32("next total reaches hold", plan.next_total_us, 10000);
    expect_true("hold completed", plan.hold_completed);
}

void test_hold_already_complete_has_zero_step() {
    InrushHoldStepPlan plan = inrush_plan_hold_step(
        3000,
        7000,
        true,
        12000,
        10000
    );

    expect_eq_u32("step is zero when complete", plan.step_us, 0);
    expect_eq_u32("next total unchanged", plan.next_total_us, 12000);
    expect_true("hold completed", plan.hold_completed);
}

} // namespace

int main() {
    test_hold_step_uses_remaining_time_without_underflow();
    test_hold_step_caps_to_remaining_time();
    test_hold_already_complete_has_zero_step();

    if (g_failures == 0) {
        std::cout << "PASS: host_inrush_logic_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_inrush_logic_tests failures=" << g_failures << std::endl;
    return 1;
}
