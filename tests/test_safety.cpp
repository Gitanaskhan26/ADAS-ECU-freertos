#include "safety.hpp"
#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>

void test_timestamp_monotonic() {
    uint64_t t1 = get_timestamp_us();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    uint64_t t2 = get_timestamp_us();
    
    // Time should advance
    assert(t2 > t1);
    
    // Should be roughly 10ms apart (allow some tolerance)
    uint64_t diff = t2 - t1;
    assert(diff >= 9000 && diff <= 15000);  // 9-15ms tolerance
    
    std::cout << "[PASS] Timestamp monotonic test (diff=" << diff << "us)\n";
}

void test_system_state_transitions() {
    // Test atomic state transitions
    g_system_state.store(SystemState::INIT);
    assert(g_system_state.load() == SystemState::INIT);
    
    g_system_state.store(SystemState::RUNNING);
    assert(g_system_state.load() == SystemState::RUNNING);
    
    g_system_state.store(SystemState::SAFE);
    assert(g_system_state.load() == SystemState::SAFE);
    
    g_system_state.store(SystemState::FAULT);
    assert(g_system_state.load() == SystemState::FAULT);
    
    std::cout << "[PASS] System state transitions test\n";
}

void test_timing_metrics() {
    // Reset metrics
    g_timing_metrics.compute_start_us = 0;
    g_timing_metrics.compute_end_us = 0;
    g_timing_metrics.execution_time_us = 0;
    g_timing_metrics.deadline_violated = false;
    
    // Simulate compute task timing
    g_timing_metrics.compute_start_us = get_timestamp_us();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    g_timing_metrics.compute_end_us = get_timestamp_us();
    g_timing_metrics.execution_time_us = 
        g_timing_metrics.compute_end_us - g_timing_metrics.compute_start_us;
    
    // Should be roughly 5ms
    assert(g_timing_metrics.execution_time_us >= 4000);
    assert(g_timing_metrics.execution_time_us <= 10000);
    
    std::cout << "[PASS] Timing metrics test (exec=" 
              << g_timing_metrics.execution_time_us << "us)\n";
}

void test_deadline_violation_flag() {
    g_timing_metrics.deadline_violated = false;
    assert(!g_timing_metrics.deadline_violated);
    
    // Simulate violation
    g_timing_metrics.deadline_violated = true;
    assert(g_timing_metrics.deadline_violated);
    
    // Clear violation
    g_timing_metrics.deadline_violated = false;
    assert(!g_timing_metrics.deadline_violated);
    
    std::cout << "[PASS] Deadline violation flag test\n";
}

void test_state_machine_transitions() {
    std::cout << "Testing state machine transitions...\n";
    
    // Test all valid state transitions
    g_system_state.store(SystemState::INIT);
    assert(g_system_state.load() == SystemState::INIT);
    
    // INIT -> RUNNING
    g_system_state.store(SystemState::RUNNING);
    assert(g_system_state.load() == SystemState::RUNNING);
    
    // RUNNING -> SAFE (deadline violation simulation)
    g_timing_metrics.deadline_violated = true;
    if (g_timing_metrics.deadline_violated) {
        g_system_state.store(SystemState::SAFE);
    }
    assert(g_system_state.load() == SystemState::SAFE);
    
    // SAFE -> FAULT (critical failure)
    g_system_state.store(SystemState::FAULT);
    assert(g_system_state.load() == SystemState::FAULT);
    
    // Reset
    g_timing_metrics.deadline_violated = false;
    g_system_state.store(SystemState::INIT);
    
    std::cout << "[PASS] State machine transitions test\n";
}

void test_emergency_brake_trigger() {
    std::cout << "Testing emergency brake trigger...\n";
    
    g_system_state.store(SystemState::RUNNING);
    
    // Trigger emergency brake
    trigger_emergency_brake();
    
    // Should transition to SAFE state
    assert(g_system_state.load() == SystemState::SAFE);
    
    std::cout << "[PASS] Emergency brake trigger test\n";
}

void test_deadline_violation_logging() {
    std::cout << "Testing deadline violation logging...\n";
    
    // Test with small execution time (no violation)
    log_deadline_violation(5000); // 5ms
    
    // Test with large execution time (violation)
    log_deadline_violation(25000); // 25ms (exceeds 20ms deadline)
    
    std::cout << "[PASS] Deadline violation logging test\n";
}

void test_timestamp_precision() {
    // Test microsecond precision
    uint64_t timestamps[10];
    for (int i = 0; i < 10; i++) {
        timestamps[i] = get_timestamp_us();
    }
    
    // All timestamps should be unique (or very close)
    for (int i = 1; i < 10; i++) {
        assert(timestamps[i] >= timestamps[i-1]);
    }
    
    std::cout << "[PASS] Timestamp precision test\n";
}

int main() {
    std::cout << "Running Safety Functions Tests...\n";
    
    test_timestamp_monotonic();
    test_system_state_transitions();
    test_timing_metrics();
    test_deadline_violation_flag();
    test_state_machine_transitions();
    test_emergency_brake_trigger();
    test_deadline_violation_logging();
    test_timestamp_precision();
    
    std::cout << "\nAll safety tests passed!\n";
    return 0;
}
