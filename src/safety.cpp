#include "safety.hpp"
#include "can_interface.hpp"
#include "config.hpp"
#include <ctime>
#include <cstdio>

// Global system state
std::atomic<SystemState> g_system_state{SystemState::INIT};

// Timing metrics
TimingMetrics g_timing_metrics = {0, 0, 0, false};

uint64_t get_timestamp_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + 
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

void trigger_emergency_brake() {
    static CANInterface can_interface(CAN_INTERFACE);
    
    // Send brake command over CAN
    can_interface.send_brake_command();
    
    // Update system state
    g_system_state.store(SystemState::SAFE);
    
    printf("[SAFETY] Emergency brake triggered\n");
}

void log_deadline_violation(uint64_t execution_time_us) {
    printf("[SAFETY] DEADLINE VIOLATION: Compute took %lu us (limit: %u ms)\n",
           execution_time_us, COMPUTE_DEADLINE_MS);
    
    // Log to file in production system
    // For now, just console output
}
