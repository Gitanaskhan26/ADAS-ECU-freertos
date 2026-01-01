#ifndef SAFETY_HPP
#define SAFETY_HPP

#include "sensor_data.hpp"
#include <cstdint>
#include <atomic>

// Global system state (atomic for thread-safe access)
extern std::atomic<SystemState> g_system_state;

// Timing metrics for watchdog
struct TimingMetrics {
    uint64_t compute_start_us;
    uint64_t compute_end_us;
    uint64_t execution_time_us;
    bool deadline_violated;
};

extern TimingMetrics g_timing_metrics;

// Safety functions
void trigger_emergency_brake();
void log_deadline_violation(uint64_t execution_time_us);
uint64_t get_timestamp_us();

#endif // SAFETY_HPP
