#include "config.hpp"
#include "safety.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <cstdio>

void task_watchdog(void* params) {
    (void)params;  // Unused
    
    printf("[WATCHDOG] Task started, monitoring deadline=%u ms\n", COMPUTE_DEADLINE_MS);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TASK_WATCHDOG_PERIOD_MS);
    
    const uint64_t deadline_us = COMPUTE_DEADLINE_MS * 1000ULL;
    
    while (1) {
        // Check if compute task exceeded deadline
        uint64_t exec_time = g_timing_metrics.execution_time_us;
        
        if (exec_time > deadline_us) {
            if (!g_timing_metrics.deadline_violated) {
                // First violation
                g_timing_metrics.deadline_violated = true;
                
                log_deadline_violation(exec_time);
                trigger_emergency_brake();
                
                printf("[WATCHDOG] System entering SAFE state\n");
            }
        } else {
            // Reset violation flag if back to normal
            if (g_timing_metrics.deadline_violated && exec_time < deadline_us / 2) {
                g_timing_metrics.deadline_violated = false;
                printf("[WATCHDOG] Deadline violations cleared\n");
            }
        }
        
        // Monitor system state
        SystemState state = g_system_state.load();
        if (state == SystemState::RUNNING && g_timing_metrics.deadline_violated) {
            // Keep state as SAFE
        } else if (state == SystemState::INIT && exec_time > 0) {
            // Transition to running after first compute cycle
            g_system_state.store(SystemState::RUNNING);
            printf("[WATCHDOG] System state: RUNNING\n");
        }
        
        // Wait until next period
        vTaskDelayUntil(&last_wake_time, period);
    }
}
