#include "config.hpp"
#include "sensor_data.hpp"
#include "kalman.hpp"
#include "safety.hpp"
#include "collision_detection.hpp"
#include "data_logger.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <cstdio>

// External queue handle (defined in main.cpp)
extern QueueHandle_t g_sensor_queue;

// Global state estimate
static StateVector g_state_estimate = {0.0f, 0.0f, 0.0f, 0.0f, 0};

void task_compute(void* params) {
    (void)params;  // Unused
    
    KalmanFilter kalman;
    CollisionDetector collision_detector;
    DataLogger logger;
    
    // Initialize logger
    if (!logger.initialize()) {
        printf("[COMPUTE] Warning: Failed to initialize data logger\n");
    }
    
    printf("[COMPUTE] Task started, period=%u ms\n", TASK_COMPUTE_PERIOD_MS);
    printf("[COMPUTE] Collision detection enabled (Warning: %.1fs, Critical: %.1fs)\n",
           COLLISION_WARNING_TTC_SEC, COLLISION_CRITICAL_TTC_SEC);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TASK_COMPUTE_PERIOD_MS);
    
    uint64_t last_timestamp = 0;
    
    while (1) {
        // Record start time for watchdog
        g_timing_metrics.compute_start_us = get_timestamp_us();
        
        SensorData sensor_data;
        
        // Receive latest sensor data from queue
        if (xQueueReceive(g_sensor_queue, &sensor_data, 0) == pdTRUE) {
            
            // Log raw sensor data
            logger.log_sensor_data(sensor_data);
            
            // Calculate dt in seconds
            float dt = 0.0f;
            if (last_timestamp > 0) {
                dt = (sensor_data.timestamp - last_timestamp) / 1e6f;
            }
            last_timestamp = sensor_data.timestamp;
            
            // Run Kalman filter
            if (dt > 0.0f && kalman.is_initialized()) {
                kalman.predict(dt);
            }
            kalman.update(sensor_data);
            
            // Get state estimate
            g_state_estimate = kalman.get_state();
            g_state_estimate.timestamp = sensor_data.timestamp;
            
            // Log state estimate
            logger.log_state_estimate(g_state_estimate);
            
            // Collision detection
            CollisionInfo collision = collision_detector.detect_collision(g_state_estimate);
            
            // Log collision info
            logger.log_collision_info(collision, sensor_data.timestamp);
            
            // Handle collision warnings
            if (collision.risk_level == CollisionRisk::CRITICAL) {
                printf("[COMPUTE] ⚠️  CRITICAL: TTC=%.2fs, dist=%.2fm\n",
                       collision.time_to_collision, collision.distance);
                logger.log_safety_event("COLLISION_CRITICAL", sensor_data.timestamp);
                if (collision_detector.should_trigger_brake(collision)) {
                    trigger_emergency_brake();
                    logger.log_safety_event("EMERGENCY_BRAKE", sensor_data.timestamp);
                }
            } else if (collision.risk_level == CollisionRisk::WARNING) {
                printf("[COMPUTE] ⚠️  WARNING: TTC=%.2fs, dist=%.2fm\n",
                       collision.time_to_collision, collision.distance);
                logger.log_safety_event("COLLISION_WARNING", sensor_data.timestamp);
            }
            
            // Log state (optional, for debugging)
            // printf("[COMPUTE] State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f\n",
            //        g_state_estimate.x, g_state_estimate.y,
            //        g_state_estimate.vx, g_state_estimate.vy);
        }
        
        // Record end time for watchdog
        g_timing_metrics.compute_end_us = get_timestamp_us();
        g_timing_metrics.execution_time_us = 
            g_timing_metrics.compute_end_us - g_timing_metrics.compute_start_us;
        
        // Wait until next period
        vTaskDelayUntil(&last_wake_time, period);
    }
}
