#include "config.hpp"
#include "sensor_data.hpp"
#include "can_interface.hpp"
#include "safety.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <cstdio>

// External queue handle (defined in main.cpp)
extern QueueHandle_t g_sensor_queue;

void task_sensor_receiver(void* params) {
    (void)params;  // Unused
    
    CANInterface can(CAN_INTERFACE);
    
    if (!can.initialize()) {
        printf("[SENSOR] Failed to initialize CAN interface\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("[SENSOR] Task started, period=%u ms\n", TASK_SENSOR_PERIOD_MS);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS);
    
    while (1) {
        SensorData data = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0};
        data.timestamp = get_timestamp_us();
        
        // Read from CAN bus (non-blocking or with timeout)
        if (can.read_sensor_data(data)) {
            // Send to queue (non-blocking)
            if (xQueueSend(g_sensor_queue, &data, 0) != pdTRUE) {
                printf("[SENSOR] Queue full, dropping frame\n");
            }
        }
        
        // Wait until next period
        vTaskDelayUntil(&last_wake_time, period);
    }
}
