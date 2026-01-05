#include "config.hpp"
#include "sensor_data.hpp"
#include "safety.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <cstdio>
#include <cstdlib>

extern void task_sensor_receiver(void* params);
extern void task_compute(void* params);
extern void task_watchdog(void* params);

QueueHandle_t g_sensor_queue = NULL;

int main(int argc, char* argv[]) {
    (void)argc;  // Unused
    (void)argv;  // Unused
    
    printf("==============================================");
    printf("  ADAS ECU Real-Time Simulation\n");
    printf("  FreeRTOS + SocketCAN + Kalman Filter\n");
    printf("==============================================\n\n");
    
    // Initialize system state
    g_system_state.store(SystemState::INIT);
    
    // Create sensor data queue
    g_sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(SensorData));
    if (g_sensor_queue == NULL) {
        printf("[ERROR] Failed to create sensor queue\n");
        return EXIT_FAILURE;
    }
    printf("[INIT] Sensor queue created (depth=%u)\n", SENSOR_QUEUE_LENGTH);
    
    BaseType_t result = xTaskCreate(
        task_sensor_receiver,
        "SensorRx",
        STACK_SIZE_SENSOR,
        NULL,
        PRIORITY_SENSOR,
        NULL
    );
    if (result != pdPASS) {
        printf("ERR: Sensor task failed\n");
        return EXIT_FAILURE;
    }
    
    result = xTaskCreate(
        task_compute,
        "Compute",
        STACK_SIZE_COMPUTE,
        NULL,
        PRIORITY_COMPUTE,
        NULL
    );
    if (result != pdPASS) {
        printf("ERR: Compute task failed\n");
        return EXIT_FAILURE;
    }
    
    result = xTaskCreate(
        task_watchdog,
        "Watchdog",
        STACK_SIZE_WATCHDOG,
        NULL,
        PRIORITY_WATCHDOG,
        NULL
    );
    if (result != pdPASS) {
        printf("ERR: Watchdog task failed\n");
        return EXIT_FAILURE;
    }
    
    printf("Starting scheduler...\n");
    
    vTaskStartScheduler();
    
    printf("FATAL: Scheduler exit\n");
    return EXIT_FAILURE;
}
