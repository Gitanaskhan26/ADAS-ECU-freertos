#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cstdint>

// Task timing configuration (all in milliseconds)
constexpr uint32_t TASK_SENSOR_PERIOD_MS = 10;    // 100 Hz sensor read
constexpr uint32_t TASK_COMPUTE_PERIOD_MS = 20;   // 50 Hz Kalman update
constexpr uint32_t TASK_WATCHDOG_PERIOD_MS = 5;   // 200 Hz safety check

// Deadline enforcement
constexpr uint32_t COMPUTE_DEADLINE_MS = 50;      // Max allowed compute time

// Queue configuration
constexpr uint32_t SENSOR_QUEUE_LENGTH = 10;      // Bounded queue depth

// CAN configuration
constexpr const char* CAN_INTERFACE = "vcan0";
constexpr uint32_t LIDAR_CAN_ID = 0x100;
constexpr uint32_t RADAR_CAN_ID = 0x200;
constexpr uint32_t CAMERA_CAN_ID = 0x250;      // Camera object detection
constexpr uint32_t ULTRASONIC_CAN_ID = 0x300;  // Ultrasonic distance sensors
constexpr uint32_t BRAKE_CAN_ID = 0x400;

// FreeRTOS task priorities (higher = more important)
constexpr uint32_t PRIORITY_WATCHDOG = 3;         // Highest - safety critical
constexpr uint32_t PRIORITY_COMPUTE = 2;          // Medium - data processing
constexpr uint32_t PRIORITY_SENSOR = 1;           // Lower - I/O bound

// FreeRTOS stack sizes (bytes)
constexpr uint32_t STACK_SIZE_SENSOR = 2048;
constexpr uint32_t STACK_SIZE_COMPUTE = 4096;
constexpr uint32_t STACK_SIZE_WATCHDOG = 1024;

#endif // CONFIG_HPP
