#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <cstdint>

// Structured sensor data after CAN parsing
struct SensorData {
    float x;              // Lidar position X (meters)
    float y;              // Lidar position Y (meters)
    float vx;             // Radar velocity X (m/s)
    float vy;             // Radar velocity Y (m/s)
    uint64_t timestamp;   // Microseconds since boot
    
    // Extended sensors
    uint8_t camera_object_class;   // Camera: detected object class (0=none, 1=vehicle, 2=pedestrian, 3=obstacle)
    float camera_confidence;       // Camera: detection confidence (0.0-1.0)
    float ultrasonic_distance_cm;  // Ultrasonic: distance in centimeters
    uint8_t sensor_flags;          // Bit flags: which sensors have valid data
};

// Sensor flags
constexpr uint8_t SENSOR_FLAG_LIDAR = 0x01;
constexpr uint8_t SENSOR_FLAG_RADAR = 0x02;
constexpr uint8_t SENSOR_FLAG_CAMERA = 0x04;
constexpr uint8_t SENSOR_FLAG_ULTRASONIC = 0x08;

// State vector for Kalman filter
// [x, y, vx, vy]
struct StateVector {
    float x;              // Position X (meters)
    float y;              // Position Y (meters)
    float vx;             // Velocity X (m/s)
    float vy;             // Velocity Y (m/s)
    uint64_t timestamp;   // Microseconds
};

// System state enumeration
enum class SystemState : uint8_t {
    INIT,                 // Initializing
    RUNNING,              // Normal operation
    SAFE,                 // Safe state (deadline violated)
    FAULT                 // Unrecoverable fault
};

#endif // SENSOR_DATA_HPP
