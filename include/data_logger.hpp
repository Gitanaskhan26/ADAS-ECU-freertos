#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include "sensor_data.hpp"
#include "collision_detection.hpp"
#include <cstdio>
#include <cstdint>

// Data logging for post-analysis
class DataLogger {
public:
    DataLogger(const char* log_dir = "logs");
    ~DataLogger();
    
    // Initialize logging (create files)
    bool initialize();
    
    // Log sensor data
    void log_sensor_data(const SensorData& data);
    
    // Log state estimate
    void log_state_estimate(const StateVector& state);
    
    // Log collision detection result
    void log_collision_info(const CollisionInfo& info, uint64_t timestamp);
    
    // Log safety event
    void log_safety_event(const char* event, uint64_t timestamp);
    
    // Close all log files
    void close();
    
private:
    const char* log_dir_;
    FILE* sensor_log_;
    FILE* state_log_;
    FILE* collision_log_;
    FILE* safety_log_;
    
    bool create_log_directory();
    FILE* open_log_file(const char* filename);
};

#endif // DATA_LOGGER_HPP
