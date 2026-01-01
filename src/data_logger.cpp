#include "data_logger.hpp"
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

DataLogger::DataLogger(const char* log_dir)
    : log_dir_(log_dir)
    , sensor_log_(nullptr)
    , state_log_(nullptr)
    , collision_log_(nullptr)
    , safety_log_(nullptr)
{
}

DataLogger::~DataLogger() {
    close();
}

bool DataLogger::create_log_directory() {
    struct stat st;
    if (stat(log_dir_, &st) == -1) {
        if (mkdir(log_dir_, 0755) != 0) {
            return false;
        }
    }
    return true;
}

FILE* DataLogger::open_log_file(const char* filename) {
    char filepath[256];
    snprintf(filepath, sizeof(filepath), "%s/%s", log_dir_, filename);
    
    FILE* file = fopen(filepath, "w");
    if (!file) {
        return nullptr;
    }
    
    return file;
}

bool DataLogger::initialize() {
    // Create log directory
    if (!create_log_directory()) {
        printf("[LOGGER] Failed to create log directory: %s\n", log_dir_);
        return false;
    }
    
    // Open sensor data log
    sensor_log_ = open_log_file("sensor_data.csv");
    if (sensor_log_) {
        fprintf(sensor_log_, "timestamp_us,x,y,vx,vy\n");
        fflush(sensor_log_);
    }
    
    // Open state estimate log
    state_log_ = open_log_file("state_estimate.csv");
    if (state_log_) {
        fprintf(state_log_, "timestamp_us,x,y,vx,vy\n");
        fflush(state_log_);
    }
    
    // Open collision detection log
    collision_log_ = open_log_file("collision_detection.csv");
    if (collision_log_) {
        fprintf(collision_log_, "timestamp_us,risk_level,ttc,distance,is_approaching\n");
        fflush(collision_log_);
    }
    
    // Open safety events log
    safety_log_ = open_log_file("safety_events.csv");
    if (safety_log_) {
        fprintf(safety_log_, "timestamp_us,event\n");
        fflush(safety_log_);
    }
    
    printf("[LOGGER] Initialized in directory: %s\n", log_dir_);
    return true;
}

void DataLogger::log_sensor_data(const SensorData& data) {
    if (!sensor_log_) return;
    
    fprintf(sensor_log_, "%lu,%.6f,%.6f,%.6f,%.6f\n",
            data.timestamp, data.x, data.y, data.vx, data.vy);
    fflush(sensor_log_);
}

void DataLogger::log_state_estimate(const StateVector& state) {
    if (!state_log_) return;
    
    fprintf(state_log_, "%lu,%.6f,%.6f,%.6f,%.6f\n",
            state.timestamp, state.x, state.y, state.vx, state.vy);
    fflush(state_log_);
}

void DataLogger::log_collision_info(const CollisionInfo& info, uint64_t timestamp) {
    if (!collision_log_) return;
    
    const char* risk_str = "NONE";
    switch (info.risk_level) {
        case CollisionRisk::NONE:     risk_str = "NONE"; break;
        case CollisionRisk::LOW:      risk_str = "LOW"; break;
        case CollisionRisk::WARNING:  risk_str = "WARNING"; break;
        case CollisionRisk::CRITICAL: risk_str = "CRITICAL"; break;
    }
    
    fprintf(collision_log_, "%lu,%s,%.6f,%.6f,%d\n",
            timestamp, risk_str, info.time_to_collision, 
            info.distance, info.is_approaching ? 1 : 0);
    fflush(collision_log_);
}

void DataLogger::log_safety_event(const char* event, uint64_t timestamp) {
    if (!safety_log_) return;
    
    fprintf(safety_log_, "%lu,%s\n", timestamp, event);
    fflush(safety_log_);
}

void DataLogger::close() {
    if (sensor_log_) {
        fclose(sensor_log_);
        sensor_log_ = nullptr;
    }
    if (state_log_) {
        fclose(state_log_);
        state_log_ = nullptr;
    }
    if (collision_log_) {
        fclose(collision_log_);
        collision_log_ = nullptr;
    }
    if (safety_log_) {
        fclose(safety_log_);
        safety_log_ = nullptr;
    }
}
