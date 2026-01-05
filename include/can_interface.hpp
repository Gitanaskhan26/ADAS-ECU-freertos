#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include "sensor_data.hpp"
#include <cstdint>

// CAN interface for SocketCAN
class CANInterface {
public:
    CANInterface(const char* interface_name);
    ~CANInterface();
    
    // Initialize CAN socket
    bool initialize();
    
    // Read sensor data from CAN bus
    // Returns true if valid frame received
    bool read_sensor_data(SensorData& data);
    
    // Send emergency brake command
    void send_brake_command();
    
    // Check if interface is open
    bool is_open() const { return socket_fd_ >= 0; }
    
private:
    const char* interface_name_;
    int socket_fd_;
    
    // Parse CAN frame into sensor data
    bool parse_lidar_frame(const uint8_t* data, SensorData& out);
    bool parse_radar_frame(const uint8_t* data, SensorData& out);
    bool parse_camera_frame(const uint8_t* data, SensorData& out);
    bool parse_ultrasonic_frame(const uint8_t* data, SensorData& out);
};

#endif // CAN_INTERFACE_HPP
