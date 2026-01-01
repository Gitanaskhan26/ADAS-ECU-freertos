#ifndef CAN_FD_INTERFACE_HPP
#define CAN_FD_INTERFACE_HPP

#include "sensor_data.hpp"
#include <cstdint>
#include <linux/can.h>

// Check if CAN FD definitions are already available
#ifndef CANFD_BRS
// CAN FD frame flags
constexpr uint8_t CANFD_BRS = 0x01;  // Bit Rate Switch
constexpr uint8_t CANFD_ESI = 0x02;  // Error State Indicator
constexpr uint8_t CANFD_FDF = 0x04;  // FD Format indicator

// CAN FD frame structure (Linux SocketCAN)
struct canfd_frame {
    uint32_t can_id;   // CAN ID + EFF/RTR/ERR flags
    uint8_t  len;      // Frame payload length (0..64)
    uint8_t  flags;    // CANFD_BRS | CANFD_ESI | CANFD_FDF
    uint8_t  __res0;   // Reserved / padding
    uint8_t  __res1;   // Reserved / padding
    uint8_t  data[64]; // CAN FD payload (up to 64 bytes)
};
#endif  // CANFD_BRS

// Extended sensor data for CAN FD (more data per frame)
struct ExtendedSensorData {
    // Lidar (16 bytes)
    float x, y, z;                  // 3D position
    float intensity;                // Point cloud intensity
    
    // Radar (16 bytes)
    float vx, vy, vz;              // 3D velocity
    float doppler;                  // Doppler shift
    
    // Camera (16 bytes)
    uint8_t object_class;
    float confidence;
    float bbox_x, bbox_y;           // Bounding box
    float bbox_width, bbox_height;
    
    // IMU (16 bytes)
    float accel_x, accel_y, accel_z;  // Acceleration
    float gyro_roll, gyro_pitch, gyro_yaw;  // Angular rates
    
    uint64_t timestamp;
};

class CANFDInterface {
public:
    CANFDInterface(const char* interface_name);
    ~CANFDInterface();
    
    // Initialize CAN FD socket with specific bitrate
    bool initialize(uint32_t arbitration_bitrate = 500000,
                   uint32_t data_bitrate = 2000000);
    
    // Read CAN FD frame
    bool read_canfd_frame(canfd_frame& frame);
    
    // Write CAN FD frame
    bool write_canfd_frame(const canfd_frame& frame);
    
    // Read extended sensor data from CAN FD
    bool read_extended_sensor_data(ExtendedSensorData& data);
    
    // Send extended sensor data via CAN FD
    bool send_extended_sensor_data(const ExtendedSensorData& data, uint32_t can_id);
    
    // Check if interface supports CAN FD
    bool is_canfd_supported() const { return canfd_supported_; }
    
    bool is_open() const { return socket_fd_ >= 0; }
    
private:
    const char* interface_name_;
    int socket_fd_;
    bool canfd_supported_;
    
    // Parse extended frames
    bool parse_extended_lidar(const uint8_t* data, ExtendedSensorData& out);
    bool parse_extended_radar(const uint8_t* data, ExtendedSensorData& out);
    bool parse_extended_camera(const uint8_t* data, ExtendedSensorData& out);
    bool parse_extended_imu(const uint8_t* data, ExtendedSensorData& out);
};

#endif // CAN_FD_INTERFACE_HPP
