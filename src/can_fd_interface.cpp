#include "can_fd_interface.hpp"
#include "config.hpp"
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

CANFDInterface::CANFDInterface(const char* interface_name)
    : interface_name_(interface_name)
    , socket_fd_(-1)
    , canfd_supported_(false)
{
}

CANFDInterface::~CANFDInterface() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

bool CANFDInterface::initialize(uint32_t arbitration_bitrate, uint32_t data_bitrate) {
    // Create CAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        perror("CAN FD socket creation failed");
        return false;
    }
    
    // Enable CAN FD mode
    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &enable_canfd, sizeof(enable_canfd)) < 0) {
        printf("Warning: CAN FD not supported, falling back to classical CAN\n");
        canfd_supported_ = false;
    } else {
        canfd_supported_ = true;
        printf("CAN FD enabled: Arbitration=%u bps, Data=%u bps\n",
               arbitration_bitrate, data_bitrate);
    }
    
    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_, IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("CAN interface not found");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Bind socket to CAN interface
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("CAN FD bind failed");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    return true;
}

bool CANFDInterface::read_canfd_frame(canfd_frame& frame) {
    if (socket_fd_ < 0) {
        return false;
    }
    
    ssize_t nbytes = read(socket_fd_, &frame, sizeof(canfd_frame));
    
    if (nbytes == sizeof(canfd_frame)) {
        return true;  // CAN FD frame
    } else if (nbytes == sizeof(struct can_frame)) {
        return true;  // Classical CAN frame (compatible)
    }
    
    return false;
}

bool CANFDInterface::write_canfd_frame(const canfd_frame& frame) {
    if (socket_fd_ < 0) {
        return false;
    }
    
    ssize_t nbytes = write(socket_fd_, &frame, sizeof(canfd_frame));
    return (nbytes == sizeof(canfd_frame));
}

bool CANFDInterface::parse_extended_lidar(const uint8_t* data, ExtendedSensorData& out) {
    // Parse 16-byte Lidar data: [x(4)] [y(4)] [z(4)] [intensity(4)]
    std::memcpy(&out.x, &data[0], sizeof(float));
    std::memcpy(&out.y, &data[4], sizeof(float));
    std::memcpy(&out.z, &data[8], sizeof(float));
    std::memcpy(&out.intensity, &data[12], sizeof(float));
    return true;
}

bool CANFDInterface::parse_extended_radar(const uint8_t* data, ExtendedSensorData& out) {
    // Parse 16-byte Radar data: [vx(4)] [vy(4)] [vz(4)] [doppler(4)]
    std::memcpy(&out.vx, &data[0], sizeof(float));
    std::memcpy(&out.vy, &data[4], sizeof(float));
    std::memcpy(&out.vz, &data[8], sizeof(float));
    std::memcpy(&out.doppler, &data[12], sizeof(float));
    return true;
}

bool CANFDInterface::parse_extended_camera(const uint8_t* data, ExtendedSensorData& out) {
    // Parse 16-byte Camera data
    out.object_class = data[0];
    std::memcpy(&out.confidence, &data[1], sizeof(float));
    std::memcpy(&out.bbox_x, &data[5], sizeof(float));
    std::memcpy(&out.bbox_y, &data[9], sizeof(float));
    // bbox_width and bbox_height would be in next bytes if needed
    return true;
}

bool CANFDInterface::parse_extended_imu(const uint8_t* data, ExtendedSensorData& out) {
    // Parse IMU data: 6 floats (24 bytes)
    std::memcpy(&out.accel_x, &data[0], sizeof(float));
    std::memcpy(&out.accel_y, &data[4], sizeof(float));
    std::memcpy(&out.accel_z, &data[8], sizeof(float));
    std::memcpy(&out.gyro_roll, &data[12], sizeof(float));
    std::memcpy(&out.gyro_pitch, &data[16], sizeof(float));
    std::memcpy(&out.gyro_yaw, &data[20], sizeof(float));
    return true;
}

bool CANFDInterface::read_extended_sensor_data(ExtendedSensorData& data) {
    canfd_frame frame;
    
    if (!read_canfd_frame(frame)) {
        return false;
    }
    
    // Parse based on CAN ID
    switch (frame.can_id) {
        case LIDAR_CAN_ID:
            return parse_extended_lidar(frame.data, data);
        case RADAR_CAN_ID:
            return parse_extended_radar(frame.data, data);
        case CAMERA_CAN_ID:
            return parse_extended_camera(frame.data, data);
        default:
            return false;
    }
}

bool CANFDInterface::send_extended_sensor_data(const ExtendedSensorData& data, uint32_t can_id) {
    canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    
    frame.can_id = can_id;
    frame.flags = CANFD_BRS | CANFD_FDF;  // Enable bit-rate switching and FD format
    frame.len = 64;  // Use full CAN FD payload
    
    // Pack all sensor data into 64-byte payload
    size_t offset = 0;
    
    // Lidar (16 bytes)
    std::memcpy(&frame.data[offset], &data.x, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.y, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.z, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.intensity, sizeof(float)); offset += 4;
    
    // Radar (16 bytes)
    std::memcpy(&frame.data[offset], &data.vx, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.vy, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.vz, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.doppler, sizeof(float)); offset += 4;
    
    // Camera (16 bytes)
    frame.data[offset++] = data.object_class;
    std::memcpy(&frame.data[offset], &data.confidence, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.bbox_x, sizeof(float)); offset += 4;
    std::memcpy(&frame.data[offset], &data.bbox_y, sizeof(float)); offset += 4;
    offset += 3;  // Padding
    
    // Timestamp (8 bytes)
    std::memcpy(&frame.data[offset], &data.timestamp, sizeof(uint64_t));
    
    return write_canfd_frame(frame);
}
