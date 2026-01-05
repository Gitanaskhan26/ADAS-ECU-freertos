#include "can_interface.hpp"
#include "config.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

CANInterface::CANInterface(const char* interface_name)
    : interface_name_(interface_name)
    , socket_fd_(-1)
{
}

CANInterface::~CANInterface() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

bool CANInterface::initialize() {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        return false;
    }
    
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_, IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    return true;
}

bool CANInterface::read_sensor_data(SensorData& data) {
    if (socket_fd_ < 0) {
        return false;
    }
    
    struct can_frame frame;
    ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
    
    if (nbytes != sizeof(frame)) {
        return false;
    }
    
    // Parse based on CAN ID
    if (frame.can_id == LIDAR_CAN_ID) {
        data.sensor_flags |= SENSOR_FLAG_LIDAR;
        return parse_lidar_frame(frame.data, data);
    } else if (frame.can_id == RADAR_CAN_ID) {
        data.sensor_flags |= SENSOR_FLAG_RADAR;
        return parse_radar_frame(frame.data, data);
    } else if (frame.can_id == CAMERA_CAN_ID) {
        data.sensor_flags |= SENSOR_FLAG_CAMERA;
        return parse_camera_frame(frame.data, data);
    } else if (frame.can_id == ULTRASONIC_CAN_ID) {
        data.sensor_flags |= SENSOR_FLAG_ULTRASONIC;
        return parse_ultrasonic_frame(frame.data, data);
    }
    
    return false;
}

bool CANInterface::parse_lidar_frame(const uint8_t* data, SensorData& out) {
    // Frame format: [x_bytes(4)] [y_bytes(4)]
    // Assuming float encoding (little-endian)
    
    if (data == nullptr) {
        return false;
    }
    
    std::memcpy(&out.x, &data[0], sizeof(float));
    std::memcpy(&out.y, &data[4], sizeof(float));
    
    return true;
}

bool CANInterface::parse_radar_frame(const uint8_t* data, SensorData& out) {
    // Frame format: [vx_bytes(4)] [vy_bytes(4)]
    // Assuming float encoding (little-endian)
    
    if (data == nullptr) {
        return false;
    }
    
    std::memcpy(&out.vx, &data[0], sizeof(float));
    std::memcpy(&out.vy, &data[4], sizeof(float));
    
    return true;
}

bool CANInterface::parse_camera_frame(const uint8_t* data, SensorData& out) {
    // Frame format: [object_class(1)] [confidence(4)] [reserved(3)]
    // object_class: 0=none, 1=vehicle, 2=pedestrian, 3=obstacle
    // confidence: float 0.0-1.0
    
    if (data == nullptr) {
        return false;
    }
    
    out.camera_object_class = data[0];
    std::memcpy(&out.camera_confidence, &data[1], sizeof(float));
    
    return true;
}

bool CANInterface::parse_ultrasonic_frame(const uint8_t* data, SensorData& out) {
    // Frame format: [distance_cm(4)] [reserved(4)]
    // distance_cm: float distance in centimeters
    
    if (data == nullptr) {
        return false;
    }
    
    std::memcpy(&out.ultrasonic_distance_cm, &data[0], sizeof(float));
    
    return true;
}

void CANInterface::send_brake_command() {
    if (socket_fd_ < 0) {
        return;
    }
    
    struct can_frame frame;
    frame.can_id = BRAKE_CAN_ID;
    frame.can_dlc = 1;
    frame.data[0] = 0xFF;  // Emergency brake signal
    
    ssize_t result = write(socket_fd_, &frame, sizeof(frame));
    if (result < 0) {
        perror("Failed to send brake command");
    }
}
