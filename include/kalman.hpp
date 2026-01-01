#ifndef KALMAN_HPP
#define KALMAN_HPP

#include "sensor_data.hpp"
#include <Eigen/Dense>

// Kalman filter for 2D position and velocity estimation
// State: [x, y, vx, vy]
class KalmanFilter {
public:
    KalmanFilter();
    
    // Initialize with first measurement
    void initialize(const SensorData& data);
    
    // Prediction step (time update)
    void predict(float dt);
    
    // Update step (measurement update)
    void update(const SensorData& measurement);
    
    // Get current state estimate
    StateVector get_state() const;
    
    // Check if filter is initialized
    bool is_initialized() const { return initialized_; }
    
private:
    Eigen::Vector4f x_;           // State [x, y, vx, vy]
    Eigen::Matrix4f P_;           // Covariance matrix
    Eigen::Matrix4f F_;           // State transition matrix
    Eigen::Matrix4f Q_;           // Process noise covariance
    Eigen::Matrix4f R_;           // Measurement noise covariance
    Eigen::Matrix4f H_;           // Measurement matrix
    
    bool initialized_;
};

#endif // KALMAN_HPP
