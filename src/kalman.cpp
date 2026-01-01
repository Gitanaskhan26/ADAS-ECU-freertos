#include "kalman.hpp"
#include <cmath>

KalmanFilter::KalmanFilter()
    : initialized_(false)
{
    // Initialize state vector to zero
    x_.setZero();
    
    // Initialize covariance matrix (high uncertainty initially)
    P_.setIdentity();
    P_ *= 100.0f;
    
    // State transition matrix (constant velocity model)
    // Will be updated with actual dt in predict()
    F_.setIdentity();
    
    // Process noise covariance (tune based on system dynamics)
    Q_.setIdentity();
    Q_ *= 0.1f;
    
    // Measurement noise covariance (sensor accuracy)
    R_.setIdentity();
    R_(0, 0) = 0.5f;   // Lidar X uncertainty
    R_(1, 1) = 0.5f;   // Lidar Y uncertainty
    R_(2, 2) = 0.2f;   // Radar Vx uncertainty
    R_(3, 3) = 0.2f;   // Radar Vy uncertainty
    
    // Measurement matrix (direct observation)
    H_.setIdentity();
}

void KalmanFilter::initialize(const SensorData& data) {
    x_(0) = data.x;
    x_(1) = data.y;
    x_(2) = data.vx;
    x_(3) = data.vy;
    
    initialized_ = true;
}

void KalmanFilter::predict(float dt) {
    if (!initialized_) {
        return;
    }
    
    // Update state transition matrix with actual timestep
    // x_new = x_old + vx * dt
    // y_new = y_old + vy * dt
    // vx_new = vx_old
    // vy_new = vy_old
    
    F_(0, 2) = dt;  // x depends on vx
    F_(1, 3) = dt;  // y depends on vy
    
    // Predict state: x = F * x
    x_ = F_ * x_;
    
    // Predict covariance: P = F * P * F^T + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const SensorData& measurement) {
    if (!initialized_) {
        initialize(measurement);
        return;
    }
    
    // Measurement vector
    Eigen::Vector4f z;
    z(0) = measurement.x;
    z(1) = measurement.y;
    z(2) = measurement.vx;
    z(3) = measurement.vy;
    
    // Innovation: y = z - H * x
    Eigen::Vector4f y = z - H_ * x_;
    
    // Innovation covariance: S = H * P * H^T + R
    Eigen::Matrix4f S = H_ * P_ * H_.transpose() + R_;
    
    // Kalman gain: K = P * H^T * S^-1
    Eigen::Matrix4f K = P_ * H_.transpose() * S.inverse();
    
    // Update state: x = x + K * y
    x_ = x_ + K * y;
    
    // Update covariance: P = (I - K * H) * P
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    P_ = (I - K * H_) * P_;
}

StateVector KalmanFilter::get_state() const {
    StateVector state;
    state.x = x_(0);
    state.y = x_(1);
    state.vx = x_(2);
    state.vy = x_(3);
    state.timestamp = 0;  // Set by caller
    return state;
}
