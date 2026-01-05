#include "kalman.hpp"
#include <cmath>

KalmanFilter::KalmanFilter()
    : initialized_(false)
{
    x_.setZero();
    P_.setIdentity();
    P_ *= 100.0f;
    F_.setIdentity();
    Q_.setIdentity();
    Q_ *= 0.1f;
    R_.setIdentity();
    R_(0, 0) = 0.5f;
    R_(1, 1) = 0.5f;
    R_(2, 2) = 0.2f;
    R_(3, 3) = 0.2f;
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
    
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const SensorData& measurement) {
    if (!initialized_) {
        initialize(measurement);
        return;
    }
    
    Eigen::Vector4f z;
    z(0) = measurement.x;
    z(1) = measurement.y;
    z(2) = measurement.vx;
    z(3) = measurement.vy;
    
    Eigen::Vector4f y = z - H_ * x_;
    Eigen::Matrix4f S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix4f K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
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
