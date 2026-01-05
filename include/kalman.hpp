#ifndef KALMAN_HPP
#define KALMAN_HPP

#include "sensor_data.hpp"
#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    void initialize(const SensorData& data);
    void predict(float dt);
    void update(const SensorData& measurement);
    StateVector get_state() const;
    bool is_initialized() const { return initialized_; }
    
private:
    Eigen::Vector4f x_;
    Eigen::Matrix4f P_;
    Eigen::Matrix4f F_;
    Eigen::Matrix4f Q_;
    Eigen::Matrix4f R_;
    Eigen::Matrix4f H_;
    bool initialized_;
};

#endif
