#include "kalman.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

void test_initialization() {
    KalmanFilter kf;
    
    SensorData data;
    data.x = 1.0f;
    data.y = 2.0f;
    data.vx = 0.5f;
    data.vy = 0.3f;
    data.timestamp = 0;
    
    kf.initialize(data);
    
    assert(kf.is_initialized());
    
    StateVector state = kf.get_state();
    assert(std::abs(state.x - 1.0f) < 0.01f);
    assert(std::abs(state.y - 2.0f) < 0.01f);
    
    std::cout << "[PASS] Initialization test\n";
}

void test_prediction() {
    KalmanFilter kf;
    
    SensorData data;
    data.x = 0.0f;
    data.y = 0.0f;
    data.vx = 1.0f;
    data.vy = 1.0f;
    data.timestamp = 0;
    
    kf.initialize(data);
    
    // Predict 1 second forward
    kf.predict(1.0f);
    
    StateVector state = kf.get_state();
    
    // Position should have moved by velocity
    assert(std::abs(state.x - 1.0f) < 0.1f);
    assert(std::abs(state.y - 1.0f) < 0.1f);
    
    std::cout << "[PASS] Prediction test\n";
}

void test_update() {
    KalmanFilter kf;
    
    SensorData init_data;
    init_data.x = 0.0f;
    init_data.y = 0.0f;
    init_data.vx = 0.0f;
    init_data.vy = 0.0f;
    init_data.timestamp = 0;
    
    kf.initialize(init_data);
    
    // Update with new measurement
    SensorData measurement;
    measurement.x = 1.0f;
    measurement.y = 1.0f;
    measurement.vx = 0.5f;
    measurement.vy = 0.5f;
    measurement.timestamp = 1000000;
    
    kf.update(measurement);
    
    StateVector state = kf.get_state();
    
    // State should be influenced by measurement
    assert(state.x > 0.0f && state.x < 1.5f);
    assert(state.y > 0.0f && state.y < 1.5f);
    
    std::cout << "[PASS] Update test\n";
}

void test_multiple_updates() {
    std::cout << "Testing multiple sequential updates...\n";
    KalmanFilter kf;
    
    SensorData init;
    init.x = 0.0f;
    init.y = 0.0f;
    init.vx = 10.0f;
    init.vy = 0.0f;
    init.timestamp = 0;
    
    kf.initialize(init);
    
    // Apply multiple updates
    for (int i = 1; i <= 10; i++) {
        SensorData measurement;
        measurement.x = i * 1.0f;
        measurement.y = i * 0.5f;
        measurement.vx = 10.0f;
        measurement.vy = 0.0f;
        measurement.timestamp = i * 100000;
        
        kf.predict(0.1f);
        kf.update(measurement);
    }
    
    StateVector final_state = kf.get_state();
    
    // State should track measurements
    assert(final_state.x > 5.0f);
    assert(final_state.y > 2.0f);
    assert(std::isfinite(final_state.x));
    assert(std::isfinite(final_state.y));
    
    std::cout << "[PASS] Multiple updates test (x=" << final_state.x << ", y=" << final_state.y << ")\n";
}

void test_long_prediction_sequence() {
    std::cout << "Testing long prediction sequence...\n";
    KalmanFilter kf;
    
    SensorData data;
    data.x = 0.0f;
    data.y = 0.0f;
    data.vx = 10.0f;
    data.vy = 5.0f;
    data.timestamp = 0;
    
    kf.initialize(data);
    
    // Many prediction steps (tests numerical stability)
    for (int i = 0; i < 100; i++) {
        kf.predict(0.1f);
    }
    
    StateVector state = kf.get_state();
    
    // Position should have moved significantly
    assert(state.x > 50.0f);   // 10 m/s * 10s
    assert(state.y > 20.0f);   // 5 m/s * 10s
    assert(std::isfinite(state.x));
    assert(std::isfinite(state.y));
    
    std::cout << "[PASS] Long prediction test (x=" << state.x << ", y=" << state.y << ")\n";
}

void test_measurement_correction() {
    std::cout << "Testing measurement correction...\n";
    KalmanFilter kf;
    
    SensorData init;
    init.x = 10.0f;
    init.y = 5.0f;
    init.vx = 2.0f;
    init.vy = 1.0f;
    init.timestamp = 0;
    
    kf.initialize(init);
    
    // Predict forward
    kf.predict(1.0f);
    StateVector predicted = kf.get_state();
    
    // Measurement is different from prediction
    SensorData measurement;
    measurement.x = 11.0f;
    measurement.y = 6.0f;
    measurement.vx = 2.5f;
    measurement.vy = 1.2f;
    measurement.timestamp = 1000000;
    
    kf.update(measurement);
    StateVector corrected = kf.get_state();
    
    // State should have been corrected toward measurement
    assert(corrected.x != predicted.x);
    assert(corrected.y != predicted.y);
    assert(std::isfinite(corrected.x));
    
    std::cout << "[PASS] Measurement correction test\n";
}

int main() {
    std::cout << "Running Kalman Filter Tests...\n";
    
    test_initialization();
    test_prediction();
    test_update();
    test_multiple_updates();
    test_long_prediction_sequence();
    test_measurement_correction();
    
    std::cout << "\nAll tests passed!\n";
    return 0;
}
