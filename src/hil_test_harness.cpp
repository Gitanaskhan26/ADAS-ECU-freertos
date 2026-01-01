#include "hil_test_harness.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>

HILTestHarness::HILTestHarness(const char* can_interface)
    : can_interface_(can_interface)
    , scenario_start_time_(0)
{
    std::memset(&stats_, 0, sizeof(stats_));
}

HILTestHarness::~HILTestHarness() {
}

bool HILTestHarness::initialize() {
    if (!can_interface_.initialize()) {
        printf("HIL: Failed to initialize CAN FD interface\n");
        return false;
    }
    
    printf("HIL Test Harness initialized\n");
    return true;
}

uint64_t HILTestHarness::get_current_time_us() const {
    using namespace std::chrono;
    auto now = high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return duration_cast<microseconds>(duration).count();
}

void HILTestHarness::wait_until(uint64_t timestamp_us) {
    uint64_t current = get_current_time_us();
    if (timestamp_us > current) {
        std::this_thread::sleep_for(
            std::chrono::microseconds(timestamp_us - current));
    }
}

bool HILTestHarness::inject_stimulus(const TestStimulus& stimulus) {
    canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    
    frame.can_id = stimulus.can_id;
    frame.len = stimulus.length;
    frame.flags = CANFD_BRS | CANFD_FDF;
    std::memcpy(frame.data, stimulus.data, stimulus.length);
    
    // Wait until scheduled time
    uint64_t target_time = scenario_start_time_ + stimulus.timestamp_us;
    wait_until(target_time);
    
    bool success = can_interface_.write_canfd_frame(frame);
    if (success) {
        stats_.total_stimuli++;
    }
    
    return success;
}

bool HILTestHarness::compare_data(const uint8_t* actual, const uint8_t* expected,
                                  const uint8_t* mask, uint8_t length) const {
    for (uint8_t i = 0; i < length; i++) {
        if ((actual[i] & mask[i]) != (expected[i] & mask[i])) {
            return false;
        }
    }
    return true;
}

TestResult HILTestHarness::validate_response(const ExpectedResponse& expected) {
    TestResult result;
    result.expected_timestamp_us = expected.timestamp_us;
    
    // Read response frame
    canfd_frame frame;
    uint64_t deadline = scenario_start_time_ + expected.timestamp_us + expected.tolerance_us;
    
    bool received = false;
    while (get_current_time_us() < deadline) {
        if (can_interface_.read_canfd_frame(frame)) {
            if (frame.can_id == expected.can_id) {
                received = true;
                result.actual_timestamp_us = get_current_time_us() - scenario_start_time_;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    if (!received) {
        result.passed = false;
        result.error_message = "No response received within tolerance";
        result.timing_error_us = static_cast<int64_t>(get_current_time_us() - scenario_start_time_ - expected.timestamp_us);
        stats_.failed_tests++;
        return result;
    }
    
    // Check timing
    result.timing_error_us = static_cast<int64_t>(result.actual_timestamp_us - expected.timestamp_us);
    
    if (static_cast<uint64_t>(std::abs(result.timing_error_us)) > expected.tolerance_us) {
        result.passed = false;
        result.error_message = "Response timing out of tolerance";
        stats_.failed_tests++;
        return result;
    }
    
    // Check data
    if (frame.len != expected.length) {
        result.passed = false;
        result.error_message = "Payload length mismatch";
        stats_.failed_tests++;
        return result;
    }
    
    if (!compare_data(frame.data, expected.data, expected.data_mask, frame.len)) {
        result.passed = false;
        result.error_message = "Payload data mismatch";
        stats_.failed_tests++;
        return result;
    }
    
    result.passed = true;
    stats_.passed_tests++;
    stats_.total_responses++;
    
    return result;
}

void HILTestHarness::generate_normal_operation(uint64_t duration_ms) {
    // Generate periodic sensor updates
    const uint64_t period_us = 10000;  // 10ms (100Hz)
    uint64_t num_cycles = (duration_ms * 1000) / period_us;
    
    for (uint64_t i = 0; i < num_cycles; i++) {
        TestStimulus stimulus;
        stimulus.timestamp_us = i * period_us;
        stimulus.can_id = 0x100;  // Lidar CAN ID
        stimulus.length = 16;
        
        // Simulated lidar: object at constant distance
        float x = 50.0f;
        float y = 0.0f;
        std::memcpy(&stimulus.data[0], &x, sizeof(float));
        std::memcpy(&stimulus.data[4], &y, sizeof(float));
        
        stimuli_.push_back(stimulus);
    }
}

void HILTestHarness::generate_collision_imminent(uint64_t duration_ms) {
    // Generate approaching object scenario
    const uint64_t period_us = 10000;
    uint64_t num_cycles = (duration_ms * 1000) / period_us;
    
    for (uint64_t i = 0; i < num_cycles; i++) {
        TestStimulus stimulus;
        stimulus.timestamp_us = i * period_us;
        stimulus.can_id = 0x100;
        stimulus.length = 16;
        
        // Object approaching at 10 m/s
        float t = static_cast<float>(i) * 0.01f;
        float x = 100.0f - 10.0f * t;
        float y = 0.0f;
        
        std::memcpy(&stimulus.data[0], &x, sizeof(float));
        std::memcpy(&stimulus.data[4], &y, sizeof(float));
        
        stimuli_.push_back(stimulus);
        
        // Expect collision warning when TTC < 3s
        if (x < 30.0f && x > 0.0f) {
            ExpectedResponse response;
            response.timestamp_us = (i + 1) * period_us;
            response.tolerance_us = 5000;  // 5ms tolerance
            response.can_id = 0x500;  // Safety warning CAN ID
            response.length = 1;
            response.data[0] = 0x01;  // Collision warning flag
            std::memset(response.data_mask, 0xFF, sizeof(response.data_mask));
            
            expected_responses_.push_back(response);
        }
    }
}

void HILTestHarness::generate_sensor_failure(uint64_t duration_ms) {
    // Generate scenario with sensor dropout
    const uint64_t period_us = 10000;
    uint64_t num_cycles = (duration_ms * 1000) / period_us;
    
    for (uint64_t i = 0; i < num_cycles; i++) {
        // Skip frames from i=50 to i=100 to simulate dropout
        if (i >= 50 && i < 100) {
            continue;
        }
        
        TestStimulus stimulus;
        stimulus.timestamp_us = i * period_us;
        stimulus.can_id = 0x100;
        stimulus.length = 8;
        
        float x = 50.0f;
        std::memcpy(&stimulus.data[0], &x, sizeof(float));
        
        stimuli_.push_back(stimulus);
    }
    
    // Expect safety system to detect dropout
    ExpectedResponse response;
    response.timestamp_us = 60 * period_us;
    response.tolerance_us = 50000;  // 50ms tolerance
    response.can_id = 0x500;
    response.length = 1;
    response.data[0] = 0x02;  // Sensor fault flag
    std::memset(response.data_mask, 0xFF, sizeof(response.data_mask));
    
    expected_responses_.push_back(response);
}

void HILTestHarness::generate_rapid_acceleration(uint64_t duration_ms) {
    const uint64_t period_us = 10000;
    uint64_t num_cycles = (duration_ms * 1000) / period_us;
    
    for (uint64_t i = 0; i < num_cycles; i++) {
        TestStimulus stimulus;
        stimulus.timestamp_us = i * period_us;
        stimulus.can_id = 0x101;  // Radar CAN ID
        stimulus.length = 16;
        
        // Simulate rapid acceleration: v = 2*t
        float t = static_cast<float>(i) * 0.01f;
        float vx = 2.0f * t;
        
        std::memcpy(&stimulus.data[0], &vx, sizeof(float));
        stimuli_.push_back(stimulus);
    }
}

void HILTestHarness::generate_emergency_braking(uint64_t duration_ms) {
    const uint64_t period_us = 10000;
    uint64_t num_cycles = (duration_ms * 1000) / period_us;
    
    for (uint64_t i = 0; i < num_cycles; i++) {
        TestStimulus stimulus;
        stimulus.timestamp_us = i * period_us;
        stimulus.can_id = 0x101;
        stimulus.length = 16;
        
        // Simulate emergency braking: deceleration = -8 m/s^2
        float t = static_cast<float>(i) * 0.01f;
        float vx = std::max(0.0f, 20.0f - 8.0f * t);
        
        std::memcpy(&stimulus.data[0], &vx, sizeof(float));
        stimuli_.push_back(stimulus);
    }
}

void HILTestHarness::generate_multi_object_tracking(uint64_t duration_ms) {
    // Generate scenario with 3 objects
    const uint64_t period_us = 10000;
    uint64_t num_cycles = (duration_ms * 1000) / period_us;
    
    for (uint64_t i = 0; i < num_cycles; i++) {
        float t = static_cast<float>(i) * 0.01f;
        
        // Object 1: stationary
        TestStimulus obj1;
        obj1.timestamp_us = i * period_us;
        obj1.can_id = 0x100;
        obj1.length = 16;
        float x1 = 30.0f, y1 = 0.0f;
        std::memcpy(&obj1.data[0], &x1, sizeof(float));
        std::memcpy(&obj1.data[4], &y1, sizeof(float));
        stimuli_.push_back(obj1);
        
        // Object 2: moving left
        TestStimulus obj2;
        obj2.timestamp_us = i * period_us + 1000;
        obj2.can_id = 0x100;
        obj2.length = 16;
        float x2 = 50.0f, y2 = -5.0f * t;
        std::memcpy(&obj2.data[0], &x2, sizeof(float));
        std::memcpy(&obj2.data[4], &y2, sizeof(float));
        stimuli_.push_back(obj2);
        
        // Object 3: approaching
        TestStimulus obj3;
        obj3.timestamp_us = i * period_us + 2000;
        obj3.can_id = 0x100;
        obj3.length = 16;
        float x3 = 80.0f - 3.0f * t, y3 = 2.0f;
        std::memcpy(&obj3.data[0], &x3, sizeof(float));
        std::memcpy(&obj3.data[4], &y3, sizeof(float));
        stimuli_.push_back(obj3);
    }
}

bool HILTestHarness::generate_scenario(TestScenarioType type, uint64_t duration_ms) {
    stimuli_.clear();
    expected_responses_.clear();
    results_.clear();
    
    switch (type) {
        case TestScenarioType::NORMAL_OPERATION:
            generate_normal_operation(duration_ms);
            break;
        case TestScenarioType::COLLISION_IMMINENT:
            generate_collision_imminent(duration_ms);
            break;
        case TestScenarioType::SENSOR_FAILURE:
            generate_sensor_failure(duration_ms);
            break;
        case TestScenarioType::RAPID_ACCELERATION:
            generate_rapid_acceleration(duration_ms);
            break;
        case TestScenarioType::EMERGENCY_BRAKING:
            generate_emergency_braking(duration_ms);
            break;
        case TestScenarioType::MULTI_OBJECT_TRACKING:
            generate_multi_object_tracking(duration_ms);
            break;
        default:
            return false;
    }
    
    printf("HIL: Generated scenario with %zu stimuli\n", stimuli_.size());
    return true;
}

bool HILTestHarness::run_scenario() {
    if (stimuli_.empty()) {
        printf("HIL: No scenario loaded\n");
        return false;
    }
    
    printf("HIL: Starting scenario with %zu stimuli, %zu expected responses\n",
           stimuli_.size(), expected_responses_.size());
    
    scenario_start_time_ = get_current_time_us();
    
    // Inject all stimuli
    for (const auto& stimulus : stimuli_) {
        if (!inject_stimulus(stimulus)) {
            printf("HIL: Failed to inject stimulus at t=%lu us\n", stimulus.timestamp_us);
        }
    }
    
    // Validate expected responses
    for (const auto& expected : expected_responses_) {
        TestResult result = validate_response(expected);
        results_.push_back(result);
        
        if (!result.passed) {
            printf("HIL: Validation failed at t=%lu us: %s\n",
                   expected.timestamp_us, result.error_message.c_str());
        }
    }
    
    // Calculate statistics
    stats_.pass_rate = (stats_.total_responses > 0)
        ? (static_cast<double>(stats_.passed_tests) / stats_.total_responses) * 100.0
        : 0.0;
    
    int64_t total_error = 0;
    stats_.max_timing_error_us = 0;
    for (const auto& result : results_) {
        total_error += result.timing_error_us;
        if (std::abs(result.timing_error_us) > std::abs(stats_.max_timing_error_us)) {
            stats_.max_timing_error_us = result.timing_error_us;
        }
    }
    stats_.avg_timing_error_us = results_.empty() ? 0 : total_error / static_cast<int64_t>(results_.size());
    
    printf("HIL: Scenario complete - Pass rate: %.1f%% (%u/%u)\n",
           stats_.pass_rate, stats_.passed_tests, stats_.total_responses);
    
    return (stats_.failed_tests == 0);
}

bool HILTestHarness::export_report(const std::string& filename) const {
    std::ofstream report(filename);
    if (!report.is_open()) {
        return false;
    }
    
    report << "HIL Test Report\n";
    report << "===============\n\n";
    report << "Statistics:\n";
    report << "  Total Stimuli: " << stats_.total_stimuli << "\n";
    report << "  Total Responses: " << stats_.total_responses << "\n";
    report << "  Passed Tests: " << stats_.passed_tests << "\n";
    report << "  Failed Tests: " << stats_.failed_tests << "\n";
    report << "  Pass Rate: " << stats_.pass_rate << "%\n";
    report << "  Max Timing Error: " << stats_.max_timing_error_us << " us\n";
    report << "  Avg Timing Error: " << stats_.avg_timing_error_us << " us\n\n";
    
    report << "Detailed Results:\n";
    for (size_t i = 0; i < results_.size(); i++) {
        const auto& result = results_[i];
        report << "  Test " << i << ": " << (result.passed ? "PASS" : "FAIL") << "\n";
        report << "    Expected Time: " << result.expected_timestamp_us << " us\n";
        report << "    Actual Time: " << result.actual_timestamp_us << " us\n";
        report << "    Timing Error: " << result.timing_error_us << " us\n";
        if (!result.passed) {
            report << "    Error: " << result.error_message << "\n";
        }
    }
    
    report.close();
    return true;
}

bool HILTestHarness::load_scenario(const std::string& /* scenario_file */) {
    // Placeholder for loading custom scenarios from file
    printf("HIL: Custom scenario loading not yet implemented\n");
    return false;
}
