#ifndef HIL_TEST_HARNESS_HPP
#define HIL_TEST_HARNESS_HPP

#include "sensor_data.hpp"
#include "can_fd_interface.hpp"
#include <vector>
#include <string>
#include <cstdint>

/**
 * Hardware-in-the-Loop (HIL) Testing Framework
 * 
 * Enables real-time testing with simulated/physical hardware:
 * - Stimulus generation (sensor inputs, CAN messages)
 * - Response validation (output verification, timing checks)
 * - Automated test scenarios
 * - Performance metrics collection
 */

struct TestStimulus {
    uint64_t timestamp_us;      // Microseconds from test start
    uint32_t can_id;            // CAN message ID
    uint8_t data[64];           // CAN FD payload
    uint8_t length;             // Payload length
};

struct ExpectedResponse {
    uint64_t timestamp_us;      // Expected response time
    uint64_t tolerance_us;      // Timing tolerance
    uint32_t can_id;            // Expected CAN ID
    uint8_t data[64];           // Expected payload
    uint8_t data_mask[64];      // Mask for "don't care" bits
    uint8_t length;
};

struct TestResult {
    bool passed;
    uint64_t actual_timestamp_us;
    uint64_t expected_timestamp_us;
    int64_t timing_error_us;
    std::string error_message;
};

enum class TestScenarioType {
    NORMAL_OPERATION,
    COLLISION_IMMINENT,
    SENSOR_FAILURE,
    RAPID_ACCELERATION,
    EMERGENCY_BRAKING,
    MULTI_OBJECT_TRACKING,
    CUSTOM
};

class HILTestHarness {
public:
    HILTestHarness(const char* can_interface);
    ~HILTestHarness();
    
    // Initialize test harness
    bool initialize();
    
    // Load test scenario from file
    bool load_scenario(const std::string& scenario_file);
    
    // Generate built-in test scenario
    bool generate_scenario(TestScenarioType type, uint64_t duration_ms);
    
    // Execute test scenario
    bool run_scenario();
    
    // Inject single stimulus
    bool inject_stimulus(const TestStimulus& stimulus);
    
    // Validate response against expectations
    TestResult validate_response(const ExpectedResponse& expected);
    
    // Get test statistics
    struct Statistics {
        uint32_t total_stimuli;
        uint32_t total_responses;
        uint32_t passed_tests;
        uint32_t failed_tests;
        int64_t max_timing_error_us;
        int64_t avg_timing_error_us;
        double pass_rate;
    };
    Statistics get_statistics() const { return stats_; }
    
    // Export test report
    bool export_report(const std::string& filename) const;
    
private:
    CANFDInterface can_interface_;
    std::vector<TestStimulus> stimuli_;
    std::vector<ExpectedResponse> expected_responses_;
    std::vector<TestResult> results_;
    Statistics stats_;
    uint64_t scenario_start_time_;
    
    // Scenario generators
    void generate_normal_operation(uint64_t duration_ms);
    void generate_collision_imminent(uint64_t duration_ms);
    void generate_sensor_failure(uint64_t duration_ms);
    void generate_rapid_acceleration(uint64_t duration_ms);
    void generate_emergency_braking(uint64_t duration_ms);
    void generate_multi_object_tracking(uint64_t duration_ms);
    
    // Timing utilities
    uint64_t get_current_time_us() const;
    void wait_until(uint64_t timestamp_us);
    
    // Validation helpers
    bool compare_data(const uint8_t* actual, const uint8_t* expected,
                     const uint8_t* mask, uint8_t length) const;
};

#endif // HIL_TEST_HARNESS_HPP
