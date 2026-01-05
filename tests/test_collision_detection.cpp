// Test suite for collision detection module
// Verifies TTC calculation and risk level assignment

#include "../include/collision_detection.hpp"
#include "../include/sensor_data.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

void test_collision_detector_initialization() {
    printf("Test: CollisionDetector initialization...\n");
    CollisionDetector detector;
    printf("  ✓ CollisionDetector created\n");
}

void test_no_collision_stationary() {
    printf("Test: No collision for stationary distant object...\n");
    CollisionDetector detector;
    
    StateVector state;
    state.x = 100.0f;
    state.y = 0.0f;
    state.vx = 0.0f;
    state.vy = 0.0f;
    state.timestamp = 0;
    
    CollisionInfo info = detector.detect_collision(state);
    
    assert(info.risk_level == CollisionRisk::NONE);
    assert(!info.is_approaching);
    printf("  ✓ No collision detected for stationary object at 100m\n");
}

void test_collision_approaching() {
    printf("Test: Collision warning for approaching object...\n");
    CollisionDetector detector;
    
    StateVector state;
    state.x = 20.0f;
    state.y = 0.0f;
    state.vx = -10.0f;
    state.vy = 0.0f;
    state.timestamp = 0;
    
    CollisionInfo info = detector.detect_collision(state);
    
    assert(info.is_approaching);
    assert(info.time_to_collision < 3.0f);
    printf("  ✓ Collision detected: TTC=%.2fs, Risk=%d\n", 
           info.time_to_collision, static_cast<int>(info.risk_level));
}

void test_critical_collision() {
    printf("Test: Critical collision for imminent impact...\n");
    CollisionDetector detector;
    
    StateVector state;
    state.x = 5.0f;
    state.y = 0.0f;
    state.vx = -10.0f;
    state.vy = 0.0f;
    state.timestamp = 0;
    
    CollisionInfo info = detector.detect_collision(state);
    
    assert(info.risk_level == CollisionRisk::CRITICAL);
    assert(detector.should_trigger_brake(info));
    printf("  ✓ Critical collision: TTC=%.2fs, emergency brake triggered\n", 
           info.time_to_collision);
}

void test_fsr_001_5_requirement() {
    printf("Test: FSR-001.5 - Collision warning at TTC < 3.0s...\n");
    CollisionDetector detector;
    
    StateVector state;
    state.x = 25.0f;
    state.y = 0.0f;
    state.vx = -10.0f;
    state.vy = 0.0f;
    state.timestamp = 0;
    
    CollisionInfo info = detector.detect_collision(state);
    
    assert(info.time_to_collision < 3.0f);
    assert(info.risk_level >= CollisionRisk::WARNING);
    printf("  ✓ FSR-001.5 verified: Warning triggers at TTC < 3.0s\n");
}

void test_multiple_simultaneous_collisions() {
    printf("Test: Multiple simultaneous collision threats...\n");
    CollisionDetector detector;
    
    // Test with first object
    StateVector state1;
    state1.x = 40.0f;
    state1.y = 0.0f;
    state1.vx = -10.0f;
    state1.vy = 0.0f;
    state1.timestamp = 0;
    
    CollisionInfo info1 = detector.detect_collision(state1);
    assert(info1.is_approaching);
    
    // Test with second object
    StateVector state2;
    state2.x = 50.0f;
    state2.y = -2.0f;
    state2.vx = -15.0f;
    state2.vy = 0.0f;
    state2.timestamp = 0;
    
    CollisionInfo info2 = detector.detect_collision(state2);
    assert(info2.is_approaching);
    
    printf("  ✓ Multiple threats handled: TTC1=%.2fs, TTC2=%.2fs\n", 
           info1.time_to_collision, info2.time_to_collision);
}

void test_extreme_relative_velocity() {
    printf("Test: Extreme relative velocity handling...\n");
    CollisionDetector detector;
    
    StateVector state;
    state.x = 100.0f;
    state.y = 0.0f;
    state.vx = -50.0f;  // Very high closing speed
    state.vy = 0.0f;
    state.timestamp = 0;
    
    CollisionInfo info = detector.detect_collision(state);
    
    // High relative velocity should give low TTC
    assert(info.is_approaching);
    assert(info.time_to_collision < 3.0f);
    assert(info.risk_level >= CollisionRisk::WARNING);
    
    printf("  ✓ Extreme velocity handled: TTC=%.2fs (v=-50m/s)\n", 
           info.time_to_collision);
}

int main() {
    printf("========================================\n");
    printf(" Collision Detection Test Suite\n");
    printf(" Traceability: SR-005, FSR-001.5\n");
    printf("========================================\n\n");
    
    test_collision_detector_initialization();
    test_no_collision_stationary();
    test_collision_approaching();
    test_critical_collision();
    test_fsr_001_5_requirement();
    test_multiple_simultaneous_collisions();
    test_extreme_relative_velocity();
    
    printf("\n========================================\n");
    printf(" ✓ All collision detection tests PASSED\n");
    printf(" Requirements verified: SR-005, FSR-001.5\n");
    printf("========================================\n");
    
    return 0;
}
